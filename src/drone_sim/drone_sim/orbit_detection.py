#!/usr/bin/env python3
"""
orbit_detection.py

Runs everything in a single terminal:
  1. Circular orbit around person_1 (velocity setpoint + PurePursuitOrbit)
  2. Realtime YOLO person detection from drone camera
  3. Synchronized IMU + Camera data collection -> saved to ~/orbit_imu_data/

Run:
  source /opt/ros/jazzy/setup.bash
  source ~/DIPLOM/ws_drone/install/setup.bash
  ~/venv_ros2_yolo/bin/python ~/DIPLOM/ws_drone/src/drone_sim/drone_sim/orbit_detection.py
"""

import csv
import math
import os
import time
from collections import deque
from datetime import datetime

import cv2
import torch
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import (OffboardControlMode, SensorCombined, TrajectorySetpoint,
                           VehicleCommand, VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from sensor_msgs.msg import Image
from ultralytics import YOLO

from drone_sim.coord_convert import enu_to_ned, ned_to_enu
from drone_sim.drone_controller import PurePursuitOrbit

# ── Constants ──────────────────────────────────────────────────────────────────
# Orbit center = person_1 ENU(10, 0, 0) converted to NED
_center_ned    = enu_to_ned(10.0, 0.0, 0.0)
CENTER_X       = _center_ned.x    #  0.0  NED North
CENTER_Y       = _center_ned.y    # 10.0  NED East

ORBIT_RADIUS   = 9.0    # orbit radius (m)
ORBIT_ALTITUDE = 5.0    # flight altitude (m, positive = above ground)
ORBIT_SPEED    = 0.4    # angular speed (rad/s) — ~1 lap per 16s
MAX_XY_SPEED   = 4.0    # max horizontal velocity (m/s)
MAX_Z_SPEED    = 1.5    # max vertical velocity (m/s)
ALT_KP         = 1.2    # proportional gain for altitude hold
LAPS           = 2      # number of laps

# YOLO
MODEL_NAME  = 'yolov8s.pt'
CONF_THRESH = 0.7
PERSON_CLS  = 0
DEVICE      = 'cuda' if torch.cuda.is_available() else 'cpu'

# IMU-Camera collector
IMU_BUF_SIZE  = 200     # buffer ~1s (200 x 5ms)
MAX_SYNC_MS   = 20      # timestamp sync tolerance (ms)
JPEG_QUALITY  = 90
OUTPUT_BASE   = os.path.expanduser('~/orbit_imu_data')

DT = 0.05   # 20 Hz


# ── State machine ──────────────────────────────────────────────────────────────
class State:
    INIT    = 'INIT'
    TAKEOFF = 'TAKEOFF'
    ORBIT   = 'ORBIT'
    LAND    = 'LAND'
    DONE    = 'DONE'


class OrbitDetection(Node):

    def __init__(self):
        super().__init__('orbit_detection')

        # ── QoS ───────────────────────────────────────────────────────────────
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_pub)

        # ── Subscribers ────────────────────────────────────────────────────────
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v3',
            self._status_cb, qos_sub)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, qos_sub)
        self.create_subscription(
            Image, '/camera/image_raw',
            self._image_cb, 10)
        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self._imu_cb, qos_sub)   # BEST_EFFORT + TRANSIENT_LOCAL to match PX4 publisher

        # ── Flight state ───────────────────────────────────────────────────────
        self.state         = State.INIT
        self.pos_x         = 0.0
        self.pos_y         = 0.0
        self.pos_z         = 0.0
        self.counter       = 0
        self.nav_state     = -1
        self.arming_state  = -1
        self._pos_received = False

        # ── PurePursuitOrbit ───────────────────────────────────────────────────
        self.orbit = PurePursuitOrbit(
            center_x=CENTER_X,
            center_y=CENTER_Y,
            radius=ORBIT_RADIUS,
            altitude=ORBIT_ALTITUDE,
            speed=ORBIT_SPEED,
            lookahead=2.0,
        )

        # ── Detection state ────────────────────────────────────────────────────
        self.bridge           = CvBridge()
        self._prev_person_cnt = -1
        self._display_state   = State.INIT
        self._display_laps    = 0

        # ── IMU buffer ─────────────────────────────────────────────────────────
        self._imu_buf: deque = deque(maxlen=IMU_BUF_SIZE)
        self._last_imu = None   # most recent IMU sample received

        # ── IMU-Camera collector output ────────────────────────────────────────
        session = datetime.now().strftime('session_%Y%m%d_%H%M%S')
        self._session_dir = os.path.join(OUTPUT_BASE, session)
        self._frames_dir  = os.path.join(self._session_dir, 'frames')
        os.makedirs(self._frames_dir, exist_ok=True)
        csv_path = os.path.join(self._session_dir, 'imu_log.csv')
        self._csv_file   = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'frame_id', 'cam_ts_us', 'imu_ts_us', 'dt_sync_ms',
            'gx_rad_s', 'gy_rad_s', 'gz_rad_s',
            'ax_m_s2',  'ay_m_s2',  'az_m_s2',
        ])
        self._frame_id    = 0
        self._sync_ok     = 0
        self._sync_fail   = 0
        self._last_stat_t = time.time()

        # ── Load YOLO ─────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading {MODEL_NAME} on {DEVICE} ...')
        self.model = YOLO(MODEL_NAME)
        self.model.to(DEVICE)

        # ── Timer 20 Hz ────────────────────────────────────────────────────────
        self.create_timer(DT, self._loop)

        # ── Startup log ────────────────────────────────────────────────────────
        self.get_logger().info('=' * 55)
        self.get_logger().info('Node started  [velocity setpoint mode]')
        self.get_logger().info(f'  Orbit center ENU (Gazebo) : x=10.0  y=0.0  (person_1)')
        self.get_logger().info(f'  Orbit center NED (PX4)    : x={CENTER_X}  y={CENTER_Y}')
        self.get_logger().info(f'  Trajectory : circle  r={ORBIT_RADIUS}m  h={ORBIT_ALTITUDE}m')
        self.get_logger().info(f'  Angular speed : {ORBIT_SPEED} rad/s  (~{2*math.pi/ORBIT_SPEED:.0f}s/lap)')
        self.get_logger().info(f'  Max XY speed  : {MAX_XY_SPEED} m/s')
        self.get_logger().info(f'  Laps          : {LAPS}')
        self.get_logger().info(f'  Model         : {MODEL_NAME}  conf={CONF_THRESH}  device={DEVICE}')
        self.get_logger().info(f'  Output        : {self._session_dir}')
        self.get_logger().info('=' * 55)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _status_cb(self, msg):
        self.nav_state    = msg.nav_state
        self.arming_state = msg.arming_state

    def _pos_cb(self, msg):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z

        if not self._pos_received:
            self._pos_received = True
            drone_enu = ned_to_enu(self.pos_x, self.pos_y, self.pos_z)
            self.get_logger().info('=' * 55)
            self.get_logger().info('[DRONE SPAWN]')
            self.get_logger().info(
                f'  NED : x={self.pos_x:.3f}  y={self.pos_y:.3f}  z={self.pos_z:.3f}')
            self.get_logger().info(
                f'  ENU : x={drone_enu.x:.3f}  y={drone_enu.y:.3f}  z={drone_enu.z:.3f}')
            self.get_logger().info('=' * 55)

    def _imu_cb(self, msg: SensorCombined):
        """Store the latest IMU sample (overwrite — always keep the most recent)."""
        self._last_imu = {
            'px4_ts_us': msg.timestamp,   # PX4 internal clock (us)
            'wall_s':    time.time(),      # wall clock at reception
            'gx':  float(msg.gyro_rad[0]),
            'gy':  float(msg.gyro_rad[1]),
            'gz':  float(msg.gyro_rad[2]),
            'ax':  float(msg.accelerometer_m_s2[0]),
            'ay':  float(msg.accelerometer_m_s2[1]),
            'az':  float(msg.accelerometer_m_s2[2]),
        }

    def _image_cb(self, msg):
        """Receive frame -> YOLO inference -> save IMU+frame -> display."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')
            return

        h, w = frame.shape[:2]
        self._frame_id += 1

        # ── YOLO inference ─────────────────────────────────────────────────────
        results    = self.model(frame, classes=[PERSON_CLS],
                                conf=CONF_THRESH, device=DEVICE, verbose=False)
        boxes      = results[0].boxes
        person_cnt = 0
        annotated  = frame.copy()

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                conf = float(box.conf[0])
                person_cnt += 1
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated,
                    f'person {conf:.2f}',
                    (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # ── Log on detection state change ──────────────────────────────────────
        if person_cnt != self._prev_person_cnt:
            if person_cnt > 0:
                self.get_logger().info(
                    f'[DETECTION] Person detected — count: {person_cnt}')
            else:
                if self._prev_person_cnt > 0:
                    self.get_logger().info(
                        '[DETECTION] No person in frame')
            self._prev_person_cnt = person_cnt

        # ── IMU sync + save ────────────────────────────────────────────────────
        # Use the most recent IMU sample without timestamp comparison.
        # PX4 and Gazebo use different clock epochs so wall-clock is used instead.
        # IMU runs at 250 Hz so the latest sample is always fresh enough.
        imu = self._last_imu

        if imu is not None:
            # Save frame
            fpath = os.path.join(self._frames_dir,
                                 f'frame_{self._frame_id:06d}.jpg')
            cv2.imwrite(fpath, frame,
                        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            # Write CSV row
            cam_ts = float(msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9
            dt_ms  = (time.time() - imu['wall_s']) * 1000.0
            self._csv_writer.writerow([
                self._frame_id,
                f'{cam_ts:.6f}',
                imu['px4_ts_us'],
                f'{dt_ms:.1f}',
                f"{imu['gx']:.6f}", f"{imu['gy']:.6f}", f"{imu['gz']:.6f}",
                f"{imu['ax']:.6f}", f"{imu['ay']:.6f}", f"{imu['az']:.6f}",
            ])
            self._csv_file.flush()
            self._sync_ok += 1

            gyro_norm = math.sqrt(imu['gx']**2 + imu['gy']**2 + imu['gz']**2)
            cv2.putText(annotated,
                f'|w|={gyro_norm:.3f} rad/s  gz={imu["gz"]:.3f}',
                (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        else:
            self._sync_fail += 1
            if self._frame_id <= 3:
                self.get_logger().warn(
                    '[IMU-CAM] No IMU data yet — '
                    'check /fmu/out/sensor_combined')

        # ── Print stats every 5s ───────────────────────────────────────────────
        now = time.time()
        if now - self._last_stat_t >= 5.0:
            self._last_stat_t = now
            total = self._sync_ok + self._sync_fail
            rate  = (self._sync_ok / total * 100) if total > 0 else 0.0
            self.get_logger().info(
                f'[IMU-CAM] saved={self._sync_ok}  fail={self._sync_fail}'
                f'  sync_rate={rate:.0f}%')

        # ── Main overlay ───────────────────────────────────────────────────────
        status     = f'DETECTED: {person_cnt} person(s)' if person_cnt > 0 else 'Searching...'
        status_col = (0, 255, 0) if person_cnt > 0 else (0, 165, 255)
        cv2.putText(annotated, status,
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_col, 2)
        cv2.putText(annotated,
            f'State:{self._display_state}  Lap:{self._display_laps}/{LAPS}'
            f'  Angle:{self.orbit.current_angle_deg:.0f}deg  [{DEVICE.upper()}]',
            (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.putText(annotated,
            f'IMU saved:{self._sync_ok}',
            (w - 160, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        cv2.imshow('Orbit Detection', annotated)
        cv2.waitKey(1)

    # ── Control loop 20 Hz ─────────────────────────────────────────────────────

    def _loop(self):
        self._pub_offboard_mode()
        self._display_state = self.state

        if self.state == State.INIT:
            self._do_init()
        elif self.state == State.TAKEOFF:
            self._do_takeoff()
        elif self.state == State.ORBIT:
            self._do_orbit()
        elif self.state == State.LAND:
            self._do_land()

    # ── States ─────────────────────────────────────────────────────────────────

    def _do_init(self):
        # Use position setpoint during INIT to keep drone in place
        self._pub_position_setpoint(0.0, 0.0, -ORBIT_ALTITUDE)
        self.counter += 1

        in_offboard = (self.nav_state == 14)
        is_armed    = (self.arming_state == 2)

        if self.counter % 40 == 0:
            self.get_logger().info(
                f'[INIT] counter={self.counter} '
                f'nav={self.nav_state}(offboard={in_offboard}) '
                f'arm={self.arming_state}(armed={is_armed})')

        if self.counter < 10:
            return

        if not in_offboard:
            self._pub_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                              param1=1.0, param2=6.0)
            return

        if not is_armed:
            if not hasattr(self, '_arm_sent'):
                self._arm_sent = True
                self._pub_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                  param1=1.0)
                self.get_logger().info('[INIT] OFFBOARD confirmed -> ARM sent')
            return

        self.state = State.TAKEOFF
        self.get_logger().info('[INIT] ARMED + OFFBOARD -> TAKEOFF')

    def _do_takeoff(self):
        """
        Take off in place to ORBIT_ALTITUDE using position setpoint.
        Switch to ORBIT (velocity mode) once altitude is reached.
        """
        self._pub_position_setpoint(0.0, 0.0, -ORBIT_ALTITUDE)

        dist_z = abs(self.pos_z - (-ORBIT_ALTITUDE))
        if dist_z < 0.5:
            # Initialize orbit angle from current drone position to avoid sudden jump
            self.orbit.init_angle_from_position(self.pos_x, self.pos_y)
            self.state = State.ORBIT
            self.get_logger().info('=' * 55)
            self.get_logger().info('[ORBIT START] Switching to velocity setpoint')
            self.get_logger().info(
                f'  drone NED : x={self.pos_x:.2f}  y={self.pos_y:.2f}  z={self.pos_z:.2f}')
            self.get_logger().info(
                f'  initial orbit angle : {self.orbit.current_angle_deg:.1f} deg')
            self.get_logger().info('=' * 55)

    def _do_orbit(self):
        """
        Smooth circular flight using velocity setpoint.
        PurePursuitOrbit computes the target point -> velocity vector toward target.
        """
        sp = self.orbit.step(self.pos_x, self.pos_y, dt=DT)

        # ── XY velocity: proportional to position error ────────────────────────
        # gain=2.0 -> 2m error gives vx=4m/s (clamped by MAX_XY_SPEED)
        gain   = 2.0
        err_x  = sp.x - self.pos_x
        err_y  = sp.y - self.pos_y
        vx     = _clamp(err_x * gain, -MAX_XY_SPEED, MAX_XY_SPEED)
        vy     = _clamp(err_y * gain, -MAX_XY_SPEED, MAX_XY_SPEED)

        # ── Z velocity: altitude hold ──────────────────────────────────────────
        target_z = -ORBIT_ALTITUDE   # NED: negative = above ground
        vz = _clamp((target_z - self.pos_z) * ALT_KP, -MAX_Z_SPEED, MAX_Z_SPEED)

        # ── Yaw: always face orbit center ─────────────────────────────────────
        yaw = sp.yaw

        self._pub_velocity_setpoint(vx, vy, vz, yaw)

        # ── Lap counting ──────────────────────────────────────────────────────
        self._display_laps = self.orbit.laps_done
        if self.orbit.laps_done >= LAPS:
            self.get_logger().info(
                f'[ORBIT] Completed {LAPS} lap(s) -> LAND')
            self.state = State.LAND

    def _do_land(self):
        self._pub_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = State.DONE
        self.get_logger().info('[LAND] Land command sent -> DONE')
        self.create_timer(2.0, self._shutdown)

    def _shutdown(self):
        self.get_logger().info('[DONE] Shutting down')
        raise SystemExit

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _pub_offboard_mode(self):
        msg = OffboardControlMode()
        # ORBIT state: velocity mode; INIT/TAKEOFF/LAND: position mode
        if self.state in (State.ORBIT,):
            msg.position     = False
            msg.velocity     = True
        else:
            msg.position     = True
            msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def _pub_position_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position  = [float(x), float(y), float(z)]
        msg.velocity  = [float('nan'), float('nan'), float('nan')]
        msg.yaw       = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _pub_velocity_setpoint(self, vx, vy, vz, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position  = [float('nan'), float('nan'), float('nan')]
        msg.velocity  = [float(vx), float(vy), float(vz)]
        msg.yaw       = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _pub_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


# ── Utility ────────────────────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ── Main ───────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OrbitDetection()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node._csv_file.flush()
            node._csv_file.close()
            node.get_logger().info(
                f'[DONE] CSV closed. Saved {node._sync_ok} frame(s) -> {node._session_dir}')
        except Exception:
            pass
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
