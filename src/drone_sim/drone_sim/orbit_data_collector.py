#!/usr/bin/env python3
"""
orbit_data_collector.py

1. Wait for EKF2 ready -> OFFBOARD + ARM automatically
2. Fly to orbit start point (target + radius, 0)
3. Fly 3 circular laps at fixed radius 5m around person_1 (10, 0)
4. After MAX_RECORD_SEC seconds from recording start -> save video + land + exit

Video recorded from the first frame received until landing.
No keyboard input, no ACK retry, no variable radius.

Coordinates:
  - SDF/Gazebo uses ENU : x=East,  y=North, z=Up
  - PX4/this code uses NED: x=North, y=East, z=Down (negative = above ground)
  - All TARGET_* values are in NED, converted from SDF via coord_convert.enu_to_ned()

Usage:
  ros2 run drone_sim orbit_data_collector
"""

import math
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                           VehicleCommand, VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from sensor_msgs.msg import Image

from drone_sim.coord_convert import enu_to_ned, ned_to_enu


# ── Constants ─────────────────────────────────────────────────────────────────
# person_1 coordinates from baylands.sdf (ENU / Gazebo frame):
#   <pose>10 0 1.0 0 0 0</pose>  ->  ENU: x=10, y=0, z=1.0
#
# Converted to NED (PX4 frame) via coord_convert:
#   enu_to_ned(x=10, y=0, z=1.0) -> NED: x=0, y=10, z=-1.0
#
_person1_ned  = enu_to_ned(10.0, 0.0, 1.0)   # ENU from SDF -> NED for PX4
_person1_enu  = ned_to_enu(_person1_ned.x, _person1_ned.y, _person1_ned.z)  # keep original ENU for logging

TARGET_X       = _person1_ned.x   #  0.0  (NED North)
TARGET_Y       = _person1_ned.y   # 10.0  (NED East)
ORBIT_RADIUS   =  9.0   # fixed orbit radius (m)
ORBIT_ALTITUDE =  5.0   # flight altitude (m, positive = above ground)
ORBIT_SPEED    =  0.3   # angular speed (rad/s)
LAPS           =  3     # number of laps
MAX_RECORD_SEC = 300.0  # auto-stop after 5 minutes (300s)

# ── Video filename — change before each run ───────────────────────────────────
VIDEO_NAME     = 'orbit_9m'   # output file will be: orbit_9m_YYYYMMDD_HHMMSS.mp4

VIDEO_DIR = os.path.expanduser('~/orbit_videos')


# ── State machine ─────────────────────────────────────────────────────────────
class State:
    INIT     = 'INIT'
    TAKEOFF  = 'TAKEOFF'
    ORBIT    = 'ORBIT'
    LAND     = 'LAND'
    DONE     = 'DONE'


class OrbitDataCollector(Node):

    def __init__(self):
        super().__init__('orbit_data_collector')

        os.makedirs(VIDEO_DIR, exist_ok=True)

        # ── QoS ──────────────────────────────────────────────────────────────
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

        # ── Publishers ────────────────────────────────────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_pub)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v3',
            self._status_cb, qos_sub)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, qos_sub)
        self.create_subscription(
            Image, '/camera/image_raw',
            self._image_cb, 10)

        # ── State ─────────────────────────────────────────────────────────────
        self.state           = State.INIT
        self.pos_x           = 0.0
        self.pos_y           = 0.0
        self.pos_z           = 0.0
        self.counter         = 0
        self.nav_state       = -1
        self.arming_state    = -1
        self._pos_received   = False   # flag: first position not yet received

        self.orbit_angle     = 0.0
        self.lap_angle_acc   = 0.0
        self.current_lap     = 0

        # ── Accumulate positions to compute actual orbit center ───────────────
        self._orbit_sum_x    = 0.0   # sum of pos_x in current lap
        self._orbit_sum_y    = 0.0   # sum of pos_y in current lap
        self._orbit_sum_dist = 0.0   # sum of distances to person_1
        self._orbit_count    = 0     # sample count in current lap

        # ── Video ─────────────────────────────────────────────────────────────
        self.bridge          = CvBridge()
        self.video_writer    = None
        self.record_start_t  = None   # time.monotonic() when recording started
        timestamp            = time.strftime('%Y%m%d_%H%M%S')
        self.video_path      = os.path.join(VIDEO_DIR, f'{VIDEO_NAME}_{timestamp}.mp4')

        # ── Timer 20 Hz ───────────────────────────────────────────────────────
        self.create_timer(0.05, self._loop)

        self.get_logger().info('='*55)
        self.get_logger().info('OrbitDataCollector started')
        self.get_logger().info(f'  person_1 ENU (SDF) : x={_person1_enu.x}  y={_person1_enu.y}  z={_person1_enu.z}')
        self.get_logger().info(f'  person_1 NED (PX4) : x={TARGET_X}  y={TARGET_Y}')
        self.get_logger().info(f'  [drone spawn position will be printed on first position received from PX4]')
        self.get_logger().info(f'  Radius   : {ORBIT_RADIUS} m')
        self.get_logger().info(f'  Altitude : {ORBIT_ALTITUDE} m')
        self.get_logger().info(f'  Laps     : {LAPS}')
        self.get_logger().info(f'  Max time : {MAX_RECORD_SEC:.0f} s  ({MAX_RECORD_SEC/60:.0f} min)')
        self.get_logger().info(f'  Video    : {self.video_path}')
        self.get_logger().info(f'  [Recording starts on ORBIT entry, stops after {LAPS} laps or {MAX_RECORD_SEC/60:.0f} min]')
        self.get_logger().info('='*55)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _status_cb(self, msg):
        self.nav_state    = msg.nav_state
        self.arming_state = msg.arming_state

    def _pos_cb(self, msg):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z

        # Print drone coordinates on first position received from simulator
        if not self._pos_received:
            self._pos_received = True
            drone_enu = ned_to_enu(self.pos_x, self.pos_y, self.pos_z)
            self.get_logger().info('=' * 55)
            self.get_logger().info('[DRONE SPAWN] Drone coordinates from simulator:')
            self.get_logger().info(
                f'  NED (PX4)    : x={self.pos_x:.3f}  y={self.pos_y:.3f}  z={self.pos_z:.3f}')
            self.get_logger().info(
                f'  ENU (Gazebo) : x={drone_enu.x:.3f}  y={drone_enu.y:.3f}  z={drone_enu.z:.3f}')
            self.get_logger().info('=' * 55)

    def _image_cb(self, msg):
        """Record video during ORBIT. Stop after MAX_RECORD_SEC or after all laps."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')
            return

        h, w = frame.shape[:2]

        # ── Start recording when entering ORBIT ───────────────────────────────
        if self.video_writer is None and self.state == State.ORBIT:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                self.video_path, fourcc, 20.0, (w, h))
            self.record_start_t = time.monotonic()
            self.get_logger().info(f'[VIDEO] Recording started: {self.video_path}')

        # ── Write frame while in ORBIT ────────────────────────────────────────
        if self.video_writer is not None and self.state == State.ORBIT and self.record_start_t is not None:
            elapsed = time.monotonic() - self.record_start_t
            overlay = frame.copy()
            cv2.putText(overlay,
                f'State:{self.state}  Lap:{self.current_lap+1}/{LAPS}'
                f'  Angle:{math.degrees(self.orbit_angle)%360:.0f}deg'
                f'  T:{elapsed:.1f}s/{MAX_RECORD_SEC:.0f}s',
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(overlay,
                f'NED:({self.pos_x:.1f},{self.pos_y:.1f},{self.pos_z:.1f})',
                (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 0), 1)
            cv2.circle(overlay, (w - 20, 20), 10, (0, 0, 255), -1)
            cv2.putText(overlay, 'REC', (w - 58, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            self.video_writer.write(overlay)

            # ── Timeout -> switch to LAND ─────────────────────────────────────
            if elapsed >= MAX_RECORD_SEC:
                self.get_logger().info(
                    f'[VIDEO] {MAX_RECORD_SEC/60:.0f} min elapsed -> auto land')
                self.state = State.LAND

        # ── Always show camera window ─────────────────────────────────────────
        cv2.imshow('Camera', frame)
        cv2.waitKey(1)

    # ── Control loop 20 Hz ────────────────────────────────────────────────────

    def _loop(self):
        self._pub_offboard_mode()

        if self.state == State.INIT:
            self._do_init()
        elif self.state == State.TAKEOFF:
            self._do_takeoff()
        elif self.state == State.ORBIT:
            self._do_orbit()
        elif self.state == State.LAND:
            self._do_land()
        # DONE: do nothing

    # ── States ────────────────────────────────────────────────────────────────

    def _do_init(self):
        """
        Correct PX4 sequence:
          1. Stream setpoint continuously from the start (required)
          2. t >= 0.5s: send OFFBOARD mode command until nav_state == 14
          3. Once OFFBOARD: send ARM once
          4. Once ARMED (arming_state == 2): switch to TAKEOFF
        """
        self._pub_setpoint(0.0, 0.0, -ORBIT_ALTITUDE)
        self.counter += 1

        in_offboard = (self.nav_state == 14)
        is_armed    = (self.arming_state == 2)

        # Log every 2s
        if self.counter % 40 == 0:
            self.get_logger().info(
                f'[INIT] counter={self.counter} '
                f'nav={self.nav_state}(offboard={in_offboard}) '
                f'arm={self.arming_state}(armed={is_armed})')

        # Wait at least 0.5s to stream setpoints first
        if self.counter < 10:
            return

        # Step 1: send OFFBOARD continuously until PX4 confirms
        if not in_offboard:
            self._pub_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                              param1=1.0, param2=6.0)
            return

        # Step 2: OFFBOARD confirmed -> ARM once
        if not is_armed:
            if not hasattr(self, '_arm_sent'):
                self._arm_sent = True
                self._pub_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                  param1=1.0)
                self.get_logger().info('[INIT] OFFBOARD confirmed -> ARM sent')
            return

        # Step 3: ARMED + OFFBOARD -> TAKEOFF
        self.state = State.TAKEOFF
        self.get_logger().info('[INIT] ARMED + OFFBOARD -> TAKEOFF')

    def _do_takeoff(self):
        """Fly to orbit start point: (target + radius, 0) at the correct altitude."""
        sx = TARGET_X + ORBIT_RADIUS
        sy = TARGET_Y
        self._pub_setpoint(sx, sy, -ORBIT_ALTITUDE,
                           yaw=self._yaw_to_target(sx, sy))

        dist_xy = math.sqrt((self.pos_x - sx)**2 + (self.pos_y - sy)**2)
        dist_z  = abs(self.pos_z - (-ORBIT_ALTITUDE))

        if dist_xy < 1.0 and dist_z < 0.5:
            self.orbit_angle   = 0.0
            self.lap_angle_acc = 0.0
            self.state         = State.ORBIT

            # Print initial position when orbit starts
            dist_to_person = math.sqrt(
                (self.pos_x - TARGET_X)**2 + (self.pos_y - TARGET_Y)**2)
            self.get_logger().info('=' * 55)
            self.get_logger().info('[ORBIT START] Starting circular flight')
            self.get_logger().info(
                f'  person_1  NED : x={TARGET_X:.2f}  y={TARGET_Y:.2f}  z=0.00')
            self.get_logger().info(
                f'  drone initial : x={self.pos_x:.2f}  y={self.pos_y:.2f}  z={self.pos_z:.2f}')
            self.get_logger().info(
                f'  distance to person_1 : {dist_to_person:.2f} m  (expected={ORBIT_RADIUS:.1f} m)')
            self.get_logger().info('=' * 55)

    def _do_orbit(self):
        """Fly circular orbit. After LAPS laps -> LAND."""
        pos_x = TARGET_X + ORBIT_RADIUS * math.cos(self.orbit_angle)
        pos_y = TARGET_Y + ORBIT_RADIUS * math.sin(self.orbit_angle)
        self._pub_setpoint(pos_x, pos_y, -ORBIT_ALTITUDE,
                           yaw=self._yaw_to_target(pos_x, pos_y))

        # ── Accumulate actual drone position ──────────────────────────────────
        dist_now = math.sqrt(
            (self.pos_x - TARGET_X)**2 + (self.pos_y - TARGET_Y)**2)
        self._orbit_sum_x    += self.pos_x
        self._orbit_sum_y    += self.pos_y
        self._orbit_sum_dist += dist_now
        self._orbit_count    += 1

        delta = ORBIT_SPEED * 0.05  # rad/tick @ 20 Hz
        self.orbit_angle   += delta
        self.lap_angle_acc += delta

        if self.lap_angle_acc >= 2.0 * math.pi:
            self.lap_angle_acc -= 2.0 * math.pi
            self.current_lap   += 1

            # ── Print end-of-lap report ───────────────────────────────────────
            center_x = TARGET_X
            center_y = TARGET_Y
            avg_dist = ORBIT_RADIUS
            err_center = 0.0
            if self._orbit_count > 0:
                center_x   = self._orbit_sum_x    / self._orbit_count
                center_y   = self._orbit_sum_y    / self._orbit_count
                avg_dist   = self._orbit_sum_dist / self._orbit_count
                err_center = math.sqrt(
                    (center_x - TARGET_X)**2 + (center_y - TARGET_Y)**2)

            self.get_logger().info('-' * 55)
            self.get_logger().info(
                f'[ORBIT] Lap {self.current_lap}/{LAPS} completed')
            self.get_logger().info(
                f'  drone current position    : x={self.pos_x:.2f}  y={self.pos_y:.2f}  z={self.pos_z:.2f}')
            self.get_logger().info(
                f'  actual orbit center       : x={center_x:.2f}  y={center_y:.2f}')
            self.get_logger().info(
                f'  theoretical orbit center  : x={TARGET_X:.2f}  y={TARGET_Y:.2f}')
            self.get_logger().info(
                f'  center deviation          : {err_center:.3f} m')
            self.get_logger().info(
                f'  mean actual radius        : {avg_dist:.3f} m  (expected={ORBIT_RADIUS:.1f} m)')
            self.get_logger().info('-' * 55)

            # Reset accumulation for the next lap
            self._orbit_sum_x    = 0.0
            self._orbit_sum_y    = 0.0
            self._orbit_sum_dist = 0.0
            self._orbit_count    = 0

            if self.current_lap >= LAPS:
                self.get_logger().info('[ORBIT] Done -> LAND')
                self.state = State.LAND

    def _do_land(self):
        """Save video then land."""
        self._save_video()
        self._pub_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = State.DONE
        self.get_logger().info('[LAND] Land command sent -> DONE')
        # Exit after 2s to let PX4 receive the command
        self.create_timer(2.0, self._shutdown)

    def _shutdown(self):
        self.get_logger().info('[DONE] Shutting down node')
        raise SystemExit

    def _save_video(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            self.get_logger().info(f'[VIDEO] Saved: {self.video_path}')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _yaw_to_target(self, drone_x, drone_y) -> float:
        """NED yaw to always face the orbit center."""
        dx = TARGET_X - drone_x
        dy = TARGET_Y - drone_y
        return math.atan2(dy, dx)

    def _pub_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def _pub_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position  = [float(x), float(y), float(z)]
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


# ── Main ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OrbitDataCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._save_video()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
