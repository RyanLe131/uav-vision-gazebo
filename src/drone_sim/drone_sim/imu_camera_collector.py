#!/usr/bin/env python3
"""
imu_camera_collector.py

Synchronized IMU + Camera data collection for neural network training
(e.g. learning the relationship between motion blur and angular velocity).

Pipeline:
  1. Subscribe /fmu/out/sensor_combined  -> gyroscope (rad/s) + timestamp
  2. Subscribe /camera/image_raw         -> frame + timestamp
  3. On each new frame:
       - Find the closest IMU sample by timestamp (±MAX_SYNC_MS ms)
       - If synced: save frame .jpg + write row to imu_log.csv
  4. Print stats every 5 seconds: total frames, sync OK, sync fail, rate

Output directory: ~/orbit_imu_data/<session_YYYYMMDD_HHMMSS>/
  frames/
    frame_000001.jpg
    frame_000002.jpg
    ...
  imu_log.csv   (columns: frame_id, cam_ts_us, imu_ts_us, dt_sync_ms,
                           gx_rad_s, gy_rad_s, gz_rad_s,
                           ax_m_s2, ay_m_s2, az_m_s2)

Run (separate terminal, while drone is flying):
  source /opt/ros/jazzy/setup.bash
  source ~/DIPLOM/ws_drone/install/setup.bash
  ~/venv_ros2_yolo/bin/python \\
    ~/DIPLOM/ws_drone/src/drone_sim/drone_sim/imu_camera_collector.py

Stop with Ctrl+C — CSV file is closed cleanly.
"""

import csv
import math
import os
import time
from collections import deque
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import SensorCombined
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from sensor_msgs.msg import Image

# ── Config ─────────────────────────────────────────────────────────────────────
MAX_SYNC_MS   = 20      # maximum timestamp difference to consider "synced" (ms)
IMU_BUF_SIZE  = 200     # number of IMU samples kept in buffer (200 × 5ms = 1s)
JPEG_QUALITY  = 90      # JPEG compression quality for saved frames
STATS_EVERY_S = 5.0     # print stats every N seconds
SHOW_PREVIEW  = True    # show preview window (disable if no display)

OUTPUT_BASE   = os.path.expanduser('~/orbit_imu_data')


class ImuCameraCollector(Node):

    def __init__(self):
        super().__init__('imu_camera_collector')

        # ── QoS ───────────────────────────────────────────────────────────────
        qos_imu = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_cam = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscribers ────────────────────────────────────────────────────────
        self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self._imu_cb,
            qos_imu,
        )
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_cb,
            qos_cam,
        )

        # ── IMU circular buffer ────────────────────────────────────────────────
        # Each element: dict {ts_us, gx, gy, gz, ax, ay, az}
        self._imu_buf: deque = deque(maxlen=IMU_BUF_SIZE)

        # ── Output setup ──────────────────────────────────────────────────────
        session = datetime.now().strftime('session_%Y%m%d_%H%M%S')
        self._session_dir = os.path.join(OUTPUT_BASE, session)
        self._frames_dir  = os.path.join(self._session_dir, 'frames')
        os.makedirs(self._frames_dir, exist_ok=True)

        csv_path = os.path.join(self._session_dir, 'imu_log.csv')
        self._csv_file   = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'frame_id',
            'cam_ts_us', 'imu_ts_us', 'dt_sync_ms',
            'gx_rad_s',  'gy_rad_s',  'gz_rad_s',
            'ax_m_s2',   'ay_m_s2',   'az_m_s2',
        ])

        # ── Counters ──────────────────────────────────────────────────────────
        self._frame_id     = 0
        self._sync_ok      = 0
        self._sync_fail    = 0
        self._last_stat_t  = time.time()

        # ── Bridge ────────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Startup log ───────────────────────────────────────────────────────
        self.get_logger().info('=' * 55)
        self.get_logger().info('[IMU-CAM COLLECTOR] Starting up')
        self.get_logger().info(f'  Output         : {self._session_dir}')
        self.get_logger().info(f'  Sync tolerance : ±{MAX_SYNC_MS} ms')
        self.get_logger().info(f'  IMU buffer     : {IMU_BUF_SIZE} samples')
        self.get_logger().info(f'  JPEG quality   : {JPEG_QUALITY}')
        self.get_logger().info('  Waiting for /fmu/out/sensor_combined ...')
        self.get_logger().info('  Waiting for /camera/image_raw ...')
        self.get_logger().info('  Press Ctrl+C to stop and close CSV')
        self.get_logger().info('=' * 55)

    # ── IMU callback ───────────────────────────────────────────────────────────

    def _imu_cb(self, msg: SensorCombined):
        """Store IMU sample into the circular buffer."""
        # px4_msgs SensorCombined:
        #   timestamp             (uint64, microseconds)
        #   gyro_rad[3]           (float32[3], rad/s)
        #   accelerometer_m_s2[3] (float32[3], m/s²)
        sample = {
            'ts_us': msg.timestamp,
            'gx':    float(msg.gyro_rad[0]),
            'gy':    float(msg.gyro_rad[1]),
            'gz':    float(msg.gyro_rad[2]),
            'ax':    float(msg.accelerometer_m_s2[0]),
            'ay':    float(msg.accelerometer_m_s2[1]),
            'az':    float(msg.accelerometer_m_s2[2]),
        }
        self._imu_buf.append(sample)

    # ── Camera callback ────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        """Receive frame -> find closest IMU sample -> save."""
        self._frame_id += 1
        cam_ts_us = int(msg.header.stamp.sec * 1_000_000
                        + msg.header.stamp.nanosec / 1_000)

        # ── Find closest IMU sample ────────────────────────────────────────────
        best = self._find_closest_imu(cam_ts_us)

        if best is None:
            self._sync_fail += 1
            if self._frame_id <= 5 or self._frame_id % 50 == 0:
                self.get_logger().warn(
                    f'[SYNC FAIL] frame={self._frame_id}  '
                    f'IMU buffer empty or no data yet')
            self._print_stats_maybe()
            return

        dt_ms = abs(cam_ts_us - best['ts_us']) / 1_000.0
        if dt_ms > MAX_SYNC_MS:
            self._sync_fail += 1
            if self._frame_id % 50 == 0:
                self.get_logger().warn(
                    f'[SYNC FAIL] frame={self._frame_id}  dt={dt_ms:.1f}ms > {MAX_SYNC_MS}ms')
            self._print_stats_maybe()
            return

        # ── Decode frame ───────────────────────────────────────────────────────
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')
            self._sync_fail += 1
            return

        # ── Save frame ────────────────────────────────────────────────────────
        fname = f'frame_{self._frame_id:06d}.jpg'
        fpath = os.path.join(self._frames_dir, fname)
        cv2.imwrite(fpath, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

        # ── Write CSV row ──────────────────────────────────────────────────────
        self._csv_writer.writerow([
            self._frame_id,
            cam_ts_us,
            best['ts_us'],
            f'{dt_ms:.3f}',
            f"{best['gx']:.6f}",
            f"{best['gy']:.6f}",
            f"{best['gz']:.6f}",
            f"{best['ax']:.6f}",
            f"{best['ay']:.6f}",
            f"{best['az']:.6f}",
        ])
        self._csv_file.flush()  # flush to disk immediately

        self._sync_ok += 1

        # ── Preview ────────────────────────────────────────────────────────────
        if SHOW_PREVIEW:
            gyro_norm = math.sqrt(best['gx']**2 + best['gy']**2 + best['gz']**2)
            h, w = frame.shape[:2]
            preview = frame.copy()
            cv2.putText(preview,
                f'Frame #{self._frame_id}  sync dt={dt_ms:.1f}ms',
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(preview,
                f'Gyro |w|={gyro_norm:.3f} rad/s  '
                f'gz={best["gz"]:.3f}',
                (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
            cv2.putText(preview,
                f'OK={self._sync_ok}  FAIL={self._sync_fail}',
                (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
            cv2.imshow('IMU-Camera Collector', preview)
            cv2.waitKey(1)

        self._print_stats_maybe()

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _find_closest_imu(self, cam_ts_us: int):
        """Return the IMU sample with timestamp closest to cam_ts_us."""
        if not self._imu_buf:
            return None
        return min(self._imu_buf, key=lambda s: abs(s['ts_us'] - cam_ts_us))

    def _print_stats_maybe(self):
        now = time.time()
        if now - self._last_stat_t < STATS_EVERY_S:
            return
        self._last_stat_t = now
        total = self._sync_ok + self._sync_fail
        rate  = (self._sync_ok / total * 100) if total > 0 else 0.0
        self.get_logger().info(
            f'[STATS] frames_total={total}  '
            f'sync_ok={self._sync_ok}  '
            f'sync_fail={self._sync_fail}  '
            f'rate={rate:.1f}%  '
            f'saved_to={self._session_dir}')

    def destroy_node(self):
        """Close CSV when node is destroyed."""
        try:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(
                f'[DONE] CSV closed. '
                f'Total frames saved: {self._sync_ok}  '
                f'Directory: {self._session_dir}')
        except Exception:
            pass
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()
        super().destroy_node()


# ── Main ───────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ImuCameraCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
