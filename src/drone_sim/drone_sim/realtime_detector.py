#!/usr/bin/env python3
"""
realtime_detector.py

Subscribe /camera/image_raw -> run YOLOv8s inference (GPU) -> display
realtime window with bounding boxes and confidence scores.

Run:
  source /opt/ros/jazzy/setup.bash
  source ~/DIPLOM/ws_drone/install/setup.bash
  ~/venv_ros2_yolo/bin/python ~/DIPLOM/ws_drone/src/drone_sim/drone_sim/realtime_detector.py
"""

import cv2
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# ── Config ────────────────────────────────────────────────────────────────────
MODEL_NAME  = 'yolov8s.pt'
CONF_THRESH = 0.25
PERSON_CLS  = 0       # class 'person' in COCO
DEVICE      = 'cuda' if torch.cuda.is_available() else 'cpu'


class RealtimeDetector(Node):

    def __init__(self):
        super().__init__('realtime_detector')

        # ── Load model ────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading {MODEL_NAME} on {DEVICE} ...')
        self.model = YOLO(MODEL_NAME)
        self.model.to(DEVICE)
        self.get_logger().info(f'Model loaded — device: {DEVICE}')

        self.bridge     = CvBridge()
        self._frame_cnt = 0
        self._det_cnt   = 0

        # ── QoS ──────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, qos)

        self.get_logger().info('Subscribed to /camera/image_raw')
        self.get_logger().info('Press Q in the camera window to quit')

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')
            return

        self._frame_cnt += 1
        h, w = frame.shape[:2]

        # ── Inference ─────────────────────────────────────────────────────────
        results = self.model(
            frame,
            classes=[PERSON_CLS],
            conf=CONF_THRESH,
            device=DEVICE,
            verbose=False,
        )

        # ── Annotate ──────────────────────────────────────────────────────────
        annotated = frame.copy()
        boxes     = results[0].boxes
        n_det     = 0

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                conf = float(box.conf[0])
                n_det += 1
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated,
                    f'person {conf:.2f}',
                    (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if n_det > 0:
            self._det_cnt += 1

        # ── Overlay info ──────────────────────────────────────────────────────
        status      = f'DETECTED x{n_det}' if n_det > 0 else 'searching...'
        status_col  = (0, 255, 0) if n_det > 0 else (0, 165, 255)
        det_rate    = self._det_cnt / self._frame_cnt * 100

        cv2.putText(annotated, status,
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_col, 2)
        cv2.putText(annotated,
            f'Frame:{self._frame_cnt}  DetRate:{det_rate:.0f}%  [{DEVICE.upper()}]',
            (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)

        # ── REC indicator ─────────────────────────────────────────────────────
        cv2.circle(annotated, (w - 20, 20), 8, (0, 0, 255), -1)
        cv2.putText(annotated, 'LIVE', (w - 62, 26),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255), 1)

        # ── Show ──────────────────────────────────────────────────────────────
        cv2.imshow('Realtime Detection', annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Q pressed — shutting down')
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
