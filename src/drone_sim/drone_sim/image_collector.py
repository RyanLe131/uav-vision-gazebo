#!/usr/bin/env python3
"""
Image Collector Node for YOLO Training Data
Captures images from drone camera and saves them for YOLO training.

Controls (OpenCV window must be focused):
  - SPACE : Capture single image manually
  - A     : Toggle auto-capture mode (saves every N frames)
  - Q     : Quit

Parameters:
  - output_dir  : Directory to save images (default: ~/yolo_dataset/images)
  - auto_capture: Start with auto-capture enabled (default: False)
  - capture_interval: Frames between auto-captures (default: 30)
  - min_diff_threshold: Min pixel difference to avoid duplicate frames (default: 5.0)
"""

import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageCollector(Node):
    def __init__(self):
        super().__init__('image_collector')

        # Declare parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/yolo_dataset/images'))
        self.declare_parameter('auto_capture', False)
        self.declare_parameter('capture_interval', 30)
        self.declare_parameter('min_diff_threshold', 5.0)

        self.output_dir = self.get_parameter('output_dir').value
        self.auto_capture = self.get_parameter('auto_capture').value
        self.capture_interval = self.get_parameter('capture_interval').value
        self.min_diff_threshold = self.get_parameter('min_diff_threshold').value

        # Create output directories (YOLO structure)
        os.makedirs(self.output_dir, exist_ok=True)
        labels_dir = os.path.join(os.path.dirname(self.output_dir), 'labels')
        os.makedirs(labels_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0
        self.last_saved_frame = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        mode = 'AUTO' if self.auto_capture else 'MANUAL'
        self.get_logger().info(
            f'Image Collector started | Mode: {mode} | '
            f'Output: {self.output_dir} | Interval: {self.capture_interval}'
        )
        self.get_logger().info('Controls: SPACE=capture, A=toggle auto, Q=quit')

    def is_different_frame(self, frame: np.ndarray) -> bool:
        """Check if frame is sufficiently different from last saved frame."""
        if self.last_saved_frame is None:
            return True
        if frame.shape != self.last_saved_frame.shape:
            return True
        diff = np.mean(np.abs(frame.astype(float) - self.last_saved_frame.astype(float)))
        return diff > self.min_diff_threshold

    def save_image(self, frame: np.ndarray) -> None:
        """Save frame to disk with timestamp-based filename."""
        if not self.is_different_frame(frame):
            self.get_logger().info('Frame too similar to last saved — skipped')
            return

        timestamp = int(time.time() * 1000)
        filename = f'img_{timestamp}_{self.saved_count:06d}.jpg'
        filepath = os.path.join(self.output_dir, filename)

        cv2.imwrite(filepath, frame)
        self.last_saved_frame = frame.copy()
        self.saved_count += 1
        self.get_logger().info(f'Saved [{self.saved_count}]: {filename}')

    def image_callback(self, msg: Image) -> None:
        self.frame_count += 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Draw HUD overlay
        display = cv_image.copy()
        mode_text = 'AUTO' if self.auto_capture else 'MANUAL'
        cv2.putText(
            display, f'Mode: {mode_text} | Saved: {self.saved_count} | Frame: {self.frame_count}',
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
        )
        if self.auto_capture:
            cv2.circle(display, (display.shape[1] - 20, 20), 8, (0, 0, 255), -1)

        cv2.imshow('Image Collector [SPACE=capture A=auto Q=quit]', display)

        # Auto-capture
        if self.auto_capture and self.frame_count % self.capture_interval == 0:
            self.save_image(cv_image)

        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            self.save_image(cv_image)
        elif key == ord('a') or key == ord('A'):
            self.auto_capture = not self.auto_capture
            state = 'ON' if self.auto_capture else 'OFF'
            self.get_logger().info(f'Auto-capture: {state}')
        elif key == ord('q') or key == ord('Q'):
            self.get_logger().info(f'Quitting. Total images saved: {self.saved_count}')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.get_logger().info(f'Total images saved: {node.saved_count}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
