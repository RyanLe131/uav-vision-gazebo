#!/usr/bin/env python3
"""
Camera viewer for Gazebo using OpenCV.
Run: ros2 run drone_bridge camera_viewer.py
Or:  python3 camera_viewer.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/'
            'pitch_link/sensor/camera/image',
            self.image_callback,
            1
        )
        self.get_logger().info('Camera viewer started. Waiting for images...')
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1

            # Add frame info to display
            cv2.putText(
                cv_image,
                f'Frame: {self.frame_count}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
            cv2.putText(
                cv_image,
                f'Resolution: {cv_image.shape[1]}x{cv_image.shape[0]}',
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            # Display the frame
            cv2.imshow('Gazebo Camera Feed (Press Q to exit)', cv_image)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Shutting down camera viewer...')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()

    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        camera_viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
