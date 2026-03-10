#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys


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

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the frame
            cv2.imshow('Gazebo Camera Feed', cv_image)

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
