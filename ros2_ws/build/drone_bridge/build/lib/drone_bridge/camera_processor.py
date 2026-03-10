#!/usr/bin/env python3
"""
Enhanced Camera Processor with Frame Saving and Statistics

This is an example ROS2 node that extends the basic camera viewer with:
- Frame statistics (resolution, encoding, timestamp)
- Frame saving capability (saves to disk periodically)
- Basic image processing (edge detection, etc.)
- Logging and metrics

Usage:
    ros2 run drone_bridge camera_processor
    
Or with custom save directory:
    ros2 run drone_bridge camera_processor --save-dir /path/to/frames
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from pathlib import Path
import numpy as np


class CameraProcessor(Node):
    """Enhanced camera processor with statistics and frame saving."""
    
    def __init__(self):
        super().__init__('camera_processor')
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Frame statistics
        self.frame_count = 0
        self.last_timestamp = None
        self.frame_times = []
        
        # Frame saving
        self.save_frames = False
        self.save_every_n_frames = 30  # Save every 30th frame
        self.save_directory = Path.home() / 'camera_frames'
        
        # Declare parameters
        self.declare_parameter('save_frames', False)
        self.declare_parameter('save_every_n', 30)
        self.declare_parameter('save_dir', str(self.save_directory))
        
        # Get parameters
        self.save_frames = self.get_parameter('save_frames').value
        self.save_every_n_frames = self.get_parameter('save_every_n').value
        self.save_directory = Path(self.get_parameter('save_dir').value)
        
        # Create save directory if needed
        if self.save_frames:
            self.save_directory.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f'Frame saving enabled. Directory: {self.save_directory}')
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/'
            'pitch_link/sensor/camera/image',
            self.image_callback,
            1  # Queue size
        )
        
        self.get_logger().info(
            f'Camera processor started.\n'
            f'  Frame saving: {"Enabled" if self.save_frames else "Disabled"}\n'
            f'  Save directory: {self.save_directory}'
        )
    
    def image_callback(self, msg: Image):
        """
        Process incoming image from camera.
        
        Args:
            msg: ROS2 Image message
        """
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Update statistics
            self.frame_count += 1
            height, width = cv_image.shape[:2]
            
            # Log every 100 frames
            if self.frame_count % 100 == 0:
                self.get_logger().info(
                    f'Received {self.frame_count} frames | '
                    f'Resolution: {width}x{height} | '
                    f'Encoding: {msg.encoding}'
                )
            
            # Save frame if enabled
            if self.save_frames and self.frame_count % self.save_every_n_frames == 0:
                self._save_frame(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def _save_frame(self, cv_image: np.ndarray):
        """
        Save a frame to disk.
        
        Args:
            cv_image: OpenCV image (numpy array)
        """
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = f'frame_{self.frame_count:06d}_{timestamp}.png'
            filepath = self.save_directory / filename
            
            cv2.imwrite(str(filepath), cv_image)
            self.get_logger().debug(f'Saved frame: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving frame: {e}')
    
    def _detect_edges(self, cv_image: np.ndarray) -> np.ndarray:
        """
        Example processing: Detect edges using Canny edge detection.
        
        Args:
            cv_image: Input image (BGR)
        
        Returns:
            Edge-detected image
        """
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        return edges
    
    def shutdown(self):
        """Cleanup on shutdown."""
        self.get_logger().info(
            f'Camera processor shutting down.\n'
            f'Total frames processed: {self.frame_count}'
        )


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    processor = CameraProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Interrupted by user')
    finally:
        processor.shutdown()
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
