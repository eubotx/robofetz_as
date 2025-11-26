#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import numpy as np

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Camera parameters for 800x600 image
        self.image_width = 800
        self.image_height = 600
        
        # Intrinsic matrix (approximate for 170° FOV)
        # fx, fy = focal length in pixels
        # cx, cy = principal point (image center)
        self.k = [300.0, 0.0, 400.0,   # fx, 0, cx - lower fx for wider FOV
                  0.0, 300.0, 300.0,   # 0, fy, cy  
                  0.0, 0.0, 1.0]       # 0, 0, 1
        
        # Distortion coefficients for FISHEYE model that match your custom lens:
        # r = 1.8 * sin(θ / 1.5) creates strong fisheye distortion
        # For fisheye model: [k1, k2, k3, k4] - radial distortion only
        self.distortion_coeffs = [-0.423185, 0.128456, -0.031892, 0.005234]  # k1, k2, k3, k4
        
        # Projection matrix
        self.p = [300.0, 0.0, 400.0, 0.0,
                  0.0, 300.0, 300.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        # Rotation matrix (identity)
        self.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 
                  0.0, 0.0, 1.0]
        
        # Create publisher - match your real camera topic structure
        self.publisher = self.create_publisher(
            CameraInfo,
            'camera_info',  # Same as your real camera
            10)
        
        # Publish at 30Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_camera_info)
        
        self.get_logger().info('Camera info publisher started for fisheye model')
        self.get_logger().info(f'Fisheye distortion coefficients: {self.distortion_coeffs}')
    
    def publish_camera_info(self):
        """Publish camera calibration info"""
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arena_camera/link/arena_camera'
        
        msg.height = self.image_height
        msg.width = self.image_width
        msg.distortion_model = 'fisheye'  # Use fisheye model for your rectification node
        msg.d = self.distortion_coeffs
        msg.k = self.k
        msg.r = self.r
        msg.p = self.p
        msg.binning_x = 0
        msg.binning_y = 0
        
        # ROI (full image)
        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = 0
        msg.roi.width = 0
        msg.roi.do_rectify = False
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()