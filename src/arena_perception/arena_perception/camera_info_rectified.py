#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import CameraInfo

class RectifiedCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('rectified_camera_info_publisher')
        
        # Subscriber to original camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            'camera_info',  # Subscribe to original camera info
            self.camera_info_callback, 
            10
        )
        
        # Publisher for rectified camera info
        self.rectified_camera_info_pub = self.create_publisher(
            CameraInfo,
            'camera_info_rect',  # Publish rectified camera info
            10
        )
        
        self.rectified_K = None
        self.original_camera_info = None
        
        self.get_logger().info("Rectified Camera Info Publisher initialized")

    def camera_info_callback(self, msg):
        # Only process if we haven't computed the rectified matrix yet
        # or if camera parameters changed
        if self.rectified_K is not None and self.original_camera_info is not None:
            # Check if parameters changed
            if (np.array_equal(np.array(msg.k), np.array(self.original_camera_info.k)) and
                np.array_equal(np.array(msg.d), np.array(self.original_camera_info.d))):
                # Parameters unchanged, just update header and republish
                self.publish_rectified_camera_info(msg.header)
                return
        
        # Store original camera info
        self.original_camera_info = msg
        
        if msg.distortion_model not in ['fisheye', 'rational_polynomial']:
            self.get_logger().warn(f"Unsupported distortion model: {msg.distortion_model}")
            return

        # Extract camera parameters
        K = np.array(msg.k).reshape(3, 3)
        D = np.array(msg.d)
        resolution = (msg.width, msg.height)
        
        self.get_logger().info(f"Computing rectified camera matrix for {resolution}")
        
        try:
            # Compute rectified camera matrix
            self.rectified_K = self.compute_rectified_camera_matrix(K, D, resolution)
            
            self.get_logger().info("Rectified camera matrix computed successfully")
            
            # Publish rectified camera info
            self.publish_rectified_camera_info(msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Failed to compute rectified camera matrix: {str(e)}")

    def compute_rectified_camera_matrix(self, K, D, resolution):
        """Compute the rectified camera matrix for fisheye images"""
        import cv2
        
        # For fisheye images, estimate new camera matrix
        rectified_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K=K,
            D=D,
            image_size=resolution,
            R=np.eye(3),
            balance=1.0,  # Adjust this for desired FOV (0.0 = full FOV, 1.0 = cropped)
            new_size=resolution,
            fov_scale=1.0
        )
        
        return rectified_K

    def publish_rectified_camera_info(self, header):
        """Publish the rectified camera info"""
        if self.rectified_K is None or self.original_camera_info is None:
            return
            
        # Create new camera info message
        rectified_info = CameraInfo()
        
        # Copy header and basic info from original (with updated timestamp)
        rectified_info.header = header
        rectified_info.header.stamp = self.get_clock().now().to_msg()
        rectified_info.height = self.original_camera_info.height
        rectified_info.width = self.original_camera_info.width
        rectified_info.distortion_model = 'plumb_bob'  # Rectified = no distortion
        
        # Use rectified camera matrix
        rectified_info.k = self.rectified_K.ravel().tolist()
        
        # For rectified images, use identity projection matrix
        # Typically P = K for monocular cameras
        rectified_info.p = list(self.rectified_K.ravel()) + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectified images have no distortion
        rectified_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Identity rectification matrix (no additional rotation)
        rectified_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Copy binning and ROI if present
        rectified_info.binning_x = self.original_camera_info.binning_x
        rectified_info.binning_y = self.original_camera_info.binning_y
        rectified_info.roi = self.original_camera_info.roi
        
        self.rectified_camera_info_pub.publish(rectified_info)
        self.get_logger().debug("Published rectified camera info")

def main(args=None):
    rclpy.init(args=args)
    node = RectifiedCameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()