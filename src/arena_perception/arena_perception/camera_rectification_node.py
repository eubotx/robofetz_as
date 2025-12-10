#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge

class CameraRectificationNode(Node):
    def __init__(self):
        super().__init__('camera_rectification_node')
        
        self.bridge = CvBridge()
        self.map1 = None
        self.map2 = None
        self.needs_rectification = False
        self.initialized = False  # Track if we've made the decision
        self._warned_for_camera_info = False  # Track if we've already warned
        
        # Use relative topics
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            'camera_info',
            self.camera_info_callback, 
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback, 
            10
        )
        
        # Publishers
        self.rectified_image_pub = self.create_publisher(
            Image, 
            'image_rect',
            10
        )
        
        self.rectified_compressed_pub = self.create_publisher(
            CompressedImage,
            'image_rect/compressed',
            10
        )

        self.get_logger().info("Camera rectification node initialized")

    def camera_info_callback(self, msg):
        # Only initialize once
        if self.initialized:
            return
            
        self.initialized = True
        
        # Check if we need rectification (non-zero distortion with fisheye model)
        D = np.array(msg.d)
        has_distortion = not np.allclose(D, 0)
        
        if not has_distortion:
            self.get_logger().info("Pinhole camera detected - passing through images")
            self.needs_rectification = False
            self.map1 = None
            self.map2 = None
            return
            
        if msg.distortion_model != 'fisheye':
            self.get_logger().warn(f"Distortion present but model is {msg.distortion_model} - passing through")
            self.needs_rectification = False
            self.map1 = None
            self.map2 = None
            return
        
        # We have fisheye distortion - setup rectification
        self.needs_rectification = True
        K = np.array(msg.k).reshape(3, 3)
        resolution = (msg.width, msg.height)
        
        self.get_logger().info(f"Setting up fisheye rectification for {resolution}")
        
        try:
            # Take first 4 coefficients for fisheye
            D_fisheye = D[:4] if len(D) >= 4 else D
            
            P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K=K,
                D=D_fisheye,
                image_size=resolution,
                R=np.eye(3),
                balance=1.0,
                new_size=resolution,
                fov_scale=1.0
            )
            
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                K=K,
                D=D_fisheye,
                R=np.eye(3),
                P=P,
                size=resolution,
                m1type=cv2.CV_16SC2
            )
            
            self.get_logger().info("Fisheye rectification ready")
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup rectification: {str(e)}")
            self.needs_rectification = False

    def image_callback(self, msg):
        if not self.initialized:
            # Only warn once about waiting for camera info
            if not self._warned_for_camera_info:
                self.get_logger().warn("Waiting for camera info...")
                self._warned_for_camera_info = True
            return
            
        try:
            # For pinhole or failed setup, just republish
            if not self.needs_rectification or self.map1 is None:
                # Publish same image to rectified topic
                rectified_msg = msg  # Just pass through (shallow copy)
                self.rectified_image_pub.publish(rectified_msg)
                
                # For compressed, need to convert
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                compressed_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
                compressed_msg.header = msg.header
                self.rectified_compressed_pub.publish(compressed_msg)
                return
                
            # We have fisheye distortion - apply rectification
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            rectified_image = cv2.remap(
                cv_image, 
                self.map1, 
                self.map2, 
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT
            )
            
            # Publish rectified image
            rectified_msg = self.bridge.cv2_to_imgmsg(rectified_image, encoding='bgr8')
            rectified_msg.header = msg.header
            self.rectified_image_pub.publish(rectified_msg)
            
            # Publish compressed image
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(rectified_image)
            compressed_msg.header = msg.header
            self.rectified_compressed_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraRectificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()