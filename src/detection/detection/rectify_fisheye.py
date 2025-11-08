#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge

class FisheyeRectificationNode(Node):
    def __init__(self):
        super().__init__('fisheye_rectification_node')
        
        self.bridge = CvBridge()
        self.map1 = None
        self.map2 = None
        
        # Use relative topics (without leading slash)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            'camera_info',  # Relative to node's namespace
            self.camera_info_callback, 
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',  # Relative to node's namespace
            self.image_callback, 
            10
        )
        
        # Publishers - use relative topics
        self.rectified_image_pub = self.create_publisher(
            Image, 
            'image_rect',  # Relative to node's namespace
            10
        )
        
        self.rectified_compressed_pub = self.create_publisher(
            CompressedImage,
            'image_rect/compressed',  # Relative to node's namespace
            10
        )

        self.get_logger().info("Fisheye rectification node initialized")

    def camera_info_callback(self, msg):
        if self.map1 is not None:
            return  # Already initialized
            
        if msg.distortion_model not in ['fisheye', 'rational_polynomial']:
            self.get_logger().warn(f"Unsupported distortion model: {msg.distortion_model}")
            return

        # Extract camera parameters
        K = np.array(msg.k).reshape(3, 3)
        D = np.array(msg.d)
        resolution = (msg.width, msg.height)
        
        self.get_logger().info(f"Setting up rectification for {resolution}")
        
        # Setup fisheye rectification
        try:
            P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K=K,
                D=D,
                image_size=resolution,
                R=np.eye(3),
                balance=1.0,
                new_size=resolution,
                fov_scale=1.0
            )
            
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                K=K,
                D=D,
                R=np.eye(3),
                P=P,
                size=resolution,
                m1type=cv2.CV_16SC2
            )
            
            self.get_logger().info("Fisheye rectification maps computed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup rectification: {str(e)}")

    def image_callback(self, msg):
        if self.map1 is None or self.map2 is None:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Apply rectification
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
    node = FisheyeRectificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()