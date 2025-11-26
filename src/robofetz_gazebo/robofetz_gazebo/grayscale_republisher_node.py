#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

#This node is needed because Gazebo does not support grayscale cameras. So instead it publishes rgb and we convert here to grayscale.

class GrayscaleRepublisher(Node):
    def __init__(self):
        super().__init__('grayscale_republisher')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.input_topic = 'arena_camera/image_sim_color'
        self.output_topic = 'arena_camera/image_raw'
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            self.output_topic,
            10)
        
        self.get_logger().info(f'Grayscale republisher initialized')
        self.get_logger().info(f'Subscribed to: {self.input_topic}')
        self.get_logger().info(f'Publishing to: {self.output_topic}')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Convert back to ROS Image message
            gray_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
            
            # Copy the header from original message
            gray_msg.header = msg.header
            
            # Publish the grayscale image
            self.publisher.publish(gray_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = GrayscaleRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()