#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import yaml

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('image_topic', '/arena_camera/image')
        
        # Get parameters
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        if not config_file:
            self.get_logger().error('No config file specified! Please provide a config_file parameter.')
            raise ValueError('config_file parameter is required')
        
        # Load camera calibration from YAML file
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}')
            raise
        
        # Store camera info
        self.image_width = config['image_width']
        self.image_height = config['image_height']
        self.distortion_model = config['distortion_model']
        self.k = config['camera_matrix']['data']
        self.d = config['distortion_coefficients']['data']
        self.r = config['rectification_matrix']['data']
        self.p = config['projection_matrix']['data']
        
        self.get_logger().info(f'Loaded camera config from: {config_file}')
        
        # Create publisher
        self.publisher = self.create_publisher(CameraInfo, 'camera_info', 10)
        
        # Subscribe to image topic
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        
        # Track last published timestamp to avoid duplicates
        self.last_timestamp = None
        
        self.get_logger().info(f'Subscribed to image topic: {image_topic}')
        
    def image_callback(self, image_msg):
        # Avoid publishing duplicate camera_info for same timestamp
        if self.last_timestamp == image_msg.header.stamp:
            return
            
        self.last_timestamp = image_msg.header.stamp
        
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = image_msg.header.frame_id
        
        msg.height = self.image_height
        msg.width = self.image_width
        msg.distortion_model = self.distortion_model
        msg.d = self.d
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
    
    try:
        node = CameraInfoPublisher()
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger('camera_info_publisher').error(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()