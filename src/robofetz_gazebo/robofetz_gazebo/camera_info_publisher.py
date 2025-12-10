#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Declare parameter for config file path
        self.declare_parameter('config_file', '')
        
        # Get config file path
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        if not config_file:
            self.get_logger().error('No config file specified! Please provide a config_file parameter.')
            self.get_logger().info('Example usage: ros2 run your_package camera_info_publisher --ros-args -p config_file:=/path/to/camera.yaml')
            raise ValueError('config_file parameter is required')
        
        # Load camera calibration from YAML file
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f'Config file not found: {config_file}')
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')
            raise
        
        # Extract parameters from config
        self.image_width = config['image_width']
        self.image_height = config['image_height']
        self.distortion_model = config['distortion_model']
        
        # Convert matrix data to flat lists for CameraInfo message
        self.k = config['camera_matrix']['data']
        self.d = config['distortion_coefficients']['data']
        self.r = config['rectification_matrix']['data']
        self.p = config['projection_matrix']['data']
        
        self.get_logger().info(f'Loaded camera config from: {config_file}')
        self.get_logger().info(f'Image size: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'Distortion model: {self.distortion_model}')
        
        # Create publisher
        self.publisher = self.create_publisher(
            CameraInfo,
            'camera_info',
            10)
        
        # Publish at 15Hz
        self.timer = self.create_timer(1.0/15.0, self.publish_camera_info)
    
    def publish_camera_info(self):
        """Publish camera calibration info"""
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arena_camera/link/arena_camera'
        
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
    except ValueError as e:
        rclpy.logging.get_logger('camera_info_publisher').error(str(e))
    except Exception as e:
        rclpy.logging.get_logger('camera_info_publisher').error(f'Unexpected error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()