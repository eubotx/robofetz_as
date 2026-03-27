#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelMuxer(Node):
    def __init__(self):
        super().__init__('cmd_vel_muxer')
        
        # Parameters
        self.declare_parameter('teleop_timeout', 0.5)  # seconds
        self.teleop_timeout = self.get_parameter('teleop_timeout').get_parameter_value().double_value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.teleop_sub = self.create_subscription(
            Twist,
            'cmd_vel_teleop',
            self.teleop_callback,
            10)
        
        self.autonomous_sub = self.create_subscription(
            Twist,
            'cmd_vel_autonomous',
            self.autonomous_callback,
            10)
        
        # State variables
        self.last_teleop_time = 0.0
        self.has_received_teleop = False  # Track if we've ever received teleop
        self.current_teleop_cmd = None
        self.current_autonomous_cmd = None
        
        # Timer to check for timeout and publish
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)  # 20 Hz
        
        self.get_logger().info('CmdVelMuxer node started')
        self.get_logger().info(f'Teleop timeout: {self.teleop_timeout} seconds')
    
    def teleop_callback(self, msg):
        """Handle incoming teleop commands"""
        self.current_teleop_cmd = msg
        self.last_teleop_time = self.get_clock().now().nanoseconds / 1e9
        self.has_received_teleop = True
        self.get_logger().debug('Received teleop command', throttle_duration_sec=1.0)
    
    def autonomous_callback(self, msg):
        """Handle incoming autonomous commands"""
        self.current_autonomous_cmd = msg
        self.get_logger().debug('Received autonomous command', throttle_duration_sec=1.0)
    
    def publish_cmd_vel(self):
        """Decide which command to publish and publish it"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        cmd_to_publish = None
        
        # Check if we have recent teleop commands
        if self.has_received_teleop and self.current_teleop_cmd is not None:
            time_since_teleop = current_time - self.last_teleop_time
            
            if time_since_teleop <= self.teleop_timeout:
                # Use teleop command if it's recent
                cmd_to_publish = self.current_teleop_cmd
                self.get_logger().debug('Publishing teleop command', throttle_duration_sec=1.0)
            else:
                # Teleop timeout, switch to autonomous
                self.get_logger().debug('Teleop timeout, switching to autonomous', throttle_duration_sec=1.0)
                # Clear teleop command to ensure we don't use it again until new one arrives
                self.current_teleop_cmd = None
                self.has_received_teleop = False
        
        # If no teleop command available, use autonomous
        if cmd_to_publish is None:
            if self.current_autonomous_cmd is not None:
                cmd_to_publish = self.current_autonomous_cmd
                self.get_logger().debug('Publishing autonomous command', throttle_duration_sec=1.0)
            else:
                # No commands available, publish zero velocity
                cmd_to_publish = Twist()
                self.get_logger().debug('No commands available, publishing zero velocity', throttle_duration_sec=1.0)
        
        # Publish the selected command
        self.cmd_vel_pub.publish(cmd_to_publish)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()