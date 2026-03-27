#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class CmdVelMuxer(Node):
    def __init__(self):
        super().__init__('cmd_vel_muxer')
        
        # Parameters
        self.declare_parameter('teleop_timeout', 0.5)  # seconds
        self.teleop_timeout = self.get_parameter('teleop_timeout').get_parameter_value().double_value
        
        # Inversion parameters
        self.declare_parameter('invert_linear_x', True)
        self.declare_parameter('invert_linear_y', True)
        self.declare_parameter('invert_angular_z', False)
        self.invert_linear_x = self.get_parameter('invert_linear_x').get_parameter_value().bool_value
        self.invert_linear_y = self.get_parameter('invert_linear_y').get_parameter_value().bool_value
        self.invert_angular_z = self.get_parameter('invert_angular_z').get_parameter_value().bool_value
        
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
        
        self.marker_sub = self.create_subscription(
            String,
            'arena_perception/robot/visible_marker',
            self.marker_callback,
            10)
        
        # State variables
        self.last_teleop_time = 0.0
        self.has_received_teleop = False
        self.current_teleop_cmd = None
        self.current_autonomous_cmd = None
        self.current_marker = None  # "top" or "bottom"
        
        # Timer to check for timeout and publish
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)  # 20 Hz
        
        self.get_logger().info('CmdVelMuxer node started')
        self.get_logger().info(f'Teleop timeout: {self.teleop_timeout} seconds')
        self.get_logger().info(f'Inversion settings: linear_x={self.invert_linear_x}, linear_y={self.invert_linear_y}, angular_z={self.invert_angular_z}')
        self.get_logger().info('Listening for marker state on arena_perception/robot/visible_marker')
    
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
    
    def marker_callback(self, msg):
        """Handle marker state changes"""
        self.current_marker = msg.data
        self.get_logger().info(f'Marker state changed to: {self.current_marker}')
    
    def invert_for_upside_down(self, twist_msg):
        """
        Invert velocities for an upside-down robot based on configurable parameters.
        Only inverts axes that are enabled via parameters.
        """
        inverted = Twist()
        
        # Apply inversion based on parameters
        inverted.linear.x = -twist_msg.linear.x if self.invert_linear_x else twist_msg.linear.x
        inverted.linear.y = -twist_msg.linear.y if self.invert_linear_y else twist_msg.linear.y
        inverted.linear.z = twist_msg.linear.z  # Keep vertical as is (usually 0)
        
        # Keep angular.x and angular.y as is (usually 0 for ground robots)
        inverted.angular.x = twist_msg.angular.x
        inverted.angular.y = twist_msg.angular.y
        
        # Apply inversion for rotation
        inverted.angular.z = -twist_msg.angular.z if self.invert_angular_z else twist_msg.angular.z
        
        return inverted
    
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
        
        # Apply inversion if marker is "bottom" (upside down)
        if self.current_marker == "bottom":
            cmd_to_publish = self.invert_for_upside_down(cmd_to_publish)
            self.get_logger().debug('Applying inversion for upside-down operation', throttle_duration_sec=1.0)
        
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