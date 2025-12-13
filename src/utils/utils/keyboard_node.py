#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import sys
import tty
import select

class EnemyTeleop(Node):
    def __init__(self):
        super().__init__('enemy_teleop_node')
        
        # Create publisher for enemy cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/enemy/cmd_vel', 10)
        
        # Timer for periodic publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Velocity parameters
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.linear_step = 0.5
        self.angular_step = 1.0
        
        self.get_logger().info('Enemy teleop node started!')
        self.get_logger().info('Use arrow keys to control the enemy robot')
        self.get_logger().info('Press SPACE to stop, Q to quit')
        
    def get_key(self):
        """Get a single key press without requiring Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            [i, o, e] = select.select([sys.stdin], [], [], 0.1)
            if i:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def timer_callback(self):
        """Timer callback to handle key presses and publish"""
        key = self.get_key()
        
        if key == '\x1b':  # Escape sequence for arrow keys
            # Check if it's an arrow key
            next_key = self.get_key()
            if next_key == '[':
                arrow_key = self.get_key()
                
                if arrow_key == 'A':  # Up arrow - forward
                    self.linear_vel = self.linear_step
                    self.angular_vel = 0.0
                    self.get_logger().info('UP: Moving forward')
                    
                elif arrow_key == 'B':  # Down arrow - backward
                    self.linear_vel = -self.linear_step
                    self.angular_vel = 0.0
                    self.get_logger().info('DOWN: Moving backward')
                    
                elif arrow_key == 'C':  # Right arrow - turn right
                    self.linear_vel = 0.0
                    self.angular_vel = -self.angular_step
                    self.get_logger().info('RIGHT: Turning right')
                    
                elif arrow_key == 'D':  # Left arrow - turn left
                    self.linear_vel = 0.0
                    self.angular_vel = self.angular_step
                    self.get_logger().info('LEFT: Turning left')
        
        elif key == ' ':  # Space bar - stop
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.get_logger().info('SPACE: Stopped')
            
        elif key.lower() == 'q':  # Quit
            self.get_logger().info('Q: Shutting down...')
            self.stop_robot()
            raise KeyboardInterrupt
            
        elif key == 'w':  # Alternative: W - forward
            self.linear_vel = self.linear_step
            self.angular_vel = 0.0
            self.get_logger().info('W: Moving forward')
            
        elif key == 's':  # Alternative: S - backward
            self.linear_vel = -self.linear_step
            self.angular_vel = 0.0
            self.get_logger().info('S: Moving backward')
            
        elif key == 'a':  # Alternative: A - turn left
            self.linear_vel = 0.0
            self.angular_vel = self.angular_step
            self.get_logger().info('A: Turning left')
            
        elif key == 'd':  # Alternative: D - turn right
            self.linear_vel = 0.0
            self.angular_vel = -self.angular_step
            self.get_logger().info('D: Turning right')
        
        # Publish the command
        self.publish_cmd_vel()
    
    def publish_cmd_vel(self):
        """Publish the current velocity command"""
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel
        twist_msg.angular.z = self.angular_vel
        self.publisher_.publish(twist_msg)
    
    def stop_robot(self):
        """Stop the robot before shutting down"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    
    enemy_teleop = EnemyTeleop()
    
    try:
        rclpy.spin(enemy_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        enemy_teleop.stop_robot()
        enemy_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()