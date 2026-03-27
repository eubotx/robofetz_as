#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class BattleBotNavigator(Node):
    def __init__(self):
        super().__init__('battle_bot_navigator')
        
        # Declare parameters - more aggressive defaults for battle bot
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_kp', 2.0),        # Increased for faster response
                ('angular_kp', 3.0),        # Increased for quicker turns
                ('angular_ki', 0.5),         # Keep some integral for steady-state error
                ('max_linear', 1.0),          # Increased max speed
                ('max_angular', 3.5),          # Increased turn rate
                ('position_tolerance', 0.1),   # Slightly larger tolerance
                ('orientation_tolerance', 0.3), # Larger orientation tolerance
                ('goal_frame', 'map'),
                ('integral_windup_limit', 2.0),
                ('min_speed', 0.1),            # Minimum speed to overcome friction
                ('angle_threshold_for_driving', 0.5), # Radians - drive even if slightly misaligned
                ('distance_to_slow_down', 0.5),  # When to start slowing down
            ]
        )
        
        # Get parameters
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.angular_ki = self.get_parameter('angular_ki').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.integral_windup_limit = self.get_parameter('integral_windup_limit').value
        self.min_speed = self.get_parameter('min_speed').value
        self.angle_threshold = self.get_parameter('angle_threshold_for_driving').value
        self.slow_distance = self.get_parameter('distance_to_slow_down').value
        
        # Current state
        self.current_pose = None
        self.current_pose_header = None
        self.goal_pose = None
        self.goal_pose_header = None
        
        # Control variables
        self.yaw_integral = 0.0
        self.last_yaw_error = 0.0
        self.last_time = self.get_clock().now()
        
        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/robot/pose', 
            self.pose_callback, 
            10)
            
        self.goal_sub = self.create_subscription(
            PoseStamped, 
            '/goal_pose', 
            self.goal_callback, 
            10)
            
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer (30Hz)
        self.control_timer = self.create_timer(1.0/30.0, self.control_loop)
        
        self.get_logger().info("Battle Bot Navigator initialized - Ready to fight!")

    def pose_callback(self, msg):
        """Store the current robot pose from camera"""
        self.current_pose = msg.pose
        self.current_pose_header = msg.header
        
    def goal_callback(self, msg):
        """Store the goal pose"""
        self.goal_pose = msg.pose
        self.goal_pose_header = msg.header
        self.yaw_integral = 0.0
        self.get_logger().info(f"Target acquired at: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})")

    def control_loop(self):
        if self.current_pose is None or self.current_pose_header is None:
            self.get_logger().warn("Waiting for position fix...", throttle_duration_sec=1.0)
            return
            
        if self.goal_pose is None or self.goal_pose_header is None:
            self.get_logger().warn("Waiting for target...", throttle_duration_sec=1.0)
            return
            
        try:
            # Get transform between frames if needed
            if self.goal_pose_header.frame_id != self.current_pose_header.frame_id:
                transform = self.tf_buffer.lookup_transform(
                    self.goal_pose_header.frame_id,
                    self.current_pose_header.frame_id,
                    rclpy.time.Time())
                # Apply transform to current pose (simplified - would need full transform in reality)
            
            # Calculate errors
            dx = self.goal_pose.position.x - self.current_pose.position.x
            dy = self.goal_pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Calculate angle to goal
            desired_yaw = math.atan2(dy, dx)
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            yaw_error = self.normalize_angle(desired_yaw - current_yaw)
            
            # Calculate time difference
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 0.1:  # Cap dt to prevent spikes
                dt = 0.1
            self.last_time = now
            
            # Update integral with anti-windup
            if abs(yaw_error) < math.pi/1.5:  # Wider integration range
                self.yaw_integral += yaw_error * dt
                self.yaw_integral = max(min(self.yaw_integral, self.integral_windup_limit), 
                                     -self.integral_windup_limit)
            else:
                self.yaw_integral *= 0.5  # Decay integral instead of resetting completely
            
            # Generate commands - SIMULTANEOUS drive and steer
            cmd = Twist()
            
            if distance > self.position_tolerance:
                # Calculate angular command (always active)
                angular_cmd = (self.angular_kp * yaw_error + 
                             self.angular_ki * self.yaw_integral)
                
                # Calculate linear command with angle weighting
                # Reduce speed when turning sharp, but don't stop
                angle_factor = max(0.3, 1.0 - abs(yaw_error) / math.pi)  # Scale 0.3-1.0
                
                # Base linear speed from distance
                if distance < self.slow_distance:
                    # Slow down when approaching target
                    linear_cmd = self.linear_kp * distance * 0.5
                else:
                    linear_cmd = self.linear_kp * self.max_linear
                
                # Apply angle factor
                linear_cmd *= angle_factor
                
                # Apply minimum speed if we're still far away
                if distance > self.slow_distance * 2 and linear_cmd < self.min_speed:
                    linear_cmd = self.min_speed
                
                # Clamp commands
                cmd.linear.x = max(min(linear_cmd, self.max_linear), 0.0)  # No reverse for now
                cmd.angular.z = max(min(angular_cmd, self.max_angular), -self.max_angular)
                
                # Log occasionally
                if self.get_clock().now().nanoseconds % 1000000000 < 30000000:  # ~once per second
                    self.get_logger().info(
                        f"Dist: {distance:.2f}, Yaw err: {math.degrees(yaw_error):.1f}°, "
                        f"Cmd: L={cmd.linear.x:.2f}, A={cmd.angular.z:.2f}")
            else:
                # At goal, do a little victory spin? ;)
                cmd.angular.z = 0.0
                self.get_logger().info("Target reached!")
            
            self.cmd_pub.publish(cmd)
            
        except TransformException as ex:
            self.get_logger().warn(f"Transform error: {ex}")
            # Publish zero velocity on transform errors for safety
            self.cmd_pub.publish(Twist())
            return
        except Exception as ex:
            self.get_logger().error(f"Control error: {ex}")
            return

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle - optimized for battle bot"""
        # Using the standard conversion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    navigator = BattleBotNavigator()
    
    # Create a kill switch for emergency stop (optional)
    def shutdown_handler():
        navigator.get_logger().info("Shutting down - stopping robot")
        cmd = Twist()
        navigator.cmd_pub.publish(cmd)
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        shutdown_handler()
    finally:
        shutdown_handler()
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()