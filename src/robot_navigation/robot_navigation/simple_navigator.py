#!/usr/bin/env python3
import math

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class BattleBotNavigator(Node):
    def __init__(self):
        super().__init__('battle_bot_navigator')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_kp', 5.0),
                ('angular_kp', 3.0),
                ('angular_ki', 0.5),
                ('angular_kd', 0.1),
                ('max_linear', 5.0),
                ('max_angular', 3.5),
                ('position_tolerance', 0.1),
                ('orientation_tolerance', 0.3),
                ('goal_frame', 'map'),
                ('integral_windup_limit', 2.0),
                ('min_speed', 0.5),
                ('angle_threshold_for_driving', 0.5),
                ('distance_to_slow_down', 0.1),
            ]
        )

        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.angular_ki = self.get_parameter('angular_ki').value
        self.angular_kd = self.get_parameter('angular_kd').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.integral_windup_limit = self.get_parameter('integral_windup_limit').value
        self.min_speed = self.get_parameter('min_speed').value
        self.angle_threshold = self.get_parameter('angle_threshold_for_driving').value
        self.slow_distance = self.get_parameter('distance_to_slow_down').value

        self.current_pose = None
        self.current_pose_header = None
        self.goal_pose = None
        self.goal_pose_header = None

        self.yaw_integral = 0.0
        self.last_yaw_error = 0.0
        self.last_time = self.get_clock().now()

        self.escape_mode = False
        self.escape_phase = 'orient'
        self.escape_opponent_pose = None
        self.escape_safe_pose = None
        self.escape_yaw_integral = 0.0
        self.escape_last_yaw_error = 0.0
        self.escape_last_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/arena_perception_robot_base_footprint_pose',
            self.pose_callback,
            10)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)

        self.escape_opponent_sub = self.create_subscription(
            PoseStamped,
            '/escape_opponent',
            self.escape_opponent_callback,
            10)

        self.escape_safe_sub = self.create_subscription(
            PoseStamped,
            '/escape_safe',
            self.escape_safe_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.control_timer = self.create_timer(1.0/30.0, self.control_loop)

        self.get_logger().info('Battle Bot Navigator initialized - Ready to fight!')

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.current_pose_header = msg.header

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.goal_pose_header = msg.header
        self.yaw_integral = 0.0
        self.get_logger().info(
            f'Target acquired at: ({self.goal_pose.position.x:.2f}, '
            f'{self.goal_pose.position.y:.2f})'
        )

    def escape_opponent_callback(self, msg):
        self.escape_opponent_pose = msg.pose

    def escape_safe_callback(self, msg):
        self.escape_safe_pose = msg.pose
        self.escape_mode = True
        self.escape_phase = 'orient'
        self.goal_pose = None
        self.escape_yaw_integral = 0.0
        self.escape_last_yaw_error = 0.0
        self.escape_last_time = self.get_clock().now()
        self.get_logger().info(
            f'Escape mode activated! Safe position: '
            f'({self.escape_safe_pose.position.x:.2f}, '
            f'{self.escape_safe_pose.position.y:.2f})'
        )

    def control_loop(self):
        if self.current_pose is None or self.current_pose_header is None:
            self.get_logger().warn('Waiting for position fix...', throttle_duration_sec=1.0)
            return

        if self.escape_mode:
            self._escape_control()
            return

        if self.goal_pose is None or self.goal_pose_header is None:
            self.get_logger().warn('Waiting for target...', throttle_duration_sec=1.0)
            return

        try:
            if self.goal_pose_header.frame_id != self.current_pose_header.frame_id:
                self.tf_buffer.lookup_transform(
                    self.goal_pose_header.frame_id,
                    self.current_pose_header.frame_id,
                    rclpy.time.Time())

            dx = self.goal_pose.position.x - self.current_pose.position.x
            dy = self.goal_pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)

            desired_yaw = math.atan2(dy, dx)
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            yaw_error = self.normalize_angle(desired_yaw - current_yaw)

            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 0.1:
                dt = 0.1
            self.last_time = now

            if abs(yaw_error) < math.pi/1.5:
                self.yaw_integral += yaw_error * dt
                self.yaw_integral = max(min(self.yaw_integral, self.integral_windup_limit),
                                        -self.integral_windup_limit)
            else:
                self.yaw_integral *= 0.5

            cmd = Twist()

            if distance > self.position_tolerance:
                angular_cmd = (self.angular_kp * yaw_error +
                               self.angular_ki * self.yaw_integral)

                angle_factor = max(0.6, 1.0 - abs(yaw_error) / math.pi)

                # if distance < self.slow_distance:
                #     linear_cmd = self.linear_kp * distance * 0.5
                # else:
                #     linear_cmd = self.linear_kp * self.max_linear
                linear_cmd = self.linear_kp * self.max_linear

                self.get_logger().info(f'Linear cmd before angle factor: {linear_cmd:.2f}, Angle factor: {angle_factor:.2f}')

                linear_cmd *= angle_factor

                if distance > self.slow_distance * 2 and linear_cmd < self.min_speed:
                    linear_cmd = self.min_speed

                cmd.linear.x = max(min(linear_cmd, self.max_linear), 0.0)
                cmd.angular.z = max(min(angular_cmd, self.max_angular), -self.max_angular)

                if self.get_clock().now().nanoseconds % 1000000000 < 30000000:
                    self.get_logger().info(
                        f'Dist: {distance:.2f}, Yaw err: {math.degrees(yaw_error):.1f}°, '
                        f'Cmd: L={cmd.linear.x:.2f}, A={cmd.angular.z:.2f}'
                    )
            else:
                cmd.angular.z = 0.0
                self.get_logger().info('Target reached!')

            self.get_logger().info(f'Publishing cmd: Linear={cmd.linear.x:.2f}, Angular={cmd.angular.z:.2f}')

            self.cmd_pub.publish(cmd)

        except TransformException as ex:
            self.get_logger().warn(f'Transform error: {ex}')
            self.cmd_pub.publish(Twist())
            return
        except Exception as ex:
            self.get_logger().error(f'Control error: {ex}')
            return

    def _escape_control(self):
        if self.escape_safe_pose is None:
            self.get_logger().warn('No safe position for escape')
            return

        dx_escape = self.escape_safe_pose.position.x - self.current_pose.position.x
        dy_escape = self.escape_safe_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx_escape**2 + dy_escape**2)

        if distance < self.position_tolerance:
            self.escape_mode = False
            self.escape_phase = 'orient'
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Escape complete!')
            return

        now = self.get_clock().now()
        dt = (now - self.escape_last_time).nanoseconds * 1e-9
        if dt > 0.1:
            dt = 0.1
        self.escape_last_time = now

        if self.escape_phase == 'orient':
            if self.escape_opponent_pose is None:
                self.get_logger().warn(
                    'No opponent pose for escape orientation',
                    throttle_duration_sec=1.0
                )
                return

            dx_opp = self.escape_opponent_pose.position.x - self.current_pose.position.x
            dy_opp = self.escape_opponent_pose.position.y - self.current_pose.position.y
            desired_yaw = math.atan2(dy_opp, dx_opp)
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            yaw_error = self.normalize_angle(desired_yaw - current_yaw)

            self.escape_yaw_integral += yaw_error * dt
            self.escape_yaw_integral = max(
                min(self.escape_yaw_integral, self.integral_windup_limit),
                -self.integral_windup_limit
            )
            angular_derivative = (yaw_error - self.escape_last_yaw_error) / dt if dt > 0 else 0.0

            angular_cmd = (self.angular_kp * yaw_error +
                           self.angular_ki * self.escape_yaw_integral +
                           self.angular_kd * angular_derivative)

            self.escape_last_yaw_error = yaw_error

            cmd = Twist()
            cmd.angular.z = max(min(angular_cmd, self.max_angular), -self.max_angular)
            self.cmd_pub.publish(cmd)

            if self.get_clock().now().nanoseconds % 1000000000 < 30000000:
                self.get_logger().info(
                    f'ESCAPE ORIENT: Yaw err={math.degrees(yaw_error):.1f}°, '
                    f'Cmd: A={cmd.angular.z:.2f}'
                )

            if abs(yaw_error) < self.orientation_tolerance:
                self.escape_phase = 'drive'
                self.escape_yaw_integral = 0.0
                self.escape_last_yaw_error = 0.0
                self.get_logger().info('Escape ORIENT complete, switching to DRIVE')

        else:
            angle_to_escape = math.atan2(dy_escape, dx_escape)
            desired_yaw = angle_to_escape + math.pi
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            yaw_error = self.normalize_angle(desired_yaw - current_yaw)

            self.escape_yaw_integral += yaw_error * dt
            self.escape_yaw_integral = max(
                min(self.escape_yaw_integral, self.integral_windup_limit),
                -self.integral_windup_limit
            )
            angular_derivative = (yaw_error - self.escape_last_yaw_error) / dt if dt > 0 else 0.0

            angular_cmd = (self.angular_kp * yaw_error +
                           self.angular_ki * self.escape_yaw_integral +
                           self.angular_kd * angular_derivative)

            self.escape_last_yaw_error = yaw_error

            angle_factor = max(0.3, 1.0 - abs(yaw_error) / math.pi)

            # if distance < self.slow_distance:
            #     linear_cmd = self.linear_kp * distance * 0.5
            # else:
            #     linear_cmd = self.linear_kp * self.max_linear
            linear_cmd = self.linear_kp * self.max_linear

            linear_cmd *= angle_factor

            if distance > self.slow_distance * 2 and linear_cmd < self.min_speed:
                linear_cmd = self.min_speed

            cmd = Twist()
            cmd.linear.x = max(min(-linear_cmd, -self.min_speed), -self.max_linear)
            cmd.angular.z = max(min(angular_cmd, self.max_angular), -self.max_angular)
            self.cmd_pub.publish(cmd)

            if self.get_clock().now().nanoseconds % 1000000000 < 30000000:
                self.get_logger().info(
                    f'ESCAPE DRIVE: Dist={distance:.2f}, Yaw err={math.degrees(yaw_error):.1f}°, '
                    f'Cmd: L={cmd.linear.x:.2f} (REV), A={cmd.angular.z:.2f}'
                )

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    navigator = BattleBotNavigator()

    def shutdown_handler():
        navigator.get_logger().info('Shutting down - stopping robot')
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
