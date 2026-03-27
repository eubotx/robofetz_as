#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ElfNavigator(Node):

    def __init__(self):
        super().__init__('elf_navigator')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('default_linear_speed', 0.3),
                ('default_angular_speed', 3.0),
                ('forward_speed', 3.0),
                ('backward_speed', 3.0),
                ('turn_speed', 3.5),
                ('position_tolerance', 0.1),
                ('angle_threshold_for_driving', 0.17),
                ('linear_kp',1.0),
                ('angular_kp', 1.8),
                ('angular_ki', 0.5),
                ('integral_windup_limit', 2.0),
            ]
        )

        self.default_linear_speed = self.get_parameter(
            'default_linear_speed'
        ).value
        self.default_angular_speed = self.get_parameter(
            'default_angular_speed'
        ).value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_threshold = self.get_parameter(
            'angle_threshold_for_driving'
        ).value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.angular_ki = self.get_parameter('angular_ki').value
        self.integral_windup_limit = self.get_parameter('integral_windup_limit').value

        self.goal_pose: PoseStamped | None = None
        self.robot_pose: PoseStamped | None = None
        self.drive_mode = 'FORWARD'
        self.tracking_active = False
        self.yaw_integral = 0.0
        self.last_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.command_sub = self.create_subscription(
            String,
            '/elf_navigator_cmd',
            self.command_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )

        self.drive_mode_sub = self.create_subscription(
            String,
            '/drive_mode',
            self.drive_mode_callback,
            10
        )

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Elf Navigator initialized - Ready!')

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.tracking_active = True
        self.yaw_integral = 0.0

    def pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    def drive_mode_callback(self, msg: String):
        self.drive_mode = msg.data.upper()

    def command_callback(self, msg: String):
        command = msg.data.strip()
        parts = command.upper().split(':')
        action = parts[0]

        self.tracking_active = False
        self.goal_pose = None

        cmd = Twist()

        if action == 'STOP':
            pass

        elif action == 'FORWARD':
            speed = float(parts[1]) if len(parts) > 1 else self.default_linear_speed
            cmd.linear.x = abs(speed)

        elif action == 'BACKWARD':
            speed = float(parts[1]) if len(parts) > 1 else self.default_linear_speed
            cmd.linear.x = -abs(speed)

        elif action == 'TURN':
            angular = float(parts[1]) if len(parts) > 1 else self.default_angular_speed
            cmd.angular.z = angular

        elif action == 'TURN_LEFT':
            speed = float(parts[1]) if len(parts) > 1 else self.default_angular_speed
            cmd.angular.z = abs(speed)

        elif action == 'TURN_RIGHT':
            speed = float(parts[1]) if len(parts) > 1 else self.default_angular_speed
            cmd.angular.z = -abs(speed)

        else:
            self.get_logger().warn(f'Unknown command: {action}')
            return

        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if not self.tracking_active:
            return
        if self.goal_pose is None or self.robot_pose is None:
            return

        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < self.position_tolerance:
            self.tracking_active = False
            self.cmd_pub.publish(Twist())
            return

        desired_yaw = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.robot_pose.pose.orientation)
        yaw_error = self.normalize_angle(desired_yaw - current_yaw)

        if self.drive_mode == 'FORWARD':
            effective_yaw_error = yaw_error
        else:
            effective_yaw_error = self.normalize_angle(yaw_error + math.pi)

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt > 0.1:
            dt = 0.1
        self.last_time = now

        if abs(effective_yaw_error) < math.pi / 1.5:
            self.yaw_integral += effective_yaw_error * dt
            self.yaw_integral = max(min(self.yaw_integral, self.integral_windup_limit),
                                    -self.integral_windup_limit)
        else:
            self.yaw_integral *= 0.5

        cmd = Twist()

        angular_cmd = self.angular_kp * effective_yaw_error + self.angular_ki * self.yaw_integral
        cmd.angular.z = max(min(angular_cmd, self.turn_speed), -self.turn_speed)

        angle_factor = max(0.3, 1.0 - abs(effective_yaw_error) / math.pi)
        linear_cmd = self.linear_kp * distance * angle_factor
        if self.drive_mode == 'FORWARD':
            cmd.linear.x = max(min(linear_cmd, self.forward_speed), 0.0)
        else:
            cmd.linear.x = max(min(-linear_cmd, 0.0), -self.backward_speed)

        self.cmd_pub.publish(cmd)

    @staticmethod
    def quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    navigator = ElfNavigator()

    def shutdown_handler():
        cmd = Twist()
        navigator.cmd_pub.publish(cmd)
        navigator.get_logger().info('Shutting down - stopping robot')

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
