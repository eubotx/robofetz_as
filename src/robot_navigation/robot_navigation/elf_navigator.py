#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class ElfNavigator(Node):

    def __init__(self):
        super().__init__('elf_navigator')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('default_linear_speed', 0.3),
                ('default_angular_speed', 1.0),
            ]
        )

        self.default_linear_speed = self.get_parameter(
            'default_linear_speed'
        ).value
        self.default_angular_speed = self.get_parameter(
            'default_angular_speed'
        ).value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.command_sub = self.create_subscription(
            String,
            '/elf_navigator_cmd',
            self.command_callback,
            10
        )

        self.get_logger().info('Elf Navigator initialized - Ready!')

    def command_callback(self, msg: String):
        command = msg.data.strip()
        parts = command.upper().split(':')
        action = parts[0]

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
