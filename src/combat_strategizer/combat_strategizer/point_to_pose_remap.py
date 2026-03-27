#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node


class PointToPoseRemap(Node):

    def __init__(self):
        super().__init__('point_to_pose_remap')

        # Declare parameters with default values
        self.declare_parameter('input_topic', '/detected_opponent/viz_point')
        self.declare_parameter('output_topic', '/opponent/pose')

        # Get parameter values
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Create subscriber
        self.subscription = self.create_subscription(
            PointStamped,
            input_topic,
            self.point_callback,
            10
        )

        # Create publisher
        self.publisher = self.create_publisher(
            PoseStamped,
            output_topic,
            10
        )

        self.get_logger().info(f'PointToPoseRemap started')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')

    def point_callback(self, msg: PointStamped):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x = msg.point.x
        pose_msg.pose.position.y = msg.point.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointToPoseRemap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()