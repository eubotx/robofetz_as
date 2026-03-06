#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node


class PointToPoseRemap(Node):

    def __init__(self):
        super().__init__('point_to_pose_remap')

        self.subscription = self.create_subscription(
            PointStamped,
            '/detected_opponent/viz_point',
            self.point_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            '/arena_perception_opponent_base_footprint_pose',
            10
        )

        self.get_logger().info('PointToPoseRemap node started')

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
