#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TFToPose(Node):

    def __init__(self):
        super().__init__('tf_to_pose')

        self.declare_parameter('source_frame', '')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('pose_topic', 'pose')
        self.declare_parameter('rate', 60.0)

        self.source_frame = self.get_parameter('source_frame').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.rate = self.get_parameter('rate').value

        if not self.source_frame:
            self.get_logger().error('source_frame parameter is required')
            raise ValueError('source_frame parameter is required')

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'Publishing {self.reference_frame} -> {self.source_frame} '
            f'as PoseStamped on {self.pose_topic}'
        )

        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.source_frame,
                rclpy.time.Time()
            )

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.reference_frame
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation

            self.pose_pub.publish(pose_msg)

        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TFToPose()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
