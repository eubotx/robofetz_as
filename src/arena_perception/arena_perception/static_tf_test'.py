#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticTfTest(Node):
    def __init__(self):
        super().__init__('static_tf_test')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'static_test'
        t.transform.translation.x = 2.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 0.5
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('📌 Published static transform')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTfTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()