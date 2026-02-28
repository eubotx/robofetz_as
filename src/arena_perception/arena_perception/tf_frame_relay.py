#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped # Added PoseStamped
import sys

class TFFrameRelay(Node):
    def __init__(self):
        super().__init__('tf_frame_relay')
        
        self.declare_parameter('source_frame', 'robot/base_footprint_sim')
        self.declare_parameter('target_frame', 'arena_perception/robot/base_footprint')
        self.declare_parameter('rate', 60.0)
        
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.rate = self.get_parameter('rate').value
        
        # 1. Create the Pose Publisher
        # This creates a topic like "arena_perception/robot/pose"
        topic_name = self.target_frame.replace('/', '_') + '_pose'
        self.pose_pub = self.create_publisher(PoseStamped, topic_name, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(f'Relaying to topic: {topic_name}')
        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)
        
    def timer_callback(self):
        try:
            # Look up transform
            t = self.tf_buffer.lookup_transform(
                'world', 
                self.source_frame, 
                rclpy.time.Time()
            )
            
            # --- KEEP TF BROADCAST (Optional) ---
            new_tf = TransformStamped()
            new_tf.header.stamp = self.get_clock().now().to_msg()
            new_tf.header.frame_id = 'world'
            new_tf.child_frame_id = self.target_frame
            new_tf.transform = t.transform
            self.tf_broadcaster.sendTransform(new_tf)

            # --- NEW: PUBLISH AS POSE TOPIC ---
            pose_msg = PoseStamped()
            pose_msg.header = new_tf.header
            # Convert Transform to Pose
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation
            
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            pass # Keep logs clean during startup

def main(args=None):
    rclpy.init(args=args)
    node = TFFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()