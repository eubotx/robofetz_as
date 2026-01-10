#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_inverse, quaternion_multiply
import numpy as np

class TagTransformPublisher(Node):
    def __init__(self):
        super().__init__('tag_transform_publisher')
        
        # Create static transform broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Create buffer and listener for dynamic transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to publish transforms
        self.timer = self.create_timer(1.0, self.publish_static_transforms)
        
        self.get_logger().info("Tag Transform Publisher started")
        self.transforms_published = False
        
    def publish_static_transforms(self):
        if self.transforms_published:
            return
            
        try:
            # Get transform from base_footprint to bottom_apriltag_link
            bottom_tag_tf = self.tf_buffer.lookup_transform(
                'robot/base_footprint',
                'robot/bottom_apriltag_link',
                rclpy.time.Time()
            )
            
            # Get transform from base_footprint to top_apriltag_link
            top_tag_tf = self.tf_buffer.lookup_transform(
                'robot/base_footprint',
                'robot/top_apriltag_link',
                rclpy.time.Time()
            )
            
            # Create static transforms
            bottom_static_tf = TransformStamped()
            top_static_tf = TransformStamped()
            
            # Get current time
            now = self.get_clock().now()
            
            # Bottom tag transform
            bottom_static_tf.header.stamp = now.to_msg()
            bottom_static_tf.header.frame_id = 'arena_perception/robot_bottom_tag'
            bottom_static_tf.child_frame_id = 'arena_perception/robot_base_footprint'
            
            # Invert the transform (we want tag -> base_footprint)
            # For translation: T_tag^base = -T_base^tag
            bottom_static_tf.transform.translation.x = -bottom_tag_tf.transform.translation.x
            bottom_static_tf.transform.translation.y = -bottom_tag_tf.transform.translation.y
            bottom_static_tf.transform.translation.z = -bottom_tag_tf.transform.translation.z
            
            # For rotation: q_tag^base = (q_base^tag)^-1
            q_orig = bottom_tag_tf.transform.rotation
            q_inv = quaternion_inverse([q_orig.x, q_orig.y, q_orig.z, q_orig.w])
            bottom_static_tf.transform.rotation.x = q_inv[0]
            bottom_static_tf.transform.rotation.y = q_inv[1]
            bottom_static_tf.transform.rotation.z = q_inv[2]
            bottom_static_tf.transform.rotation.w = q_inv[3]
            
            # Top tag transform
            top_static_tf.header.stamp = now.to_msg()
            top_static_tf.header.frame_id = 'arena_perception/robot_top_tag'
            top_static_tf.child_frame_id = 'arena_perception/robot_base_footprint'
            
            # Invert the transform
            top_static_tf.transform.translation.x = -top_tag_tf.transform.translation.x
            top_static_tf.transform.translation.y = -top_tag_tf.transform.translation.y
            top_static_tf.transform.translation.z = -top_tag_tf.transform.translation.z
            
            q_orig_top = top_tag_tf.transform.rotation
            q_inv_top = quaternion_inverse([q_orig_top.x, q_orig_top.y, q_orig_top.z, q_orig_top.w])
            top_static_tf.transform.rotation.x = q_inv_top[0]
            top_static_tf.transform.rotation.y = q_inv_top[1]
            top_static_tf.transform.rotation.z = q_inv_top[2]
            top_static_tf.transform.rotation.w = q_inv_top[3]
            
            # Publish static transforms
            self.static_broadcaster.sendTransform([bottom_static_tf, top_static_tf])
            self.get_logger().info("Published static transforms for AprilTags")
            
            # Mark as published and cancel timer
            self.transforms_published = True
            self.timer.cancel()
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f"Could not get transform: {ex}")
            # Try again next timer tick

def main(args=None):
    rclpy.init(args=args)
    node = TagTransformPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()