#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations
import numpy as np

class RobotDetectionNode(Node):
    def __init__(self):
        super().__init__('robot_detection_node')
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store static transforms
        self.tag_to_base_transforms = {}
        
        # Load static transforms once
        self.load_static_tag_to_base_transforms()
        
        # Timer
        self.timer = self.create_timer(0.01, self.update_transform)
        
        self.get_logger().info("Robot Detection Node started")
    
    def load_static_tag_to_base_transforms(self):
        """Load static transforms from URDF once"""
        self.get_logger().info("Waiting for static robot description transforms...")
        
        # Load both tag-to-base transforms
        for tag_type, tag_frame in [('top', 'robot/top_apriltag_optical'),
                                     ('bottom', 'robot/bottom_apriltag_optical')]:
            for _ in range(10):  # Try for 1 second
                try:
                    transform = self.tf_buffer.lookup_transform(
                        tag_frame,
                        'robot/base_footprint',
                        rclpy.time.Time()
                    )
                    self.tag_to_base_transforms[tag_type] = transform
                    self.get_logger().info(f"Loaded static transform: {tag_frame} -> base_footprint")
                    break
                except tf2_ros.TransformException:
                    rclpy.spin_once(self, timeout_sec=0.1)
    
    def compose_transforms(self, transform_a, transform_b):
        """Compose two transforms: result = A * B (apply B then A)"""
        # Convert to numpy
        t1 = np.array([transform_a.transform.translation.x,
                       transform_a.transform.translation.y,
                       transform_a.transform.translation.z])
        q1 = np.array([transform_a.transform.rotation.x,
                       transform_a.transform.rotation.y,
                       transform_a.transform.rotation.z,
                       transform_a.transform.rotation.w])
        
        t2 = np.array([transform_b.transform.translation.x,
                       transform_b.transform.translation.y,
                       transform_b.transform.translation.z])
        q2 = np.array([transform_b.transform.rotation.x,
                       transform_b.transform.rotation.y,
                       transform_b.transform.rotation.z,
                       transform_b.transform.rotation.w])
        
        # Compose rotations
        q_result = tf_transformations.quaternion_multiply(q1, q2)
        
        # Rotate t2 by q1 and add to t1
        rot_matrix = tf_transformations.quaternion_matrix(q1)[:3, :3]
        t_rotated = rot_matrix @ t2
        t_result = t1 + t_rotated
        
        # Create result
        result = TransformStamped()
        result.transform.translation.x = t_result[0]
        result.transform.translation.y = t_result[1]
        result.transform.translation.z = t_result[2]
        result.transform.rotation.x = q_result[0]
        result.transform.rotation.y = q_result[1]
        result.transform.rotation.z = q_result[2]
        result.transform.rotation.w = q_result[3]
        
        return result
    
    def update_transform(self):
        # Try to get both transforms and use the most recent one
        camera_to_top = None
        camera_to_bottom = None
        
        # Get timestamp for comparison
        current_time = self.get_clock().now()
        
        # Try to get top tag transform
        try:
            camera_to_top = self.tf_buffer.lookup_transform(
                'arena_camera_optical',
                'arena_perception/robot_top_tag',
                rclpy.time.Time()
            )
            # Check if transform is recent (within 0.2 seconds)
            transform_time = rclpy.time.Time.from_msg(camera_to_top.header.stamp)
            if (current_time - transform_time).nanoseconds > 200_000_000:  # 0.2 seconds
                camera_to_top = None  # Too old
        except tf2_ros.TransformException:
            pass
        
        # Try to get bottom tag transform
        try:
            camera_to_bottom = self.tf_buffer.lookup_transform(
                'arena_camera_optical',
                'arena_perception/robot_bottom_tag',
                rclpy.time.Time()
            )
            # Check if transform is recent
            transform_time = rclpy.time.Time.from_msg(camera_to_bottom.header.stamp)
            if (current_time - transform_time).nanoseconds > 200_000_000:  # 0.2 seconds
                camera_to_bottom = None  # Too old
        except tf2_ros.TransformException:
            pass
        
        # Determine which transform to use (prefer more recent)
        chosen_transform = None
        chosen_tag_type = None
        
        if camera_to_top and camera_to_bottom:
            # Both available, use the more recent one
            top_time = rclpy.time.Time.from_msg(camera_to_top.header.stamp)
            bottom_time = rclpy.time.Time.from_msg(camera_to_bottom.header.stamp)
            
            if top_time > bottom_time:
                chosen_transform = camera_to_top
                chosen_tag_type = 'top'
            else:
                chosen_transform = camera_to_bottom
                chosen_tag_type = 'bottom'
        elif camera_to_top:
            chosen_transform = camera_to_top
            chosen_tag_type = 'top'
        elif camera_to_bottom:
            chosen_transform = camera_to_bottom
            chosen_tag_type = 'bottom'
        else:
            # No valid transforms available
            return
        
        # Check if we have the corresponding static transform
        if chosen_tag_type not in self.tag_to_base_transforms:
            return
        
        # Get the pre-loaded static transform
        tag_to_base = self.tag_to_base_transforms[chosen_tag_type]
        
        # Compose transforms: camera -> tag -> base
        camera_to_base = self.compose_transforms(chosen_transform, tag_to_base)
        
        # Set header and publish
        camera_to_base.header.stamp = self.get_clock().now().to_msg()
        camera_to_base.header.frame_id = 'arena_camera_optical'
        camera_to_base.child_frame_id = 'robot/base_footprint'
        
        self.tf_broadcaster.sendTransform(camera_to_base)

def main(args=None):
    rclpy.init(args=args)
    node = RobotDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()