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
        
        # ========== CONFIGURATION ==========
        # Define all tag pairs you want to support
        # Each pair: (detection_frame, robot_optical_frame)
        self.TAG_PAIRS = [
            # Format: (detection_frame, robot_optical_frame, tag_name)
            ('arena_perception/robot_top_tag', 'robot/top_apriltag_optical', 'top'),
            ('arena_perception/robot_bottom_tag', 'robot/bottom_apriltag_optical', 'bottom'),
            # Add more pairs as needed:
        ]
        
        # Frame definitions
        self.CAMERA_FRAME = 'arena_camera_optical'
        self.ROBOT_BASE_FRAME = 'robot/base_footprint'
        
        # Timing parameters
        self.UPDATE_RATE = 0.01  # 100 Hz
        self.TRANSFORM_TIMEOUT = 0.2  # seconds (max age of transform)
        
        # ========== INITIALIZATION ==========
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store static transforms
        self.tag_to_base_transforms = {}
        
        # Load static transforms once
        self.load_static_transforms()
        
        # Timer
        self.timer = self.create_timer(self.UPDATE_RATE, self.update_transform)
        
        self.get_logger().info("Robot Detection Node started")
        self.get_logger().info(f"Camera frame: {self.CAMERA_FRAME}")
        self.get_logger().info(f"Robot Base frame: {self.ROBOT_BASE_FRAME}")
        self.get_logger().info(f"Configured {len(self.TAG_PAIRS)} tag pair(s)")
    
    def load_static_transforms(self):
        """Load static transforms from URDF for all configured tag pairs"""
        self.get_logger().info("Loading static transforms...")
        
        for detection_frame, robot_frame, tag_name in self.TAG_PAIRS:
            self.get_logger().info(f"Waiting for static transform: {robot_frame} -> {self.ROBOT_BASE_FRAME}")
            
            loaded = False
            for attempt in range(20):  # Try for 2 seconds
                try:
                    transform = self.tf_buffer.lookup_transform(
                        robot_frame,
                        self.ROBOT_BASE_FRAME,
                        rclpy.time.Time()
                    )
                    self.tag_to_base_transforms[tag_name] = {
                        'transform': transform,
                        'robot_frame': robot_frame,
                        'detection_frame': detection_frame
                    }
                    self.get_logger().info(f"✓ Loaded static transform for '{tag_name}' tag")
                    loaded = True
                    break
                except tf2_ros.TransformException as e:
                    if attempt == 19:  # Last attempt
                        self.get_logger().warn(f"✗ Failed to load transform for '{tag_name}': {e}")
                    rclpy.spin_once(self, timeout_sec=0.1)
            
            if not loaded:
                self.get_logger().error(f"Could not load transform for tag '{tag_name}'. Node may not function properly.")
    
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
    
    def get_detected_transform(self, detection_frame):
        """Try to get a transform for a detected tag, checking its age"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.CAMERA_FRAME,
                detection_frame,
                rclpy.time.Time()
            )
            
            # Check if transform is recent
            current_time = self.get_clock().now()
            transform_time = rclpy.time.Time.from_msg(transform.header.stamp)
            age_ns = (current_time - transform_time).nanoseconds
            
            return transform, age_ns
                
        except tf2_ros.TransformException:
            return None, None
    
    def find_best_transform(self):
        """Find the best (most recent) transform among all available tag pairs"""
        max_age_ns = int(self.TRANSFORM_TIMEOUT * 1e9)
        
        best_transform = None
        best_tag_info = None
        best_age_ns = float('inf')
        
        # Check all configured tag pairs
        for tag_name, tag_info in self.tag_to_base_transforms.items():
            detection_frame = tag_info['detection_frame']
            
            transform, age_ns = self.get_detected_transform(detection_frame)
            
            if transform is not None and age_ns <= max_age_ns:
                # This transform is valid and recent enough
                if age_ns < best_age_ns:
                    best_transform = transform
                    best_tag_info = tag_info
                    best_age_ns = age_ns
        
        return best_transform, best_tag_info, best_age_ns
    
    def update_transform(self):
        """Main update loop to compute and publish camera->base transform"""
        
        # Find the best available transform
        detected_transform, tag_info, age_ns = self.find_best_transform()
        
        if detected_transform is None or tag_info is None:
            # No valid transforms available
            self.get_logger().debug("No valid tag transforms available")
            return
        
        # Get the static transform for this tag
        static_transform = tag_info['transform']
        tag_name = next(name for name, info in self.tag_to_base_transforms.items() 
                       if info == tag_info)
        
        # Compose transforms: camera -> tag -> base
        camera_to_base = self.compose_transforms(detected_transform, static_transform)
        
        # Set header and publish
        camera_to_base.header.stamp = self.get_clock().now().to_msg()
        camera_to_base.header.frame_id = self.CAMERA_FRAME
        camera_to_base.child_frame_id = self.ROBOT_BASE_FRAME
        
        self.tf_broadcaster.sendTransform(camera_to_base)

def main(args=None):
    rclpy.init(args=args)
    
    # Use the basic version
    node = RobotDetectionNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()