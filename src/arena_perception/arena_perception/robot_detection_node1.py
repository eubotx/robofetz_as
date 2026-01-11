#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
import tf_transformations

class CameraToBaseTransform(Node):
    def __init__(self):
        super().__init__('camera_to_base_transform')
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF broadcaster for the computed transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store static transforms
        self.static_transforms = {}
        
        # Timer to continuously compute and publish the transform
        self.timer = self.create_timer(0.1, self.update_transform)  # 10 Hz
        
        self.get_logger().info("Camera to Base Transform Node started")
        
    def update_transform(self):
        try:
            # Try to get camera to tag transforms (one will be available)
            camera_to_top_tag = None
            camera_to_bottom_tag = None
            
            try:
                camera_to_top_tag = self.tf_buffer.lookup_transform(
                    'arena_camera_optical',
                    'arena_perception/robot_top_tag',
                    rclpy.time.Time()
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                pass
                
            try:
                camera_to_bottom_tag = self.tf_buffer.lookup_transform(
                    'arena_camera_optical',
                    'arena_perception/robot_bottom_tag',
                    rclpy.time.Time()
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                pass
            
            # Determine which tag is visible
            if camera_to_top_tag is not None:
                self.compute_and_publish_base_footprint(camera_to_top_tag, 'top')
            elif camera_to_bottom_tag is not None:
                self.compute_and_publish_base_footprint(camera_to_bottom_tag, 'bottom')
            else:
                self.get_logger().debug("No tag transforms available")
                
        except Exception as e:
            self.get_logger().debug(f"Error in update_transform: {e}")
    
    def compute_and_publish_base_footprint(self, camera_to_tag_transform, tag_type):
        try:
            # Get the corresponding static transform from robot state publisher
            if tag_type == 'top':
                base_to_tag_link = 'robot/top_apriltag_optical'
                tag_name = 'arena_perception/robot_top_tag'
            else:  # bottom
                base_to_tag_link = 'robot/bottom_apriltag_optical'
                tag_name = 'arena_perception/robot_bottom_tag'
            
            # Get the static transform (base_footprint -> apriltag_link)
            base_to_apriltag = self.tf_buffer.lookup_transform(
                'robot/base_footprint',
                base_to_tag_link,
                rclpy.time.Time()
            )
            
            # Transform chain: camera_optical -> tag -> base_footprint
            # We need to invert base_to_apriltag to get apriltag_link -> base_footprint
            
            # Method 1: Using tf2 transformations
            # Get the transform from apriltag_link to base_footprint (inverse of base_to_apriltag)
            apriltag_to_base = self.invert_transform(base_to_apriltag)
            
            # Now compose: camera_optical -> tag -> base_footprint
            # Since tag and apriltag_link are the same physical tag in different frames,
            # we assume identity transform between them
            
            # Compose the transforms: camera_to_tag * tag_to_base
            # Here we use the fact that tag (perception frame) corresponds to apriltag_link (robot frame)
            camera_to_base = self.compose_transforms(
                camera_to_tag_transform,
                apriltag_to_base
            )
            
            # Publish the computed transform
            camera_to_base.header.stamp = self.get_clock().now().to_msg()
            camera_to_base.header.frame_id = 'arena_camera_optical'
            camera_to_base.child_frame_id = 'arena_perception/robot/base_footprint'
            
            self.tf_broadcaster.sendTransform(camera_to_base)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f"Could not get transform for {tag_type}: {e}")
        except Exception as e:
            self.get_logger().error(f"Error computing transform: {e}")
    
    def invert_transform(self, transform):
        """Invert a transform (returns B->A given A->B)"""
        inverse = TransformStamped()
        inverse.header.stamp = transform.header.stamp
        inverse.header.frame_id = transform.child_frame_id
        inverse.child_frame_id = transform.header.frame_id
        
        # Extract transform components
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # Convert to transformation matrix
        mat = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z
        
        # Invert the matrix
        mat_inv = tf_transformations.inverse_matrix(mat)
        
        # Extract translation and rotation from inverted matrix
        inverse.transform.translation.x = mat_inv[0, 3]
        inverse.transform.translation.y = mat_inv[1, 3]
        inverse.transform.translation.z = mat_inv[2, 3]
        
        quat = tf_transformations.quaternion_from_matrix(mat_inv)
        inverse.transform.rotation.x = quat[0]
        inverse.transform.rotation.y = quat[1]
        inverse.transform.rotation.z = quat[2]
        inverse.transform.rotation.w = quat[3]
        
        return inverse
    
    def compose_transforms(self, transform_a, transform_b):
        """Compose two transforms: C = A * B (apply B then A)"""
        result = TransformStamped()
        result.header.stamp = transform_a.header.stamp
        result.header.frame_id = transform_a.header.frame_id
        result.child_frame_id = transform_b.child_frame_id
        
        # Convert both transforms to matrices
        t1 = transform_a.transform.translation
        q1 = transform_a.transform.rotation
        mat1 = tf_transformations.quaternion_matrix([q1.x, q1.y, q1.z, q1.w])
        mat1[0, 3] = t1.x
        mat1[1, 3] = t1.y
        mat1[2, 3] = t1.z
        
        t2 = transform_b.transform.translation
        q2 = transform_b.transform.rotation
        mat2 = tf_transformations.quaternion_matrix([q2.x, q2.y, q2.z, q2.w])
        mat2[0, 3] = t2.x
        mat2[1, 3] = t2.y
        mat2[2, 3] = t2.z
        
        # Compose matrices: result = mat1 * mat2
        mat_result = tf_transformations.concatenate_matrices(mat1, mat2)
        
        # Extract translation and rotation
        result.transform.translation.x = mat_result[0, 3]
        result.transform.translation.y = mat_result[1, 3]
        result.transform.translation.z = mat_result[2, 3]
        
        quat_result = tf_transformations.quaternion_from_matrix(mat_result)
        result.transform.rotation.x = quat_result[0]
        result.transform.rotation.y = quat_result[1]
        result.transform.rotation.z = quat_result[2]
        result.transform.rotation.w = quat_result[3]
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CameraToBaseTransform()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()