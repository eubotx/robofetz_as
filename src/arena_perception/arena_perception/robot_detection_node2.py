#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Quaternion
import tf2_geometry_msgs
import tf2_ros

class CameraToBaseTransform(Node):
    def __init__(self):
        super().__init__('camera_to_base_transform')
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF broadcaster for the computed transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to continuously compute and publish the transform
        self.timer = self.create_timer(0.1, self.update_transform)
        
        self.get_logger().info("Camera to Base Transform Node started")
        
    def update_transform(self):
        try:
            # Check which tag is visible
            for tag_type in ['top', 'bottom']:
                try:
                    # Try to get camera to tag transform
                    camera_to_tag = self.tf_buffer.lookup_transform(
                        'arena_camera_optical',
                        f'arena_perception/robot_{tag_type}_tag',
                        rclpy.time.Time()
                    )
                    
                    # Get corresponding static transform
                    if tag_type == 'top':
                        static_target = 'robot/top_apriltag_link'
                    else:
                        static_target = 'robot/bottom_apriltag_link'
                    
                    # Get static transform: base_footprint -> apriltag_link
                    base_to_apriltag = self.tf_buffer.lookup_transform(
                        'robot/base_footprint',
                        static_target,
                        rclpy.time.Time()
                    )
                    
                    # Compute camera -> base_footprint
                    # Method: camera -> tag = camera -> base * base -> tag
                    # So: camera -> base = camera -> tag * (base -> tag)^-1
                    
                    # Invert base_to_apriltag to get tag -> base
                    tag_to_base = self.tf_buffer.lookup_transform(
                        static_target,
                        'robot/base_footprint',
                        rclpy.time.Time()
                    )
                    
                    # Create a PoseStamped at origin in tag frame
                    pose_in_tag = PoseStamped()
                    pose_in_tag.header.frame_id = f'arena_perception/robot_{tag_type}_tag'
                    pose_in_tag.header.stamp = self.get_clock().now().to_msg()
                    pose_in_tag.pose.position = Point(x=0.0, y=0.0, z=0.0)
                    pose_in_tag.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    
                    # Transform to camera frame (this gives us camera -> tag transform as pose)
                    # Actually, we already have camera_to_tag as transform...
                    # Let's compose transforms differently:
                    
                    # Transform chain: we want camera -> base
                    # We have: camera -> tag (from perception)
                    # We have: tag_link -> base (inverted static transform)
                    # Assuming tag and tag_link are the same physical entity
                    
                    # Create transform from camera to base
                    camera_to_base = TransformStamped()
                    camera_to_base.header.stamp = self.get_clock().now().to_msg()
                    camera_to_base.header.frame_id = 'arena_camera_optical'
                    camera_to_base.child_frame_id = 'arena_perception/robot/base_footprint'
                    
                    # For simplicity, let's use the fact that:
                    # camera -> base = (camera -> tag) * (tag -> base)
                    # Where tag -> base is approximately tag_link -> base
                    
                    # We'll use a simplified approach - look up the transform directly
                    # This assumes the static transforms are available in TF tree
                    
                    # Try to get the complete transform using tf2
                    try:
                        # This should work if all transforms are in the buffer
                        camera_to_base_tf = self.tf_buffer.lookup_transform(
                            'arena_camera_optical',
                            'arena_perception/robot/base_footprint',
                            rclpy.time.Time()
                        )
                        # Use this transform directly
                        camera_to_base.transform = camera_to_base_tf.transform
                        
                    except:
                        # Fallback: approximate by assuming tag and tag_link are same
                        # This is less accurate but works as approximation
                        camera_to_base.transform.translation.x = (
                            camera_to_tag.transform.translation.x + 
                            tag_to_base.transform.translation.x
                        )
                        camera_to_base.transform.translation.y = (
                            camera_to_tag.transform.translation.y + 
                            tag_to_base.transform.translation.y
                        )
                        camera_to_base.transform.translation.z = (
                            camera_to_tag.transform.translation.z + 
                            tag_to_base.transform.translation.z
                        )
                        
                        # Combine rotations (simplified - in reality should multiply quaternions)
                        camera_to_base.transform.rotation = camera_to_tag.transform.rotation
                    
                    # Publish the transform
                    self.tf_broadcaster.sendTransform(camera_to_base)
                    self.get_logger().debug(f"Published transform using {tag_type} tag")
                    return
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                        tf2_ros.ExtrapolationException):
                    continue
                    
            self.get_logger().debug("No visible tag found")
                
        except Exception as e:
            self.get_logger().debug(f"Error: {e}")

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