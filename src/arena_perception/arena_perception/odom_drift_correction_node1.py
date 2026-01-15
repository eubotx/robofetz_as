#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

class OdometryCorrectionNode(Node):
    def __init__(self):
        super().__init__('odometry_correction_node')
        
        # Create transform broadcaster for map->odom
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create buffer and listener for TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Track previous odom->base_footprint for delta calculation
        self.prev_odom_to_base = None
        
        # Timer for continuous correction
        self.timer = self.create_timer(1/60, self.publish_correction)  # 60 Hz
        
        self.get_logger().info("Odometry Correction Node started")
        
    def get_transform_matrix(self, transform):
        """Convert TransformStamped to 4x4 homogeneous transformation matrix"""
        # Extract translation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        
        # Extract quaternion
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
        # Normalize quaternion
        norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        
        # Convert quaternion to rotation matrix
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
        ])
        
        # Create homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        
        return T
    
    def matrix_to_transform(self, T, frame_id, child_frame_id):
        """Convert 4x4 homogeneous matrix to TransformStamped"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id
        
        # Extract translation
        transform.transform.translation.x = T[0, 3]
        transform.transform.translation.y = T[1, 3]
        transform.transform.translation.z = T[2, 3]
        
        # Extract rotation matrix
        R = T[:3, :3]
        
        # Convert rotation matrix to quaternion
        # Using the method from: https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        
        # Normalize quaternion
        norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        transform.transform.rotation.w = qw / norm
        transform.transform.rotation.x = qx / norm
        transform.transform.rotation.y = qy / norm
        transform.transform.rotation.z = qz / norm
        
        return transform
    
    def invert_transform_matrix(self, T):
        """Invert a 4x4 homogeneous transformation matrix"""
        R = T[:3, :3]
        t = T[:3, 3]
        
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T  # Transpose of rotation matrix
        T_inv[:3, 3] = -R.T @ t  # Negative rotated translation
        
        return T_inv
    
    def publish_correction(self):
        try:
            # Get all required transforms
            # 1. world -> map (usually static)
            world_to_map = self.tf_buffer.lookup_transform(
                'world',
                'map',
                rclpy.time.Time()
            )
            
            # 2. world -> arena_perception/robot/base_footprint (ground truth)
            world_to_arena_base = self.tf_buffer.lookup_transform(
                'world',
                'arena_perception/robot/base_footprint',
                rclpy.time.Time()
            )
            
            # 3. odom -> robot/base_footprint (odometry)
            odom_to_base = self.tf_buffer.lookup_transform(
                'odom',
                'robot/base_footprint',
                rclpy.time.Time()
            )
            
            # Convert all transforms to matrices for easier computation
            T_world_map = self.get_transform_matrix(world_to_map)
            T_world_arena_base = self.get_transform_matrix(world_to_arena_base)
            T_odom_base = self.get_transform_matrix(odom_to_base)
            
            # We want: T_map_odom such that:
            # T_world_map * T_map_odom * T_odom_base = T_world_arena_base
            
            # Solve for T_map_odom:
            # T_map_odom = T_world_map^{-1} * T_world_arena_base * T_odom_base^{-1}
            
            T_world_map_inv = self.invert_transform_matrix(T_world_map)
            T_odom_base_inv = self.invert_transform_matrix(T_odom_base)
            
            T_map_odom = T_world_map_inv @ T_world_arena_base @ T_odom_base_inv
            
            # Convert back to TransformStamped and publish
            map_to_odom_tf = self.matrix_to_transform(
                T_map_odom, 'map', 'odom'
            )
            
            self.tf_broadcaster.sendTransform(map_to_odom_tf)
            
            # Log for debugging (optional, can be removed)
            self.get_logger().debug(
                f"Published map->odom: "
                f"x={map_to_odom_tf.transform.translation.x:.3f}, "
                f"y={map_to_odom_tf.transform.translation.y:.3f}"
            )
            
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f"Waiting for transforms: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in correction: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryCorrectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()