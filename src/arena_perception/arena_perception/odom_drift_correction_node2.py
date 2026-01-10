#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformException
import tf_transformations as tf_trans

class SimpleOdometryCorrection(Node):
    def __init__(self):
        super().__init__('simple_odometry_correction')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for correction
        self.timer = self.create_timer(0.1, self.correct_odometry)  # 10 Hz
        
        self.get_logger().info("Odometry Correction Node started")
        
    def correct_odometry(self):
        try:
            # Get current time for all lookups
            now = rclpy.time.Time()
            
            # Get world -> map transform
            world_to_map = self.tf_buffer.lookup_transform(
                'world', 'map', now
            )
            
            # Get world -> arena_base (ground truth)
            world_to_arena_base = self.tf_buffer.lookup_transform(
                'world', 'arena_perception/robot_base_footprint', now
            )
            
            # Get odom -> base_footprint (robot odometry)
            odom_to_base = self.tf_buffer.lookup_transform(
                'robot/odom', 'robot/base_footprint', now
            )
            
            # We need to compute: map -> odom
            # chain: world -> map -> odom -> base = world -> arena_base
            # So: map -> odom = (world -> map)^{-1} * (world -> arena_base) * (odom -> base)^{-1}
            
            # Method using tf_transformations
            # Convert all to transformation matrices
            w_to_m = self.transform_to_matrix(world_to_map.transform)
            w_to_a = self.transform_to_matrix(world_to_arena_base.transform)
            o_to_b = self.transform_to_matrix(odom_to_base.transform)
            
            # Compute inverses
            m_to_w = self.inverse_matrix(w_to_m)
            b_to_o = self.inverse_matrix(o_to_b)
            
            # Compute map->odom: m_to_w * w_to_a * b_to_o
            m_to_o = np.matmul(m_to_w, np.matmul(w_to_a, b_to_o))
            
            # Convert back to transform
            map_to_odom_tf = TransformStamped()
            map_to_odom_tf.header.stamp = self.get_clock().now().to_msg()
            map_to_odom_tf.header.frame_id = 'map'
            map_to_odom_tf.child_frame_id = 'robot/odom'
            map_to_odom_tf.transform = self.matrix_to_transform(m_to_o)
            
            # Publish the correction
            self.tf_broadcaster.sendTransform(map_to_odom_tf)
            
            self.get_logger().debug("Published odometry correction")
            
        except TransformException as e:
            self.get_logger().debug(f"Transform error: {e}")
    
    def transform_to_matrix(self, transform):
        """Convert geometry_msgs/Transform to 4x4 matrix"""
        trans = [transform.translation.x,
                 transform.translation.y,
                 transform.translation.z]
        rot = [transform.rotation.x,
               transform.rotation.y,
               transform.rotation.z,
               transform.rotation.w]
        
        return tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(trans),
            tf_trans.quaternion_matrix(rot)
        )
    
    def inverse_matrix(self, matrix):
        """Invert a 4x4 transformation matrix"""
        # For homogeneous transformation matrix:
        # [R t]^-1 = [R^T -R^T*t]
        # [0 1]      [0    1    ]
        R = matrix[:3, :3]
        t = matrix[:3, 3]
        
        inv = np.eye(4)
        inv[:3, :3] = R.T
        inv[:3, 3] = -R.T @ t
        
        return inv
    
    def matrix_to_transform(self, matrix):
        """Convert 4x4 matrix to geometry_msgs/Transform"""
        from geometry_msgs.msg import Transform
        
        transform = Transform()
        
        # Extract translation
        transform.translation.x = matrix[0, 3]
        transform.translation.y = matrix[1, 3]
        transform.translation.z = matrix[2, 3]
        
        # Extract rotation as quaternion
        # Get 3x3 rotation matrix
        R = matrix[:3, :3]
        # Ensure it's a proper rotation matrix
        # by normalizing (SVD could be better but this is simpler)
        q = tf_trans.quaternion_from_matrix(matrix)
        
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        
        return transform

def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometryCorrection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()