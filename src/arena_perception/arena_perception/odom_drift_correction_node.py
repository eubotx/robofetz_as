#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import tf_transformations

class OdometryCorrectionNode(Node):
    def __init__(self):
        super().__init__('odometry_correction_node')
        
        # Declare parameters with default values 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('world_frame', 'world'),
                ('map_frame', 'map'),
                ('odom_frame', 'odom'),
                ('base_frame', 'robot/base_footprint'),
                ('detection_base_frame', 'arena_perception/robot/base_footprint'),
                ('update_rate', 60.0),
                ('transform_timeout', 0.1),
                ('log_level', 'INFO')
            ]
        )
        
        # Get parameters
        self.world_frame = self.get_parameter('world_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.detection_base_frame = self.get_parameter('detection_base_frame').value
        self.update_rate = self.get_parameter('update_rate').value
        self.transform_timeout = self.get_parameter('transform_timeout').value
        
        # Initialize TF components
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create timer with parameterized rate
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.publish_correction)
        
        # Log parameter values
        self.get_logger().info("Odometry Correction Node started with parameters:")
        self.get_logger().info(f"  world_frame: {self.world_frame}")
        self.get_logger().info(f"  map_frame: {self.map_frame}")
        self.get_logger().info(f"  odom_frame: {self.odom_frame}")
        self.get_logger().info(f"  base_frame: {self.base_frame}")
        self.get_logger().info(f"  detection_base_frame: {self.detection_base_frame}")
        self.get_logger().info(f"  update_rate: {self.update_rate} Hz")
        self.get_logger().info(f"  transform_timeout: {self.transform_timeout} s")
    
    def compose_transforms(self, transform_a, transform_b):
        """Compose two transforms: result = A * B (apply B then A)"""
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
        
        result = TransformStamped()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = transform_a.header.frame_id
        result.child_frame_id = transform_b.child_frame_id
        
        result.transform.translation.x = t_result[0]
        result.transform.translation.y = t_result[1]
        result.transform.translation.z = t_result[2]
        result.transform.rotation.x = q_result[0]
        result.transform.rotation.y = q_result[1]
        result.transform.rotation.z = q_result[2]
        result.transform.rotation.w = q_result[3]
        
        return result
    
    def invert_transform(self, transform):
        """Invert a TransformStamped"""
        t = np.array([transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z])
        q = np.array([transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z,
                      transform.transform.rotation.w])
        
        # Invert quaternion
        q_inv = tf_transformations.quaternion_inverse(q)
        
        # Invert translation: t_inv = -q_inv * t * q
        rot_matrix_inv = tf_transformations.quaternion_matrix(q_inv)[:3, :3]
        t_inv = -rot_matrix_inv @ t
        
        inverted = TransformStamped()
        inverted.header.stamp = transform.header.stamp
        inverted.header.frame_id = transform.child_frame_id
        inverted.child_frame_id = transform.header.frame_id
        
        inverted.transform.translation.x = t_inv[0]
        inverted.transform.translation.y = t_inv[1]
        inverted.transform.translation.z = t_inv[2]
        inverted.transform.rotation.x = q_inv[0]
        inverted.transform.rotation.y = q_inv[1]
        inverted.transform.rotation.z = q_inv[2]
        inverted.transform.rotation.w = q_inv[3]
        
        return inverted
    
    def publish_correction(self):
        try:            
            # Get transforms using parameterized frame names

            world_to_map = self.tf_buffer.lookup_transform(
                self.world_frame, self.map_frame, rclpy.time.Time())
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
            # world_to_detection_base = self.tf_buffer.lookup_transform(
            #     self.world_frame, self.detection_base_frame, rclpy.time.Time())
            
            world_to_detection_base = self.get_newest_transform(
                self.world_frame, self.detection_base_frame, self.transform_timeout)          
  
            if world_to_map is None or world_to_detection_base is None or odom_to_base is None:
                self.get_logger().debug(
                    f"Could not publish {self.map_frame} -> {self.odom_frame} correction.",
                    throttle_duration_sec=2.0
                )
                return

            # Invert to get directions we need for composition
            map_to_world = self.invert_transform(world_to_map)
            base_to_odom = self.invert_transform(odom_to_base)
            
            # Compute: map->odom = map->world * world->detection_base * base->odom
            map_to_detection_base = self.compose_transforms(map_to_world, world_to_detection_base)
            map_to_odom = self.compose_transforms(map_to_detection_base, base_to_odom)
            
            # Set correct frames using parameters
            map_to_odom.header.stamp = self.get_clock().now().to_msg()
            map_to_odom.header.frame_id = self.map_frame
            map_to_odom.child_frame_id = self.odom_frame
            
            self.tf_broadcaster.sendTransform(map_to_odom)
            
            # Log success at debug level
            self.get_logger().debug(
                f"Published transform from {self.map_frame} to {self.odom_frame}",
                throttle_duration_sec=1.0
            )

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(
                f"Waiting for transforms: {e}", 
                throttle_duration_sec=2.0
            )
        except Exception as e:
            self.get_logger().error(f"Error in correction: {e}")

    def get_newest_transform(self, from_frame, to_frame, max_age_s):
        try:
            transform = self.tf_buffer.lookup_transform(
                from_frame,
                to_frame,
                rclpy.time.Time(),
            )
            
            # check if transform is recent
            current_time = self.get_clock().now()
            transform_time = rclpy.time.Time.from_msg(transform.header.stamp)
            age_ns = (current_time - transform_time).nanoseconds
            max_age_ns = int(max_age_s * 10**9)

            if  age_ns > max_age_ns:
                self.get_logger().info(f"Transform too old ({from_frame}->{to_frame}): {age_ns/1e9:.3f}s", throttle_duration_sec=5.0)
                return None 
            return transform
        
        except (LookupException, ConnectivityException, ExtrapolationException, tf2_ros.TransformException) as e:
            self.get_logger().error(f"TF lookup failed ({from_frame}->{to_frame}): {str(e)}")
            return None

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