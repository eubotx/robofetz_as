import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf2_ros
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster, Buffer, TransformListener
import tf_transformations
import yaml
import os
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from scipy.spatial.transform import Rotation
import statistics
from ament_index_python.packages import get_package_share_directory

class FindCameraInWorldService(Node):
    def __init__(self):
        super().__init__('find_camera_in_world_service')
        
        # Declare all parameters with descriptions and default values
        self.declare_parameters(
            namespace='',
            parameters=[
                # Required parameter for calibration file
                ('calibration_file', '', 
                 ParameterDescriptor(
                     description='Optional path to YAML file for storing calibration results. Can use package:// URLs.',
                     type=ParameterType.PARAMETER_STRING)),
                
                # Frame names
                ('world_frame', 'world'),
                ('calibration_marker_frame', 'arena_marker'),
                ('camera_frame', 'arena_camera_optical'),
                
                # Marker position in world
                ('marker_position.x', 0.0),
                ('marker_position.y', 0.0),
                ('marker_position.z', 0.0),
                
                # Marker orientation in world (radians)
                ('marker_orientation.roll', 0.0),
                ('marker_orientation.pitch', 0.0),
                ('marker_orientation.yaw', 0.0),
                
                # Rates
                ('calibration_attempt_rate', 1.0,
                 ParameterDescriptor(
                     description='Rate (Hz) for attempting calibration',
                     type=ParameterType.PARAMETER_DOUBLE)),
                ('publish_rate', 60.0,
                 ParameterDescriptor(
                     description='Rate (Hz) for publishing camera transform dynamically',
                     type=ParameterType.PARAMETER_DOUBLE)),
                
                # Transform timeout
                ('transform_timeout', 0.1,
                 ParameterDescriptor(
                     description='Timeout (seconds) for transform lookups',
                     type=ParameterType.PARAMETER_DOUBLE)),
                
                # NEW: Simple median filter configuration
                ('median_filter_window', 1,
                 ParameterDescriptor(
                     description='Window size for median filtering (1 = no filtering)',
                     type=ParameterType.PARAMETER_INTEGER))
            ]
        )
        
        calibration_file = self.get_parameter('calibration_file').value
        
        # Resolve package:// URL if present
        calibration_file = self.resolve_package_url(calibration_file)
        
        # Frame names from parameters
        self.world_frame = self.get_parameter('world_frame').value
        self.calibration_marker_frame = self.get_parameter('calibration_marker_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # Add "_calibrated" suffix to marker frame for the static transform
        self.calibration_marker_calibrated_frame = f"{self.calibration_marker_frame}_calibrated"
        
        # Marker position in world coordinate system from parameters
        self.marker_position_in_world = np.array([
            self.get_parameter('marker_position.x').value,
            self.get_parameter('marker_position.y').value,
            self.get_parameter('marker_position.z').value
        ])
        
        # Marker orientation in world coordinate system from parameters
        self.marker_orientation_in_world = np.array([
            self.get_parameter('marker_orientation.roll').value,
            self.get_parameter('marker_orientation.pitch').value,
            self.get_parameter('marker_orientation.yaw').value
        ])
        
        # Initialize median filter
        self.median_window = self.get_parameter('median_filter_window').value
        self.calibration_buffer = []  # Simple list instead of deque
        
        # Service for manual calibration
        self.srv = self.create_service(Trigger, 'find_camera_in_world', self.calibrate_callback)
        
        # TF broadcasters
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)  # For marker (static)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)        # For camera (dynamic)
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Transform storage
        self.world_to_marker_transform = None      # world -> marker_calibrated (static, from params)
        self.world_to_camera_transform = None      # world -> camera (dynamic, computed)
        self.previous_world_to_camera_transform = None  # For movement tracking
        
        # Calibration state
        self.calibration_successful = False
        self.attempt_count = 0
        
        # Get rates from parameters
        self.calibration_rate = self.get_parameter('calibration_attempt_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.transform_timeout = self.get_parameter('transform_timeout').value

        # Log configuration
        self.get_logger().info("Configuration loaded:")
        self.get_logger().info(f"  World frame: {self.world_frame}")
        self.get_logger().info(f"  Calibration marker frame (detected): {self.calibration_marker_frame}")
        self.get_logger().info(f"  Calibration marker frame (calibrated): {self.calibration_marker_calibrated_frame}")
        self.get_logger().info(f"  Camera frame: {self.camera_frame}")
        self.get_logger().info(f"  Median filter window: {self.median_window} ({'disabled' if self.median_window <= 1 else 'enabled'})")
        self.get_logger().info(f"  Marker position in world: [{self.marker_position_in_world[0]:.3f}, "
                              f"{self.marker_position_in_world[1]:.3f}, {self.marker_position_in_world[2]:.3f}] m")
        self.get_logger().info(f"  Marker orientation (roll,pitch,yaw): [{self.marker_orientation_in_world[0]:.3f}, "
                              f"{self.marker_orientation_in_world[1]:.3f}, {self.marker_orientation_in_world[2]:.3f}] rad")
        self.get_logger().info(f"  Calibration attempt rate: {self.get_parameter('calibration_attempt_rate').value} Hz")
        self.get_logger().info(f"  Dynamic publish rate: {self.get_parameter('publish_rate').value} Hz")
        self.get_logger().info(f"  Transform timeout: {self.transform_timeout} s")
        self.get_logger().info(f"  Calibration file path: {calibration_file}")
        
        # Publish static marker transform
        self.publish_static_marker_transform()

        # Try to load existing calibration
        if not self.load_calibration(calibration_file):
            # Start continuous calibration attempts since no calibration via file was provided
            self.calibration_timer = self.create_timer(1.0 / self.calibration_rate, self.attempt_calibration)
            self.get_logger().info("Started continuous calibration attempts...")
        
        # Start continuous dynamic publishing of camera transform
        self.dynamic_publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_camera_dynamic)

    def resolve_package_url(self, path):
        """Resolve package:// URLs to absolute file paths"""
        if not path:
            return path
            
        if path.startswith('package://'):
            try:
                # Extract package name and relative path
                # Format: package://package_name/relative/path/to/file
                path_without_prefix = path[10:]  # Remove 'package://'
                
                # Split into package name and relative path
                if '/' in path_without_prefix:
                    package_name, relative_path = path_without_prefix.split('/', 1)
                else:
                    package_name = path_without_prefix
                    relative_path = ""

                package_dir = get_package_share_directory(package_name)
                
                # Construct full path
                full_path = os.path.join(package_dir, relative_path)
                
                self.get_logger().info(f"Resolved package URL: {path} -> {full_path}")
                return full_path
                
            except Exception as e:
                self.get_logger().error(f"Failed to resolve package URL '{path}': {e}")
                return path
        else:
            # Not a package URL, return as-is
            return path
        
    # Simple median filter
    def _apply_median_filter(self):
        """Apply median filter when buffer is full"""
        if len(self.calibration_buffer) < self.median_window:
            # Buffer not full yet, return latest
            return self.calibration_buffer[-1] if self.calibration_buffer else None
        
        # Buffer is full, apply median filter
        x_vals = [t.transform.translation.x for t in self.calibration_buffer]
        y_vals = [t.transform.translation.y for t in self.calibration_buffer]
        z_vals = [t.transform.translation.z for t in self.calibration_buffer]
        
        # Apply median to positions
        median_x = statistics.median(x_vals)
        median_y = statistics.median(y_vals)
        median_z = statistics.median(z_vals)
        
        # For orientation, average the quaternions
        quats = []
        for t in self.calibration_buffer:
            q = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ]
            quats.append(q)
        
        # Average quaternions
        quat_array = np.array(quats)
        avg_quat = np.mean(quat_array, axis=0)
        avg_quat = avg_quat / np.linalg.norm(avg_quat)
        
        # Create filtered transform
        filtered = TransformStamped()
        filtered.header.stamp = self.get_clock().now().to_msg()
        filtered.header.frame_id = self.world_frame
        filtered.child_frame_id = self.camera_frame
        
        filtered.transform.translation.x = float(median_x)
        filtered.transform.translation.y = float(median_y)
        filtered.transform.translation.z = float(median_z)
        
        filtered.transform.rotation.x = float(avg_quat[0])
        filtered.transform.rotation.y = float(avg_quat[1])
        filtered.transform.rotation.z = float(avg_quat[2])
        filtered.transform.rotation.w = float(avg_quat[3])
        
        return filtered
        
    def load_calibration(self, calibration_file):
        """Try to load existing camera calibration of world -> camera tf from file"""

        if not calibration_file:
            self.get_logger().info("No calibration file specified")
            return False
            
        if not os.path.exists(calibration_file):
            self.get_logger().info(f"No calibration file found at {calibration_file}")
            return False
        
        try:
            self.get_logger().info(f"Using calibration file: {calibration_file}")
            with open(calibration_file, 'r') as f:
                calibration_data = yaml.safe_load(f)
            
            # Check if calibration data is valid
            if 'camera_in_world' not in calibration_data:
                self.get_logger().warning(f"Invalid calibration file format: {calibration_file}")
                return False
            
            camera_data = calibration_data['camera_in_world']
            
            # Create transform from loaded data
            world_to_camera_transform = TransformStamped()
            world_to_camera_transform.header.stamp = self.get_clock().now().to_msg()
            world_to_camera_transform.header.frame_id = self.world_frame
            world_to_camera_transform.child_frame_id = self.camera_frame
            
            # Set translation
            world_to_camera_transform.transform.translation.x = float(camera_data['position']['x'])
            world_to_camera_transform.transform.translation.y = float(camera_data['position']['y'])
            world_to_camera_transform.transform.translation.z = float(camera_data['position']['z'])
            
            # Load from Euler angles (radians)
            euler_data = camera_data['orientation']
            roll = float(euler_data.get('roll', 0.0))
            pitch = float(euler_data.get('pitch', 0.0))
            yaw = float(euler_data.get('yaw', 0.0))
            
            # Convert Euler to quaternion
            rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
            quat = rotation.as_quat()
            
            world_to_camera_transform.transform.rotation.x = float(quat[0])
            world_to_camera_transform.transform.rotation.y = float(quat[1])
            world_to_camera_transform.transform.rotation.z = float(quat[2])
            world_to_camera_transform.transform.rotation.w = float(quat[3])
            
            # Update stored transform
            self.world_to_camera_transform = world_to_camera_transform
            self.calibration_successful = True

            self.log_calibration_details(
                marker_to_camera_transform=None,
                world_to_camera_transform=self.world_to_camera_transform,
                calibration_source="Calibration file",
                previous_world_to_camera_transform=self.previous_world_to_camera_transform
            )

            # Store for movement comparison in next calibration
            self.previous_world_to_camera_transform = self.world_to_camera_transform
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading calibration file: {e}")
            return False
        
    def create_world_to_marker_transform(self):
        """Create the transform from world to marker_calibrated"""
        world_to_marker_transform = TransformStamped()
        world_to_marker_transform.header.stamp = self.get_clock().now().to_msg()
        world_to_marker_transform.header.frame_id = self.world_frame
        world_to_marker_transform.child_frame_id = self.calibration_marker_calibrated_frame
        
        # Set translation based on parameters
        world_to_marker_transform.transform.translation.x = float(self.marker_position_in_world[0])
        world_to_marker_transform.transform.translation.y = float(self.marker_position_in_world[1])
        world_to_marker_transform.transform.translation.z = float(self.marker_position_in_world[2])
        
        # Convert Euler angles from parameters to quaternion
        rotation = Rotation.from_euler('xyz', self.marker_orientation_in_world)
        quat = rotation.as_quat()
        world_to_marker_transform.transform.rotation.x = float(quat[0])
        world_to_marker_transform.transform.rotation.y = float(quat[1])
        world_to_marker_transform.transform.rotation.z = float(quat[2])
        world_to_marker_transform.transform.rotation.w = float(quat[3])
        
        return world_to_marker_transform
        
    def publish_static_marker_transform(self):
        """Publish STATIC transform from world to marker_calibrated based on parameters"""
        self.world_to_marker_transform = self.create_world_to_marker_transform()
        
        # Publish just the marker transform as static
        self.static_tf_broadcaster.sendTransform([self.world_to_marker_transform])
        
        self.get_logger().info(f"Published STATIC transform: {self.world_frame} -> {self.calibration_marker_calibrated_frame}")
        
    def publish_camera_dynamic(self):
        """Publish camera transform dynamically"""
        if self.world_to_camera_transform is not None:
            # Update timestamp for dynamic publishing
            self.world_to_camera_transform.header.stamp = self.get_clock().now().to_msg()
            self.dynamic_tf_broadcaster.sendTransform(self.world_to_camera_transform)
            
    def attempt_calibration(self):
        """Attempt calibration - runs continuously until successful"""
        self.attempt_count += 1
        
        # If already calibrated, just return (we keep running for manual re-calibration via service)
        if self.calibration_successful:
            return
            
        calibration_source = f"initial (attempt {self.attempt_count})"
        success = self.perform_calibration(calibration_source)
        
        if not success and self.attempt_count % 10 == 0:
            self.get_logger().debug(f"Still waiting for calibration after {self.attempt_count} attempts")
            
    def perform_calibration(self, calibration_source="manual"):
        """Shared calibration logic - used by both timer and service"""
        try:
            marker_to_camera_transform = self.get_newest_transform(self.calibration_marker_frame, self.camera_frame, self.transform_timeout)
            
            if marker_to_camera_transform is not None:

                # Compute world-to-camera transform
                # Ensure we have the world-to-marker transform
                if self.world_to_marker_transform is None:
                    self.world_to_marker_transform = self.create_world_to_marker_transform()
                
                # Use composition to compute: world_to_camera = world_to_marker * marker_to_camera
                world_to_camera_transform = self.compose_transforms(
                    self.world_to_marker_transform,    # A: world → marker_calibrated
                    marker_to_camera_transform          # B: marker → camera
                )
                
                # Update frame IDs to match our specific frames
                world_to_camera_transform.header.frame_id = self.world_frame
                world_to_camera_transform.child_frame_id = self.camera_frame
                
                # Store in buffer
                self.calibration_buffer.append(world_to_camera_transform)
                
                # Only calibrate when buffer is full
                if len(self.calibration_buffer) < self.median_window:
                    self.get_logger().debug(f"Collecting samples: {len(self.calibration_buffer)}/{self.median_window}")
                    return False
                
                # Apply median filter
                filtered_transform = self._apply_median_filter()
                
                if filtered_transform:
                    world_to_camera_transform = filtered_transform
                    self.get_logger().info(f"Applied median filter over {self.median_window} samples")
                
                # Update stored transform for dynamic publishing
                self.world_to_camera_transform = world_to_camera_transform
                
                # Update calibration state
                self.calibration_successful = True
                
                # Log calibration details with movement info
                self.log_calibration_details(
                    marker_to_camera_transform=marker_to_camera_transform,
                    world_to_camera_transform=world_to_camera_transform,
                    calibration_source=calibration_source,
                    previous_world_to_camera_transform=self.previous_world_to_camera_transform
                )
                
                # Store for movement comparison in next calibration
                self.previous_world_to_camera_transform = world_to_camera_transform
                
                # Clear buffer after successful calibration for next service trigger
                self.calibration_buffer.clear()
                
                # If this was an initial calibration, stop the timer
                if "initial" in calibration_source and hasattr(self, 'calibration_timer'):
                    self.calibration_timer.cancel()
                    self.get_logger().info("Stopped continuous calibration attempts (calibration successful)")
                
                return True
            
            return False

        except Exception as e:
            self.get_logger().error(f"Unexpected error in calibration: {e}")
            return False
    
    def compose_transforms(self, transform_a, transform_b):
        """Compose two transforms: result = A * B (apply B then A)"""
        # Extract translation and rotation from first transform
        t1 = np.array([transform_a.transform.translation.x,
                       transform_a.transform.translation.y,
                       transform_a.transform.translation.z])
        q1 = np.array([transform_a.transform.rotation.x,
                       transform_a.transform.rotation.y,
                       transform_a.transform.rotation.z,
                       transform_a.transform.rotation.w])
        
        # Extract translation and rotation from second transform
        t2 = np.array([transform_b.transform.translation.x,
                       transform_b.transform.translation.y,
                       transform_b.transform.translation.z])
        q2 = np.array([transform_b.transform.rotation.x,
                       transform_b.transform.rotation.y,
                       transform_b.transform.rotation.z,
                       transform_b.transform.rotation.w])
        
        # Compose rotations: q_result = q1 * q2
        q_result = tf_transformations.quaternion_multiply(q1, q2)
        
        # Rotate t2 by q1 and add to t1: t_result = t1 + R(q1) * t2
        rot_matrix = tf_transformations.quaternion_matrix(q1)[:3, :3]
        t_rotated = rot_matrix @ t2
        t_result = t1 + t_rotated
        
        # Create result transform
        result = TransformStamped()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = transform_a.header.frame_id
        result.child_frame_id = transform_b.child_frame_id
        
        # Set translation
        result.transform.translation.x = t_result[0]
        result.transform.translation.y = t_result[1]
        result.transform.translation.z = t_result[2]
        
        # Set rotation
        result.transform.rotation.x = q_result[0]
        result.transform.rotation.y = q_result[1]
        result.transform.rotation.z = q_result[2]
        result.transform.rotation.w = q_result[3]
        
        return result
    
    def get_newest_transform(self, from_frame, to_frame, max_age_s):
        try:
            # Try to get the transform with timeout
            transform = self.tf_buffer.lookup_transform(
                from_frame,
                to_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )

            return transform
            
        except tf2_ros.TransformException as e:
            # More specific exception handling if needed
            if "timeout" in str(e).lower():
                self.get_logger().warn(f"Transform timeout ({from_frame}->{to_frame}): No transform within timeout")
            else:
                self.get_logger().error(f"TF lookup failed ({from_frame}->{to_frame}): {str(e)}")
            return None
    
    def log_calibration_details(self, marker_to_camera_transform, world_to_camera_transform, 
                               calibration_source, previous_world_to_camera_transform):
        """Log detailed information about the calibration"""

        # Extract translation components
        world_to_camera_translation = world_to_camera_transform.transform.translation
        world_to_camera_rotation = world_to_camera_transform.transform.rotation
        
        # Convert to Euler angles (degrees)
        rotation = Rotation.from_quat([
            world_to_camera_rotation.x,
            world_to_camera_rotation.y,
            world_to_camera_rotation.z,
            world_to_camera_rotation.w
        ])
        euler_angles = rotation.as_euler('xyz')
        
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"CALIBRATION SUCCESSFUL! ({calibration_source})")
        self.get_logger().info("=" * 50)
        
        if marker_to_camera_transform is not None:
            marker_to_camera_translation = marker_to_camera_transform.transform.translation
            # Calculate distances
            marker_distance = np.sqrt(
                marker_to_camera_translation.x**2 + 
                marker_to_camera_translation.y**2 + 
                marker_to_camera_translation.z**2
            )
            self.get_logger().info(f"Marker to camera distance: {marker_distance:.3f} m")

        self.get_logger().info(f"Camera position in world: [{world_to_camera_translation.x:.5f}, "
                              f"{world_to_camera_translation.y:.5f}, {world_to_camera_translation.z:.5f}] m")
        
        # Single line for rotation in Euler angles
        self.get_logger().info(f"Camera rotation (roll,pitch,yaw) [rad]: [{euler_angles[0]:.4f}, "
                              f"{euler_angles[1]:.4f}, {euler_angles[2]:.4f}]")
        
        self.get_logger().info(f"Published transform: {self.world_frame} -> {self.camera_frame}")
        
        # Log movement if we have a previous transform
        if previous_world_to_camera_transform is not None:
            previous_translation = previous_world_to_camera_transform.transform.translation
            previous_rotation = previous_world_to_camera_transform.transform.rotation
            
            # Position difference
            dx = world_to_camera_translation.x - previous_translation.x
            dy = world_to_camera_translation.y - previous_translation.y
            dz = world_to_camera_translation.z - previous_translation.z
            movement_distance = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # Rotation difference (Euler angles)
            previous_rotation_obj = Rotation.from_quat([
                previous_rotation.x,
                previous_rotation.y,
                previous_rotation.z,
                previous_rotation.w
            ])
            previous_euler = previous_rotation_obj.as_euler('xyz')
            
            droll = euler_angles[0] - previous_euler[0]
            dpitch = euler_angles[1] - previous_euler[1]
            dyaw = euler_angles[2] - previous_euler[2]
            
            # Normalize angles to [-π, π] for radians
            droll = (droll + np.pi) % (2 * np.pi) - np.pi
            dpitch = (dpitch + np.pi) % (2 * np.pi) - np.pi
            dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi

            # Convert to degrees for logging
            droll_deg = np.degrees(droll)
            dpitch_deg = np.degrees(dpitch)
            dyaw_deg = np.degrees(dyaw)
            
            self.get_logger().info(f"Camera moved by: [{dx:.3f}, {dy:.3f}, {dz:.3f}] m, "
                                  f"distance: {movement_distance:.3f} m")
            self.get_logger().info(f"Camera rotated by [rad]: [{droll:.2f}, {dpitch:.2f}, {dyaw:.2f}]")
            self.get_logger().info(f"Camera rotated by [deg]: [{droll_deg:.2f}°, {dpitch_deg:.2f}°, {dyaw_deg:.2f}°]")
            
        self.get_logger().info(f"Camera transform will be published dynamically at {self.publish_rate} Hz")
        
        self.get_logger().info("=" * 50)
        
    async def calibrate_callback(self, request, response):
        """External service callback for manual camera re-calibration"""
        self.get_logger().info("Manual re-calibration request received")
        
        success = self.perform_calibration(calibration_source="manual")
        
        if success:
            response.success = True
            response.message = "Re-calibration successful - publishing new tf"
        else:
            response.success = False
            response.message = "Re-calibration failed - could not detect marker"
            
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = FindCameraInWorldService()
        rclpy.spin(node)
    except ValueError as e:
        if node:
            node.get_logger().error(f"Configuration error: {e}")
        else:
            print(f"Configuration error: {e}")
        return 1
    except Exception as e:
        if node:
            node.get_logger().error(f"Node failed with error: {e}")
        else:
            print(f"Node failed with error: {e}")
        return 1
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()