import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import yaml
import os
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from scipy.spatial.transform import Rotation

class FindCameraInWorldService(Node):
    def __init__(self):
        super().__init__('find_camera_in_world_service')
        
        # Declare parameters with descriptions
        self.declare_parameter('config_file', '', 
                              ParameterDescriptor(
                                  description='Path to YAML configuration file',
                                  type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('calibration_file', '', 
                              ParameterDescriptor(
                                  description='Path to YAML file for storing calibration results (supports package:// URLs)',
                                  type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('calibration_attempt_rate', 1.0,
                              ParameterDescriptor(
                                  description='Rate (Hz) for attempting calibration',
                                  type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('dynamic_publish_rate', 30.0,
                              ParameterDescriptor(
                                  description='Rate (Hz) for publishing camera transform dynamically',
                                  type=ParameterType.PARAMETER_DOUBLE))
        
        # Load configuration
        config_file = self.get_parameter('config_file').value
        if not config_file:
            self.get_logger().error("No configuration file specified. Use '--ros-args -p config_file:=/path/to/config.yaml'")
            raise ValueError("Configuration file path is required")
        
        self.load_config(config_file)
              
        # Service for manual calibration
        self.srv = self.create_service(Trigger, 'find_camera_in_world', self.calibrate_callback)
        
        # TF broadcasters
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)  # For marker (static)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)        # For camera (dynamic)
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Transform storage
        self.world_to_marker_transform = None      # world -> marker_calibrated (static, from config)
        self.world_to_camera_transform = None      # world -> camera (dynamic, computed)
        self.previous_world_to_camera_transform = None  # For movement tracking
        
        # Calibration state
        self.calibration_successful = False
        self.attempt_count = 0
        
        # Get rates from parameters
        self.calibration_rate = self.get_parameter('calibration_attempt_rate').value
        self.dynamic_rate = self.get_parameter('dynamic_publish_rate').value
        
        # Publish static marker transform IMMEDIATELY
        self.publish_static_marker_transform()

        # Get calibration file path - FIRST check parameter, THEN config, THEN default
        calibration_file = self.get_parameter('calibration_file').value
            
        if not self.load_calibration(calibration_file):
            # Start continuous calibration attempts since no calibration via file was provided
            self.calibration_timer = self.create_timer(1.0 / self.calibration_rate, self.attempt_calibration)
            self.get_logger().info("Started continuous calibration attempts...")
        
        # Start continuous dynamic publishing of camera transform
        self.dynamic_publish_timer = self.create_timer(1.0 / self.dynamic_rate, self.publish_camera_dynamic)
           
    def load_config(self, config_path):
        """Load YAML configuration file of node"""
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found: {config_path}")
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.get_logger().info(f"Loaded configuration from {config_path}")
    
        """Initialize parameters from configuration"""
        # Frame names from configuration
        self.world_frame = config.get('world_frame', 'world')
        self.calibration_marker_frame = config.get('calibration_marker_frame', 'arena_marker')
        self.camera_frame = config.get('camera_frame', 'arena_camera_optical')
        
        # Add "_calibrated" suffix to marker frame for the static transform
        self.calibration_marker_calibrated_frame = f"{self.calibration_marker_frame}_calibrated"
        
        # Marker position in world coordinate system
        pos_config = config.get('position_marker_in_world', {})
        self.marker_position_in_world = np.array([
            pos_config.get('x', 0.0),
            pos_config.get('y', 0.0),
            pos_config.get('z', 0.0)
        ])
        
        # Marker orientation in world coordinate system
        orient_config = config.get('orientation_in_world', {})
        self.marker_orientation_in_world = np.array([
            orient_config.get('roll', 0.0),
            orient_config.get('pitch', 0.0),
            orient_config.get('yaw', 0.0)
        ])
        
        # Log configuration
        self.get_logger().info(f"World frame: {self.world_frame}")
        self.get_logger().info(f"Calibration marker frame (detected): {self.calibration_marker_frame}")
        self.get_logger().info(f"Calibration marker frame (calibrated): {self.calibration_marker_calibrated_frame}")
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        return config
      
    def load_calibration(self, calibration_file):
        """Try to load existing camera calibration of world -> camera tf from file"""

        if not os.path.exists(calibration_file):
            self.get_logger().info(f"No calibration file found at {calibration_file}")
            return False
        
        try:
            self.get_logger().info(f"Using calibration file from config/parameter: {calibration_file}")
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
        
        # Set translation based on configuration
        world_to_marker_transform.transform.translation.x = float(self.marker_position_in_world[0])
        world_to_marker_transform.transform.translation.y = float(self.marker_position_in_world[1])
        world_to_marker_transform.transform.translation.z = float(self.marker_position_in_world[2])
        
        # Convert Euler angles from configuration to quaternion
        rotation = Rotation.from_euler('xyz', self.marker_orientation_in_world)
        quat = rotation.as_quat()
        world_to_marker_transform.transform.rotation.x = float(quat[0])
        world_to_marker_transform.transform.rotation.y = float(quat[1])
        world_to_marker_transform.transform.rotation.z = float(quat[2])
        world_to_marker_transform.transform.rotation.w = float(quat[3])
        
        return world_to_marker_transform
        
    def publish_static_marker_transform(self):
        """Publish STATIC transform from world to marker_calibrated based on configuration"""
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
            # Get current marker-to-camera transform from TF
            marker_to_camera_transform = self.tf_buffer.lookup_transform(
                self.calibration_marker_frame,  # Source frame (detected marker)
                self.camera_frame,              # Target frame (camera)
                rclpy.time.Time()
            )
            
            # Compute world-to-camera transform
            world_to_camera_transform = self.compute_world_to_camera_transform(marker_to_camera_transform)
            
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
            
            # If this was an initial calibration, stop the timer
            if "initial" in calibration_source and hasattr(self, 'calibration_timer'):
                self.calibration_timer.cancel()
                self.get_logger().info("Stopped continuous calibration attempts (calibration successful)")
            
            return True
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.handle_calibration_error(e, calibration_source)
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error in calibration: {e}")
            return False
    
    def compute_world_to_camera_transform(self, marker_to_camera_transform):
        """Compute world-to-camera transform given marker-to-camera transform"""
        # Create transform from world frame to camera frame
        world_to_camera_transform = TransformStamped()
        world_to_camera_transform.header.stamp = self.get_clock().now().to_msg()
        world_to_camera_transform.header.frame_id = self.world_frame
        world_to_camera_transform.child_frame_id = self.camera_frame
        
        # Get translation and rotation from marker-to-camera transform
        marker_to_camera_translation = marker_to_camera_transform.transform.translation
        marker_to_camera_rotation = marker_to_camera_transform.transform.rotation
        
        # Convert marker_to_camera transform to rotation matrix and translation vector
        marker_to_camera_rotation_matrix = Rotation.from_quat([
            marker_to_camera_rotation.x,
            marker_to_camera_rotation.y,
            marker_to_camera_rotation.z,
            marker_to_camera_rotation.w
        ]).as_matrix()
        
        marker_to_camera_translation_vector = np.array([
            marker_to_camera_translation.x,
            marker_to_camera_translation.y,
            marker_to_camera_translation.z
        ])
        
        # Get components of world-to-marker transform (from config)
        world_to_marker_rotation_matrix = Rotation.from_euler('xyz', self.marker_orientation_in_world).as_matrix()
        world_to_marker_translation_vector = self.marker_position_in_world
        
        # Compute chained transform: world_to_camera = world_to_marker * marker_to_camera
        world_to_camera_rotation_matrix = world_to_marker_rotation_matrix @ marker_to_camera_rotation_matrix
        world_to_camera_translation_vector = world_to_marker_rotation_matrix @ marker_to_camera_translation_vector + world_to_marker_translation_vector
        
        # Convert to quaternion for ROS
        world_to_camera_rotation = Rotation.from_matrix(world_to_camera_rotation_matrix)
        world_to_camera_quaternion = world_to_camera_rotation.as_quat()
        
        # Set the computed transform
        world_to_camera_transform.transform.translation.x = float(world_to_camera_translation_vector[0])
        world_to_camera_transform.transform.translation.y = float(world_to_camera_translation_vector[1])
        world_to_camera_transform.transform.translation.z = float(world_to_camera_translation_vector[2])
        
        world_to_camera_transform.transform.rotation.x = float(world_to_camera_quaternion[0])
        world_to_camera_transform.transform.rotation.y = float(world_to_camera_quaternion[1])
        world_to_camera_transform.transform.rotation.z = float(world_to_camera_quaternion[2])
        world_to_camera_transform.transform.rotation.w = float(world_to_camera_quaternion[3])
        
        return world_to_camera_transform
    
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
            
        self.get_logger().info(f"Camera transform will be published dynamically at {self.dynamic_rate} Hz")
        
        self.get_logger().info("=" * 50)
        
    def handle_calibration_error(self, error, calibration_source):
        """Handle calibration errors with appropriate logging level"""
        if "initial" in calibration_source:
            # Only log debug messages occasionally to avoid spam
            if self.attempt_count % 10 == 0:
                self.get_logger().debug(f"Calibration failed: {str(error)}")
        else:
            self.get_logger().error(f"Calibration failed: {str(error)}")
        
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