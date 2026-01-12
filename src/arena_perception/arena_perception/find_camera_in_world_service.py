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
        
        self.config = self.load_config(config_file)
        
        # Initialize from configuration
        self.initialize_from_config()
        
        # Service for manual calibration
        self.srv = self.create_service(Trigger, 'find_camera_in_world', self.calibrate_callback)
        
        # STATIC TF broadcaster for marker (never changes)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # DYNAMIC TF broadcaster for camera (updates continuously for usecase of recalibration)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)
        
        # TF Buffer and Listener for getting transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store the marker transform (never changes)
        self.marker_transform = None
        
        # Store current camera transform (updated when calibrated)
        self.current_camera_transform = None
        self.previous_camera_transform = None
        
        # State
        self.calibration_successful = False
        self.attempt_count = 0
        
        # Get rates from parameters
        calibration_rate = self.get_parameter('calibration_attempt_rate').value
        dynamic_rate = self.get_parameter('dynamic_publish_rate').value
        
        # Publish static marker transform IMMEDIATELY
        self.publish_static_marker_transform()
        
        # Start continuous calibration attempts
        self.calibration_timer = self.create_timer(1.0 / calibration_rate, self.attempt_calibration)
        
        # Start continuous dynamic publishing of camera transform
        self.dynamic_publish_timer = self.create_timer(1.0 / dynamic_rate, self.publish_camera_dynamic)
        
        self.get_logger().info("Started continuous calibration attempts...")
        self.get_logger().info(f"Camera transform will be published dynamically at {dynamic_rate} Hz")
        
    def load_config(self, config_path):
        """Load YAML configuration file"""
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found: {config_path}")
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.get_logger().info(f"Loaded configuration from {config_path}")
        return config
    
    def initialize_from_config(self):
        """Initialize parameters from configuration"""
        # Frame names from configuration
        self.world_frame = self.config.get('world_frame', 'world')
        self.calibration_marker_frame = self.config.get('calibration_marker_frame', 'arena_marker')
        self.camera_frame = self.config.get('camera_frame', 'arena_camera_optical')
        
        # Add "_calibrated" suffix to marker frame for the static transform
        self.calibration_marker_calibrated_frame = f"{self.calibration_marker_frame}_calibrated"
        
        # Marker position in world coordinate system
        pos_config = self.config.get('position_marker_in_world', {})
        self.marker_position_in_world = np.array([
            pos_config.get('x', 0.0),
            pos_config.get('y', 0.0),
            pos_config.get('z', 0.0)
        ])
        
        # Marker orientation in world coordinate system
        orient_config = self.config.get('orientation_in_world', {})
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
        
    def create_marker_transform(self):
        """Create the marker transform (world -> marker_calibrated)"""
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = self.world_frame
        transform_stamped.child_frame_id = self.calibration_marker_calibrated_frame
        
        # Set translation based on configuration
        transform_stamped.transform.translation.x = float(self.marker_position_in_world[0])
        transform_stamped.transform.translation.y = float(self.marker_position_in_world[1])
        transform_stamped.transform.translation.z = float(self.marker_position_in_world[2])
        
        # Convert Euler angles from configuration to quaternion
        rotation = Rotation.from_euler('xyz', self.marker_orientation_in_world)
        quat = rotation.as_quat()
        transform_stamped.transform.rotation.x = float(quat[0])
        transform_stamped.transform.rotation.y = float(quat[1])
        transform_stamped.transform.rotation.z = float(quat[2])
        transform_stamped.transform.rotation.w = float(quat[3])
        
        return transform_stamped
        
    def publish_static_marker_transform(self):
        """Publish STATIC transform from world to marker_calibrated based on configuration"""
        self.marker_transform = self.create_marker_transform()
        
        # Publish just the marker transform as static
        self.static_tf_broadcaster.sendTransform([self.marker_transform])
        
        self.get_logger().info(f"Published STATIC transform: {self.world_frame} -> {self.calibration_marker_calibrated_frame}")
        
    def publish_camera_dynamic(self):
        """Publish camera transform dynamically"""
        if self.current_camera_transform is not None:
            # Update timestamp for dynamic publishing
            self.current_camera_transform.header.stamp = self.get_clock().now().to_msg()
            self.dynamic_tf_broadcaster.sendTransform(self.current_camera_transform)
            
    def attempt_calibration(self):
        """Attempt calibration - runs continuously until successful"""
        self.attempt_count += 1
        
        # If already calibrated, just return (we keep running for manual re-calibration via service)
        if self.calibration_successful:
            return
            
        success = self.perform_calibration(source=f"initial (attempt {self.attempt_count})")
        
        if not success and self.attempt_count % 10 == 0:
            self.get_logger().debug(f"Still waiting for calibration after {self.attempt_count} attempts")
            
    def perform_calibration(self, source="manual"):
        """Shared calibration logic - used by both timer and service"""
        try:
            # Look for transform from marker frame to camera frame
            transform = self.tf_buffer.lookup_transform(
                self.calibration_marker_frame,  # Source frame (detected marker)
                self.camera_frame,              # Target frame (camera)
                rclpy.time.Time()
            )
            
            # Compute camera position in world
            camera_in_world_transform = self.compute_camera_in_world(transform)
            
            # Store for dynamic publishing
            self.current_camera_transform = camera_in_world_transform
            
            # Mark as successful
            self.calibration_successful = True
            
            # Store for next comparison
            self.previous_camera_transform = camera_in_world_transform
            
            # Log success
            self.log_transform_details(transform, camera_in_world_transform, source)
            
            if "initial" in source:
                self.get_logger().info(f"✓ Calibration successful after {self.attempt_count} attempts!")
            else:
                self.get_logger().info("✓ Manual re-calibration successful!")
                
            self.get_logger().info("Camera position is now published in TF tree as dynamic transform")
            
            return True
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if "initial" in source:
                # Only log debug messages occasionally to avoid spam
                if self.attempt_count % 10 == 0:
                    self.get_logger().debug(f"Calibration failed: {str(e)}")
            else:
                self.get_logger().error(f"Calibration failed: {str(e)}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error in calibration: {e}")
            return False
    
    def compute_camera_in_world(self, marker_to_camera_transform):
        """Compute camera position in world frame given marker-to-camera transform"""
        # Create transform from world frame to camera frame
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = self.world_frame
        transform_stamped.child_frame_id = self.camera_frame
        
        # Get components of T_marker_camera (from the transform)
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
        
        # Get components of T_world_marker_calibrated (from config)
        world_to_marker_rotation_matrix = Rotation.from_euler('xyz', self.marker_orientation_in_world).as_matrix()
        world_to_marker_translation_vector = self.marker_position_in_world
        
        # Compute chained transform: T_world_camera = T_world_marker * T_marker_camera
        world_to_camera_rotation_matrix = world_to_marker_rotation_matrix @ marker_to_camera_rotation_matrix
        world_to_camera_translation_vector = world_to_marker_rotation_matrix @ marker_to_camera_translation_vector + world_to_marker_translation_vector
        
        # Convert to quaternion for ROS
        world_to_camera_rotation = Rotation.from_matrix(world_to_camera_rotation_matrix)
        world_to_camera_quaternion = world_to_camera_rotation.as_quat()
        
        # Set the computed transform
        transform_stamped.transform.translation.x = float(world_to_camera_translation_vector[0])
        transform_stamped.transform.translation.y = float(world_to_camera_translation_vector[1])
        transform_stamped.transform.translation.z = float(world_to_camera_translation_vector[2])
        
        transform_stamped.transform.rotation.x = float(world_to_camera_quaternion[0])
        transform_stamped.transform.rotation.y = float(world_to_camera_quaternion[1])
        transform_stamped.transform.rotation.z = float(world_to_camera_quaternion[2])
        transform_stamped.transform.rotation.w = float(world_to_camera_quaternion[3])
        
        return transform_stamped
    
    def log_transform_details(self, marker_to_camera_transform, world_to_camera_transform, source):
        """Log detailed information about the calibration"""
        # Log marker-to-camera details
        marker_trans = marker_to_camera_transform.transform.translation
        marker_distance = np.sqrt(marker_trans.x**2 + marker_trans.y**2 + marker_trans.z**2)
        
        # Log world-to-camera details
        world_trans = world_to_camera_transform.transform.translation
        
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"CALIBRATION SUCCESSFUL! ({source})")
        self.get_logger().info(f"Marker to camera distance: {marker_distance:.3f} m")
        self.get_logger().info(f"Camera position in world: [{world_trans.x:.3f}, {world_trans.y:.3f}, {world_trans.z:.3f}] m")
        self.get_logger().info(f"Published transform: {self.world_frame} -> {self.camera_frame}")
        # Log movement if we have previous transform
        if self.previous_camera_transform is not None:
            old_t = self.previous_camera_transform.transform.translation
            new_t = world_to_camera_transform.transform.translation
            dx = new_t.x - old_t.x
            dy = new_t.y - old_t.y
            dz = new_t.z - old_t.z
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            self.get_logger().info(f"Camera moved by: [{dx:.3f}, {dy:.3f}, {dz:.3f}] m, distance: {distance:.3f} m")
        self.get_logger().info("=" * 50)
        
    async def calibrate_callback(self, request, response):
        """External service callback for manual camera re-calibration"""
        self.get_logger().info("Manual re-calibration request received")
        
        success = self.perform_calibration(source="manual")
        
        if success:
            response.success = True
            response.message = "Re-calibration successful - camera position updated"
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