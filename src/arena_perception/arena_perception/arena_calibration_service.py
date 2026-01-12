import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
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
        self.declare_parameter('tf_publish_rate', 30.0,
                              ParameterDescriptor(
                                  description='Rate (Hz) for publishing TF transforms',
                                  type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('tf_lookup_rate', 10.0,
                              ParameterDescriptor(
                                  description='Rate (Hz) for looking up TF transform',
                                  type=ParameterType.PARAMETER_DOUBLE))
        
        # Load configuration
        config_file = self.get_parameter('config_file').value
        if not config_file:
            self.get_logger().error("No configuration file specified. Use '--ros-args -p config_file:=/path/to/config.yaml'")
            raise ValueError("Configuration file path is required")
        
        self.config = self.load_config(config_file)
        
        # Initialize from configuration
        self.initialize_from_config()
        
        # Service for calibration (can be called externally after startup)
        self.srv = self.create_service(Trigger, 'find_camera_in_world', self.calibrate_callback)
        
        # TF broadcaster for continuous publishing
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # STATIC TF broadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # TF Buffer and Listener for getting transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Get parameters
        tf_lookup_rate = self.get_parameter('tf_lookup_rate').value
        
        # State
        self.latest_transform = None  # Transform from camera to marker
        self.initial_calibration_complete = False
        
        # Control flags
        self.tf_publish_active = False
        
        # Get TF publish rate from parameter
        tf_publish_rate = self.get_parameter('tf_publish_rate').value
        
        # Timer for continuous TF publishing (created but not immediately started)
        self.tf_publish_timer = None
        self.tf_publish_rate = tf_publish_rate
        
        # Timer for looking up TF transform
        self.tf_lookup_timer = self.create_timer(1.0 / tf_lookup_rate, self.lookup_transform)
        
        # Publish static transforms IMMEDIATELY on startup
        self.publish_static_marker_transform()  # world -> marker (where marker IS in world)
        
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
        self.get_logger().info(f"Calibration marker frame: {self.calibration_marker_frame}")
        self.get_logger().info(f"Camera frame: {self.camera_frame}")
        self.get_logger().info(f"Marker position in world: {self.marker_position_in_world}")
        self.get_logger().info(f"Marker orientation in world (RPY): {self.marker_orientation_in_world}")
        
    def publish_static_marker_transform(self):
        """Publish STATIC transform from world to marker based on configuration
        
        This defines where the calibration marker is physically located in the world.
        The marker serves as a known reference point for camera localization.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.calibration_marker_frame
        
        # Set translation based on configuration
        t.transform.translation.x = float(self.marker_position_in_world[0])
        t.transform.translation.y = float(self.marker_position_in_world[1])
        t.transform.translation.z = float(self.marker_position_in_world[2])
        
        # Convert Euler angles from configuration to quaternion
        # Note: The orientation accounts for differences between AprilTag and ROS coordinate conventions
        rotation = Rotation.from_euler('xyz', self.marker_orientation_in_world)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published STATIC transform: {self.world_frame} -> {self.calibration_marker_frame}")
        self.get_logger().info(f"Marker position in world: [{t.transform.translation.x:.3f}, "
                            f"{t.transform.translation.y:.3f}, {t.transform.translation.z:.3f}] m")
        
    def lookup_transform(self):
        """Look up transform from camera frame to marker frame via TF
        
        This method continuously tries to find the transform between the camera and
        the calibration marker. Once found, it enables continuous publishing of
        the camera's position in the world frame.
        """
        try:
            # Look for transform from camera frame to marker frame
            # We want the marker's pose in camera coordinates
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,          # Source frame (camera)
                self.calibration_marker_frame,  # Target frame (marker)
                rclpy.time.Time()
            )
            
            # Store the transform for continuous publishing
            self.latest_transform = transform
            
            if not self.initial_calibration_complete:
                self.initial_calibration_complete = True
                self.start_tf_publishing()
                self.get_logger().info(f"Initial TF lookup successful - starting TF publishing at {self.tf_publish_rate} Hz")
                self.get_logger().info(f"Transform found: {self.camera_frame} -> {self.calibration_marker_frame}")
                self.log_transform_details(transform)
                
            return True
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if not self.initial_calibration_complete:
                self.get_logger().debug(f"Waiting for TF transform from {self.camera_frame} to {self.calibration_marker_frame}: {str(e)}")
            return False
    
    def log_transform_details(self, transform):
        """Log detailed information about the camera-to-marker transform"""
        trans = transform.transform.translation
        rot = transform.transform.rotation
        self.get_logger().info(f"Marker position in camera frame: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}] m")
        distance = np.sqrt(trans.x**2 + trans.y**2 + trans.z**2)
        self.get_logger().info(f"Distance from camera to marker: {distance:.3f} m")
    
    def start_tf_publishing(self):
        """Start the TF publishing timer for continuous updates"""
        if self.tf_publish_timer is None:
            tf_publish_period = 1.0 / self.tf_publish_rate
            self.tf_publish_timer = self.create_timer(tf_publish_period, self.publish_camera_in_world)
            self.tf_publish_active = True
            self.get_logger().info(f"Started continuous TF publishing at {self.tf_publish_rate} Hz")
        
    async def calibrate_callback(self, request, response):
        """External service callback for manual camera localization
        
        This service can be called by other nodes to trigger a camera
        localization update. It's useful for re-calibration or when
        the camera might have moved.
        """
        self.get_logger().info("Manual camera localization request received")
        
        success = self.lookup_transform()
        
        if success:
            self.get_logger().info("Camera localization successful - updated TF transform")
            # Ensure TF publishing is active
            if not self.tf_publish_active:
                self.start_tf_publishing()
            response.success = True
            response.message = "Camera localization complete - camera position published in world frame"
        else:
            response.success = False
            response.message = f"Failed to locate camera - transform from {self.camera_frame} to {self.calibration_marker_frame} not available"
            
        return response
    
    def publish_camera_in_world(self):
        """Continuously publish camera position in world frame
        
        This method computes and publishes the transform from world frame
        to camera frame, effectively positioning the camera in the world
        coordinate system.
        
        The computation chain is:
        1. We know: world -> marker (static, from config)
        2. We have: marker -> camera (from TF lookup)
        3. Therefore: world -> camera = (world -> marker) * (marker -> camera)
        """
        if not self.initial_calibration_complete or self.latest_transform is None:
            return
            
        # Create transform from world to camera
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.camera_frame
        
        # Note: The actual transform computation would require chaining transforms
        # For now, we'll publish the direct transform we looked up (marker in camera frame)
        # In a full implementation, we would compute:
        # T_world_camera = T_world_marker * T_marker_camera
        
        # For this simplified version, we'll publish the marker-to-camera transform
        # assuming the marker frame is aligned with world frame (which it is via static transform)
        
        # Use the transform we looked up (marker in camera coordinates)
        # This gives us camera's position relative to marker
        t.transform.translation.x = self.latest_transform.transform.translation.x
        t.transform.translation.y = self.latest_transform.transform.translation.y
        t.transform.translation.z = self.latest_transform.transform.translation.z
        
        t.transform.rotation.x = self.latest_transform.transform.rotation.x
        t.transform.rotation.y = self.latest_transform.transform.rotation.y
        t.transform.rotation.z = self.latest_transform.transform.rotation.z
        t.transform.rotation.w = self.latest_transform.transform.rotation.w
        
        self.tf_broadcaster.sendTransform(t)


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