import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from detection.detector import AprilTagDetector
from detection.math_funcs import Pose as MathPose
from scipy.spatial.transform import Rotation
import yaml
import os
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ArenaCalibrationService(Node):
    def __init__(self):
        super().__init__('arena_calibration_service')
        
        # Declare parameters with descriptions
        self.declare_parameter('config_file', '', 
                              ParameterDescriptor(
                                  description='Path to YAML configuration file',
                                  type=ParameterType.PARAMETER_STRING))
        
        # Load configuration
        config_file = self.get_parameter('config_file').value
        if not config_file:
            self.get_logger().error("No configuration file specified. Use '--ros-args -p config_file:=/path/to/config.yaml'")
            raise ValueError("Configuration file path is required")
        
        self.config = self.load_config(config_file)
        
        # Initialize from configuration
        self.initialize_from_config()
        
        # Service for calibration (can be called externally after startup)
        self.srv = self.create_service(Trigger, 'calibrate_arena', self.calibrate_callback)
        
        # TF broadcaster for continuous publishing
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # STATIC TF broadcaster for world->tag transform
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, 10)
        self.bridge = CvBridge()
        
        # State
        self.latest_frame = None
        self.camera_info = None
        self.camera_from_world = None
        self.initial_calibration_complete = False  # Track if we've calibrated at least once
        self.april_detector = None
        
        # Timer for continuous TF publishing at 1Hz (only active after initial calibration)
        self.tf_publish_timer = self.create_timer(1.0, self.publish_transform)
        
        # Initial calibration retry timer - keeps trying until success
        self.initial_calibration_timer = self.create_timer(1.0, self.perform_initial_calibration)
        
        # Publish static transform IMMEDIATELY on startup (from config)
        self.publish_static_tag_transform()
        
        # Publish static identity transform: map -> world (identical frames)
        self.publish_world_to_map_transform()
    
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
        # Arena tag configuration
        tag_config = self.config.get('arena_apriltag', {})
        self.arena_tag_id = tag_config.get('id', 2)
        self.arena_tag_family = tag_config.get('family', 'tagStandard41h12')
        self.arena_tag_size = tag_config.get('size', 0.15)
        self.arena_tag_role = tag_config.get('role', 'arena_origin_finder')
        
        # Tag position in arena coordinate system
        pos_config = tag_config.get('position_in_arena', {})
        self.tag_position_in_arena = np.array([
            pos_config.get('x', 0.0),
            pos_config.get('y', 0.0),
            pos_config.get('z', 0.0)
        ])
        
        # Tag orientation in arena coordinate system
        orient_config = tag_config.get('orientation_in_arena', {})
        self.tag_orientation_in_arena = np.array([
            orient_config.get('roll', 0.0),
            orient_config.get('pitch', 0.0),
            orient_config.get('yaw', 0.0)
        ])
        
        # Arena geometry
        arena_geom = self.config.get('arena_geometry', {})
        dims = arena_geom.get('dimensions', {})
        self.arena_dimensions = np.array([
            dims.get('width', 1.5),
            dims.get('height', 1.5),
            dims.get('depth', 0.0)
        ])
        
        boundaries = arena_geom.get('boundaries', {})
        self.arena_boundaries = {
            'x_min': boundaries.get('x_min', -0.5),
            'x_max': boundaries.get('x_max', 2.0),
            'y_min': boundaries.get('y_min', -0.5),
            'y_max': boundaries.get('y_max', 2.0),
            'z_min': boundaries.get('z_min', 0.0),
            'z_max': boundaries.get('z_max', 1.0)
        }
        
        # Frame names - also read map frame if present
        frames = self.config.get('frames', {})
        self.map_frame = frames.get('map', 'map')  # Default to 'map'
        self.world_frame = frames.get('world', 'world')
        self.camera_frame = frames.get('camera', 'arena_camera')
        self.tag_frame = frames.get('tag', 'arena_apriltag')
        
        # Log configuration
        self.get_logger().info(f"Arena tag ID: {self.arena_tag_id}, Family: {self.arena_tag_family}, Size: {self.arena_tag_size}")
        self.get_logger().info(f"Arena dimensions: {self.arena_dimensions[0]:.2f} x {self.arena_dimensions[1]:.2f} m")
        self.get_logger().info(f"TF frames: map='{self.map_frame}', world='{self.world_frame}', camera='{self.camera_frame}', tag='{self.tag_frame}'")
        
    def publish_world_to_map_transform(self):
        """Publish STATIC identity transform from world to map (world is root)"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame  # CHANGED: world is parent (root)
        t.child_frame_id = self.map_frame     # CHANGED: map is child
        
        # Identity transform (same position/orientation)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send once - this is a static transform
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published STATIC transform: {self.world_frame} -> {self.map_frame} (identity)")
        
    def camera_info_callback(self, msg):
        """Receive camera calibration from camera_info topic"""
        self.camera_info = msg
        if self.april_detector is None and self.camera_info is not None:
            calibration_data = {
                'camera_matrix': np.array(self.camera_info.k).reshape(3, 3),
                'distortion_coefficients': np.array(self.camera_info.d)
            }
            self.april_detector = AprilTagDetector(calibration_data)
            self.get_logger().info("Camera info received. AprilTag detector initialized")
        
    def image_callback(self, msg):
        """Store the latest camera frame"""
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def perform_initial_calibration(self):
        """Perform initial calibration on startup - retry until first success"""
        if self.initial_calibration_complete:
            self.initial_calibration_timer.cancel()  # Stop retrying once we have initial calibration
            return
            
        if self.latest_frame is None:
            self.get_logger().debug("Waiting for camera frames...")
            return
            
        if self.april_detector is None:
            self.get_logger().debug("Waiting for camera info...")
            return
            
        self.get_logger().info("Performing initial calibration...")
        success = self.detect_arena_tag()
        
        if success:
            if not self.initial_calibration_complete:
                self.initial_calibration_complete = True
                self.get_logger().info("Initial calibration successful - starting continuous TF publishing")
        else:
            self.get_logger().warn("Initial calibration failed, will retry in 1 second...")
        
    async def calibrate_callback(self, request, response):
        """External calibration service - can be called by other nodes"""
        self.get_logger().info("External calibration request received")
        
        if self.latest_frame is None:
            response.success = False
            response.message = "No camera frame available"
            return response
            
        if self.april_detector is None:
            response.success = False
            response.message = "Camera info not received yet"
            return response
            
        success = self.detect_arena_tag()  # DIRECT CALL
        
        if success:
            self.get_logger().info("Recalibration successful - updated TF transform")
            response.success = True
            response.message = "Arena calibration complete and published to TF"
        else:
            response.success = False
            response.message = "Failed to detect arena tag"
            
        return response
    
    def detect_arena_tag(self):
        """Detect arena tag and store cameraFromWorld transform"""
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
        april_tag_output = self.april_detector.detect(gray, self.arena_tag_size)
        
        for detection in april_tag_output["aprilTags"]:
            if (detection.tag_family.decode() == self.arena_tag_family and
                detection.tag_id == self.arena_tag_id):
                
                # Store the raw tag detection pose (camera_from_tag)
                camera_from_tag = MathPose(detection.pose_R, detection.pose_t)
                
                # Calculate tag_from_arena transform based on tag's position/orientation in arena
                tag_rotation = Rotation.from_euler('xyz', self.tag_orientation_in_arena)
                tag_from_arena_R = tag_rotation.as_matrix()
                tag_from_arena_t = self.tag_position_in_arena
                
                # Convert to numpy arrays for easier manipulation
                camera_from_tag_R = np.array(camera_from_tag.R)
                camera_from_tag_t = np.array(camera_from_tag.t).reshape(3, 1)
                tag_from_arena_R = np.array(tag_from_arena_R)
                tag_from_arena_t = np.array(tag_from_arena_t).reshape(3, 1)
                
                # Compute camera_from_world = camera_from_tag * tag_from_arena
                camera_from_arena_R = camera_from_tag_R @ tag_from_arena_R
                camera_from_arena_t = camera_from_tag_R @ tag_from_arena_t + camera_from_tag_t
                
                # Store the result
                self.camera_from_world = MathPose(camera_from_arena_R, camera_from_arena_t.flatten())
                
                self.get_logger().info("Arena tag detected - calibration complete")
                
                # Optionally publish tag pose in camera frame for debugging
                self.publish_tag_transform(camera_from_tag)
                
                return True
        
        self.get_logger().warn("Arena tag not found in frame")
        return False
    
    def publish_static_tag_transform(self):
        """Publish STATIC transform from world to tag based on configuration"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.tag_frame
        
        # Use the configured tag position in arena
        t.transform.translation.x = float(self.tag_position_in_arena[0])
        t.transform.translation.y = float(self.tag_position_in_arena[1])
        t.transform.translation.z = float(self.tag_position_in_arena[2])
        
        # Use the configured tag orientation in arena
        rotation = Rotation.from_euler('xyz', self.tag_orientation_in_arena)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        # Send once - this is a static transform
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published STATIC transform: {self.world_frame} -> {self.tag_frame}")
        self.get_logger().info(f"Tag position in world: [{t.transform.translation.x:.3f}, "
                              f"{t.transform.translation.y:.3f}, {t.transform.translation.z:.3f}]")
    
    def publish_tag_transform(self, camera_from_tag):
        """Publish DYNAMIC tag transform in camera frame (based on actual detection)"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.tag_frame
        
        # Translation from actual detection
        t.transform.translation.x = float(camera_from_tag.t[0])
        t.transform.translation.y = float(camera_from_tag.t[1])
        t.transform.translation.z = float(camera_from_tag.t[2])
        
        # Rotation matrix to quaternion conversion
        rotation = Rotation.from_matrix(camera_from_tag.R)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published DYNAMIC tag transform: {self.camera_frame} -> {self.tag_frame}")
        
        # Log detection info
        self.log_calibration_accuracy(camera_from_tag)
    
    def log_calibration_accuracy(self, camera_from_tag):
        """Log how well the detected tag matches the expected position"""
        # Convert to Python floats before formatting
        x = float(camera_from_tag.t[0])
        y = float(camera_from_tag.t[1])
        z = float(camera_from_tag.t[2])
        
        self.get_logger().info(f"Detected tag at camera-relative position: [{x:.3f}, {y:.3f}, {z:.3f}] m")
    
    def publish_transform(self):
        """Continuously publish transform at 1Hz ONLY after initial calibration"""
        if not self.initial_calibration_complete or self.camera_from_world is None:
            return  # Don't publish until we have initial calibration
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.camera_frame
        
        # Translation - convert to float first
        t.transform.translation.x = float(self.camera_from_world.t[0])
        t.transform.translation.y = float(self.camera_from_world.t[1])
        t.transform.translation.z = float(self.camera_from_world.t[2])
        
        # Rotation matrix to quaternion conversion
        rotation = Rotation.from_matrix(self.camera_from_world.R)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published camera transform to TF: {self.world_frame} -> {self.camera_frame}", 
                               throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ArenaCalibrationService()
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