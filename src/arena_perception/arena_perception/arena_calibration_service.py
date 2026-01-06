import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import pyapriltags
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
        self.declare_parameter('tf_publish_rate', 30.0,
                              ParameterDescriptor(
                                  description='Rate (Hz) for publishing TF transforms',
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
        self.srv = self.create_service(Trigger, 'calibrate_arena', self.calibrate_callback)
        
        # TF broadcaster for continuous publishing
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # STATIC TF broadcaster
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
        self.projection_matrix = None  # 3x4 projection matrix P for rectified image
        self.tag_from_camera_optical = None  # Transform from camera_optical to tag (R, t) tuple
        self.camera_optical_from_tag = None  # Inverse: transform from tag to camera_optical (R, t) tuple
        self.initial_calibration_complete = False
        self.april_detector = None
        
        # Control flags and counters
        self.tf_publish_active = False
        
        # Get TF publish rate from parameter
        tf_publish_rate = self.get_parameter('tf_publish_rate').value
        
        # Timer for continuous TF publishing (created but not immediately started)
        self.tf_publish_timer = None
        self.tf_publish_rate = tf_publish_rate
        
        # Initial calibration retry timer - keeps trying until success
        self.initial_calibration_timer = self.create_timer(1.0, self.perform_initial_calibration)
        
        # Publish static transforms IMMEDIATELY on startup
        self.publish_static_tag_transform()  # world -> tag (where tag IS in world)
        self.publish_world_to_map_transform()  # world -> map
        
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
        tag_config = self.config.get('arena_tag', {})
        self.arena_tag_id = tag_config.get('id', 2)
        self.arena_tag_family = tag_config.get('family', 'tagStandard41h12')
        self.arena_tag_size = tag_config.get('size', 0.15)
        self.arena_tag_role = tag_config.get('role', 'arena_origin_finder')
        
        # Tag position in world coordinate system
        pos_config = tag_config.get('position_in_arena', {})
        self.tag_position_in_world = np.array([
            pos_config.get('x', 0.0),
            pos_config.get('y', 0.0),
            pos_config.get('z', 0.0)
        ])
        
        # Tag orientation in world coordinate system
        orient_config = tag_config.get('orientation_in_arena', {})
        self.tag_orientation_in_world = np.array([
            orient_config.get('roll', 0.0),
            orient_config.get('pitch', 0.0),
            orient_config.get('yaw', 0.0)
        ])
        
        # Frame names
        frames = self.config.get('frames', {})
        self.frame_map = frames.get('map', 'map')
        self.frame_world = frames.get('world', 'world')
        self.frame_camera_optical = frames.get('camera_optical', 'arena_camera_optical')
        self.frame_tag = frames.get('tag', 'arena_tag')
        
        # Log configuration
        self.get_logger().info(f"Arena tag ID: {self.arena_tag_id}, Family: {self.arena_tag_family}, Size: {self.arena_tag_size}")
        self.get_logger().info(f"Tag position in world: {self.tag_position_in_world}")
        
    def publish_world_to_map_transform(self):
        """Publish STATIC identity transform from world to map"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_world
        t.child_frame_id = self.frame_map
        
        # Identity transform
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published STATIC: {self.frame_world} -> {self.frame_map}")
        

    def publish_static_tag_transform(self):
        """Publish STATIC transform from world to tag based on configuration"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_world
        t.child_frame_id = self.frame_tag
        
        t.transform.translation.x = float(self.tag_position_in_world[0])
        t.transform.translation.y = float(self.tag_position_in_world[1])
        t.transform.translation.z = float(self.tag_position_in_world[2])
        
        rotation = Rotation.from_euler('xyz', self.tag_orientation_in_world)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published STATIC: {self.frame_world} -> {self.frame_tag}")
        self.get_logger().info(f"Tag position in world: [{t.transform.translation.x:.3f}, "
                            f"{t.transform.translation.y:.3f}, {t.transform.translation.z:.3f}]")
        
    def camera_info_callback(self, msg):
        """Receive camera calibration from camera_info topic"""
        self.camera_info = msg
        
        # Extract the projection matrix P (3x4) - this is for rectified images
        self.projection_matrix = np.array(self.camera_info.p).reshape(3, 4)
        
        # For rectified images (image_rect), we should use the projection matrix P
        # The left 3x3 part of P is the intrinsic matrix for the rectified image
        K_rect = self.projection_matrix[:, :3]
        
        # Extract camera intrinsics from the rectified camera matrix for AprilTag
        fx = K_rect[0, 0]
        fy = K_rect[1, 1]
        cx = K_rect[0, 2]
        cy = K_rect[1, 2]
        
        if self.april_detector is None:
            # Initialize AprilTag detector directly
            self.april_detector = pyapriltags.Detector(families=self.arena_tag_family)
            
            # Store intrinsics only for AprilTag detector
            self.april_intrinsics = [fx, fy, cx, cy]
            
            self.get_logger().info("Camera info received. AprilTag detector initialized")
            self.get_logger().info(f"Using projection matrix P for rectified image (image_rect)")
            self.get_logger().info(f"Rectified camera intrinsics from P: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        
    def image_callback(self, msg):
        """Store the latest camera frame"""
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def perform_initial_calibration(self):
        """Perform initial calibration on startup - retry until first success"""
        if self.initial_calibration_complete:
            self.initial_calibration_timer.cancel()
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
                # Start the TF publishing timer only after calibration is complete
                self.start_tf_publishing()
                self.get_logger().info(f"Initial calibration successful - starting TF publishing at {self.tf_publish_rate} Hz")
        else:
            self.get_logger().warn("Initial calibration failed, will retry in 1 second...")
    
    def start_tf_publishing(self):
        """Start the TF publishing timer"""
        if self.tf_publish_timer is None:
            tf_publish_period = 1.0 / self.tf_publish_rate
            self.tf_publish_timer = self.create_timer(tf_publish_period, self.publish_transform)
            self.tf_publish_active = True
        
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
            
        success = self.detect_arena_tag()
        
        if success:
            self.get_logger().info("Recalibration successful - updated TF transform")
            # Ensure TF publishing is active
            if not self.tf_publish_active:
                self.start_tf_publishing()
            response.success = True
            response.message = "Arena calibration complete and published to TF"
        else:
            response.success = False
            response.message = "Failed to detect arena tag"
            
        return response
    
    def detect_arena_tag(self):
        """Detect arena tag and compute tag -> camera_optical transform"""
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
        
        # Direct AprilTag detection using the rectified camera intrinsics
        detections = self.april_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.april_intrinsics,
            tag_size=self.arena_tag_size
        )
        
        for detection in detections:
            if (detection.tag_family.decode() == self.arena_tag_family and
                detection.tag_id == self.arena_tag_id):
                
                # AprilTag gives us: tag pose in camera optical frame (OpenCV)
                # This is T_tag_camera_optical: transform FROM camera_optical TO tag
                tag_from_camera_optical_R = np.array(detection.pose_R)
                tag_from_camera_optical_t = np.array(detection.pose_t).reshape(3, 1)
                
                # Store transform as tuple (R, t)
                self.tag_from_camera_optical = (tag_from_camera_optical_R, tag_from_camera_optical_t)
                
                # INVERSE: Compute camera_optical FROM tag transform
                # T_camera_optical_tag = T_tag_camera_optical^(-1)
                camera_optical_from_tag_R = tag_from_camera_optical_R.T
                camera_optical_from_tag_t = -camera_optical_from_tag_R @ tag_from_camera_optical_t
                
                # Store the inverse transform as tuple (R, t)
                self.camera_optical_from_tag = (camera_optical_from_tag_R, camera_optical_from_tag_t.flatten())
                
                self.get_logger().info("=== TAG DETECTED ===")
                self.get_logger().info(f"Tag position in camera_optical frame: [{tag_from_camera_optical_t[0][0]:.3f}, "
                                      f"{tag_from_camera_optical_t[1][0]:.3f}, {tag_from_camera_optical_t[2][0]:.3f}] m")
                self.get_logger().info(f"Camera_optical position in tag frame: [{camera_optical_from_tag_t[0][0]:.3f}, "
                                      f"{camera_optical_from_tag_t[1][0]:.3f}, {camera_optical_from_tag_t[2][0]:.3f}] m")
                self.get_logger().info(f"Distance to tag: {np.linalg.norm(tag_from_camera_optical_t):.3f} m")
                
                return True
        
        self.get_logger().warn("Arena tag not found in frame")
        return False
    
    def publish_transform(self):
        """Continuously publish tag -> camera_optical transform"""
        if not self.initial_calibration_complete or self.camera_optical_from_tag is None:
            return
            
        # Extract R and t from tuple
        camera_optical_from_tag_R, camera_optical_from_tag_t = self.camera_optical_from_tag
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_tag
        t.child_frame_id = self.frame_camera_optical
        
        # Use the INVERSE transform: FROM tag TO camera_optical
        t.transform.translation.x = float(camera_optical_from_tag_t[0])
        t.transform.translation.y = float(camera_optical_from_tag_t[1])
        t.transform.translation.z = float(camera_optical_from_tag_t[2])
        
        rotation = Rotation.from_matrix(camera_optical_from_tag_R)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)


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