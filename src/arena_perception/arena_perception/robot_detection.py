import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation
import pyapriltags
import yaml
import os

class BotDetectionNode(Node):
    def __init__(self):
        super().__init__('robot_detection_node')
        
        # Declare parameters
        self.declare_parameter('config_file', '', 
                              descriptor=ParameterDescriptor(
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
        
        # QoS profiles
        image_qos = rclpy.qos.qos_profile_sensor_data
        info_qos = rclpy.qos.QoSProfile(depth=10)
        
        # Subscribers with appropriate QoS
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, image_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, info_qos)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, self.topics['robot_pose'], 10)
        
        # Debug publishers
        self.tag_pose_pub = self.create_publisher(PoseStamped, '/debug/tag_pose', 10)
        
        # Only create overlay publisher if debug images are enabled
        if self.debug_image:
            self.overlay_pub = self.create_publisher(Image, self.topics['detection_overlay'], 10)
            self.get_logger().info("Debug image publishing ENABLED")
        else:
            self.overlay_pub = None
            self.get_logger().info("Debug image publishing DISABLED")
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State
        self.bridge = CvBridge()
        self.april_detector = None
        self.projection_matrix = None  # 3x4 projection matrix P
        self.camera_intrinsics = None  # [fx, fy, cx, cy]
        
        # Timer for transform publishing
        self.tf_timer = self.create_timer(0.1, self.publish_latest_tf)  # 10Hz
        self.latest_robot_base_pose = None  # (R, t) tuple for world_from_robot_base
        self.latest_camera_from_tag = None  # For debug TF publishing (R, t) tuple
        self.latest_tag_id = None  # Store which tag was detected
        
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
        # Robot tags configuration
        robot_tags_config = self.config.get('robot_tags', {})
        self.robot_tags = {
            'top_id': robot_tags_config.get('top_id', 12),
            'bottom_id': robot_tags_config.get('bottom_id', 31),
            'family': robot_tags_config.get('family', 'tagStandard41h12'),
            'size': robot_tags_config.get('size', 0.0556)
        }
        
        # Load tag-to-base transforms for both tags
        self.tag_transforms = {}
        
        # Top tag transform
        top_transform_config = robot_tags_config.get('top_tag_to_base_transform', {})
        top_translation = top_transform_config.get('translation', {})
        top_orientation = top_transform_config.get('orientation', {})
        
        top_translation_vec = np.array([
            top_translation.get('x', 0.0),
            top_translation.get('y', 0.0),
            top_translation.get('z', 0.05)
        ]).reshape(3, 1)
        
        top_rotation_matrix = Rotation.from_euler('xyz', [
            top_orientation.get('roll', 0.0),
            top_orientation.get('pitch', 0.0),
            top_orientation.get('yaw', 0.0)
        ]).as_matrix()
        
        self.tag_transforms[self.robot_tags['top_id']] = (top_rotation_matrix, top_translation_vec)
        
        # Bottom tag transform
        bottom_transform_config = robot_tags_config.get('bottom_tag_to_base_transform', {})
        bottom_translation = bottom_transform_config.get('translation', {})
        bottom_orientation = bottom_transform_config.get('orientation', {})
        
        bottom_translation_vec = np.array([
            bottom_translation.get('x', 0.0),
            bottom_translation.get('y', 0.0),
            bottom_translation.get('z', 0.05)
        ]).reshape(3, 1)
        
        bottom_rotation_matrix = Rotation.from_euler('xyz', [
            bottom_orientation.get('roll', 0.0),
            bottom_orientation.get('pitch', 0.0),
            bottom_orientation.get('yaw', 0.0)
        ]).as_matrix()
        
        self.tag_transforms[self.robot_tags['bottom_id']] = (bottom_rotation_matrix, bottom_translation_vec)
        
        # Detection settings
        detection_config = self.config.get('detection', {})
        self.debug_image = detection_config.get('debug_image', True)
        
        # Robot outline
        outline_points = detection_config.get('outline_points', [
            [0.0, -0.52, 0.0],
            [0.26, 0.0, 0.0],
            [0.26, 0.13, 0.0],
            [-0.26, 0.13, 0.0],
            [-0.26, 0.0, 0.0]
        ])
        self.robot_outline = np.array(outline_points)
        
        # Frame names
        frames_config = self.config.get('frames', {})
        self.frames = {
            'robot_base': frames_config.get('robot_base', 'robot_base'),
            'camera_optical': frames_config.get('camera_optical', 'arena_camera_optical'),
            'world': frames_config.get('world', 'world'),
            'arena_tag': frames_config.get('arena_tag', 'arena_tag')
        }
        
        # Topics
        topics_config = self.config.get('topics', {})
        self.topics = {
            'robot_pose': topics_config.get('robot_pose', '/robot/pose'),
            'detection_overlay': topics_config.get('detection_overlay', '/arena_camera/image_robot_detection_debug')
        }
        
        # Log configuration
        self.get_logger().info(f"Robot tags: top_id={self.robot_tags['top_id']}, bottom_id={self.robot_tags['bottom_id']}")
        self.get_logger().info(f"Debug image: {self.debug_image}")
        self.get_logger().info(f"Robot outline points: {len(self.robot_outline)}")
        
    def publish_latest_tf(self):
        """Publish latest robot_base transform at fixed rate"""
        if self.latest_robot_base_pose is not None:
            self.publish_bot_tf(self.latest_robot_base_pose, self.frames['robot_base'])
        
        # Also publish debug TF for robot_tag
        if self.latest_camera_from_tag is not None and self.latest_tag_id is not None:
            # We need world_from_tag for TF, not camera_from_tag
            camera_from_world = self.get_camera_from_world()
            if camera_from_world is not None:
                # Transform multiplication: world_from_tag = camera_from_world * camera_from_tag
                camera_from_world_R, camera_from_world_t = camera_from_world
                camera_from_tag_R, camera_from_tag_t = self.latest_camera_from_tag
                
                # world_from_tag_R = camera_from_world_R @ camera_from_tag_R
                world_from_tag_R = camera_from_world_R @ camera_from_tag_R
                # world_from_tag_t = camera_from_world_R @ camera_from_tag_t + camera_from_world_t
                world_from_tag_t = camera_from_world_R @ camera_from_tag_t + camera_from_world_t
                
                world_from_tag = (world_from_tag_R, world_from_tag_t)
                self.publish_bot_tf(world_from_tag, f'debug_robot_tag_{self.latest_tag_id}')
        
    def camera_info_callback(self, msg):
        """Initialize detector when camera info is received"""
        if self.april_detector is None:
            # Extract the projection matrix P (3x4) - this is for rectified images
            self.projection_matrix = np.array(msg.p).reshape(3, 4)
            
            # For rectified images (image_rect), we should use the projection matrix P
            # The left 3x3 part of P is the intrinsic matrix for the rectified image
            K_rect = self.projection_matrix[:, :3]
            
            # Extract camera intrinsics from the rectified camera matrix
            fx = K_rect[0, 0]
            fy = K_rect[1, 1]
            cx = K_rect[0, 2]
            cy = K_rect[1, 2]
            
            self.camera_intrinsics = np.array([fx, fy, cx, cy])
            
            # Initialize AprilTag detector directly
            self.april_detector = pyapriltags.Detector(families=self.robot_tags['family'])
            
            self.get_logger().info("AprilTag detector initialized with projection matrix P")
            self.get_logger().info(f"Rectified camera intrinsics from P: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        
    def get_camera_from_world(self):
        """Get camera_from_world transform from TF2 by manually chaining transforms"""
        try:
            # First get world -> arena_apriltag transform
            transform_world_to_apriltag = self.tf_buffer.lookup_transform(
                self.frames['world'], self.frames['arena_tag'], rclpy.time.Time()
            )
            
            # Then get arena_apriltag -> arena_camera_optical transform
            transform_apriltag_to_camera = self.tf_buffer.lookup_transform(
                self.frames['arena_tag'], self.frames['camera_optical'], rclpy.time.Time()
            )
            
            # Convert world -> arena_apriltag to (R, t) tuple
            translation_world_to_apriltag = np.array([
                transform_world_to_apriltag.transform.translation.x,
                transform_world_to_apriltag.transform.translation.y, 
                transform_world_to_apriltag.transform.translation.z
            ]).reshape(3, 1)
            
            quaternion_world_to_apriltag = np.array([
                transform_world_to_apriltag.transform.rotation.x,
                transform_world_to_apriltag.transform.rotation.y,
                transform_world_to_apriltag.transform.rotation.z, 
                transform_world_to_apriltag.transform.rotation.w
            ])
            
            world_from_apriltag_R = Rotation.from_quat(quaternion_world_to_apriltag).as_matrix()
            world_from_apriltag = (world_from_apriltag_R, translation_world_to_apriltag)
            
            # Convert arena_apriltag -> arena_camera_optical to (R, t) tuple
            translation_apriltag_to_camera = np.array([
                transform_apriltag_to_camera.transform.translation.x,
                transform_apriltag_to_camera.transform.translation.y, 
                transform_apriltag_to_camera.transform.translation.z
            ]).reshape(3, 1)
            
            quaternion_apriltag_to_camera = np.array([
                transform_apriltag_to_camera.transform.rotation.x,
                transform_apriltag_to_camera.transform.rotation.y,
                transform_apriltag_to_camera.transform.rotation.z, 
                transform_apriltag_to_camera.transform.rotation.w
            ])
            
            apriltag_from_camera_R = Rotation.from_quat(quaternion_apriltag_to_camera).as_matrix()
            apriltag_from_camera = (apriltag_from_camera_R, translation_apriltag_to_camera)
            
            # Chain the transforms: world_from_camera = world_from_apriltag * apriltag_from_camera
            world_from_apriltag_R, world_from_apriltag_t = world_from_apriltag
            apriltag_from_camera_R, apriltag_from_camera_t = apriltag_from_camera
            
            # world_from_camera_R = world_from_apriltag_R @ apriltag_from_camera_R
            world_from_camera_R = world_from_apriltag_R @ apriltag_from_camera_R
            # world_from_camera_t = world_from_apriltag_R @ apriltag_from_camera_t + world_from_apriltag_t
            world_from_camera_t = world_from_apriltag_R @ apriltag_from_camera_t + world_from_apriltag_t
            
            world_from_camera = (world_from_camera_R, world_from_camera_t)
            
            # Return the inverse: camera_from_world
            # camera_from_world_R = world_from_camera_R.T
            camera_from_world_R = world_from_camera_R.T
            # camera_from_world_t = -camera_from_world_R @ world_from_camera_t
            camera_from_world_t = -camera_from_world_R @ world_from_camera_t
            
            return (camera_from_world_R, camera_from_world_t)
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}", throttle_duration_sec=5.0)
            return None
    
    def project_3d_to_2d(self, point_3d):
        """Project 3D point to 2D image coordinates using rectified camera matrix from P"""
        if self.projection_matrix is None:
            return None
            
        # Use the rectified camera matrix from projection matrix
        K_rect = self.projection_matrix[:, :3]
        
        # For monocular camera, projection is just K * [X, Y, Z]
        point_homogeneous = np.array([point_3d[0], point_3d[1], point_3d[2], 1.0])
        image_point_homogeneous = K_rect @ point_homogeneous[:3]
        
        if abs(image_point_homogeneous[2]) < 1e-6:
            return None
            
        # Convert from homogeneous coordinates
        u = image_point_homogeneous[0] / image_point_homogeneous[2]
        v = image_point_homogeneous[1] / image_point_homogeneous[2]
        
        return np.array([u, v])
        
    def calculate_robot_outline(self, camera_from_robot_base):
        """Calculate robot outline projection based on robot_base"""
        outline_2d = []
        camera_from_robot_base_R, camera_from_robot_base_t = camera_from_robot_base
        
        for point in self.robot_outline:
            # Transform point from robot_base to camera directly
            # point_in_camera = R * point + t
            point_3d = camera_from_robot_base_R @ point.reshape(3, 1) + camera_from_robot_base_t
            outline_2d.append(self.project_3d_to_2d(point_3d.flatten()))
        return outline_2d
        
    def detect_robot(self, image_gray, camera_from_world):
        """Core robot detection - returns robot_base pose in world coordinates and camera_from_tag for debug"""
        if self.april_detector is None:
            return None, None, None
            
        # Direct AprilTag detection using the rectified camera intrinsics
        detections = self.april_detector.detect(
            image_gray,
            estimate_tag_pose=True,
            camera_params=self.camera_intrinsics,
            tag_size=self.robot_tags['size']
        )
        
        for detection in detections:
            if (detection.tag_family.decode() == self.robot_tags['family'] and
                detection.tag_id in [self.robot_tags['top_id'], self.robot_tags['bottom_id']]):
                
                tag_id = detection.tag_id
                
                # Get camera_from_robot_tag from detection (store as tuple)
                camera_from_robot_tag_R = np.array(detection.pose_R)
                camera_from_robot_tag_t = np.array(detection.pose_t).reshape(3, 1)
                camera_from_robot_tag = (camera_from_robot_tag_R, camera_from_robot_tag_t)
                
                # Convert to world_from_robot_tag
                camera_from_world_R, camera_from_world_t = camera_from_world
                
                # First compute world_from_robot_tag = camera_from_world * camera_from_robot_tag
                # Need to invert camera_from_world to get world_from_camera
                # world_from_camera_R = camera_from_world_R.T
                world_from_camera_R = camera_from_world_R.T
                # world_from_camera_t = -world_from_camera_R @ camera_from_world_t
                world_from_camera_t = -world_from_camera_R @ camera_from_world_t
                
                # world_from_robot_tag_R = world_from_camera_R @ camera_from_robot_tag_R
                world_from_robot_tag_R = world_from_camera_R @ camera_from_robot_tag_R
                # world_from_robot_tag_t = world_from_camera_R @ camera_from_robot_tag_t + world_from_camera_t
                world_from_robot_tag_t = world_from_camera_R @ camera_from_robot_tag_t + world_from_camera_t
                world_from_robot_tag = (world_from_robot_tag_R, world_from_robot_tag_t)
                
                # Convert to world_from_robot_base using the appropriate tag transform
                robot_tag_to_base_R, robot_tag_to_base_t = self.tag_transforms[tag_id]
                
                # world_from_robot_base = world_from_robot_tag * robot_tag_to_base
                world_from_robot_base_R = world_from_robot_tag_R @ robot_tag_to_base_R
                world_from_robot_base_t = world_from_robot_tag_R @ robot_tag_to_base_t + world_from_robot_tag_t
                world_from_robot_base = (world_from_robot_base_R, world_from_robot_base_t)
                
                # Return both poses for debug along with tag_id
                return world_from_robot_base, camera_from_robot_tag, tag_id
        
        return None, None, None
    
    def image_callback(self, msg):
        """Process each camera frame"""
        camera_from_world = self.get_camera_from_world()
        if camera_from_world is None:
            return
            
        if self.april_detector is None:
            self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=5.0)
            return
            
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Core detection - now returns both base and tag poses
            world_pose_base, camera_pose_tag, tag_id = self.detect_robot(gray, camera_from_world)
            
            if world_pose_base is not None and camera_pose_tag is not None and tag_id is not None:
                # Store for TF publishing
                self.latest_robot_base_pose = world_pose_base
                self.latest_camera_from_tag = camera_pose_tag
                self.latest_tag_id = tag_id
                
                # Publish debug poses
                self.publish_debug_poses(camera_pose_tag, world_pose_base, camera_from_world, tag_id)
                
                # Only publish overlay if debug images are enabled
                if self.debug_image:
                    # Calculate camera_from_robot_base for overlay
                    # camera_from_robot_base = camera_from_world * world_pose_base
                    camera_from_world_R, camera_from_world_t = camera_from_world
                    world_pose_base_R, world_pose_base_t = world_pose_base
                    
                    # First need world_from_camera from camera_from_world
                    # world_from_camera_R = camera_from_world_R.T
                    world_from_camera_R = camera_from_world_R.T
                    # world_from_camera_t = -world_from_camera_R @ camera_from_world_t
                    world_from_camera_t = -world_from_camera_R @ camera_from_world_t
                    
                    # camera_from_robot_base = (world_from_camera * world_from_robot_base)^-1
                    # First compute world_from_camera * world_from_robot_base
                    temp_R = world_from_camera_R @ world_pose_base_R
                    temp_t = world_from_camera_R @ world_pose_base_t + world_from_camera_t
                    
                    # Then invert to get camera_from_robot_base
                    camera_from_robot_base_R = temp_R.T
                    camera_from_robot_base_t = -camera_from_robot_base_R @ temp_t
                    camera_from_robot_base = (camera_from_robot_base_R, camera_from_robot_base_t)
                    
                    self.publish_overlay(cv_image, camera_from_robot_base, camera_pose_tag, tag_id)
                
                # Publish robot_base pose
                self.publish_pose(world_pose_base)
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def publish_debug_poses(self, camera_pose_tag, world_pose_base, camera_from_world, tag_id):
        """Publish debug poses for visualization"""
        # Publish camera_from_tag pose (in camera frame)
        tag_pose_msg = PoseStamped()
        tag_pose_msg.header.frame_id = self.frames['camera_optical']
        tag_pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Extract from tuple
        camera_pose_tag_R, camera_pose_tag_t = camera_pose_tag
        
        # Position of tag in camera frame
        tag_pose_msg.pose.position.x = float(camera_pose_tag_t[0])
        tag_pose_msg.pose.position.y = float(camera_pose_tag_t[1])
        tag_pose_msg.pose.position.z = float(camera_pose_tag_t[2])
        
        # Orientation of tag in camera frame
        rotation_tag = Rotation.from_matrix(camera_pose_tag_R)
        quat_tag = rotation_tag.as_quat()
        tag_pose_msg.pose.orientation.x = float(quat_tag[0])
        tag_pose_msg.pose.orientation.y = float(quat_tag[1])
        tag_pose_msg.pose.orientation.z = float(quat_tag[2])
        tag_pose_msg.pose.orientation.w = float(quat_tag[3])
        
        self.tag_pose_pub.publish(tag_pose_msg)
        
        # Log debug information
        tag_pos = camera_pose_tag_t.flatten()
        base_pos = world_pose_base[1].flatten()  # world_pose_base is (R, t) tuple
        
        self.get_logger().debug(
            f"Tag {tag_id} in camera frame: "
            f"pos=[{float(tag_pos[0]):.3f}, {float(tag_pos[1]):.3f}, {float(tag_pos[2]):.3f}], "
            f"Base in world: "
            f"pos=[{float(base_pos[0]):.3f}, {float(base_pos[1]):.3f}, {float(base_pos[2]):.3f}]",
            throttle_duration_sec=2.0
        )
    
    def publish_overlay(self, cv_image, camera_from_robot_base, camera_pose_tag, tag_id):
        """Publish visualization overlay with just the outline based on robot_base"""
        if not self.debug_image or self.overlay_pub is None:
            return
            
        overlay = cv_image.copy()
        
        # Draw robot outline based on robot_base
        outline_2d = self.calculate_robot_outline(camera_from_robot_base)
        if outline_2d and all(p is not None for p in outline_2d):
            for i in range(len(outline_2d)):
                pt1 = tuple(outline_2d[i].astype(int))
                pt2 = tuple(outline_2d[(i + 1) % len(outline_2d)].astype(int))
                cv2.line(overlay, pt1, pt2, (0, 255, 0), 2)
        
        # Draw tag position (project tag center to image)
        tag_center_2d = self.project_3d_to_2d(camera_pose_tag[1].flatten())
        if tag_center_2d is not None and not np.isnan(tag_center_2d).any():
            center = tuple(tag_center_2d.astype(int))
            cv2.circle(overlay, center, 5, (0, 0, 255), -1)  # Red circle at tag center
            cv2.putText(overlay, f"Tag {tag_id}", (center[0]+10, center[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Draw base position (project base center to image)
        base_center_2d = self.project_3d_to_2d(camera_from_robot_base[1].flatten())
        if base_center_2d is not None and not np.isnan(base_center_2d).any():
            center = tuple(base_center_2d.astype(int))
            cv2.circle(overlay, center, 5, (255, 0, 0), -1)  # Blue circle at base center
            cv2.putText(overlay, "Base", (center[0]+10, center[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Publish overlay
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
        overlay_msg.header.stamp = self.get_clock().now().to_msg()
        self.overlay_pub.publish(overlay_msg)
    
    def publish_pose(self, world_pose_base):
        """Publish robot_base pose as message"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.frames['world']
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Extract from tuple
        world_pose_base_R, world_pose_base_t = world_pose_base
        
        # Position of robot_base
        pose_msg.pose.position.x = float(world_pose_base_t[0])
        pose_msg.pose.position.y = float(world_pose_base_t[1])
        pose_msg.pose.position.z = float(world_pose_base_t[2])
        
        # Orientation of robot_base
        rotation = Rotation.from_matrix(world_pose_base_R)
        quat = rotation.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        self.pose_pub.publish(pose_msg)
        
        # Log position
        base_pos = world_pose_base_t.flatten()
        self.get_logger().info(
            f"Robot BASE pose: [{float(base_pos[0]):.2f}, {float(base_pos[1]):.2f}, {float(base_pos[2]):.2f}]",
            throttle_duration_sec=2.0
        )
    
    def publish_bot_tf(self, world_pose, frame_name):
        """Publish robot transform to TF tree"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frames['world']
        t.child_frame_id = frame_name
        
        # Extract from tuple
        world_pose_R, world_pose_t = world_pose
        
        # Position
        t.transform.translation.x = float(world_pose_t[0])
        t.transform.translation.y = float(world_pose_t[1])
        t.transform.translation.z = float(world_pose_t[2])
        
        # Orientation
        rotation = Rotation.from_matrix(world_pose_R)
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
        node = BotDetectionNode()
        rclpy.spin(node)
    except ValueError as e:
        if node:
            node.get_logger().error(f"Configuration error: {e}")
        else:
            print(f"Configuration error: {e}")
        return 1
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Node failed: {e}")
        else:
            print(f"Node failed: {e}")
        return 1
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()