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
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, self.topics['robot_pose'], 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, 10)
        self.bridge = CvBridge()
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State
        self.latest_frame = None
        self.camera_info = None
        self.projection_matrix = None  # 3x4 projection matrix P for rectified image
        self.world_from_robot_base = None  # Transform from world to robot_base (R, t) tuple - FOR POSE MESSAGE ONLY
        self.world_from_top_tag = None  # Transform from world to top tag (R, t) tuple
        self.world_from_bottom_tag = None  # Transform from world to bottom tag (R, t) tuple
        self.initial_detection_complete = False
        self.april_detector = None
        
        # Camera pose cache for fixed camera (compute once, reuse forever)
        self.cached_camera_from_world = None
        self.cache_valid = False
        
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
        
        # Tag position relative to robot base coordinate system
        self.tag_transforms = {}
        
        # Top tag transform
        top_config = robot_tags_config.get('top_tag_to_base_transform', {})
        top_translation = top_config.get('translation', {})
        top_orientation = top_config.get('orientation', {})
        
        top_translation_vec = np.array([
            top_translation.get('x', 0.0),
            top_translation.get('y', 0.0),
            top_translation.get('z', 0.05)  # Note: Changed from 0.0 to 0.05 to match YAML
        ]).reshape(3, 1)
        
        top_rotation_matrix = Rotation.from_euler('xyz', [
            top_orientation.get('roll', 0.0),
            top_orientation.get('pitch', 3.14159),  # Matching YAML
            top_orientation.get('yaw', 1.5708)      # Matching YAML
        ]).as_matrix()
        
        self.tag_transforms[self.robot_tags['top_id']] = (top_rotation_matrix, top_translation_vec)
        
        # Bottom tag transform
        bottom_config = robot_tags_config.get('bottom_tag_to_base_transform', {})
        bottom_translation = bottom_config.get('translation', {})
        bottom_orientation = bottom_config.get('orientation', {})
        
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
        
        # Robot outline for visualization
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
            'arena_tag': frames_config.get('arena_tag', 'arena_tag'),
            'robot_top_tag': frames_config.get('robot_top_tag', 'robot_top_tag'),
            'robot_bottom_tag': frames_config.get('robot_bottom_tag', 'robot_bottom_tag')
        }
        
        # Topics
        topics_config = self.config.get('topics', {})
        self.topics = {
            'robot_pose': topics_config.get('robot_pose', 'arena_perception/robot/pose'),
            'detection_overlay': topics_config.get('detection_overlay', '/arena_camera/image_robot_detection_debug')
        }
        
        # Log configuration
        self.get_logger().info(f"Robot tags: ID={self.robot_tags['top_id']}/{self.robot_tags['bottom_id']}, Family={self.robot_tags['family']}, Size={self.robot_tags['size']}")
        self.get_logger().info(f"Frame names: robot_base={self.frames['robot_base']}, top_tag={self.frames['robot_top_tag']}, bottom_tag={self.frames['robot_bottom_tag']}")
        
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
            self.april_detector = pyapriltags.Detector(families=self.robot_tags['family'])
            
            # Store intrinsics only for AprilTag detector
            self.april_intrinsics = [fx, fy, cx, cy]
            
            self.get_logger().info("Camera info received. AprilTag detector initialized")
            self.get_logger().info(f"Using projection matrix P for rectified image (image_rect)")
            self.get_logger().info(f"Rectified camera intrinsics from P: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
        
    def compute_camera_pose(self):
        """Compute and cache camera_from_world transform for fixed camera"""
        if self.cache_valid:
            return self.cached_camera_from_world
            
        try:
            # First get world -> arena_tag transform
            transform_world_to_tag = self.tf_buffer.lookup_transform(
                self.frames['world'], self.frames['arena_tag'], rclpy.time.Time())
            
            # Then get arena_tag -> camera_optical transform
            transform_tag_to_camera = self.tf_buffer.lookup_transform(
                self.frames['arena_tag'], self.frames['camera_optical'], rclpy.time.Time())
            
            # Helper to extract (R, t) tuple from TransformStamped
            def extract_transform(transform_msg):
                translation = np.array([
                    transform_msg.transform.translation.x,
                    transform_msg.transform.translation.y,
                    transform_msg.transform.translation.z
                ]).reshape(3, 1)
                
                quaternion = np.array([
                    transform_msg.transform.rotation.x,
                    transform_msg.transform.rotation.y,
                    transform_msg.transform.rotation.z,
                    transform_msg.transform.rotation.w
                ])
                
                rotation_matrix = Rotation.from_quat(quaternion).as_matrix()
                return (rotation_matrix, translation)
            
            # Compute world_from_camera = world_from_tag * tag_from_camera
            world_from_tag_R, world_from_tag_t = extract_transform(transform_world_to_tag)
            tag_from_camera_R, tag_from_camera_t = extract_transform(transform_tag_to_camera)
            
            world_from_camera_R = world_from_tag_R @ tag_from_camera_R
            world_from_camera_t = world_from_tag_R @ tag_from_camera_t + world_from_tag_t
            world_from_camera = (world_from_camera_R, world_from_camera_t)
            
            # Return the inverse: camera_from_world
            camera_from_world_R = world_from_camera_R.T
            camera_from_world_t = -camera_from_world_R @ world_from_camera_t
            camera_from_world = (camera_from_world_R, camera_from_world_t)
            
            # Cache the result
            self.cached_camera_from_world = camera_from_world
            self.cache_valid = True
            
            self.get_logger().info(f"Camera pose cached for fixed camera")
            return camera_from_world
            
        except Exception as e:
            self.get_logger().warn(f"Failed to compute camera pose: {str(e)}")
            return None
    
    def get_camera_from_world(self):
        """Get camera_from_world transform (cached for fixed camera)"""
        if not self.cache_valid:
            return self.compute_camera_pose()
        return self.cached_camera_from_world
    
    def image_callback(self, msg):
        """Process each camera frame"""
        camera_from_world = self.get_camera_from_world()
        if camera_from_world is None:
            self.get_logger().warn("Waiting for camera pose...", throttle_duration_sec=5.0)
            return
            
        if self.april_detector is None:
            self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=5.0)
            return
            
        try:
            # Convert image
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
            
            # Core detection
            success = self.detect_robot(gray, camera_from_world)
            
            if success:
                if not self.initial_detection_complete:
                    self.initial_detection_complete = True
                    self.get_logger().info("Initial robot detection successful")
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def detect_robot(self, image_gray, camera_from_world):
        """Detect robot and compute robot_base pose in world coordinates"""
        # Direct AprilTag detection using the rectified camera intrinsics
        detections = self.april_detector.detect(
            image_gray,
            estimate_tag_pose=True,
            camera_params=self.april_intrinsics,
            tag_size=self.robot_tags['size']
        )
        
        # Reset tag transforms since they may not be detected in this frame
        self.world_from_top_tag = None
        self.world_from_bottom_tag = None
        
        for detection in detections:
            if (detection.tag_family.decode() == self.robot_tags['family'] and
                detection.tag_id in [self.robot_tags['top_id'], self.robot_tags['bottom_id']]):
                
                tag_id = detection.tag_id
                
                # AprilTag gives us: tag pose in camera optical frame (OpenCV)
                # This is T_tag_camera_optical: transform FROM camera_optical TO tag
                tag_from_camera_optical_R = np.array(detection.pose_R)
                tag_from_camera_optical_t = np.array(detection.pose_t).reshape(3, 1)
                
                # Convert to world_from_robot_tag
                camera_from_world_R, camera_from_world_t = camera_from_world
                
                # First compute world_from_camera = camera_from_world^-1
                world_from_camera_R = camera_from_world_R.T
                world_from_camera_t = -world_from_camera_R @ camera_from_world_t
                
                # world_from_tag = world_from_camera * tag_from_camera_optical
                world_from_tag_R = world_from_camera_R @ tag_from_camera_optical_R
                world_from_tag_t = world_from_camera_R @ tag_from_camera_optical_t + world_from_camera_t
                
                # Store the tag transform in world frame
                if tag_id == self.robot_tags['top_id']:
                    self.world_from_top_tag = (world_from_tag_R, world_from_tag_t)
                elif tag_id == self.robot_tags['bottom_id']:
                    self.world_from_bottom_tag = (world_from_tag_R, world_from_tag_t)
                
                # Convert to world_from_robot_base using the appropriate tag transform
                robot_tag_to_base_R, robot_tag_to_base_t = self.tag_transforms[tag_id]
                
                # world_from_robot_base = world_from_tag * robot_tag_to_base
                world_from_robot_base_R = world_from_tag_R @ robot_tag_to_base_R
                world_from_robot_base_t = world_from_tag_R @ robot_tag_to_base_t + world_from_tag_t
                
                self.world_from_robot_base = (world_from_robot_base_R, world_from_robot_base_t)
                
                # Publish transforms and pose
                # DO NOT publish robot_base TF - let TF system compute it from tags
                self.publish_tag_tfs()
                self.publish_robot_pose()
                
                self.get_logger().info(f"Tag {tag_id} detected. Robot base position in world: [{world_from_robot_base_t[0][0]:.3f}, {world_from_robot_base_t[1][0]:.3f}, {world_from_robot_base_t[2][0]:.3f}] m")
                
                return True
        
        self.get_logger().debug("Robot not found in frame", throttle_duration_sec=2.0)
        return False
    
    def publish_tag_tfs(self):
        """Publish dynamic transforms for top and bottom tags in world frame"""
        current_time = self.get_clock().now().to_msg()
        transforms_to_send = []
        
        # Publish top tag transform if available
        if self.world_from_top_tag is not None:
            top_tag_transform = TransformStamped()
            top_tag_transform.header.stamp = current_time
            top_tag_transform.header.frame_id = self.frames['world']
            top_tag_transform.child_frame_id = self.frames['robot_top_tag']
            
            world_from_top_tag_R, world_from_top_tag_t = self.world_from_top_tag
            
            top_tag_transform.transform.translation.x = float(world_from_top_tag_t[0])
            top_tag_transform.transform.translation.y = float(world_from_top_tag_t[1])
            top_tag_transform.transform.translation.z = float(world_from_top_tag_t[2])
            
            rotation = Rotation.from_matrix(world_from_top_tag_R)
            quat = rotation.as_quat()
            top_tag_transform.transform.rotation.x = float(quat[0])
            top_tag_transform.transform.rotation.y = float(quat[1])
            top_tag_transform.transform.rotation.z = float(quat[2])
            top_tag_transform.transform.rotation.w = float(quat[3])
            
            transforms_to_send.append(top_tag_transform)
        
        # Publish bottom tag transform if available
        if self.world_from_bottom_tag is not None:
            bottom_tag_transform = TransformStamped()
            bottom_tag_transform.header.stamp = current_time
            bottom_tag_transform.header.frame_id = self.frames['world']
            bottom_tag_transform.child_frame_id = self.frames['robot_bottom_tag']
            
            world_from_bottom_tag_R, world_from_bottom_tag_t = self.world_from_bottom_tag
            
            bottom_tag_transform.transform.translation.x = float(world_from_bottom_tag_t[0])
            bottom_tag_transform.transform.translation.y = float(world_from_bottom_tag_t[1])
            bottom_tag_transform.transform.translation.z = float(world_from_bottom_tag_t[2])
            
            rotation = Rotation.from_matrix(world_from_bottom_tag_R)
            quat = rotation.as_quat()
            bottom_tag_transform.transform.rotation.x = float(quat[0])
            bottom_tag_transform.transform.rotation.y = float(quat[1])
            bottom_tag_transform.transform.rotation.z = float(quat[2])
            bottom_tag_transform.transform.rotation.w = float(quat[3])
            
            transforms_to_send.append(bottom_tag_transform)
        
        # Send all transforms if any are available
        if transforms_to_send:
            self.tf_broadcaster.sendTransform(transforms_to_send)
            self.get_logger().debug(f"Published dynamic tag transforms in world frame", throttle_duration_sec=5.0)
    
    def publish_robot_pose(self):
        """Publish robot_base pose as message ONLY - no TF"""
        if self.world_from_robot_base is None:
            return
            
        world_from_robot_base_R, world_from_robot_base_t = self.world_from_robot_base
        
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.frames['world']
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = float(world_from_robot_base_t[0])
        pose_msg.pose.position.y = float(world_from_robot_base_t[1])
        pose_msg.pose.position.z = float(world_from_robot_base_t[2])
        
        rotation = Rotation.from_matrix(world_from_robot_base_R)
        quat = rotation.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        self.pose_pub.publish(pose_msg)


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