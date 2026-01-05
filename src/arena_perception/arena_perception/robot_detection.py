import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
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
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, 10)
        self.bridge = CvBridge()
        
        # TF2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State
        self.latest_frame = None
        self.camera_info = None
        self.april_detector = None
        self.april_intrinsics = None
        self.has_published_once = False
        
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
        
        # Log configuration
        self.get_logger().info(f"Robot tags: ID={self.robot_tags['top_id']}/{self.robot_tags['bottom_id']}, Family={self.robot_tags['family']}, Size={self.robot_tags['size']}")
        self.get_logger().info(f"Frame names: camera_optical={self.frames['camera_optical']}, top_tag={self.frames['robot_top_tag']}, bottom_tag={self.frames['robot_bottom_tag']}")
        
    def camera_info_callback(self, msg):
        """Receive camera calibration from camera_info topic"""
        
        if self.april_detector is None:
        
            self.camera_info = msg
            
            # Extract the projection matrix P (3x4) - this is for rectified images
            projection_matrix = np.array(self.camera_info.p).reshape(3, 4)
            
            # For rectified images (image_rect), we should use the projection matrix P
            # The left 3x3 part of P is the intrinsic matrix for the rectified image
            K_rect = projection_matrix[:, :3]
            
            # Extract camera intrinsics from the rectified camera matrix for AprilTag
            fx = K_rect[0, 0]
            fy = K_rect[1, 1]
            cx = K_rect[0, 2]
            cy = K_rect[1, 2]
        
            # Initialize AprilTag detector directly
            self.april_detector = pyapriltags.Detector(families=self.robot_tags['family'])
            
            # Store intrinsics only for AprilTag detector
            self.april_intrinsics = [fx, fy, cx, cy]
            
            self.get_logger().info("Camera info received. AprilTag detector initialized")
        
    def image_callback(self, msg):
        """Process each camera frame"""
        if self.april_detector is None:
            self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=5.0)
            return
            
        try:
            # Convert image
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
            
            # Core detection
            self.detect_tags(gray)
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def detect_tags(self, image_gray):
        """Detect tags and publish their transforms relative to camera_optical"""
        # Direct AprilTag detection using the rectified camera intrinsics
        detections = self.april_detector.detect(
            image_gray,
            estimate_tag_pose=True,
            camera_params=self.april_intrinsics,
            tag_size=self.robot_tags['size']
        )
        
        transforms_to_send = []
        
        for detection in detections:
            if (detection.tag_family.decode() == self.robot_tags['family'] and
                detection.tag_id in [self.robot_tags['top_id'], self.robot_tags['bottom_id']]):
                
                tag_id = detection.tag_id
                
                # AprilTag gives us: tag pose in camera optical frame (OpenCV)
                # This is T_tag_camera_optical: transform FROM camera_optical TO tag
                # Meaning: tag = R * camera + t
                # This is the EXACT transform we want for TF: camera_optical -> tag
                tag_from_camera_R = np.array(detection.pose_R)
                tag_from_camera_t = np.array(detection.pose_t).reshape(3, 1)
                
                # Create transform from camera_optical to tag
                tag_transform = TransformStamped()
                tag_transform.header.stamp = self.get_clock().now().to_msg()
                tag_transform.header.frame_id = self.frames['camera_optical']
                
                # Set child frame based on which tag was detected
                if tag_id == self.robot_tags['top_id']:
                    tag_transform.child_frame_id = self.frames['robot_top_tag']
                elif tag_id == self.robot_tags['bottom_id']:
                    tag_transform.child_frame_id = self.frames['robot_bottom_tag']
                else:
                    continue
                
                # Fill transform directly from AprilTag output
                # This is camera_optical -> tag transform
                tag_transform.transform.translation.x = float(tag_from_camera_t[0])
                tag_transform.transform.translation.y = float(tag_from_camera_t[1])
                tag_transform.transform.translation.z = float(tag_from_camera_t[2])
                
                # AprilTag uses OpenCV camera coordinates: +X right, +Y down, +Z forward
                # ROS uses: +X right, +Y down, +Z forward (for camera_optical frame)
                # So rotation matrix can be used directly
                rotation = Rotation.from_matrix(tag_from_camera_R)
                quat = rotation.as_quat()
                tag_transform.transform.rotation.x = float(quat[0])
                tag_transform.transform.rotation.y = float(quat[1])
                tag_transform.transform.rotation.z = float(quat[2])
                tag_transform.transform.rotation.w = float(quat[3])
                
                transforms_to_send.append(tag_transform)

        # Send all transforms if any tags were found
        if transforms_to_send:
            if not self.has_published_once:
                self.get_logger().info("Started publishing tag transforms")
                self.has_published_once = True
            
            self.tf_broadcaster.sendTransform(transforms_to_send)
        else:
            # Only log when nothing is detected
            self.get_logger().info("No robot tags found in frame", throttle_duration_sec=2.0)


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