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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ApriltagDetectionNode(Node):
    def __init__(self):
        super().__init__('apriltag_detection_node')
        
        # Use QoS settings for better performance - prevents backpressure
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Only keep the latest message
        )
        
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
        
        # Subscribers with QoS profile
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile=qos_profile)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile=qos_profile)
        self.bridge = CvBridge()
        
        # TF2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Debug image publisher (if enabled)
        if self.debug_enabled:
            self.debug_pub = self.create_publisher(
                Image, self.debug_image_topic, qos_profile=qos_profile)
        
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
        # Topics
        self.camera_info_topic = self.config.get('camera_info_topic', 'camera/camera_info')
        self.image_topic = self.config.get('input_image_topic', 'camera/image_rect')
        
        # Camera frame
        self.camera_frame = self.config.get('camera_frame', 'camera_optical')
        
        # Debug configuration
        self.debug_enabled = self.config.get('debug_image', False)
        self.debug_image_topic = self.config.get('debug_image_topic', 'apriltag_detection/debug')
        
        # Tag configuration
        tag_config = self.config.get('tag', {})
        
        # Get tag IDs, frames, and sizes
        self.tag_ids = tag_config.get('ids', [])
        self.tag_frames = tag_config.get('frames', [])
        self.tag_sizes = tag_config.get('sizes', [])
        
        # Validate configuration
        if not self.tag_ids:
            raise ValueError("No tag IDs specified in configuration")
        
        if len(self.tag_frames) != len(self.tag_ids):
            raise ValueError(
                f"Number of frames ({len(self.tag_frames)}) doesn't match number of tags ({len(self.tag_ids)}). "
                f"Please specify exactly {len(self.tag_ids)} frame names."
            )
    
        if len(self.tag_sizes) != len(self.tag_ids):
            raise ValueError(
                f"Number of sizes ({len(self.tag_sizes)}) doesn't match number of tags ({len(self.tag_ids)}). "
                f"Please specify exactly {len(self.tag_ids)} sizes."
            )
        
        # Create a mapping from tag ID to frame name and size
        self.tag_mapping = {}
        for i, tag_id in enumerate(self.tag_ids):
            self.tag_mapping[tag_id] = {
                'frame': self.tag_frames[i],
                'size': self.tag_sizes[i]
            }
        
        # Tag family (default to standard)
        self.tag_family = tag_config.get('family', 'tag36h11')
        
        # Log configuration
        self.get_logger().info(f"Topics: camera_info={self.camera_info_topic}, image={self.image_topic}")
        self.get_logger().info(f"Camera frame: {self.camera_frame}")
        self.get_logger().info(f"Tag family: {self.tag_family}")
        self.get_logger().info(f"Debug image: {'enabled' if self.debug_enabled else 'disabled'}")
        if self.debug_enabled:
            self.get_logger().info(f"Debug image topic: {self.debug_image_topic}")
        self.get_logger().info(f"Configured {len(self.tag_ids)} tags:")
        for i, tag_id in enumerate(self.tag_ids):
            self.get_logger().info(f"  ID {tag_id}: frame='{self.tag_frames[i]}', size={self.tag_sizes[i]}m")
        
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
        
            # Initialize AprilTag detector
            self.april_detector = pyapriltags.Detector(families=self.tag_family)
            
            # Store intrinsics for AprilTag detector
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
            
            # Create a copy for debug visualization if enabled
            if self.debug_enabled:
                debug_frame = self.latest_frame.copy()
            else:
                debug_frame = None
            
            # Core detection
            transforms_to_send = self.detect_tags(gray, debug_frame)
            
            # Publish debug image if enabled and frame was modified
            if self.debug_enabled and debug_frame is not None:
                self.publish_debug_image(debug_frame)
            
            # Send transforms if any tags were found
            if transforms_to_send:
                if not self.has_published_once:
                    self.get_logger().info("Started publishing tag transforms")
                    self.has_published_once = True
                
                self.tf_broadcaster.sendTransform(transforms_to_send)
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def detect_tags(self, image_gray, debug_frame=None):
        """Detect tags and publish their transforms relative to camera_optical"""
        
        transforms_to_send = []
        
        for tag_id, tag_info in self.tag_mapping.items():
            # Detect this specific tag
            detections = self.april_detector.detect(
                image_gray,
                estimate_tag_pose=True,
                camera_params=self.april_intrinsics,
                tag_size=tag_info['size']
            )
            
            for detection in detections:
                if (detection.tag_family.decode() == self.tag_family and
                    detection.tag_id == tag_id):
                    
                    # AprilTag gives us: tag pose in camera optical frame (OpenCV)
                    # This is T_tag_camera_optical: transform FROM camera_optical TO tag
                    tag_from_camera_R = np.array(detection.pose_R)
                    tag_from_camera_t = np.array(detection.pose_t).reshape(3, 1)
                    
                    # Create transform from camera_optical to tag
                    tag_transform = TransformStamped()
                    tag_transform.header.stamp = self.get_clock().now().to_msg()
                    tag_transform.header.frame_id = self.camera_frame
                    tag_transform.child_frame_id = tag_info['frame']
                    
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
                    
                    # Debug visualization (if enabled)
                    if debug_frame is not None:
                        self.draw_tag_detection(debug_frame, detection, tag_id)
        
        # Log if no tags were found
        if not transforms_to_send:
            self.get_logger().info("No configured tags found in frame", throttle_duration_sec=2.0)
        
        return transforms_to_send
    
    def draw_tag_detection(self, frame, detection, tag_id):
        """Draw tag detection on frame: red dot in center and bounding box"""
        # Convert corners to integer for drawing
        corners = detection.corners.astype(int)

        # Draw bounding box (square) - red color
        # Points are in order: bottom-left, bottom-right, top-right, top-left
        cv2.polylines(frame, [corners], isClosed=True, color=(0, 0, 255), thickness=2)

        # Calculate center point (average of all corners)
        center = np.center.astype(int)

        # Draw red dot in the center
        cv2.circle(frame, tuple(center), radius=4, color=(0, 0, 255), thickness=-1)  # -1 for filled circle

        # Draw tag ID near the center
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        text = f"ID: {tag_id}"

        # Calculate text size for background rectangle
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, 1)

        # Draw background rectangle for text
        text_x = center[0] - text_width // 2
        text_y = center[1] + text_height + 20  # Position below the red dot
        cv2.rectangle(frame, 
                        (text_x - 2, text_y - text_height - 2),
                        (text_x + text_width + 2, text_y + 2),
                        (0, 0, 255), -1)  # Red background

        # Draw text in white
        cv2.putText(frame, text, 
                    (text_x, text_y), 
                    font, font_scale, (255, 255, 255), 1, cv2.LINE_AA)

        # Optional: Draw small coordinate axes on the tag
        # This helps visualize tag orientation
        axis_length = 20

        # Use detection center (which is already 2D image coordinates)
        center_pt = detection.center.astype(int)

        # Get rotation matrix and convert to proper 2D vectors
        # We need to project the 3D axes onto the image plane
        # For simplicity, we can draw using the corners to estimate orientation

        # Draw x-axis (red) - from center to midpoint of right side
        right_mid = ((corners[1] + corners[2]) / 2).astype(int)
        cv2.arrowedLine(frame, tuple(center_pt), tuple(right_mid), 
                        (0, 0, 255), 2, tipLength=0.3)

        # Draw y-axis (green) - from center to midpoint of bottom side  
        bottom_mid = ((corners[0] + corners[1]) / 2).astype(int)
        cv2.arrowedLine(frame, tuple(center_pt), tuple(bottom_mid), 
                        (0, 255, 0), 2, tipLength=0.3)
    
    def publish_debug_image(self, frame):
        """Publish debug image with tag detection visualization"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = self.camera_frame
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ApriltagDetectionNode()
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