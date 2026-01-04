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
from arena_perception.detector import AprilTagDetector
from arena_perception.math_funcs import Pose as MathPose

class BotDetectionNode(Node):
    def __init__(self):
        super().__init__('robot_detection_node')
        
        # Declare parameters with descriptions
        self.declare_parameter('debug_image', False, 
                              descriptor=ParameterDescriptor(  # Use ParameterDescriptor directly
                                  description='Enable debug image publishing'))
        
        self.declare_parameter('robot_tag_to_base_translation', [0.0, 0.0, 0.05],
                              descriptor=ParameterDescriptor(  # Use ParameterDescriptor directly
                                  description='Translation from robot_tag to robot_base [x, y, z] in meters'))
        
        self.declare_parameter('robot_tag_to_base_rotation', [0.0, 0.0, 0.0, 1.0],
                              descriptor=ParameterDescriptor(  # Use ParameterDescriptor directly
                                  description='Quaternion rotation from robot_tag to robot_base [x, y, z, w]'))
        
        # Get parameters
        self.debug_image = self.get_parameter('debug_image').get_parameter_value().bool_value
        
        translation_param = self.get_parameter('robot_tag_to_base_translation').get_parameter_value().double_array_value
        rotation_param = self.get_parameter('robot_tag_to_base_rotation').get_parameter_value().double_array_value
        
        # Validate and create transform
        if len(translation_param) != 3:
            self.get_logger().error("Invalid translation parameter, using default [0,0,0]")
            translation_param = [0.0, 0.0, 0.0]
        
        if len(rotation_param) != 4:
            self.get_logger().error("Invalid rotation parameter, using default identity")
            rotation_param = [0.0, 0.0, 0.0, 1.0]
        
        # Create robot_tag to robot_base transform from parameters
        translation = np.array(translation_param)
        rotation_matrix = Rotation.from_quat(rotation_param).as_matrix()
        self.robot_tag_to_base = MathPose(rotation_matrix, translation)
        
        self.get_logger().info(f"Robot tag to base transform: translation={translation.tolist()}")
        
        # Configuration
        self.robot_tags = {
            'sizes': 0.0778, 
            'family': 'tagStandard41h12', 
            'top_id': 12, 
            'bottom_id': 31
        }
        self.robot_outline = 0.13 * np.array([
            [0, -4, 0], [2, 0, 0], [2, 1, 0], [-2, 1, 0], [-2, 0, 0]
        ])
        
        # QoS profiles
        image_qos = rclpy.qos.qos_profile_sensor_data
        info_qos = rclpy.qos.QoSProfile(depth=10)
        
        # Subscribers with appropriate QoS
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, image_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, info_qos)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/bot/pose', 10)
        
        # Debug publishers
        self.tag_pose_pub = self.create_publisher(PoseStamped, '/debug/tag_pose', 10)
        
        # Only create overlay publisher if debug images are enabled
        if self.debug_image:
            self.overlay_pub = self.create_publisher(Image, '/detection/robot_overlay', 10)
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
        self.camera_matrix = None
        
        # Timer for transform publishing
        self.tf_timer = self.create_timer(0.1, self.publish_latest_tf)  # 10Hz
        self.latest_robot_base_pose = None
        self.latest_camera_from_tag = None  # For debug TF publishing
        
    def publish_latest_tf(self):
        """Publish latest robot_base transform at fixed rate"""
        if self.latest_robot_base_pose is not None:
            self.publish_bot_tf(self.latest_robot_base_pose, 'robot_base')
        
        # Also publish debug TF for robot_tag
        if self.latest_camera_from_tag is not None:
            # We need world_from_tag for TF, not camera_from_tag
            camera_from_world = self.get_camera_from_world()
            if camera_from_world is not None:
                world_from_tag = camera_from_world * self.latest_camera_from_tag
                self.publish_bot_tf(world_from_tag, 'debug_robot_tag')
        
    def camera_info_callback(self, msg):
        """Initialize detector when camera info is received"""
        if self.april_detector is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            calibration_data = {
                'camera_matrix': self.camera_matrix,
                'distortion_coefficients': np.array(msg.d)
            }
            self.april_detector = AprilTagDetector(calibration_data)
            self.get_logger().info("AprilTag detector initialized")
        
    def get_camera_from_world(self):
        """Get camera_from_world transform from TF2 by manually chaining transforms"""
        try:
            # First get world -> arena_apriltag transform
            transform_world_to_apriltag = self.tf_buffer.lookup_transform(
                'world', 'arena_apriltag', rclpy.time.Time()
            )
            
            # Then get arena_apriltag -> arena_camera_optical transform
            transform_apriltag_to_camera = self.tf_buffer.lookup_transform(
                'arena_apriltag', 'arena_camera_optical', rclpy.time.Time()
            )
            
            # Convert world -> arena_apriltag to MathPose
            translation_world_to_apriltag = np.array([
                transform_world_to_apriltag.transform.translation.x,
                transform_world_to_apriltag.transform.translation.y, 
                transform_world_to_apriltag.transform.translation.z
            ])
            
            quaternion_world_to_apriltag = np.array([
                transform_world_to_apriltag.transform.rotation.x,
                transform_world_to_apriltag.transform.rotation.y,
                transform_world_to_apriltag.transform.rotation.z, 
                transform_world_to_apriltag.transform.rotation.w
            ])
            
            world_from_apriltag = MathPose(
                Rotation.from_quat(quaternion_world_to_apriltag).as_matrix(),
                translation_world_to_apriltag
            )
            
            # Convert arena_apriltag -> arena_camera_optical to MathPose
            translation_apriltag_to_camera = np.array([
                transform_apriltag_to_camera.transform.translation.x,
                transform_apriltag_to_camera.transform.translation.y, 
                transform_apriltag_to_camera.transform.translation.z
            ])
            
            quaternion_apriltag_to_camera = np.array([
                transform_apriltag_to_camera.transform.rotation.x,
                transform_apriltag_to_camera.transform.rotation.y,
                transform_apriltag_to_camera.transform.rotation.z, 
                transform_apriltag_to_camera.transform.rotation.w
            ])
            
            apriltag_from_camera = MathPose(
                Rotation.from_quat(quaternion_apriltag_to_camera).as_matrix(),
                translation_apriltag_to_camera
            )
            
            # Chain the transforms: world_from_camera = world_from_apriltag * apriltag_from_camera
            world_from_camera = world_from_apriltag * apriltag_from_camera
            
            # Return the inverse: camera_from_world
            return world_from_camera.inv()
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}", throttle_duration_sec=5.0)
            return None
    
    def project_3d_to_2d(self, point_3d):
        """Project 3D point to 2D image coordinates"""
        if self.camera_matrix is None:
            return None
            
        point_2d, _ = cv2.projectPoints(
            point_3d, np.eye(3), np.zeros(3), self.camera_matrix, np.zeros(5)
        )
        return point_2d.squeeze()
    
    def calculate_robot_outline(self, camera_from_robot_base):
        """Calculate robot outline projection based on robot_base"""
        outline_2d = []
        for point in self.robot_outline:
            # Transform point from robot_base to camera directly
            point_in_camera = camera_from_robot_base * MathPose(np.eye(3), point)
            outline_2d.append(self.project_3d_to_2d(point_in_camera.t))
        return outline_2d
        
    def detect_robot(self, image_gray, camera_from_world):
        """Core robot detection - returns robot_base pose in world coordinates and camera_from_tag for debug"""
        if self.april_detector is None:
            return None, None
            
        detections = self.april_detector.detect(image_gray, self.robot_tags['sizes'])['aprilTags']
        
        for detection in detections:
            if (detection.tag_family.decode() == self.robot_tags['family'] and
                detection.tag_id in [self.robot_tags['top_id'], self.robot_tags['bottom_id']]):
                
                # Get camera_from_robot_tag from detection
                camera_from_robot_tag = MathPose(detection.pose_R, detection.pose_t)
                
                # Convert to world_from_robot_tag
                world_from_robot_tag = camera_from_world * camera_from_robot_tag
                
                # Convert to world_from_robot_base using the static transform
                world_from_robot_base = world_from_robot_tag * self.robot_tag_to_base
                
                # Return both poses for debug
                return world_from_robot_base, camera_from_robot_tag
        
        return None, None
    
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
            world_pose_base, camera_pose_tag = self.detect_robot(gray, camera_from_world)
            
            if world_pose_base is not None and camera_pose_tag is not None:
                # Store for TF publishing
                self.latest_robot_base_pose = world_pose_base
                self.latest_camera_from_tag = camera_pose_tag
                
                # Publish debug poses
                self.publish_debug_poses(camera_pose_tag, world_pose_base, camera_from_world)
                
                # Only publish overlay if debug images are enabled
                if self.debug_image:
                    # Calculate camera_from_robot_base for overlay
                    camera_from_robot_base = camera_from_world * world_pose_base
                    self.publish_overlay(cv_image, camera_from_robot_base, camera_pose_tag)
                
                # Publish robot_base pose
                self.publish_pose(world_pose_base)
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def publish_debug_poses(self, camera_pose_tag, world_pose_base, camera_from_world):
        """Publish debug poses for visualization"""
        # Publish camera_from_tag pose (in camera frame)
        tag_pose_msg = PoseStamped()
        tag_pose_msg.header.frame_id = "arena_camera_optical"
        tag_pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position of tag in camera frame
        tag_pose_msg.pose.position.x = float(camera_pose_tag.t[0])
        tag_pose_msg.pose.position.y = float(camera_pose_tag.t[1])
        tag_pose_msg.pose.position.z = float(camera_pose_tag.t[2])
        
        # Orientation of tag in camera frame
        rotation_tag = Rotation.from_matrix(camera_pose_tag.R)
        quat_tag = rotation_tag.as_quat()
        tag_pose_msg.pose.orientation.x = float(quat_tag[0])
        tag_pose_msg.pose.orientation.y = float(quat_tag[1])
        tag_pose_msg.pose.orientation.z = float(quat_tag[2])
        tag_pose_msg.pose.orientation.w = float(quat_tag[3])
        
        self.tag_pose_pub.publish(tag_pose_msg)
        
        # Log debug information - FIXED: Convert numpy arrays to scalars before formatting
        tag_pos = camera_pose_tag.t
        base_pos = world_pose_base.t
        
        self.get_logger().debug(
            f"Tag in camera frame: "
            f"pos=[{float(tag_pos[0]):.3f}, {float(tag_pos[1]):.3f}, {float(tag_pos[2]):.3f}], "
            f"Base in world: "
            f"pos=[{float(base_pos[0]):.3f}, {float(base_pos[1]):.3f}, {float(base_pos[2]):.3f}]",
            throttle_duration_sec=2.0
        )
    
    def publish_overlay(self, cv_image, camera_from_robot_base, camera_pose_tag):
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
        tag_center_2d = self.project_3d_to_2d(camera_pose_tag.t)
        if tag_center_2d is not None and not np.isnan(tag_center_2d).any():
            center = tuple(tag_center_2d.astype(int))
            cv2.circle(overlay, center, 5, (0, 0, 255), -1)  # Red circle at tag center
            cv2.putText(overlay, "Tag", (center[0]+10, center[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Draw base position (project base center to image)
        base_center_2d = self.project_3d_to_2d(camera_from_robot_base.t)
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
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position of robot_base
        pose_msg.pose.position.x = float(world_pose_base.t[0])
        pose_msg.pose.position.y = float(world_pose_base.t[1])
        pose_msg.pose.position.z = float(world_pose_base.t[2])
        
        # Orientation of robot_base
        rotation = Rotation.from_matrix(world_pose_base.R)
        quat = rotation.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        self.pose_pub.publish(pose_msg)
        
        # FIXED: Convert numpy array elements to floats before formatting
        base_pos = world_pose_base.t
        self.get_logger().info(
            f"Robot BASE pose: [{float(base_pos[0]):.2f}, {float(base_pos[1]):.2f}, {float(base_pos[2]):.2f}]",
            throttle_duration_sec=2.0
        )
    
    def publish_bot_tf(self, world_pose, frame_name):
        """Publish robot transform to TF tree"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = frame_name
        
        # Position
        t.transform.translation.x = float(world_pose.t[0])
        t.transform.translation.y = float(world_pose.t[1])
        t.transform.translation.z = float(world_pose.t[2])
        
        # Orientation
        rotation = Rotation.from_matrix(world_pose.R)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = BotDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()