import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation
from detection.detector import AprilTagDetector
from detection.math_funcs import Pose as MathPose

class BotDetectionNode(Node):
    def __init__(self):
        super().__init__('robot_detection_node')
        
        # Declare parameters with descriptions
        self.declare_parameter('debug_image', False, 
                              descriptor=rclpy.ParameterDescriptor(
                                  description='Enable debug image publishing'))
        
        self.declare_parameter('robot_tag_to_base_translation', [0.0, 0.0, 0.05],
                              descriptor=rclpy.ParameterDescriptor(
                                  description='Translation from robot_tag to robot_base [x, y, z] in meters'))
        
        self.declare_parameter('robot_tag_to_base_rotation', [0.0, 0.0, 0.0, 1.0],
                              descriptor=rclpy.ParameterDescriptor(
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
        
        self.get_logger().info(f"Robot tag to base transform: translation={translation}")
        
        # Configuration
        self.robot_tags = {
            'sizes': 0.125, 
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
        
    def publish_latest_tf(self):
        """Publish latest robot_base transform at fixed rate"""
        if self.latest_robot_base_pose is not None:
            self.publish_bot_tf(self.latest_robot_base_pose, 'robot_base')
        
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
        """Get camera_from_world transform from TF2"""
        try:
            transform = self.tf_buffer.lookup_transform('world', 'camera', rclpy.time.Time())
            
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y, 
                transform.transform.translation.z
            ])
            
            quaternion = np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z, 
                transform.transform.rotation.w
            ])
            
            # Convert world_from_camera to camera_from_world (inverse)
            world_from_camera = MathPose(Rotation.from_quat(quaternion).as_matrix(), translation)
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
        """Core robot detection - returns robot_base pose in world coordinates"""
        if self.april_detector is None:
            return None
            
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
                
                return world_from_robot_base
        
        return None
    
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
            
            # Core detection - now returns just the pose
            world_pose_base = self.detect_robot(gray, camera_from_world)
            if world_pose_base is not None:
                # Store for TF publishing
                self.latest_robot_base_pose = world_pose_base
                
                # Only publish overlay if debug images are enabled
                if self.debug_image:
                    # Calculate camera_from_robot_base for overlay
                    camera_from_robot_base = camera_from_world * world_pose_base
                    self.publish_overlay(cv_image, camera_from_robot_base)
                
                # Publish robot_base pose
                self.publish_pose(world_pose_base)
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def publish_overlay(self, cv_image, camera_from_robot_base):
        """Publish visualization overlay with just the outline based on robot_base"""
        if not self.debug_image or self.overlay_pub is None:
            return
            
        overlay = cv_image.copy()
        
        # Draw robot outline based on robot_base
        outline_2d = self.calculate_robot_outline(camera_from_robot_base)
        for i in range(len(outline_2d)):
            pt1 = tuple(outline_2d[i].astype(int))
            pt2 = tuple(outline_2d[(i + 1) % len(outline_2d)].astype(int))
            cv2.line(overlay, pt1, pt2, (0, 255, 0), 2)
        
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
        
        self.get_logger().info(f"Robot BASE pose: [{pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}, {pose_msg.pose.position.z:.2f}]",
                              throttle_duration_sec=2.0)
    
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