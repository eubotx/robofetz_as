import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger
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
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, 10)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/bot/pose', 10)
        self.overlay_pub = self.create_publisher(Image, '/detection/robot_overlay', 10)
        
        # Calibration client
        self.calibration_client = self.create_client(Trigger, 'calibrate_arena')
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)  # NEW: For bot TF
        
        # State
        self.bridge = CvBridge()
        self.april_detector = None
        self.camera_matrix = None
        self.calibration_requested = False
        
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
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            return None
    
    def project_3d_to_2d(self, point_3d):
        """Project 3D point to 2D image coordinates"""
        if self.camera_matrix is None:
            return None
            
        point_2d, _ = cv2.projectPoints(
            point_3d, np.eye(3), np.zeros(3), self.camera_matrix, np.zeros(5)
        )
        return point_2d.squeeze()
    
    def calculate_robot_outline(self, camera_from_robot):
        """Calculate robot outline projection - SEPARATED from detection"""
        outline_2d = []
        for point in self.robot_outline:
            point_in_camera = camera_from_robot * MathPose(np.eye(3), point)
            outline_2d.append(self.project_3d_to_2d(point_in_camera.t))
        return outline_2d
        
    def detect_robot(self, image_gray, camera_from_world):
        """Core robot detection - clean and focused on pose estimation"""
        if self.april_detector is None:
            return None
            
        detections = self.april_detector.detect(image_gray, self.robot_tags['sizes'])['aprilTags']
        
        for detection in detections:
            if (detection.tag_family.decode() == self.robot_tags['family'] and
                detection.tag_id in [self.robot_tags['top_id'], self.robot_tags['bottom_id']]):
                
                # Core detection logic only
                camera_from_robot = MathPose(detection.pose_R, detection.pose_t)
                world_from_robot = camera_from_world * camera_from_robot
                robot_center_2d = self.project_3d_to_2d(detection.pose_t)
                
                return {
                    'world_pose': world_from_robot,
                    'camera_from_robot': camera_from_robot,
                    'center_2d': robot_center_2d
                }
        
        return None
    
    def image_callback(self, msg):
        """Process each camera frame"""
        camera_from_world = self.get_camera_from_world()
        if camera_from_world is None:
            if not self.calibration_requested:
                self.request_calibration()
            return
            
        if self.april_detector is None:
            return
            
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Core detection
            detection = self.detect_robot(gray, camera_from_world)
            if detection:
                # Outline generation (separate from detection)
                outline_2d = self.calculate_robot_outline(detection['camera_from_robot'])
                detection['outline_2d'] = outline_2d
                
                self.publish_pose(detection['world_pose'])
                self.publish_overlay(cv_image, detection)
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def publish_overlay(self, cv_image, detection):
        """Publish visualization overlay"""
        overlay = cv_image.copy()
        center = tuple(detection['center_2d'].astype(int))
        
        # Draw robot center
        cv2.circle(overlay, center, 5, (0, 255, 0), -1)
        cv2.putText(overlay, "Robot", (center[0] + 10, center[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw robot outline
        outline = detection['outline_2d']
        for i in range(len(outline)):
            pt1 = tuple(outline[i].astype(int))
            pt2 = tuple(outline[(i + 1) % len(outline)].astype(int))
            cv2.line(overlay, pt1, pt2, (0, 255, 0), 2)
        
        # Publish overlay
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
        overlay_msg.header.stamp = self.get_clock().now().to_msg()
        self.overlay_pub.publish(overlay_msg)
    
    def publish_pose(self, world_pose):
        """Publish robot pose as both message and TF"""
        # Publish PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose_msg.pose.position.x = float(world_pose.t[0])
        pose_msg.pose.position.y = float(world_pose.t[1])
        pose_msg.pose.position.z = 0.0  # Ground robot
        
        # Orientation from the detected pose
        rotation = Rotation.from_matrix(world_pose.R)
        quat = rotation.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        self.pose_pub.publish(pose_msg)
        
        # NEW: Publish TF transform
        self.publish_bot_tf(world_pose)
        
        self.get_logger().info(f"Robot pose: [{pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}]")
    
    def publish_bot_tf(self, world_pose):
        """Publish robot transform to TF tree"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "robot"  # Robot frame name
        
        # Position
        t.transform.translation.x = float(world_pose.t[0])
        t.transform.translation.y = float(world_pose.t[1])
        t.transform.translation.z = 0.0  # Ground robot
        
        # Orientation
        rotation = Rotation.from_matrix(world_pose.R)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug("Published robot transform to TF")
    
    def request_calibration(self):
        """Request arena calibration service"""
        if self.calibration_client.wait_for_service(timeout_sec=1.0):
            self.calibration_requested = True
            future = self.calibration_client.call_async(Trigger.Request())
            future.add_done_callback(self.calibration_callback)
            self.get_logger().info("Requesting arena calibration...")
    
    def calibration_callback(self, future):
        """Handle calibration response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Calibration successful")
            else:
                self.get_logger().warn(f"Calibration failed: {response.message}")
                self.calibration_requested = False
        except Exception as e:
            self.get_logger().error(f"Calibration call failed: {str(e)}")
            self.calibration_requested = False


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