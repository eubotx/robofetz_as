import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
from detection.detector import AprilTagDetector
from detection.math_funcs import Pose as MathPose

def quaternion_from_vectors(v1, v2):
    v1 = np.array(v1, dtype=np.float64).squeeze()
    v2 = np.array(v2, dtype=np.float64).squeeze()
    v1 /= np.linalg.norm(v1)
    v2 /= np.linalg.norm(v2)
    dot = np.dot(v1, v2)
    
    if np.allclose(dot, 1.0):
        return np.array([0.0, 0.0, 0.0, 1.0])
    elif np.allclose(dot, -1.0):
        orthogonal = np.array([1.0, 0.0, 0.0])
        if np.allclose(v1, orthogonal):
            orthogonal = np.array([0.0, 1.0, 0.0])
        axis = np.cross(v1, orthogonal)
        axis /= np.linalg.norm(axis)
        return np.array([axis[0], axis[1], axis[2], 0.0])
    
    axis = np.cross(v1, v2)
    s = np.sqrt((1.0 + dot) * 2.0)
    invs = 1.0 / s
    return np.array([axis[0] * invs, axis[1] * invs, axis[2] * invs, s * 0.5])

class SimpleRobotDetector:
    def __init__(self, options, calibration_data):
        self.robot_tags = options['robot_tags']
        self.calibration_data = calibration_data
        self.camera_matrix = calibration_data['camera_matrix']
        self.april_detector = AprilTagDetector(calibration_data)
        
        # Define robot outline for enemy detection compatibility
        self.robot_outline = 0.13 * np.array([
            [0, -4, 0],
            [2, 0, 0], 
            [2, 1, 0],
            [-2, 1, 0],
            [-2, 0, 0]
        ])
        
    def project(self, pt3d):
        """Project 3D points to 2D image coordinates"""
        pt2d, _ = cv2.projectPoints(
            objectPoints=pt3d,
            rvec=np.eye(3),
            tvec=np.zeros(3),
            cameraMatrix=self.camera_matrix,
            distCoeffs=np.zeros(5)  # Assuming rectified image
        )
        return pt2d
        
    def detect(self, image_gray, worldFromCamera, debug_image=None):
        """Detect robot using AprilTags and return detection data"""
        # Detect AprilTags
        april_tag_detections = self.april_detector.detect(
            image_gray, self.robot_tags['sizes']
        )['aprilTags']
        
        for tag_detection in april_tag_detections:
            if (tag_detection.tag_family.decode() == self.robot_tags['family'] and
                (tag_detection.tag_id == self.robot_tags['top_id'] or
                 tag_detection.tag_id == self.robot_tags['bottom_id'])):
                
                detection = {}
                detection['cameraFromRobot'] = MathPose(tag_detection.pose_R, tag_detection.pose_t)
                detection['robotInCamera2D'] = self.project(tag_detection.pose_t).squeeze()
                detection['worldFromRobot'] = worldFromCamera * detection['cameraFromRobot']
                
                # Add robot outline for enemy detection compatibility
                robotOutlineInCamera2D = []
                for outline_point in self.robot_outline:
                    outlineInRobot = MathPose(np.eye(3), outline_point)
                    pointInCamera = detection['cameraFromRobot'] * outlineInRobot
                    robotOutlineInCamera2D.append(self.project(pointInCamera.t).squeeze())
                detection['robotOutlineInCamera2D'] = robotOutlineInCamera2D
                
                return detection
        
        return None

class BotDetectionNode(Node):
    def __init__(self):
        super().__init__('robot_detection_node')
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, 10)
        self.calibration_sub = self.create_subscription(
            Pose, '/arena_calibration', self.calibration_callback, 10)
        self.bridge = CvBridge()
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/bot/pose', 10)
        self.overlay_pub = self.create_publisher(Image, '/detection/robot_overlay', 10)
        
        # Calibration client
        self.calibration_client = self.create_client(Trigger, 'calibrate_arena')
        
        # State
        self.camera_info = None
        self.robot_detector = None
        self.camera_from_world = None
        self.calibration_requested = False
        
        # Options
        self.options = {
            'robot_tags': {'sizes': 0.125, 'family': 'tagStandard41h12', 'top_id': 12, 'bottom_id': 31},
        }
        
    def camera_info_callback(self, msg):
        """Receive camera calibration from camera_info topic"""
        self.camera_info = msg
        if self.robot_detector is None and self.camera_info is not None:
            self.initialize_detector()
        
    def initialize_detector(self):
        """Initialize robot detector with camera info"""
        calibration_data = {
            'camera_matrix': np.array(self.camera_info.k).reshape(3, 3),
            'distortion_coefficients': np.array(self.camera_info.d)
        }
        self.robot_detector = SimpleRobotDetector(self.options, calibration_data)
        self.get_logger().info("Robot detector initialized with camera info")
        
    def calibration_callback(self, msg):
        """Receive arena calibration from topic"""
        translation = np.array([msg.position.x, msg.position.y, msg.position.z])
        rotation = np.eye(3)  # Extract from quaternion (implement properly)
        self.camera_from_world = MathPose(rotation, translation)
        self.get_logger().info("Received arena calibration - ready for detection")
    
    def image_callback(self, msg):
        """Process each frame for robot detection"""
        # Ensure calibration and detector are available
        if self.camera_from_world is None:
            if not self.calibration_requested:
                self.request_calibration()
            return
            
        if self.robot_detector is None:
            self.get_logger().debug("Robot detector not initialized yet")
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect robot
            robot_detection = self.robot_detector.detect(gray, self.camera_from_world.inv(), cv_image)
            if robot_detection is None:
                self.get_logger().debug("No robot detected in frame")
                return
                
            # Publish pose
            self.publish_robot_pose(robot_detection)
            
            # Publish overlay image with detection drawings
            self.publish_robot_overlay(cv_image, robot_detection)
            
        except Exception as e:
            self.get_logger().error(f"Error in robot detection: {str(e)}")
    
    def publish_robot_overlay(self, cv_image, robot_detection):
        """Publish detection overlay as ROS Image"""
        # Create a copy for drawing
        overlay_image = cv_image.copy()
        
        # Draw robot center
        center = tuple(robot_detection['robotInCamera2D'].astype(int))
        cv2.circle(overlay_image, center, 5, (0, 255, 0), -1)
        cv2.putText(overlay_image, f"Robot", (center[0] + 10, center[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw robot outline
        outline = robot_detection['robotOutlineInCamera2D']
        for i in range(len(outline)):
            pt1 = tuple(outline[i].astype(int))
            pt2 = tuple(outline[(i + 1) % len(outline)].astype(int))
            cv2.line(overlay_image, pt1, pt2, (0, 255, 0), 2)
        
        # Convert to ROS Image and publish
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, "bgr8")
        overlay_msg.header.stamp = self.get_clock().now().to_msg()
        overlay_msg.header.frame_id = "camera"
        self.overlay_pub.publish(overlay_msg)
        
        self.get_logger().debug("Published robot detection overlay")
    
    def request_calibration(self):
        """Request arena calibration if not available"""
        if not self.calibration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Calibration service not available")
            return
            
        self.calibration_requested = True
        future = self.calibration_client.call_async(Trigger.Request())
        future.add_done_callback(self.calibration_service_callback)
        self.get_logger().info("Requesting arena calibration...")
    
    def calibration_service_callback(self, future):
        """Handle calibration service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Calibration service completed successfully")
            else:
                self.get_logger().error(f"Calibration service failed: {response.message}")
                self.calibration_requested = False
        except Exception as e:
            self.get_logger().error(f"Calibration service call failed: {str(e)}")
            self.calibration_requested = False
    
    def publish_robot_pose(self, robot_detection):
        """Publish robot pose to ROS"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose_msg.pose.position.x = float(robot_detection['worldFromRobot'].t[0])
        pose_msg.pose.position.y = float(robot_detection['worldFromRobot'].t[1])
        pose_msg.pose.position.z = 0.0
        
        # Orientation
        robotXInWorld = robot_detection['worldFromRobot'] * np.array([0, -1, 0])
        q = quaternion_from_vectors(robotXInWorld, np.array([1, 0, 0]))
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().debug(f"Published robot pose: [{pose_msg.pose.position.x:.3f}, {pose_msg.pose.position.y:.3f}]")


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