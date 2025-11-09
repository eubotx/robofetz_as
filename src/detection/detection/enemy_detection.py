import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from detection.detector import EnemyDetector
from detection.math_funcs import Pose as MathPose

class EnemyDetectionNode(Node):
    def __init__(self):
        super().__init__('enemy_detection_node')
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, 10)
        self.bot_pose_sub = self.create_subscription(
            PoseStamped, '/bot/pose', self.bot_pose_callback, 10)
        self.calibration_sub = self.create_subscription(
            Pose, '/arena_calibration', self.calibration_callback, 10)
        self.bridge = CvBridge()
        
        # Publisher
        self.enemy_pub = self.create_publisher(PoseStamped, '/enemy/pose', 10)
        
        # State
        self.camera_info = None
        self.enemy_detector = None
        self.current_bot_pose = None
        self.camera_from_world = None
        self.ready_for_detection = False
        
        # Options
        self.options = {
            # Add enemy detection options if needed
        }
        
    def camera_info_callback(self, msg):
        """Receive camera calibration from camera_info topic"""
        self.camera_info = msg
        if self.enemy_detector is None and self.camera_info is not None:
            self.initialize_detector()
        
    def initialize_detector(self):
        """Initialize enemy detector with camera info"""
        self.enemy_detector = EnemyDetector(self.options)
        self.get_logger().info("Enemy detector initialized")
        
    def calibration_callback(self, msg):
        """Receive arena calibration"""
        translation = np.array([msg.position.x, msg.position.y, msg.position.z])
        rotation = np.eye(3)
        self.camera_from_world = MathPose(rotation, translation)
        self.check_readiness()
        
    def bot_pose_callback(self, msg):
        """Store latest bot pose"""
        self.current_bot_pose = msg
        self.check_readiness()
    
    def check_readiness(self):
        """Check if we have all required data for enemy detection"""
        self.ready_for_detection = (self.camera_from_world is not None and 
                                  self.current_bot_pose is not None and
                                  self.enemy_detector is not None)
        if self.ready_for_detection:
            self.get_logger().info("Ready for enemy detection")
        
    def image_callback(self, msg):
        """Process frame for enemy detection"""
        if not self.ready_for_detection:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Convert bot pose to format expected by enemy detector
        bot_pose_data = {
            'worldFromRobot': MathPose(np.eye(3), np.array([
                self.current_bot_pose.pose.position.x,
                self.current_bot_pose.pose.position.y,
                self.current_bot_pose.pose.position.z
            ])),
            'cameraFromRobot': MathPose(np.eye(3), np.array([0, 0, 1.0]))
        }
        
        # Detect enemy
        enemy_detection = self.enemy_detector.detect(gray, bot_pose_data, cv_image)
        if enemy_detection is None:
            return
            
        # Convert to world coordinates and publish
        self.publish_enemy_pose(enemy_detection, bot_pose_data)
    
    def publish_enemy_pose(self, enemy_detection, bot_pose_data):
        """Convert enemy detection to world coordinates and publish"""
        enemyInCamera2D = enemy_detection['enemyInCamera2D']
        
        # Convert 2D pixel to 3D camera coordinates using camera info
        u, v = enemyInCamera2D
        depth = bot_pose_data['cameraFromRobot'].t[2]
        
        if depth > 0 and self.camera_info is not None:
            fx = self.camera_info.k[0]  # Focal length x
            fy = self.camera_info.k[4]  # Focal length y
            cx = self.camera_info.k[2]  # Principal point x
            cy = self.camera_info.k[5]  # Principal point y
            
            X = (u - cx) * depth / fx
            Y = (v - cy) * depth / fy
            Z = depth
            
            # Convert to world coordinates
            enemyInCamera3D = np.array([X, Y, Z])
            enemyInWorld3D = self.camera_from_world.inv() * enemyInCamera3D
            
            # Publish enemy pose
            enemy_msg = PoseStamped()
            enemy_msg.header.frame_id = "world"
            enemy_msg.header.stamp = self.get_clock().now().to_msg()
            enemy_msg.pose.position.x = float(enemyInWorld3D[0])
            enemy_msg.pose.position.y = float(-enemyInWorld3D[1])
            enemy_msg.pose.position.z = 0.0
            
            enemy_msg.pose.orientation.x = 0.0
            enemy_msg.pose.orientation.y = 0.0
            enemy_msg.pose.orientation.z = 0.0
            enemy_msg.pose.orientation.w = 1.0
            
            self.enemy_pub.publish(enemy_msg)
            self.get_logger().debug(f"Published enemy pose: [{enemy_msg.pose.position.x:.3f}, {enemy_msg.pose.position.y:.3f}]")


def main(args=None):
    rclpy.init(args=args)
    node = EnemyDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()