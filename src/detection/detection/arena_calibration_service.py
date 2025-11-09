import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from detection.detector import AprilTagDetector
from detection.math_funcs import Pose as MathPose

class SimpleArenaDetector:
    def __init__(self, options, calibration_data):
        self.options = options
        self.calibration_data = calibration_data
        self.april_detector = AprilTagDetector(calibration_data)
        
    def detect_arena_tag(self, image_gray, debug_image=None):
        """Detect arena tag and return cameraFromWorld pose"""
        april_tag_output = self.april_detector.detect(
            image_gray, self.options['arena_tag']['size'], debug_image
        )
        
        for detection in april_tag_output["aprilTags"]:
            if (detection.tag_family.decode() == self.options['arena_tag']['family'] and
                detection.tag_id == self.options['arena_tag']['id']):
                return MathPose(detection.pose_R, detection.pose_t)
        
        return None

class ArenaCalibrationService(Node):
    def __init__(self):
        super().__init__('arena_calibration_service')
        
        # Service for one-time calibration
        self.srv = self.create_service(Trigger, 'calibrate_arena', self.calibrate_callback)
        
        # Publisher for persistent calibration result
        self.calibration_pub = self.create_publisher(Pose, '/arena_calibration', 10)
        
        # TF broadcaster for spatial relationships
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'arena_camera/image_rect', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'arena_camera/camera_info', self.camera_info_callback, 10)
        self.bridge = CvBridge()
        
        # State
        self.latest_frame = None
        self.camera_info = None
        self.camera_from_world = None
        self.calibration_complete = False
        self.arena_detector = None
        
        # Options
        self.options = {
            'arena_tag': {'id': 12, 'family': 'tagStandard41h12', 'size': 0.125},
        }
        
    def camera_info_callback(self, msg):
        """Receive camera calibration from camera_info topic"""
        self.camera_info = msg
        if self.arena_detector is None and self.camera_info is not None:
            self.initialize_detector()
        
    def initialize_detector(self):
        """Initialize arena detector with camera info"""
        calibration_data = {
            'camera_matrix': np.array(self.camera_info.k).reshape(3, 3),
            'distortion_coefficients': np.array(self.camera_info.d)
        }
        self.arena_detector = SimpleArenaDetector(self.options, calibration_data)
        self.get_logger().info("Arena detector initialized with camera info")
        
    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    async def calibrate_callback(self, request, response):
        """One-time calibration service"""
        if self.latest_frame is None:
            response.success = False
            response.message = "No camera frame available"
            return response
            
        if self.arena_detector is None:
            response.success = False
            response.message = "Camera info not received yet"
            return response
            
        success = self.detect_arena_tag()
        
        if success:
            # Publish persistent result to topic
            self.publish_calibration()
            # Broadcast TF
            self.publish_transform()
            
            response.success = True
            response.message = "Arena calibration complete and published"
        else:
            response.success = False
            response.message = "Failed to detect arena tag"
            
        return response
    
    def detect_arena_tag(self):
        """Detect arena tag using the simple detector"""
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
        camera_from_world = self.arena_detector.detect_arena_tag(gray, self.latest_frame.copy())
        
        if camera_from_world is not None:
            self.camera_from_world = camera_from_world
            self.calibration_complete = True
            self.get_logger().info("Arena tag detected - calibration complete")
            return True
        
        self.get_logger().warn("Arena tag not found in frame")
        return False
    
    def publish_calibration(self):
        """Publish calibration as persistent topic message"""
        if not self.calibration_complete:
            return
            
        pose_msg = Pose()
        pose_msg.position.x = float(self.camera_from_world.t[0])
        pose_msg.position.y = float(self.camera_from_world.t[1])
        pose_msg.position.z = float(self.camera_from_world.t[2])
        
        # Convert rotation matrix to quaternion (you'll need to implement this properly)
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0
        
        self.calibration_pub.publish(pose_msg)
        self.get_logger().info("Published arena calibration to topic")
    
    def publish_transform(self):
        """Broadcast transform for TF tree"""
        if not self.calibration_complete:
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "camera"
        
        t.transform.translation.x = float(self.camera_from_world.t[0])
        t.transform.translation.y = float(self.camera_from_world.t[1])
        t.transform.translation.z = float(self.camera_from_world.t[2])
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published camera transform to TF")


def main(args=None):
    rclpy.init(args=args)
    node = ArenaCalibrationService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()