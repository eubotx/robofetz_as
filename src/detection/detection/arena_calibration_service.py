import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from detection.detector import AprilTagDetector
from detection.math_funcs import Pose as MathPose
from scipy.spatial.transform import Rotation

class ArenaCalibrationService(Node):
    def __init__(self):
        super().__init__('arena_calibration_service')
        
        # Service for one-time calibration
        self.srv = self.create_service(Trigger, 'calibrate_arena', self.calibrate_callback)
        
        # TF broadcaster
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
        self.april_detector = None
        
        # Arena tag configuration
        self.arena_tag_id = 2
        self.arena_tag_family = 'tagStandard41h12'
        self.arena_tag_size = 0.125
        
    def camera_info_callback(self, msg):
        """Receive camera calibration from camera_info topic"""
        self.camera_info = msg
        if self.april_detector is None and self.camera_info is not None:
            calibration_data = {
                'camera_matrix': np.array(self.camera_info.k).reshape(3, 3),
                'distortion_coefficients': np.array(self.camera_info.d)
            }
            self.april_detector = AprilTagDetector(calibration_data)
            self.get_logger().info("AprilTag detector initialized")
        
    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    async def calibrate_callback(self, request, response):
        """One-time calibration service"""
        if self.latest_frame is None:
            response.success = False
            response.message = "No camera frame available"
            return response
            
        if self.april_detector is None:
            response.success = False
            response.message = "Camera info not received yet"
            return response
            
        success = self.detect_arena_tag()
        
        if success:
            self.publish_transform()  # Only publish TF
            response.success = True
            response.message = "Arena calibration complete and published to TF"
        else:
            response.success = False
            response.message = "Failed to detect arena tag"
            
        return response
    
    def detect_arena_tag(self):
        """Detect arena tag and store cameraFromWorld transform"""
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
        april_tag_output = self.april_detector.detect(gray, self.arena_tag_size)
        
        for detection in april_tag_output["aprilTags"]:
            if (detection.tag_family.decode() == self.arena_tag_family and
                detection.tag_id == self.arena_tag_id):
                self.camera_from_world = MathPose(detection.pose_R, detection.pose_t)
                self.calibration_complete = True
                self.get_logger().info("Arena tag detected - calibration complete")
                return True
        
        self.get_logger().warn("Arena tag not found in frame")
        return False
    
    def publish_transform(self):
        """Broadcast transform for TF tree"""
        if self.camera_from_world is None:
            self.get_logger().error("No calibration data available")
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "camera"
        
        # Translation
        t.transform.translation.x = float(self.camera_from_world.t[0])
        t.transform.translation.y = float(self.camera_from_world.t[1])
        t.transform.translation.z = float(self.camera_from_world.t[2])
        
        # PROPER rotation matrix to quaternion conversion
        rotation = Rotation.from_matrix(self.camera_from_world.R)
        quat = rotation.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
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