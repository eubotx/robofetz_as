import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class RectifyNode(Node):
    def __init__(self):
        super().__init__('arena_camera_rectify_node')

        self.declare_parameter('image_topic', 'image')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('rectified_topic', 'image_rect')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        rect_topic = self.get_parameter('rectified_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.latest_camera_info = None
        self.map1 = None
        self.map2 = None

        # QoS best-effort
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.sub_image = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile
        )

        # Publisher
        self.pub_image = self.create_publisher(Image, rect_topic, qos_profile)

        self.get_logger().info(f"RectifyNode ready: subscribing to '{image_topic}' and '{camera_info_topic}', publishing '{rect_topic}'")

    def camera_info_callback(self, msg: CameraInfo):
        # Cache latest CameraInfo
        self.latest_camera_info = msg

        # Precompute rectification map if not already done
        if self.map1 is None or self.map2 is None:
            K = np.array(msg.k).reshape((3, 3))
            D = np.array(msg.d)
            R = np.array(msg.r).reshape((3, 3))
            P = np.array(msg.p).reshape((3, 4))

            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                K, D, R, P[:, :3], (msg.width, msg.height), cv2.CV_32FC1
            )
            self.get_logger().info("Rectification maps initialized from CameraInfo")

    def image_callback(self, msg: Image):
        if self.map1 is None or self.map2 is None:
            # No camera info yet, cannot rectify
            self.get_logger().warning("No CameraInfo received yet; skipping rectification")
            return

        # Convert image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Rectify
        rectified = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        # Publish
        rect_msg = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
        rect_msg.header = msg.header
        self.pub_image.publish(rect_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RectifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()