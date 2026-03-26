#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration


class RoiMaskNode(Node):
    def __init__(self):
        super().__init__('roi_mask_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_frame', 'arena_camera_optical'),
                ('world_frame', 'world'),
                ('roi_min_x', -0.2),
                ('roi_max_x', 1.7),
                ('roi_min_y', -0.2),
                ('roi_max_y', 1.7),
                ('roi_z', 0.0),
                ('mask_color', [0, 0, 0]),
                ('mask_alpha', 1.0)
            ]
        )

        # Get parameters
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.roi_min_x = self.get_parameter('roi_min_x').value
        self.roi_max_x = self.get_parameter('roi_max_x').value
        self.roi_min_y = self.get_parameter('roi_min_y').value
        self.roi_max_y = self.get_parameter('roi_max_y').value
        self.roi_z = self.get_parameter('roi_z').value
        self.mask_color = self.get_parameter('mask_color').value
        self.mask_alpha = self.get_parameter('mask_alpha').value

        # Initialize
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.projection_matrix = None
        self.transform = None
        self.ready = False

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera_info', self.camera_info_callback, qos)
        self.image_sub = self.create_subscription(
            Image, 'image', self.image_callback, qos)

        # Publisher
        self.masked_image_pub = self.create_publisher(
            Image, 'masked_image', qos)

        # Timers
        self.create_timer(1.0, self.check_ready)
        self.transform_timer = None

        self.get_logger().info('ROI Mask Node initialized (using projection matrix)')

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'camera_frame':
                self.camera_frame = param.value
                self.transform = None
                self.ready = False

            elif param.name == 'world_frame':
                self.world_frame = param.value
                self.transform = None
                self.ready = False

            elif param.name in ['roi_min_x', 'roi_max_x', 'roi_min_y', 'roi_max_y', 'roi_z']:
                setattr(self, param.name, param.value)

            elif param.name == 'mask_color':
                self.mask_color = param.value

            elif param.name == 'mask_alpha':
                self.mask_alpha = param.value

        return rclpy.parameter.SetParametersResult(successful=True)

    def camera_info_callback(self, msg):
        if self.projection_matrix is None:
            self.projection_matrix = np.array(msg.p).reshape(3, 4)
            self.get_logger().info('✓ Camera info received (projection matrix)')
            self.check_ready()

    def check_ready(self):
        if self.ready:
            return

        if self.projection_matrix is None:
            self.get_logger().info('Waiting for camera info...', throttle_duration_sec=2.0)
            return

        try:
            self.transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.world_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )

            self.ready = True
            self.get_logger().info('✓ All systems ready')

            if self.transform_timer is None:
                self.transform_timer = self.create_timer(0.5, self.update_transform)

        except Exception:
            self.get_logger().info(
                f'Waiting for TF transform ({self.world_frame} -> {self.camera_frame})...',
                throttle_duration_sec=2.0
            )

    def update_transform(self):
        if not self.ready:
            return

        try:
            self.transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.world_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'Transform lost: {e}')
            self.ready = False

    def world_to_pixel(self, x, y, stamp):
        if not self.ready:
            return None

        point = PointStamped()
        point.header.frame_id = self.world_frame
        point.header.stamp = stamp
        point.point.x = x
        point.point.y = y
        point.point.z = self.roi_z

        try:
            point_camera = tf2_geometry_msgs.do_transform_point(point, self.transform)
        except Exception:
            return None

        X = point_camera.point.x
        Y = point_camera.point.y
        Z = point_camera.point.z

        if Z <= 0:
            return None

        point_3d = np.array([X, Y, Z, 1.0])
        uvw = self.projection_matrix @ point_3d

        if uvw[2] == 0:
            return None

        u = uvw[0] / uvw[2]
        v = uvw[1] / uvw[2]

        return (int(round(u)), int(round(v)))

    def image_callback(self, msg):
        if not self.ready:
            self.masked_image_pub.publish(msg)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = cv_image.shape[:2]

            corners_world = [
                (self.roi_min_x, self.roi_min_y),
                (self.roi_max_x, self.roi_min_y),
                (self.roi_max_x, self.roi_max_y),
                (self.roi_min_x, self.roi_max_y)
            ]

            corners_pixel = []
            for x, y in corners_world:
                pixel = self.world_to_pixel(x, y, msg.header.stamp)
                if pixel is None:
                    self.masked_image_pub.publish(msg)
                    return
                corners_pixel.append(pixel)

            roi_mask = np.zeros((h, w), dtype=np.uint8)
            roi_corners = np.array(corners_pixel, dtype=np.int32)
            cv2.fillPoly(roi_mask, [roi_corners], 255)

            outside_mask = cv2.bitwise_not(roi_mask)

            masked_image = cv_image.copy()
            masked_image[outside_mask == 255] = self.mask_color

            masked_msg = self.bridge.cv2_to_imgmsg(masked_image, 'bgr8')
            masked_msg.header = msg.header
            self.masked_image_pub.publish(masked_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}', throttle_duration_sec=1.0)
            self.masked_image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoiMaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()