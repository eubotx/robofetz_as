import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

import numpy as np


class Detection2DTo3DConverter(Node):

    def __init__(self):
        super().__init__('detection_transformation_2D_3D_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_frame', 'world'),
                ('camera_frame', 'arena_camera_optical'),
                ('z_plane', 0.0),
                ('confidence_threshold', 0.5),
                ('tf_timeout_sec', 0.1),
                ('publish_visualization', True),
            ]
        )

        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.z_plane = self.get_parameter('z_plane').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.tf_timeout = rclpy.duration.Duration(
            seconds=self.get_parameter('tf_timeout_sec').value
        )
        self.publish_visualization = self.get_parameter('publish_visualization').value

        # State
        self.camera_info = None
        self.camera_matrix = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection3DArray,
            'detections3d_array',
            10
        )

        if self.publish_visualization:
            self.debug_point_pub = self.create_publisher(
                PointStamped,
                'detections3d_point',
                10
            )

        # QoS for camera
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, qos)
        self.create_subscription(Detection2DArray, 'detections_2d', self.detections_callback, 10)

        self.get_logger().info("Detection2DTo3DConverter initialized")

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("Camera info received")

    def pixel_to_camera_ray(self, u, v):

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        origin = np.array([0.0, 0.0, 0.0])

        direction = np.array([
            (u - cx) / fx,
            (v - cy) / fy,
            1.0
        ])

        direction = direction / np.linalg.norm(direction)

        return origin, direction

    def project_to_plane(self, ray_origin, ray_direction):

        if abs(ray_direction[2]) < 1e-6:
            return None

        t = (self.z_plane - ray_origin[2]) / ray_direction[2]

        if t < 0:
            return None

        return ray_origin + t * ray_direction

    def detections_callback(self, msg):

        if self.camera_matrix is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time(),
                self.tf_timeout
            )
        except TransformException:
            return

        detection_array_3d = Detection3DArray()
        detection_array_3d.header = Header()
        detection_array_3d.header.stamp = self.get_clock().now().to_msg()
        detection_array_3d.header.frame_id = self.target_frame

        for detection in msg.detections:

            if not detection.results:
                continue

            if detection.results[0].hypothesis.score < self.confidence_threshold:
                continue

            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y

            origin_cam, dir_cam = self.pixel_to_camera_ray(u, v)

            origin_msg = PointStamped()
            origin_msg.header.frame_id = self.camera_frame
            origin_msg.point.x = origin_cam[0]
            origin_msg.point.y = origin_cam[1]
            origin_msg.point.z = origin_cam[2]

            end_msg = PointStamped()
            end_msg.header.frame_id = self.camera_frame
            end_msg.point.x = origin_cam[0] + dir_cam[0]
            end_msg.point.y = origin_cam[1] + dir_cam[1]
            end_msg.point.z = origin_cam[2] + dir_cam[2]

            try:

                origin_target = tf2_geometry_msgs.do_transform_point(origin_msg, transform)
                end_target = tf2_geometry_msgs.do_transform_point(end_msg, transform)

                ray_origin = np.array([
                    origin_target.point.x,
                    origin_target.point.y,
                    origin_target.point.z
                ])

                ray_end = np.array([
                    end_target.point.x,
                    end_target.point.y,
                    end_target.point.z
                ])

                ray_dir = ray_end - ray_origin
                ray_dir = ray_dir / np.linalg.norm(ray_dir)

                point_3d = self.project_to_plane(ray_origin, ray_dir)

                if point_3d is None:
                    continue

                detection_3d = Detection3D()

                detection_3d.header.frame_id = self.target_frame
                detection_3d.header.stamp = detection.header.stamp

                detection_3d.bbox.center.position.x = point_3d[0]
                detection_3d.bbox.center.position.y = point_3d[1]
                detection_3d.bbox.center.position.z = point_3d[2]

                detection_3d.bbox.center.orientation.w = 1.0

                for result in detection.results:

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = result.hypothesis.class_id
                    hypothesis.hypothesis.score = result.hypothesis.score

                    detection_3d.results.append(hypothesis)

                detection_array_3d.detections.append(detection_3d)

                if self.publish_visualization:

                    point_msg = PointStamped()
                    point_msg.header.frame_id = self.target_frame
                    point_msg.header.stamp = self.get_clock().now().to_msg()

                    point_msg.point.x = point_3d[0]
                    point_msg.point.y = point_3d[1]
                    point_msg.point.z = point_3d[2]

                    self.debug_point_pub.publish(point_msg)

            except Exception:
                continue

        if detection_array_3d.detections:
            self.detection_pub.publish(detection_array_3d)


def main(args=None):

    rclpy.init(args=args)

    node = Detection2DTo3DConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()