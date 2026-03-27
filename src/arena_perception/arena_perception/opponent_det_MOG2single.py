#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import threading


class OpponentDetMOG2Single(Node):
    def __init__(self):
        super().__init__('opponent_det_MOG2single')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_frame', 'arena_camera_optical'),
                ('enabled', True),
                ('input_topic', '/arena_camera/image_rect_masked'),
                ('topic_2d', '/detections_2d/mog2'),
                ('topic_viz', '/debug/mog2_point'),
                ('min_contour_area', 300),
                ('max_contour_area', 10000),
                ('background_history', 500),
                ('var_threshold', 16),
                ('ignore_radius_px', 100),
                ('shadow_expansion_factor', 1.2),
                ('stationary_timeout', 2.0),
                ('debug', True),
            ]
        )

        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("MOG2 detector disabled")
            return

        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.max_contour_area = self.get_parameter('max_contour_area').value
        self.background_history = self.get_parameter('background_history').value
        self.var_threshold = self.get_parameter('var_threshold').value
        self.ignore_radius_px = self.get_parameter('ignore_radius_px').value
        self.shadow_expansion_factor = self.get_parameter('shadow_expansion_factor').value
        self.stationary_timeout = self.get_parameter('stationary_timeout').value
        self.debug = self.get_parameter('debug').value

        self.input_topic = self.get_parameter('input_topic').value
        self.topic_2d = self.get_parameter('topic_2d').value
        self.topic_viz = self.get_parameter('topic_viz').value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=self.background_history,
            varThreshold=self.var_threshold,
            detectShadows=True
        )

        self.projection_matrix = None  # ✅ store full P

        self.image_width = 0
        self.image_height = 0
        self.camera_frame_id = ""

        self.own_robot_position_px = None
        self.own_robot_last_update = None
        self.lock = threading.Lock()

        self.last_position = None
        self.last_seen = None
        self.stationary = False

        self.image_sub = self.create_subscription(Image, self.input_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/arena_camera/camera_info', self.camera_info_callback, 10)
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/robot/pose', self.robot_pose_callback, 10)

        self.detections_pub = self.create_publisher(Detection2DArray, self.topic_2d, 10)
        self.debug_publisher = self.create_publisher(Image, self.topic_viz, 10)

        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)

    # ---------------- Camera info ----------------
    def camera_info_callback(self, msg):
        if self.projection_matrix is not None:
            return
        try:
            # Use full rectified projection P (3x4)
            self.projection_matrix = np.array(msg.p).reshape(3, 4)
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info("Camera info received (using full P matrix).")
        except Exception as e:
            self.get_logger().error(f"Camera info error: {e}")

    # ---------------- Robot pose ----------------
    def robot_pose_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                'world',
                rclpy.time.Time()
            )

            robot_point = PointStamped()
            robot_point.header = msg.header
            robot_point.header.frame_id = 'world'
            robot_point.point = msg.pose.position
            camera_point = do_transform_point(robot_point, transform)

            if self.projection_matrix is not None:
                # ✅ homogeneous projection using full P
                point_4d = np.array([
                    camera_point.point.x,
                    camera_point.point.y,
                    camera_point.point.z,
                    1.0
                ])
                pixel_hom = self.projection_matrix @ point_4d

                if pixel_hom[2] > 0:
                    px = int(pixel_hom[0] / pixel_hom[2])
                    py = int(pixel_hom[1] / pixel_hom[2])

                    # fallback if slightly outside
                    px = np.clip(px, 0, self.image_width - 1)
                    py = np.clip(py, 0, self.image_height - 1)

                    with self.lock:
                        self.own_robot_position_px = (px, py)
                        self.own_robot_last_update = self.get_clock().now()
        except:
            pass

    # ---------------- Image processing ----------------
    def image_callback(self, msg):
        if not self.enabled:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")
            return

        debug_frame = cv_image.copy()

        if self.projection_matrix is None:
            cv2.putText(debug_frame, "WAITING FOR CAMERA INFO...", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            self.publish_debug(debug_frame)
            return

        with self.lock:
            robot_pos = self.own_robot_position_px

        fg_mask = self.bg_subtractor.apply(debug_frame)
        _, thresh = cv2.threshold(fg_mask, 250, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        if robot_pos is not None:
            mask = np.zeros(thresh.shape, dtype=np.uint8)
            radius = int(self.ignore_radius_px * self.shadow_expansion_factor)
            cv2.circle(mask, robot_pos, radius, 255, -1)
            thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_contour = None
        max_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_contour_area < area < self.max_contour_area:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect = float(w) / h
                if 0.3 < aspect < 3.0 and area > max_area:
                    max_area = area
                    best_contour = cnt

        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.camera_frame_id

        pixel_pos = None

        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M['m00'] > 0:
                cx = float(M['m10'] / M['m00'])
                cy = float(M['m01'] / M['m00'])
                pixel_pos = (int(cx), int(cy))

                x, y, w, h = cv2.boundingRect(best_contour)

                detection = Detection2D()
                detection.header = detection_array.header
                detection.bbox.center.position.x = cx
                detection.bbox.center.position.y = cy
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = "opponent"
                hypothesis.hypothesis.score = 1.0

                detection.results = [hypothesis]
                detection_array.detections.append(detection)

                self.last_position = (cx, cy)
                self.last_seen = self.get_clock().now()
                self.stationary = False
        else:
            if self.last_position is not None:
                self.stationary = True

        self.detections_pub.publish(detection_array)

        if self.debug:
            self.publish_debug(debug_frame, best_contour, pixel_pos, robot_pos)

    # ---------------- Debug visualization ----------------
    def publish_debug(self, frame, contour=None, pixel_pos=None, robot_pos=None):
        debug = frame.copy()

        if robot_pos is not None:
            radius = int(self.ignore_radius_px * self.shadow_expansion_factor)
            cv2.circle(debug, robot_pos, radius, (0, 255, 0), 2)
            cv2.putText(debug, "SELF", (robot_pos[0] - 30, robot_pos[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if contour is not None and pixel_pos is not None:
            cv2.drawContours(debug, [contour], -1, (0, 0, 255), 3)
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug, pixel_pos, 5, (0, 0, 255), -1)
            cv2.putText(debug, "opponent", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        elif self.stationary and self.last_position is not None:
            cx, cy = self.last_position
            cv2.circle(debug, (int(cx), int(cy)), 10, (128, 128, 128), 2)
            cv2.putText(debug, "opponent (stationary)", (int(cx) - 40, int(cy) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

        cv2.putText(debug, "opponent_det_MOG2single", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if contour is None:
            if self.stationary and self.last_position is not None:
                cv2.putText(debug, "Opponent stationary", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
            else:
                cv2.putText(debug, "No opponent detected", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(debug, "Opponent detected!", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
        out_msg.header.frame_id = self.camera_frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_publisher.publish(out_msg)

    # ---------------- Cleanup ----------------
    def cleanup_stale_data(self):
        now = self.get_clock().now()
        with self.lock:
            if self.own_robot_last_update:
                if (now - self.own_robot_last_update).nanoseconds > 1.0e9:
                    self.own_robot_position_px = None

        if self.stationary and self.last_seen is not None:
            time_since_seen = (now - self.last_seen).nanoseconds / 1e9
            if time_since_seen > self.stationary_timeout * 2:
                self.last_position = None
                self.last_seen = None
                self.stationary = False
                self.get_logger().info("Removed very old stationary opponent")


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetMOG2Single()
    if node.enabled:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()