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
from collections import OrderedDict


class OpponentDetMOG2Single(Node):
    """
    Opponent detector using MOG2 publishing Detection2DArray.
    Detects a single opponent with persistent ID (using opponent_0 format).
    """

    def __init__(self):
        super().__init__('opponent_det_MOG2single')

        # =================== PARAMETERS ===================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_contour_area', 300),
                ('max_contour_area', 10000),
                ('background_history', 500),
                ('var_threshold', 16),
                ('ignore_radius_px', 100),
                ('shadow_expansion_factor', 1.2),
                ('debug', True),
                ('robot_base_frame', 'robot_base'),
                ('camera_optical_frame', 'arena_camera_optical'),
                ('stationary_timeout', 2.0),  # How long to keep ID without detection
                ('match_distance', 100),  # Pixel distance for matching
            ]
        )

        # =================== INIT ===================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MOG2 Background Subtractor
        history = self.get_parameter('background_history').value
        var_threshold = self.get_parameter('var_threshold').value
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=history,
            varThreshold=var_threshold,
            detectShadows=True
        )

        # Camera variables
        self.camera_matrix = None
        self.camera_matrix_inv = None
        self.image_width = 0
        self.image_height = 0
        self.camera_frame_id = ""
        
        # Self-filtering variables
        self.own_robot_position_px = None
        self.own_robot_last_update = None
        self.lock = threading.Lock()
        
        # Tracking variables for persistent ID (always use opponent_0)
        self.opponent_id = 'MOG2_0'  # Fixed ID for single opponent
        self.last_position = None  # Last known position (cx, cy)
        self.last_seen = None  # Last time opponent was detected
        self.stationary = False  # Whether opponent is currently stationary
        
        # =================== COMMS ===================
        self.image_sub = self.create_subscription(
            Image, 
            '/arena_camera/image_rect', 
            self.image_callback, 
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            '/arena_camera/camera_info', 
            self.camera_info_callback, 
            10
        )
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, 
            '/bot/pose', 
            self.robot_pose_callback, 
            10
        )
        
        # Publisher for 2D detections
        self.detections_pub = self.create_publisher(
            Detection2DArray, 
            '/detections_2d', 
            10
        )
        
        # Debug publisher
        self.debug_publisher = self.create_publisher(
            Image, 
            '/debug/detection_image', 
            10
        )

        # Cleanup timer
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)

        self.get_logger().info("OpponentDetMOG2Single initialized - publishing single opponent with ID opponent_0")

    def camera_info_callback(self, msg):
        if self.camera_matrix is not None: 
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.image_width = msg.width
        self.image_height = msg.height
        try:
            self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)
            self.get_logger().info("Camera info received.")
        except: 
            pass

    def robot_pose_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('camera_optical_frame').value,
                'world',
                rclpy.time.Time()
            )
            robot_point = PointStamped()
            robot_point.header = msg.header
            robot_point.header.frame_id = 'world'
            robot_point.point = msg.pose.position
            camera_point = do_transform_point(robot_point, transform)

            if self.camera_matrix is not None:
                point_3d = np.array([camera_point.point.x, camera_point.point.y, camera_point.point.z])
                pixel_hom = self.camera_matrix @ point_3d
                if pixel_hom[2] > 0:
                    px = int(pixel_hom[0] / pixel_hom[2])
                    py = int(pixel_hom[1] / pixel_hom[2])
                    if 0 <= px < self.image_width and 0 <= py < self.image_height:
                        with self.lock:
                            self.own_robot_position_px = (px, py)
                            self.own_robot_last_update = self.get_clock().now()
        except Exception as e:
            # Just pass - TF might not be available yet
            pass

    def image_callback(self, msg):
        """Process image and publish detections"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_frame_id = 'arena_camera_optical'
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")
            return

        debug_frame = cv_image.copy()
        
        # Check Camera Info
        if self.camera_matrix is None:
            cv2.putText(debug_frame, "WAITING FOR CAMERA INFO...", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            self.publish_debug(debug_frame)
            return

        with self.lock:
            robot_pos = self.own_robot_position_px

        # === MOG2 Processing ===
        proc_frame = debug_frame.copy() 
        fg_mask = self.bg_subtractor.apply(proc_frame)
        
        # Threshold
        _, thresh = cv2.threshold(fg_mask, 250, 255, cv2.THRESH_BINARY)
        
        # Morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # Ego Robot Masking
        if robot_pos is not None:
            mask = np.zeros(thresh.shape, dtype=np.uint8)
            radius = int(self.get_parameter('ignore_radius_px').value * 
                        self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(mask, robot_pos, radius, 255, -1)
            thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        max_area = 0
        min_area = self.get_parameter('min_contour_area').value
        max_area_limit = self.get_parameter('max_contour_area').value
        current_position = None
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area < area < max_area_limit:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect = float(w)/h
                if 0.3 < aspect < 3.0:
                    if area > max_area:
                        max_area = area
                        best_contour = cnt

        # Create detection array
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.camera_frame_id

        pixel_pos = None

        if best_contour is not None:
            # Get centroid
            M = cv2.moments(best_contour)
            if M['m00'] > 0:
                cx = float(M['m10'] / M['m00'])
                cy = float(M['m01'] / M['m00'])
                pixel_pos = (int(cx), int(cy))
                current_position = (cx, cy)
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(best_contour)
                
                # Create Detection2D
                detection = Detection2D()
                detection.header = detection_array.header
                
                # Set bounding box center
                detection.bbox.center.position.x = cx
                detection.bbox.center.position.y = cy
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)
                
                # Create ObjectHypothesisWithPose with consistent ID
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = f"opponent_{self.opponent_id}"  # Always opponent_0
                hypothesis.hypothesis.score = 1.0
                
                detection.results = [hypothesis]
                
                detection_array.detections.append(detection)
                
                # Update tracking
                current_time = self.get_clock().now()
                self.last_position = current_position
                self.last_seen = current_time
                self.stationary = False
        else:
            # No detection - mark as stationary if we have a last position
            if self.last_position is not None:
                self.stationary = True

        # Publish
        self.detections_pub.publish(detection_array)

        # Debug
        if self.get_parameter('debug').value:
            self.publish_debug(debug_frame, best_contour, pixel_pos, robot_pos)

    def publish_debug(self, frame, contour=None, pixel_pos=None, robot_pos=None):
        debug = frame.copy()
        
        # Draw robot's own position
        if robot_pos is not None:
            radius = int(self.get_parameter('ignore_radius_px').value * 
                        self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(debug, robot_pos, radius, (0, 255, 0), 2)
            cv2.putText(debug, "SELF", (robot_pos[0] - 30, robot_pos[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw current detection if present
        if contour is not None and pixel_pos is not None:
            cv2.drawContours(debug, [contour], -1, (0, 0, 255), 3)
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(debug, pixel_pos, 5, (0, 0, 255), -1)
            
            # Draw ID
            cv2.putText(debug, f"opponent_0 (moving)", (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw stationary opponent if applicable
        elif self.stationary and self.last_position is not None:
            current_time = self.get_clock().now()
            stationary_timeout = self.get_parameter('stationary_timeout').value
            
            if self.last_seen is not None:
                time_since_seen = (current_time - self.last_seen).nanoseconds / 1e9
                if time_since_seen < stationary_timeout:
                    cx, cy = self.last_position
                    # Draw with dashed line effect or different style
                    color = (128, 128, 128)  # Gray for stationary
                    cv2.circle(debug, (int(cx), int(cy)), 10, color, 2)
                    cv2.putText(debug, f"opponent_0 (stationary)", 
                               (int(cx) - 40, int(cy) - 15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Add text to show we're publishing
        cv2.putText(debug, "opponent_det_MOG2single", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        if contour is None:
            if self.stationary and self.last_position is not None:
                cv2.putText(debug, "Opponent stationary (ID: opponent_0)", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
            else:
                cv2.putText(debug, "No opponent detected", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(debug, "Opponent detected! (ID: opponent_0)", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
        out_msg.header.frame_id = self.camera_frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_publisher.publish(out_msg)

    def cleanup_stale_data(self):
        now = self.get_clock().now()
        
        # Clean up own robot position
        with self.lock:
            if self.own_robot_last_update:
                if (now - self.own_robot_last_update).nanoseconds > 1.0e9:
                    self.own_robot_position_px = None
        
        # Clean up very old stationary opponent
        stationary_timeout = self.get_parameter('stationary_timeout').value
        if self.stationary and self.last_seen is not None:
            time_since_seen = (now - self.last_seen).nanoseconds / 1e9
            if time_since_seen > stationary_timeout * 2:  # Double timeout for cleanup
                self.last_position = None
                self.last_seen = None
                self.stationary = False
                self.get_logger().info("Removed very old stationary opponent")

def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetMOG2Single()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()