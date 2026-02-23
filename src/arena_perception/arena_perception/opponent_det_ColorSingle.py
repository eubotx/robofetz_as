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


class OpponentDetColorSingle(Node):
    """
    Opponent detector using color-based tracking.
    Detects a single opponent with persistent ID (opponent_0 format).
    Supports both parameter-based color selection and interactive clicking.
    """

    def __init__(self):
        super().__init__('opponent_det_ColorSingle')

        # =================== PARAMETERS ===================
        self.declare_parameters(
            namespace='',
            parameters=[
                # Color ranges (HSV format)
                ('hue_min', 0),
                ('hue_max', 179),
                ('sat_min', 50),
                ('sat_max', 255),
                ('val_min', 50),
                ('val_max', 255),
                
                # Detection parameters
                ('min_contour_area', 200),
                ('max_contour_area', 8000),
                ('ignore_radius_px', 60),
                ('shadow_expansion_factor', 1.0),
                
                # Color selection mode
                ('interactive_selection', True),  # True = click to select, False = use parameters
                ('selection_window_name', 'Color Selector - Click on opponent'),
                
                # Tracking parameters
                ('stationary_timeout', 2.0),
                ('match_distance', 100),
                
                # Debug
                ('debug', True),
                
                # Frames
                ('robot_base_frame', 'robot_base'),
                ('camera_optical_frame', 'arena_camera_optical'),
            ]
        )

        # =================== INIT ===================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Color range (will be updated by interactive selection if enabled)
        self.lower_color = np.array([
            self.get_parameter('hue_min').value,
            self.get_parameter('sat_min').value,
            self.get_parameter('val_min').value
        ])
        self.upper_color = np.array([
            self.get_parameter('hue_max').value,
            self.get_parameter('sat_max').value,
            self.get_parameter('val_max').value
        ])
        
        # Interactive selection variables
        self.interactive_mode = self.get_parameter('interactive_selection').value
        self.selection_complete = not self.interactive_mode  # Skip if not interactive
        self.selection_points = []  # Store clicked points
        self.selection_frame = None  # Current frame for selection
        self.selection_window_name = self.get_parameter('selection_window_name').value
        
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
        
        # Tracking variables
        self.opponent_id = 'color_0'  # Fixed ID for single opponent
        self.last_position = None
        self.last_seen = None
        self.stationary = False
        
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
        
        self.detections_pub = self.create_publisher(
            Detection2DArray, 
            '/detections_2d', 
            10
        )
        
        self.debug_publisher = self.create_publisher(
            Image, 
            '/debug/color_detection_image', 
            10
        )

        # Cleanup timer
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)

        self.get_logger().info("OpponentDetColorSingle initialized")
        if self.interactive_mode:
            self.get_logger().info("Interactive mode: Click on opponent in video window to select color")
        else:
            self.get_logger().info(f"Using parameter color range: H({self.lower_color[0]}-{self.upper_color[0]}) "
                                  f"S({self.lower_color[1]}-{self.upper_color[1]}) "
                                  f"V({self.lower_color[2]}-{self.upper_color[2]})")

    def mouse_callback(self, event, x, y, flags, param):
        """Mouse callback for color selection"""
        if event == cv2.EVENT_LBUTTONDOWN and self.selection_frame is not None:
            # Get the color at clicked position
            if y < self.selection_frame.shape[0] and x < self.selection_frame.shape[1]:
                # Convert to HSV
                hsv_frame = cv2.cvtColor(self.selection_frame, cv2.COLOR_BGR2HSV)
                clicked_color = hsv_frame[y, x]
                
                self.selection_points.append((x, y, clicked_color))
                
                # Define color range based on clicked color
                hue = clicked_color[0]
                sat = clicked_color[1]
                val = clicked_color[2]
                
                # Create range with tolerance
                hue_range = 10  # Hue tolerance
                sat_range = 50  # Saturation tolerance
                val_range = 50  # Value tolerance
                
                self.lower_color = np.array([
                    max(0, hue - hue_range),
                    max(0, sat - sat_range),
                    max(0, val - val_range)
                ])
                self.upper_color = np.array([
                    min(179, hue + hue_range),
                    min(255, sat + sat_range),
                    min(255, val + val_range)
                ])
                
                self.get_logger().info(f"Selected color at ({x}, {y}): HSV={clicked_color}")
                self.get_logger().info(f"Color range: H({self.lower_color[0]}-{self.upper_color[0]}) "
                                      f"S({self.lower_color[1]}-{self.upper_color[1]}) "
                                      f"V({self.lower_color[2]}-{self.upper_color[2]})")
                
                # Mark selection as complete after first click
                self.selection_complete = True
                
                # Display confirmation
                cv2.circle(self.selection_frame, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(self.selection_frame, "Color Selected!", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

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
            pass

    def process_color_detection(self, cv_image):
        """Process image using color-based detection"""
        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create mask based on color range
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        
        # Apply morphological operations to clean up mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Apply ego-robot masking if position known
        with self.lock:
            robot_pos = self.own_robot_position_px
            
        if robot_pos is not None:
            robot_mask = np.zeros(mask.shape, dtype=np.uint8)
            radius = int(self.get_parameter('ignore_radius_px').value * 
                        self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(robot_mask, robot_pos, radius, 255, -1)
            mask = cv2.bitwise_and(mask, mask, mask=cv2.bitwise_not(robot_mask))
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        return contours, mask, robot_pos

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

        # Handle interactive color selection
        if self.interactive_mode and not self.selection_complete:
            # Show selection window
            self.selection_frame = debug_frame.copy()
            cv2.putText(self.selection_frame, "Click on opponent to select color", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(self.selection_frame, "Press 'q' when done", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show window and wait for click
            cv2.imshow(self.selection_window_name, self.selection_frame)
            cv2.setMouseCallback(self.selection_window_name, self.mouse_callback)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or self.selection_complete:
                cv2.destroyWindow(self.selection_window_name)
                if not self.selection_complete:
                    self.selection_complete = True  # Force completion even without selection
                    self.get_logger().warn("No color selected, using default parameters")
            
            # Don't proceed with detection until selection is complete
            if not self.selection_complete:
                return

        # Process color detection
        contours, mask, robot_pos = self.process_color_detection(cv_image)
        
        # Find best contour (largest area within limits)
        best_contour = None
        max_area = 0
        min_area = self.get_parameter('min_contour_area').value
        max_area_limit = self.get_parameter('max_contour_area').value
        current_position = None
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area < area < max_area_limit:
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
                
                detection.bbox.center.position.x = cx
                detection.bbox.center.position.y = cy
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = f"opponent_{self.opponent_id}"
                hypothesis.hypothesis.score = 1.0
                
                detection.results = [hypothesis]
                detection_array.detections.append(detection)
                
                # Update tracking
                current_time = self.get_clock().now()
                self.last_position = current_position
                self.last_seen = current_time
                self.stationary = False
        else:
            if self.last_position is not None:
                self.stationary = True

        # Publish detections
        self.detections_pub.publish(detection_array)

        # Debug
        if self.get_parameter('debug').value:
            self.publish_debug(debug_frame, best_contour, pixel_pos, robot_pos, mask)

    def publish_debug(self, frame, contour=None, pixel_pos=None, robot_pos=None, mask=None):
        debug = frame.copy()
        
        # Draw color range info if interactive mode is done
        if self.interactive_mode and self.selection_complete:
            color_text = f"Color: H({self.lower_color[0]}-{self.upper_color[0]})"
            cv2.putText(debug, color_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw robot's own position
        if robot_pos is not None:
            radius = int(self.get_parameter('ignore_radius_px').value * 
                        self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(debug, robot_pos, radius, (0, 255, 0), 2)
            cv2.putText(debug, "SELF", (robot_pos[0] - 30, robot_pos[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Optionally show mask
        if mask is not None:
            # Create colored mask overlay
            mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_colored[mask > 0] = [0, 255, 0]  # Green for detection areas
            debug = cv2.addWeighted(debug, 0.7, mask_colored, 0.3, 0)

        # Draw current detection
        if contour is not None and pixel_pos is not None:
            cv2.drawContours(debug, [contour], -1, (0, 0, 255), 3)
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(debug, pixel_pos, 5, (0, 0, 255), -1)
            
            cv2.putText(debug, f"opponent_0 (moving)", (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw stationary opponent
        elif self.stationary and self.last_position is not None:
            current_time = self.get_clock().now()
            stationary_timeout = self.get_parameter('stationary_timeout').value
            
            if self.last_seen is not None:
                time_since_seen = (current_time - self.last_seen).nanoseconds / 1e9
                if time_since_seen < stationary_timeout:
                    cx, cy = self.last_position
                    color = (128, 128, 128)  # Gray for stationary
                    cv2.circle(debug, (int(cx), int(cy)), 10, color, 2)
                    cv2.putText(debug, f"opponent_0 (stationary)", 
                               (int(cx) - 40, int(cy) - 15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Add header text
        cv2.putText(debug, "opponent_det_ColorSingle", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        if contour is None:
            if self.stationary and self.last_position is not None:
                cv2.putText(debug, f"Opponent stationary (ID: {self.opponent_id})", (10, 60), 
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
        
        with self.lock:
            if self.own_robot_last_update:
                if (now - self.own_robot_last_update).nanoseconds > 1.0e9:
                    self.own_robot_position_px = None
        
        stationary_timeout = self.get_parameter('stationary_timeout').value
        if self.stationary and self.last_seen is not None:
            time_since_seen = (now - self.last_seen).nanoseconds / 1e9
            if time_since_seen > stationary_timeout * 2:
                self.last_position = None
                self.last_seen = None
                self.stationary = False

def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetColorSingle()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()