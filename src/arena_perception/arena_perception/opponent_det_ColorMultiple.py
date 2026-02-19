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
from collections import OrderedDict, defaultdict
import json
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
import math


@dataclass
class TrackedOpponent:
    """Data class for tracked opponent"""
    id: str
    color_range_lower: np.ndarray  # HSV lower bound
    color_range_upper: np.ndarray  # HSV upper bound
    last_position: Optional[Tuple[float, float]] = None
    last_seen: Optional[float] = None
    stationary: bool = False
    first_seen: float = None
    color_name: str = "unknown"
    velocity: Tuple[float, float] = (0.0, 0.0)  # pixels per second
    last_velocity_update: Optional[float] = None
    
    def __post_init__(self):
        if self.first_seen is None:
            self.first_seen = time.time()
    
    def update_position(self, new_x: float, new_y: float, current_time: float):
        """Update position and calculate velocity"""
        if self.last_position is not None and self.last_seen is not None:
            dt = current_time - self.last_seen
            if dt > 0:
                dx = new_x - self.last_position[0]
                dy = new_y - self.last_position[1]
                self.velocity = (dx / dt, dy / dt)
                self.last_velocity_update = current_time
        
        self.last_position = (new_x, new_y)
        self.last_seen = current_time
        self.stationary = False


class OpponentDetColorMultiple(Node):
    """
    Opponent detector using color-based tracking for multiple opponents.
    Detects multiple opponents with persistent IDs.
    Supports both parameter-based and interactive color selection.
    """

    def __init__(self):
        super().__init__('opponent_det_ColorMultiple')

        # =================== PARAMETERS ===================
        self.declare_parameters(
            namespace='',
            parameters=[
                # Detection parameters
                ('min_contour_area', 200),
                ('max_contour_area', 8000),
                ('ignore_radius_px', 100),
                ('shadow_expansion_factor', 1.2),
                ('contour_smoothing', 3),  # Kernel size for morphological operations
                
                # Color selection mode
                ('interactive_selection', True),
                ('selection_window_name', 'Color Selector - Click on opponents'),
                ('max_opponents', 5),  # Maximum number of opponents to track
                ('selection_timeout', 30.0),  # Seconds to wait for color selection
                
                # Predefined colors (for non-interactive mode)
                # Format: {"opponent_1": {"hue_min":0, "hue_max":10, "sat_min":100, "sat_max":255, "val_min":100, "val_max":255, "name":"red"}}
                ('opponent_configs', '{}'),  # JSON string
                
                # Tracking parameters
                ('stationary_timeout', 2.0),
                ('match_distance', 150),  # Pixel distance for matching
                ('min_time_between_detections', 0.1),
                ('max_prediction_time', 0.5),  # How far to predict position when missing
                ('velocity_smoothing', 0.7),  # Low-pass filter for velocity
                
                # Debug
                ('debug', True),
                ('publish_mask', False),  # Whether to publish segmentation mask
                
                # Frames
                ('robot_base_frame', 'robot_base'),
                ('camera_optical_frame', 'arena_camera_optical'),
            ]
        )

        # =================== INIT ===================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Tracking variables
        self.opponents: Dict[str, TrackedOpponent] = OrderedDict()
        self.next_id = 0
        self.opponent_colors = {}  # Maps color ranges to opponent IDs
        
        # Interactive selection variables
        self.interactive_mode = self.get_parameter('interactive_selection').value
        self.selection_active = self.interactive_mode
        self.selection_points = []  # Store clicked points
        self.selection_frame = None
        self.selection_window_name = self.get_parameter('selection_window_name').value
        self.selection_start_time = None
        self.max_opponents = self.get_parameter('max_opponents').value
        
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
        
        # Detection history for debugging
        self.detection_history = defaultdict(list)
        self.max_history = 100
        
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
        
        # Optional mask publisher
        if self.get_parameter('publish_mask').value:
            self.mask_publisher = self.create_publisher(
                Image,
                '/debug/color_mask',
                10
            )

        # Cleanup timer
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)

        # Initialize with predefined opponents if not in interactive mode
        if not self.interactive_mode:
            self.load_predefined_opponents()
        
        self.get_logger().info("OpponentDetColorMultiple initialized")
        if self.interactive_mode:
            self.get_logger().info(f"Interactive mode: Click on up to {self.max_opponents} opponents")
            self.get_logger().info("Press 'q' when done, 'r' to reset selection")
        else:
            self.get_logger().info(f"Loaded {len(self.opponents)} predefined opponents")

    def load_predefined_opponents(self):
        """Load opponent configurations from JSON parameter"""
        try:
            config_json = self.get_parameter('opponent_configs').value
            if config_json:
                configs = json.loads(config_json)
                for opp_id, opp_config in configs.items():
                    lower = np.array([
                        opp_config.get('hue_min', 0),
                        opp_config.get('sat_min', 50),
                        opp_config.get('val_min', 50)
                    ])
                    upper = np.array([
                        opp_config.get('hue_max', 179),
                        opp_config.get('sat_max', 255),
                        opp_config.get('val_max', 255)
                    ])
                    
                    opponent = TrackedOpponent(
                        id=opp_id,
                        color_range_lower=lower,
                        color_range_upper=upper,
                        color_name=opp_config.get('name', 'unknown')
                    )
                    self.opponents[opp_id] = opponent
                    self.get_logger().info(f"Loaded opponent {opp_id} with color {opponent.color_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to load predefined opponents: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        """Mouse callback for interactive color selection"""
        if event == cv2.EVENT_LBUTTONDOWN and self.selection_frame is not None:
            if len(self.opponents) >= self.max_opponents:
                self.get_logger().warn(f"Maximum number of opponents ({self.max_opponents}) reached")
                return
                
            if y < self.selection_frame.shape[0] and x < self.selection_frame.shape[1]:
                # Convert to HSV
                hsv_frame = cv2.cvtColor(self.selection_frame, cv2.COLOR_BGR2HSV)
                clicked_color = hsv_frame[y, x]
                
                # Create color range with tolerance
                hue_range = 10
                sat_range = 50
                val_range = 50
                
                lower = np.array([
                    max(0, clicked_color[0] - hue_range),
                    max(0, clicked_color[1] - sat_range),
                    max(0, clicked_color[2] - val_range)
                ])
                upper = np.array([
                    min(179, clicked_color[0] + hue_range),
                    min(255, clicked_color[1] + sat_range),
                    min(255, clicked_color[2] + val_range)
                ])
                
                # Create new opponent
                opp_id = f"opponent_{self.next_id}"
                self.next_id += 1
                
                # Try to guess color name
                color_name = self.guess_color_name(clicked_color[0])
                
                opponent = TrackedOpponent(
                    id=opp_id,
                    color_range_lower=lower,
                    color_range_upper=upper,
                    color_name=color_name
                )
                
                self.opponents[opp_id] = opponent
                
                self.get_logger().info(f"Added {opp_id} ({color_name}) at ({x}, {y}) - HSV={clicked_color}")
                
                # Draw marker on frame
                cv2.circle(self.selection_frame, (x, y), 8, (0, 255, 0), 2)
                cv2.putText(self.selection_frame, f"{opp_id}", (x + 10, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def guess_color_name(self, hue):
        """Guess color name based on hue value"""
        if hue < 10 or hue > 170:
            return "red"
        elif 10 <= hue < 25:
            return "orange"
        elif 25 <= hue < 35:
            return "yellow"
        elif 35 <= hue < 85:
            return "green"
        elif 85 <= hue < 130:
            return "blue"
        elif 130 <= hue < 160:
            return "purple"
        else:
            return "pink"

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

    def process_color_detections(self, cv_image):
        """Process image and detect all opponents by color"""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        with self.lock:
            robot_pos = self.own_robot_position_px
        
        detections = {}  # opponent_id -> (contour, centroid, bbox)
        
        for opp_id, opponent in self.opponents.items():
            # Create mask for this opponent's color range
            mask = cv2.inRange(hsv, opponent.color_range_lower, opponent.color_range_upper)
            
            # Apply morphological operations
            kernel_size = self.get_parameter('contour_smoothing').value
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Remove robot's own position
            if robot_pos is not None:
                robot_mask = np.zeros(mask.shape, dtype=np.uint8)
                radius = int(self.get_parameter('ignore_radius_px').value * 
                            self.get_parameter('shadow_expansion_factor').value)
                cv2.circle(robot_mask, robot_pos, radius, 255, -1)
                mask = cv2.bitwise_and(mask, mask, mask=cv2.bitwise_not(robot_mask))
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Find best contour for this opponent (largest area within limits)
            best_contour = None
            max_area = 0
            min_area = self.get_parameter('min_contour_area').value
            max_area_limit = self.get_parameter('max_contour_area').value
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if min_area < area < max_area_limit:
                    if area > max_area:
                        max_area = area
                        best_contour = cnt
            
            if best_contour is not None:
                # Get centroid
                M = cv2.moments(best_contour)
                if M['m00'] > 0:
                    cx = float(M['m10'] / M['m00'])
                    cy = float(M['m01'] / M['m00'])
                    x, y, w, h = cv2.boundingRect(best_contour)
                    
                    detections[opp_id] = {
                        'contour': best_contour,
                        'centroid': (cx, cy),
                        'bbox': (x, y, w, h),
                        'area': max_area
                    }
        
        return detections

    def match_and_update_tracks(self, detections, current_time):
        """Match detections to existing tracks and update"""
        matched_ids = set()
        
        # First, try to match detections to existing opponents
        for opp_id, detection in detections.items():
            if opp_id in self.opponents:
                # Direct match by color range
                opponent = self.opponents[opp_id]
                cx, cy = detection['centroid']
                
                # Check if this detection is close to predicted position
                predicted_pos = self.predict_position(opponent, current_time)
                if predicted_pos is not None:
                    dist = math.hypot(cx - predicted_pos[0], cy - predicted_pos[1])
                    if dist > self.get_parameter('match_distance').value:
                        # Too far from prediction, might be wrong match
                        continue
                
                opponent.update_position(cx, cy, current_time)
                matched_ids.add(opp_id)
                self.detection_history[opp_id].append((current_time, cx, cy))
        
        # Handle opponents not detected this frame
        for opp_id, opponent in self.opponents.items():
            if opp_id not in matched_ids:
                if opponent.last_seen is not None:
                    time_since_seen = current_time - opponent.last_seen
                    if time_since_seen > self.get_parameter('stationary_timeout').value:
                        opponent.stationary = True

    def predict_position(self, opponent: TrackedOpponent, current_time: float):
        """Predict current position based on velocity"""
        if opponent.last_position is None or opponent.last_seen is None:
            return None
        
        time_since_seen = current_time - opponent.last_seen
        max_prediction = self.get_parameter('max_prediction_time').value
        
        if time_since_seen > max_prediction:
            return None
        
        # Linear prediction
        pred_x = opponent.last_position[0] + opponent.velocity[0] * time_since_seen
        pred_y = opponent.last_position[1] + opponent.velocity[1] * time_since_seen
        
        # Clamp to image boundaries
        pred_x = max(0, min(self.image_width, pred_x))
        pred_y = max(0, min(self.image_height, pred_y))
        
        return (pred_x, pred_y)

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
        if self.interactive_mode and self.selection_active:
            self.handle_interactive_selection(debug_frame)
            return

        # Process detections
        current_time = time.time()
        detections = self.process_color_detections(cv_image)
        self.match_and_update_tracks(detections, current_time)
        
        # Create detection array
        detection_array = self.create_detection_array(detections, current_time)
        self.detections_pub.publish(detection_array)
        
        # Publish debug image
        if self.get_parameter('debug').value:
            self.publish_debug(debug_frame, detections, current_time)

    def handle_interactive_selection(self, frame):
        """Handle interactive color selection mode"""
        if self.selection_start_time is None:
            self.selection_start_time = time.time()
            cv2.namedWindow(self.selection_window_name)
            cv2.setMouseCallback(self.selection_window_name, self.mouse_callback)
        
        # Check timeout
        elapsed = time.time() - self.selection_start_time
        timeout = self.get_parameter('selection_timeout').value
        
        if elapsed > timeout:
            self.get_logger().info("Selection timeout reached")
            self.selection_active = False
            cv2.destroyWindow(self.selection_window_name)
            return
        
        # Update display
        self.selection_frame = frame.copy()
        
        # Draw instructions
        cv2.putText(self.selection_frame, f"Click on opponents to select colors ({len(self.opponents)}/{self.max_opponents})", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(self.selection_frame, "Press 'q' to finish, 'r' to reset", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(self.selection_frame, f"Timeout: {int(timeout - elapsed)}s", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show window
        cv2.imshow(self.selection_window_name, self.selection_frame)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            self.selection_active = False
            cv2.destroyWindow(self.selection_window_name)
            self.get_logger().info(f"Selection completed with {len(self.opponents)} opponents")
        elif key == ord('r'):
            self.opponents.clear()
            self.next_id = 0
            self.get_logger().info("Selection reset")

    def create_detection_array(self, detections, current_time):
        """Create Detection2DArray message from detections"""
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.camera_frame_id
        
        for opp_id, detection in detections.items():
            if opp_id in self.opponents:
                opponent = self.opponents[opp_id]
                cx, cy = detection['centroid']
                x, y, w, h = detection['bbox']
                
                detection_msg = Detection2D()
                detection_msg.header = detection_array.header
                
                detection_msg.bbox.center.position.x = cx
                detection_msg.bbox.center.position.y = cy
                detection_msg.bbox.center.theta = 0.0
                detection_msg.bbox.size_x = float(w)
                detection_msg.bbox.size_y = float(h)
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = opp_id
                hypothesis.hypothesis.score = 1.0
                
                detection_msg.results = [hypothesis]
                detection_array.detections.append(detection_msg)
        
        # Add stationary opponents (using predicted positions)
        for opp_id, opponent in self.opponents.items():
            if opp_id not in detections and opponent.stationary and opponent.last_position is not None:
                time_since_seen = current_time - opponent.last_seen
                if time_since_seen < self.get_parameter('stationary_timeout').value:
                    # Use predicted position for stationary opponents
                    predicted_pos = self.predict_position(opponent, current_time)
                    if predicted_pos is not None:
                        detection_msg = Detection2D()
                        detection_msg.header = detection_array.header
                        
                        detection_msg.bbox.center.position.x = predicted_pos[0]
                        detection_msg.bbox.center.position.y = predicted_pos[1]
                        detection_msg.bbox.center.theta = 0.0
                        detection_msg.bbox.size_x = 50.0  # Default size
                        detection_msg.bbox.size_y = 50.0
                        
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = opp_id
                        hypothesis.hypothesis.score = 0.5  # Lower confidence for stationary
                        
                        detection_msg.results = [hypothesis]
                        detection_array.detections.append(detection_msg)
        
        return detection_array

    def publish_debug(self, frame, detections=None, current_time=None):
        """Publish debug image with visualizations"""
        debug = frame.copy()
        
        with self.lock:
            robot_pos = self.own_robot_position_px
        
        # Draw robot's own position
        if robot_pos is not None:
            radius = int(self.get_parameter('ignore_radius_px').value * 
                        self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(debug, robot_pos, radius, (0, 255, 0), 2)
            cv2.putText(debug, "SELF", (robot_pos[0] - 30, robot_pos[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw all opponents
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
        
        if detections:
            for i, (opp_id, detection) in enumerate(detections.items()):
                color = colors[i % len(colors)]
                opponent = self.opponents.get(opp_id)
                
                if opponent:
                    cx, cy = detection['centroid']
                    x, y, w, h = detection['bbox']
                    
                    # Draw contour and bounding box
                    cv2.drawContours(debug, [detection['contour']], -1, color, 2)
                    cv2.rectangle(debug, (x, y), (x+w, y+h), color, 2)
                    cv2.circle(debug, (int(cx), int(cy)), 5, color, -1)
                    
                    # Draw ID and info
                    info_text = f"{opp_id} ({opponent.color_name})"
                    cv2.putText(debug, info_text, (x, y - 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # Draw velocity vector if available
                    if opponent.velocity[0] != 0 or opponent.velocity[1] != 0:
                        end_x = int(cx + opponent.velocity[0] * 0.5)  # Scale for visualization
                        end_y = int(cy + opponent.velocity[1] * 0.5)
                        cv2.arrowedLine(debug, (int(cx), int(cy)), (end_x, end_y), color, 2)
        
        # Draw stationary opponents
        if current_time:
            for opp_id, opponent in self.opponents.items():
                if detections and opp_id in detections:
                    continue
                    
                if opponent.stationary and opponent.last_position is not None:
                    time_since_seen = current_time - opponent.last_seen
                    if time_since_seen < self.get_parameter('stationary_timeout').value:
                        # Draw predicted position
                        predicted_pos = self.predict_position(opponent, current_time)
                        if predicted_pos:
                            color = (128, 128, 128)  # Gray for stationary
                            cv2.circle(debug, (int(predicted_pos[0]), int(predicted_pos[1])), 
                                      10, color, 2)
                            cv2.putText(debug, f"{opp_id} (stationary)", 
                                       (int(predicted_pos[0]) - 40, int(predicted_pos[1]) - 15), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Add header
        cv2.putText(debug, f"opponent_det_ColorMultiple ({len(self.opponents)} tracked)", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        out_msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
        out_msg.header.frame_id = self.camera_frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_publisher.publish(out_msg)

    def cleanup_stale_data(self):
        """Remove stale tracking data"""
        now = time.time()
        
        with self.lock:
            if self.own_robot_last_update:
                dt = (self.get_clock().now() - self.own_robot_last_update).nanoseconds / 1e9
                if dt > 1.0:
                    self.own_robot_position_px = None
        
        # Clean up very old stationary opponents
        stationary_timeout = self.get_parameter('stationary_timeout').value
        to_remove = []
        
        for opp_id, opponent in self.opponents.items():
            if opponent.stationary and opponent.last_seen is not None:
                time_since_seen = now - opponent.last_seen
                if time_since_seen > stationary_timeout * 3:
                    to_remove.append(opp_id)
        
        for opp_id in to_remove:
            del self.opponents[opp_id]
            self.get_logger().info(f"Removed stale opponent {opp_id}")
        
        # Clean up detection history
        current_time = time.time()
        for opp_id in list(self.detection_history.keys()):
            self.detection_history[opp_id] = [
                entry for entry in self.detection_history[opp_id]
                if current_time - entry[0] < 10.0  # Keep last 10 seconds
            ]


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetColorMultiple()
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