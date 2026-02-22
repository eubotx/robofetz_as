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
from scipy.spatial.transform import Rotation


class OpponentDetContourSingle(Node):
    """
    Standalone opponent tracker using CSRT/KCF tracking.
    Can be initialized with mouse ROI selection.
    Outputs Detection2DArray with ID opponent_0.
    """

    def __init__(self):
        super().__init__('opponent_det_ContourSingle')

        # =================== PARAMETERS ===================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tracker_type', 'CSRT'),  # CSRT (more accurate) or KCF (faster)
                ('debug', True),
                ('robot_base_frame', 'robot_base'),
                ('camera_optical_frame', 'arena_camera_optical'),
                ('ignore_radius_px', 60),
                ('use_polygon_mask', False),
                ('mask_expansion', 1.1),
                ('robot_model_scale', 0.13),
                ('roi_padding', 20),
                ('enable_mouse_selection', True),
                ('max_tracker_size', 300),  # Max size before reinitialization
                ('stationary_timeout', 2.0),  # How long to keep ID without detection
            ]
        )

        # =================== ROBOT OUTLINE ===================
        scale = self.get_parameter('robot_model_scale').value
        self.robot_outline_3d = scale * np.array([
            [2, 0, 0], [0, 1, 0], [-0.5, 1, 0], [-0.5, -1, 0], [0, -1, 0]
        ])

        # =================== INIT ===================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera variables
        self.camera_matrix = None
        self.camera_matrix_inv = None
        self.image_width = 0
        self.image_height = 0
        self.camera_frame_id = "arena_camera_optical"  # Set default frame
        
        # Self-filtering variables
        self.own_robot_position_px = None
        self.own_robot_polygon_px = None
        self.own_robot_last_update = None
        self.own_robot_base_radius_px = self.get_parameter('ignore_radius_px').value
        self.lock = threading.Lock()
        
        # Tracking variables for persistent ID (always use opponent_0)
        self.opponent_id = 0  # Fixed ID for single opponent
        self.tracker = None  # OpenCV tracker instance
        self.tracker_box = None  # Current tracking box (x, y, w, h)
        self.tracking_active = False
        self.last_position = None  # Last known position (cx, cy)
        self.last_box = None  # Last known bounding box
        self.last_seen = None  # Last time opponent was detected
        self.stationary = False  # Whether opponent is currently stationary
        
        # Mouse selection variables
        self.selecting_roi = False
        self.roi_start = None
        self.roi_end = None
        self.temp_roi = None
        
        # Tracker performance monitoring
        self.tracker_failures = 0
        self.max_failures = 5
        
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
            '/debug/tracker_image', 
            10
        )

        # Cleanup timer
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)

        # Create OpenCV window for mouse selection if enabled
        if self.get_parameter('enable_mouse_selection').value:
            cv2.namedWindow('Opponent Tracker')
            cv2.setMouseCallback('Opponent Tracker', self.mouse_callback)

        self.get_logger().info(
            f"OpponentDetContourSingle initialized with {self.get_parameter('tracker_type').value} - "
            "Select ROI with mouse to start tracking"
        )

    def create_tracker(self):
        """Create a new tracker instance based on parameter."""
        tracker_type = self.get_parameter('tracker_type').value
        if tracker_type == 'CSRT':
            return cv2.TrackerCSRT_create()
        elif tracker_type == 'KCF':
            return cv2.TrackerKCF_create()
        else:
            self.get_logger().warn(f"Unknown tracker type {tracker_type}, using CSRT")
            return cv2.TrackerCSRT_create()

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for ROI selection."""
        if not self.get_parameter('enable_mouse_selection').value:
            return
            
        if event == cv2.EVENT_LBUTTONDOWN:
            self.selecting_roi = True
            self.roi_start = (x, y)
            self.roi_end = None
            self.temp_roi = None
            self.get_logger().info(f"ROI selection started at ({x}, {y})")
            
        elif event == cv2.EVENT_MOUSEMOVE and self.selecting_roi:
            self.roi_end = (x, y)
            # Update temporary ROI for display
            if self.roi_start and self.roi_end:
                x1 = min(self.roi_start[0], self.roi_end[0])
                y1 = min(self.roi_start[1], self.roi_end[1])
                x2 = max(self.roi_start[0], self.roi_end[0])
                y2 = max(self.roi_start[1], self.roi_end[1])
                self.temp_roi = (x1, y1, x2-x1, y2-y1)
                
        elif event == cv2.EVENT_LBUTTONUP:
            if self.selecting_roi and self.roi_start:
                self.roi_end = (x, y)
                
                # Calculate final ROI
                x1 = min(self.roi_start[0], self.roi_end[0])
                y1 = min(self.roi_start[1], self.roi_end[1])
                x2 = max(self.roi_start[0], self.roi_end[0])
                y2 = max(self.roi_start[1], self.roi_end[1])
                
                # Ensure minimum size
                if (x2 - x1) < 20 or (y2 - y1) < 20:
                    self.get_logger().warn("ROI too small, ignoring")
                    self.selecting_roi = False
                    self.temp_roi = None
                    return
                
                # Add padding
                padding = self.get_parameter('roi_padding').value
                x1 = max(0, x1 - padding)
                y1 = max(0, y1 - padding)
                x2 = min(self.image_width, x2 + padding)
                y2 = min(self.image_height, y2 + padding)
                
                self.initialize_tracker((x1, y1, x2-x1, y2-y1))
                
                self.selecting_roi = False
                self.temp_roi = None
                
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right click to reset tracking
            self.reset_tracking()
            self.get_logger().info("Tracking reset")

    def initialize_tracker(self, bbox):
        """Initialize tracker with given bounding box."""
        self.tracker_box = bbox
        self.tracker = self.create_tracker()
        self.tracking_active = True
        self.tracker_failures = 0
        
        # Initialize tracker with current frame
        if hasattr(self, 'last_frame') and self.last_frame is not None:
            self.tracker.init(self.last_frame, tuple(bbox))
            self.get_logger().info(f"Tracking started with ROI: {bbox}")
        else:
            self.get_logger().warn("No frame available for tracker initialization")

    def reset_tracking(self):
        """Reset tracking completely."""
        self.tracking_active = False
        self.tracker = None
        self.tracker_box = None
        self.last_position = None
        self.last_box = None
        self.last_seen = None
        self.stationary = False
        self.tracker_failures = 0

    def camera_info_callback(self, msg):
        """Process camera calibration info."""
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
        """Track own robot position for visualization."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('camera_optical_frame').value,
                'world',
                rclpy.time.Time()
            )
            
            # Get robot pose in world frame
            robot_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            robot_quat = [msg.pose.orientation.x, msg.pose.orientation.y, 
                          msg.pose.orientation.z, msg.pose.orientation.w]
            robot_rotation = Rotation.from_quat(robot_quat)
            
            # Get camera transform
            trans = transform.transform.translation
            rot = transform.transform.rotation
            cam_rotation = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w])
            cam_translation = np.array([trans.x, trans.y, trans.z])
            
            if self.camera_matrix is not None:
                # Project robot to pixel
                robot_in_cam = cam_rotation.apply(robot_position) + cam_translation
                
                if robot_in_cam[2] > 0.01:
                    px = int(self.camera_matrix[0, 0] * robot_in_cam[0] / robot_in_cam[2] + self.camera_matrix[0, 2])
                    py = int(self.camera_matrix[1, 1] * robot_in_cam[1] / robot_in_cam[2] + self.camera_matrix[1, 2])
                    
                    # Boundary clamping
                    px = int(np.clip(px, 0, self.image_width - 1))
                    py = int(np.clip(py, 0, self.image_height - 1))
                    
                    # Compute polygon projection if using polygon mask
                    polygon_px = None
                    if self.get_parameter('use_polygon_mask').value:
                        expansion = self.get_parameter('mask_expansion').value
                        polygon_px = []
                        
                        for point_3d in self.robot_outline_3d:
                            point_world = robot_rotation.apply(point_3d * expansion) + robot_position
                            point_cam = cam_rotation.apply(point_world) + cam_translation
                            
                            if point_cam[2] > 0.01:
                                px_pt = int(self.camera_matrix[0, 0] * point_cam[0] / point_cam[2] + self.camera_matrix[0, 2])
                                py_pt = int(self.camera_matrix[1, 1] * point_cam[1] / point_cam[2] + self.camera_matrix[1, 2])
                                
                                px_pt = int(np.clip(px_pt, 0, self.image_width - 1))
                                py_pt = int(np.clip(py_pt, 0, self.image_height - 1))
                                polygon_px.append([px_pt, py_pt])
                        
                        if len(polygon_px) >= 3:
                            polygon_px = np.array(polygon_px, dtype=np.int32).reshape((-1, 1, 2))
                        else:
                            polygon_px = None
                    
                    with self.lock:
                        self.own_robot_position_px = (px, py)
                        self.own_robot_polygon_px = polygon_px
                        self.own_robot_last_update = self.get_clock().now()
        except Exception as e:
            pass

    def image_callback(self, msg):
        """Process image and track opponent using CSRT/KCF."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_frame_id = msg.header.frame_id
            self.image_height, self.image_width = cv_image.shape[:2]
            self.last_frame = cv_image.copy()  # Store for tracker initialization
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
            robot_polygon = self.own_robot_polygon_px

        # === Tracking ===
        pixel_pos = None
        current_bbox = None
        current_position = None

        # Initialize tracker if we have a tracker but no frame was set
        if self.tracking_active and self.tracker is not None and not hasattr(self.tracker, 'initialized'):
            self.tracker.init(cv_image, tuple(self.tracker_box))
            self.get_logger().info("Tracker initialized with current frame")

        if self.tracking_active and self.tracker is not None:
            # Update tracker
            success, box = self.tracker.update(cv_image)
            
            if success:
                self.tracker_box = [int(v) for v in box]
                x, y, w, h = self.tracker_box
                
                # Check for tracker bloat
                max_size = self.get_parameter('max_tracker_size').value
                if w > max_size or h > max_size:
                    self.get_logger().warn(f"Tracker bloat detected ({w}x{h}), reinitializing...")
                    if self.last_box is not None:
                        self.initialize_tracker(self.last_box)
                    else:
                        self.reset_tracking()
                else:
                    # Calculate center
                    cx = float(x + w/2.0)
                    cy = float(y + h/2.0)
                    pixel_pos = (int(cx), int(cy))
                    current_position = (cx, cy)
                    
                    # Store for output
                    current_bbox = (x, y, w, h)
                    self.last_position = (cx, cy)
                    self.last_box = current_bbox
                    self.last_seen = self.get_clock().now()
                    self.stationary = False
                    
                    # Reset failure counter on success
                    self.tracker_failures = 0
            else:
                # Tracker failed
                self.tracker_failures += 1
                self.get_logger().warn(f"Tracker update failed ({self.tracker_failures}/{self.max_failures})")
                
                if self.tracker_failures >= self.max_failures:
                    self.get_logger().warn("Too many tracker failures, resetting")
                    self.reset_tracking()
                else:
                    # Mark as stationary if we have a last position
                    if self.last_position is not None:
                        self.stationary = True

        # If tracking is inactive but we have a last position, mark as stationary
        if not self.tracking_active and self.last_position is not None:
            self.stationary = True

        # Create detection array (same format as working node)
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.camera_frame_id

        if current_position is not None and current_bbox is not None:
            # Create Detection2D (same format as working node)
            detection = Detection2D()
            detection.header = detection_array.header
            
            # Set bounding box center (as floats, not ints)
            detection.bbox.center.position.x = current_position[0]
            detection.bbox.center.position.y = current_position[1]
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(current_bbox[2])  # width
            detection.bbox.size_y = float(current_bbox[3])  # height
            
            # Create ObjectHypothesisWithPose with consistent ID
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = f"opponent_{self.opponent_id}"
            hypothesis.hypothesis.score = 1.0
            
            detection.results = [hypothesis]
            detection_array.detections.append(detection)
            
        # Also publish stationary opponent if within timeout
        elif self.stationary and self.last_position is not None:
            current_time = self.get_clock().now()
            stationary_timeout = self.get_parameter('stationary_timeout').value
            
            if self.last_seen is not None:
                time_since_seen = (current_time - self.last_seen).nanoseconds / 1e9
                if time_since_seen < stationary_timeout and self.last_box is not None:
                    # Publish last known position with lower confidence
                    detection = Detection2D()
                    detection.header = detection_array.header
                    
                    detection.bbox.center.position.x = self.last_position[0]
                    detection.bbox.center.position.y = self.last_position[1]
                    detection.bbox.center.theta = 0.0
                    detection.bbox.size_x = float(self.last_box[2])
                    detection.bbox.size_y = float(self.last_box[3])
                    
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = f"opponent_{self.opponent_id}"
                    hypothesis.hypothesis.score = 0.5  # Lower confidence for stationary
                    
                    detection.results = [hypothesis]
                    detection_array.detections.append(detection)

        # Publish
        self.detections_pub.publish(detection_array)

        # Debug
        if self.get_parameter('debug').value:
            self.publish_debug(debug_frame, pixel_pos, robot_pos, current_bbox)

    def publish_debug(self, frame, pixel_pos=None, robot_pos=None, bbox=None):
        """Publish debug visualization."""
        debug = frame.copy()
        
        # Draw robot's own position
        use_poly = self.get_parameter('use_polygon_mask').value
        
        if use_poly and self.own_robot_polygon_px is not None:
            # Draw polygon mask
            cv2.polylines(debug, [self.own_robot_polygon_px], True, (0, 255, 255), 2)
            
            # Add red tint to masked area
            overlay = debug.copy()
            mask_viz = np.zeros(debug.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask_viz, [self.own_robot_polygon_px], 255)
            overlay[mask_viz > 0] = (0, 0, 255)
            cv2.addWeighted(overlay, 0.3, debug, 0.7, 0, debug)
            cv2.putText(debug, "SELF (MASKED)", (10, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        elif robot_pos is not None:
            radius = int(self.get_parameter('ignore_radius_px').value)
            cv2.circle(debug, robot_pos, radius, (0, 255, 0), 2)
            cv2.putText(debug, "SELF", (robot_pos[0] - 30, robot_pos[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw tracker info
        tracker_type = self.get_parameter('tracker_type').value
        
        # Add node name to debug
        cv2.putText(debug, "opponent_det_ContourSingle", (10, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if self.tracking_active:
            # Draw current tracking box
            if self.tracker_box is not None:
                x, y, w, h = self.tracker_box
                cv2.rectangle(debug, (x, y), (x+w, y+h), (255, 0, 255), 2)
                cv2.putText(debug, f"TRACKING ({tracker_type})", (x, y-5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            
            # Draw center point for active tracking
            if pixel_pos is not None:
                cv2.circle(debug, pixel_pos, 5, (0, 0, 255), -1)
                cv2.putText(debug, f"opponent_0 (moving)", (pixel_pos[0] - 30, pixel_pos[1] - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw stationary opponent if applicable
        elif self.stationary and self.last_position is not None and self.last_box is not None:
            current_time = self.get_clock().now()
            stationary_timeout = self.get_parameter('stationary_timeout').value
            
            if self.last_seen is not None:
                time_since_seen = (current_time - self.last_seen).nanoseconds / 1e9
                if time_since_seen < stationary_timeout:
                    cx, cy = self.last_position
                    x, y, w, h = self.last_box
                    
                    # Gray for stationary
                    color = (128, 128, 128)
                    
                    # Draw bounding box (dashed effect)
                    for i in range(0, w, 10):
                        if i + 5 < w:
                            cv2.line(debug, (x + i, y), (x + i + 5, y), color, 2)
                            cv2.line(debug, (x + i, y + h), (x + i + 5, y + h), color, 2)
                    
                    for i in range(0, h, 10):
                        if i + 5 < h:
                            cv2.line(debug, (x, y + i), (x, y + i + 5), color, 2)
                            cv2.line(debug, (x + w, y + i), (x + w, y + i + 5), color, 2)
                    
                    # Draw center
                    cv2.circle(debug, (int(cx), int(cy)), 5, color, 2)
                    cv2.putText(debug, f"opponent_0 (stationary)", 
                               (int(cx) - 40, int(cy) - 15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Draw temporary ROI during selection
        if self.selecting_roi and self.temp_roi is not None:
            x, y, w, h = self.temp_roi
            cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.putText(debug, "SELECTING...", (x, y-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Draw last known position if not tracking and no stationary
        if not self.tracking_active and not self.stationary and self.last_position is not None:
            cx, cy = self.last_position
            cv2.circle(debug, (int(cx), int(cy)), 10, (128, 128, 128), 2)
            cv2.putText(debug, f"opponent_0 (last known)", 
                       (int(cx) - 40, int(cy) - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

        # Instructions and status
        cv2.putText(debug, "Left click + drag: Select ROI to track", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(debug, "Right click: Reset tracking", (10, 55), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Status
        if self.tracking_active:
            status_color = (0, 255, 0)
            status_text = f"TRACKING ACTIVE ({tracker_type})"
        else:
            status_color = (0, 0, 255)
            status_text = "TRACKING INACTIVE - Select ROI"
        
        cv2.putText(debug, status_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        if self.tracker_failures > 0:
            cv2.putText(debug, f"Failures: {self.tracker_failures}/{self.max_failures}", 
                       (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

        # Show in OpenCV window
        cv2.imshow('Opponent Tracker', debug)
        cv2.waitKey(1)

        # Publish to ROS
        out_msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
        out_msg.header.frame_id = self.camera_frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_publisher.publish(out_msg)

    def cleanup_stale_data(self):
        """Clean up stale data."""
        now = self.get_clock().now()
        
        with self.lock:
            if self.own_robot_last_update:
                if (now - self.own_robot_last_update).nanoseconds > 1.0e9:
                    self.own_robot_position_px = None
                    self.own_robot_polygon_px = None
        
        # Clean up very old stationary opponent
        stationary_timeout = self.get_parameter('stationary_timeout').value
        if self.stationary and self.last_seen is not None:
            time_since_seen = (now - self.last_seen).nanoseconds / 1e9
            if time_since_seen > stationary_timeout * 2:  # Double timeout for cleanup
                self.last_position = None
                self.last_box = None
                self.last_seen = None
                self.stationary = False
                self.get_logger().info("Removed very old stationary opponent")


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetContourSingle()
    
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