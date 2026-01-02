#!/usr/bin/env python3
#ros2 run arena_perception opponent_tracker_node --ros-arg -r /bot/pose:=/pose-sim

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import threading
from scipy.spatial.transform import Rotation


class SimpleKalmanFilter2D:
    """
    Simple 2D Kalman filter for tracking position and velocity.
    State vector: [x, y, vx, vy]
    """
    
    def __init__(self, process_noise=0.1, measurement_noise=0.5, dt=0.033):
        """
        Args:
            process_noise: Process noise covariance
            measurement_noise: Measurement noise covariance
            dt: Time step (default ~30 FPS)
        """
        self.dt = dt
        
        # State vector [x, y, vx, vy]
        self.x = np.zeros(4)
        
        # State covariance matrix
        self.P = np.eye(4) * 1000  # High initial uncertainty
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement matrix (we only measure position)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Process noise covariance
        self.Q = np.eye(4) * process_noise
        
        # Measurement noise covariance
        self.R = np.eye(2) * measurement_noise
        
        self.initialized = False
    
    def initialize(self, position):
        """Initialize filter with first measurement."""
        self.x[0:2] = position
        self.x[2:4] = 0  # Zero initial velocity
        self.initialized = True
    
    def predict(self):
        """Predict next state."""
        if not self.initialized:
            return None
        
        # Predict state
        self.x = self.F @ self.x
        
        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x[0:2]  # Return predicted position
    
    def update(self, measurement):
        """Update with new measurement."""
        if not self.initialized:
            self.initialize(measurement)
            return self.x[0:2]
        
        # Measurement residual
        z = np.array(measurement)
        y = z - self.H @ self.x
        
        # Residual covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        self.P = (np.eye(4) - K @ self.H) @ self.P
        
        return self.x[0:2]  # Return filtered position
    
    def get_velocity(self):
        """Get current velocity estimate."""
        if not self.initialized:
            return np.array([0.0, 0.0])
        return self.x[2:4]
    
    def get_speed(self):
        """Get current speed (velocity magnitude)."""
        vel = self.get_velocity()
        return np.linalg.norm(vel)
    
    def damp_velocity(self, factor=0.5):
        """Damp the velocity component of the state."""
        self.x[2:4] *= factor


class FrameDiffRobotDetector(Node):
    """
    Robust Opponent Tracker using MOG2 (Search) and CSRT (Lock) with Kalman filtering.
    Features:
    - Three-state machine: SEARCHING, TRACKING, LOST
    - Kalman filter for smooth position/velocity estimation
    - Confidence scoring
    - Orientation smoothing
    - Motion continuity validation (debris rejection)
    """

    def __init__(self):
        super().__init__('frame_diff_oponent_detector')

        # =================== PARAMETERS ===================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_contour_area', 300),
                ('max_contour_area', 10000),
                ('background_history', 500),
                ('var_threshold', 16),
                ('ignore_radius_px', 60),
                ('shadow_expansion_factor', 1.2),
                ('debug', True),
                ('robot_base_frame', 'robot_base'),
                ('camera_optical_frame', 'arena_camera_optical'),
                ('restrict_to_spawn', False),  # Disabled by default - search full arena
                ('spawn_roi_coords', [1.0, 1.0, 1.5, 1.5]),
                ('track_orientation', True),
                ('tracker_type', 'CSRT'),  # KCF is faster than CSRT
                ('smoothing_alpha', 0.2),
                ('max_tracker_size', 150),
                ('motion_threshold', 0.02),
                ('bbox_padding', 20),  # Padding to expand initial bounding box
                # New parameters
                ('orientation_smoothing_alpha', 0.3),
                ('min_confidence_threshold', 0.5),
                ('lost_state_timeout', 1.0),
                ('min_tracking_duration', 0.3),
                ('no_motion_timeout', 2.0),
                ('kalman_process_noise', 0.1),  # Restore original smoothness
                ('kalman_measurement_noise', 0.5),
                ('use_polygon_mask', True),  # Use polygon mask instead of circle
                ('mask_expansion', 1.3),  # Expansion factor for polygon mask
            ]
        )

        # =================== ROBOT OUTLINE (same as robot_detection.py) ===================
        # 3D polygon defining robot shape in robot_base frame (x forward, y left, z up)
        self.robot_outline_3d = 0.13 * np.array([
            [0, -4, 0], [2, 0, 0], [2, 1, 0], [-2, 1, 0], [-2, 0, 0]
        ])

        # =================== INIT ===================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MOG2
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
        self.image_width = 1280  # Default to common resolution until image arrives
        self.image_height = 720
        self.camera_frame_id = ""
        self.own_robot_position_px = None
        self.own_robot_polygon_px = None  # Polygon mask for own robot
        self.own_robot_last_update = None
        self.own_robot_base_radius_px = self.get_parameter('ignore_radius_px').value
        
        # =================== STATE MACHINE ===================
        self.state = "SEARCHING"  # SEARCHING, TRACKING, LOST
        self.tracker = None
        self.tracker_box = None
        self.first_acquisition = True  # Only use ROI on first search
        
        # =================== KALMAN FILTER ===================
        process_noise = self.get_parameter('kalman_process_noise').value
        measurement_noise = self.get_parameter('kalman_measurement_noise').value
        self.kalman_filter = SimpleKalmanFilter2D(process_noise, measurement_noise)
        
        # =================== TRACKING STATE ===================
        self.tracking_confidence = 0.0  # 0.0 to 1.0
        self.tracking_duration = 0.0  # Time spent in TRACKING state
        self.lost_duration = 0.0  # Time spent in LOST state
        self.no_motion_frames = 0  # Frames without MOG2 motion detection
        
        # =================== SMOOTHED OUTPUTS ===================
        self.smooth_world_pos = None
        self.smooth_world_orientation = None
        self.smooth_pixel_orientation = None
        
        # =================== PREVIOUS STATE FOR MOTION ===================
        self.prev_world_pos = None
        self.prev_pixel_pos = None
        self.last_known_world_pos = None

        self.lock = threading.Lock()

        # =================== COMMS ===================
        self.image_sub = self.create_subscription(Image, '/arena_camera/image_rect', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/arena_camera/camera_info', self.camera_info_callback, 10)
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/bot/pose', self.robot_pose_callback, 10)
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/detected_robot/pose', 10)
        self.debug_publisher = self.create_publisher(Image, '/debug/detection_image', 10)

        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)
        
        # Timer for tracking duration updates
        self.last_time = self.get_clock().now()

        self.get_logger().info("Enhanced Opponent Tracker initialized (Kalman + 3-State + Confidence)")

    def create_tracker(self):
        """Create a new tracker instance."""
        t_type = self.get_parameter('tracker_type').value
        if t_type == 'CSRT':
            return cv2.TrackerCSRT_create()
        else:
            return cv2.TrackerKCF_create()

    def camera_info_callback(self, msg):
        """Process camera calibration info using Projection matrix P for rectified images."""
        if self.camera_matrix is not None:
            return
        # Use first 3x3 of P matrix for rectified images
        self.camera_matrix = np.array(msg.p).reshape(3, 4)[:, :3]
        self.image_width = msg.width
        self.image_height = msg.height
        try:
            self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)
            self.get_logger().info("Camera info received (using P matrix).")
        except Exception as e:
            self.get_logger().error(f"Failed to invert camera matrix: {e}")

    def robot_pose_callback(self, msg):
        """Track own robot position and compute outline polygon for masking with synchronized TF."""
        try:
            # 1. TF Timing Synchronization - use message timestamp
            t_msg = msg.header.stamp
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter('camera_optical_frame').value,
                    'world',
                    t_msg,
                    rclpy.duration.Duration(seconds=0.1)
                )
                tf_type = "synced"
            except Exception as e:
                # Fallback to latest if specific time fails
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.get_parameter('camera_optical_frame').value,
                        'world',
                        rclpy.time.Time()
                    )
                    tf_type = "latest"
                    if self._debug_counter % 50 == 0:
                        self.get_logger().warn(f"TF Timestamp sync failed: {e}. Falling back to 'latest'.")
                except Exception as e_latest:
                     # This is the critical failure case
                     self.get_logger().error(f"TF Lookup FAILED completely: {e_latest}. This causes 'NO ROBOT POSE'.")
                     return

            if self._debug_counter % 50 == 0:
                self.get_logger().info(f"TF Lookup ({tf_type}) success. Time: {t_msg.sec}.{t_msg.nanosec}")

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
                # 2. Robust Projection using Rotation.apply()
                robot_in_cam = cam_rotation.apply(robot_position) + cam_translation
                
                # Z-depth check (Z > 0.01 to avoid singular projection)
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
                            # Transform point from robot_base to world using rotation object
                            point_world = robot_rotation.apply(point_3d * expansion) + robot_position
                            # Transform from world to camera
                            point_cam = cam_rotation.apply(point_world) + cam_translation
                            
                            # Z-depth check for each point
                            if point_cam[2] > 0.01:
                                px_pt = int(self.camera_matrix[0, 0] * point_cam[0] / point_cam[2] + self.camera_matrix[0, 2])
                                py_pt = int(self.camera_matrix[1, 1] * point_cam[1] / point_cam[2] + self.camera_matrix[1, 2])
                                
                                # Clamping
                                px_pt = int(np.clip(px_pt, 0, self.image_width - 1))
                                py_pt = int(np.clip(py_pt, 0, self.image_height - 1))
                                polygon_px.append([px_pt, py_pt])
                        
                        if len(polygon_px) >= 3:
                            # Standard OpenCV contour format: (N, 1, 2)
                            polygon_px = np.array(polygon_px, dtype=np.int32).reshape((-1, 1, 2))
                        else:
                            polygon_px = None
                    
                    with self.lock:
                        self.own_robot_position_px = (px, py)
                        self.own_robot_polygon_px = polygon_px
                        self.own_robot_last_update = self.get_clock().now()
                    
                    if self._debug_counter % 50 == 0:
                        self.get_logger().info(f"DEBUG MASK: Robot World={robot_position}, Robot Cam={robot_in_cam}")
                        self.get_logger().info(f"DEBUG MASK: Cam Matrix (first 3x3 of P):\n{self.camera_matrix}")
                        self.get_logger().info(f"DEBUG MASK: Robot projected at ({px}, {py}), poly_pts={len(polygon_px) if polygon_px is not None else 0}")
                else:
                    if self._debug_counter % 50 == 0:
                        self.get_logger().warn(f"DEBUG MASK: Robot in cam Z <= 0.01: {robot_in_cam[2]:.3f}. Robot World: {robot_position}")
            else:
                if self._debug_counter % 50 == 0:
                    self.get_logger().warn("DEBUG MASK: Waiting for camera_matrix in robot_pose_callback")
                        
        except Exception as e:
            self.get_logger().error(f"DEBUG MASK Error: {str(e)}", throttle_duration_sec=2.0)

    def image_callback(self, msg):
        """Main processing callback."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_frame_id = msg.header.frame_id
            self.image_height, self.image_width = cv_image.shape[:2]
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")
            return

        # IMPORTANT: Ensure deep copy - cv_bridge may return a view
        debug_frame = np.copy(cv_image)
        
        # Log image info periodically for debugging
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        if self._debug_counter % 100 == 1:
            self.get_logger().info(f"Image received: {cv_image.shape}, dtype={cv_image.dtype}, mean={np.mean(cv_image):.1f}")
        
        if self.camera_matrix is None:
            cv2.putText(debug_frame, "WAITING FOR CAMERA INFO...", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            self.publish_debug(debug_frame)
            return

        # Calculate dt for timing
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        with self.lock:
            robot_pos = self.own_robot_position_px
            robot_polygon = self.own_robot_polygon_px

        pixel_pos = None
        target_contour = None
        current_bbox = None
        roi_polygon = None

        # ==========================================================
        # STATE: TRACKING
        # ==========================================================
        if self.state == "TRACKING" and self.tracker is not None:
            success, box = self.tracker.update(cv_image)
            if success:
                self.tracker_box = [int(v) for v in box]
                x, y, w, h = self.tracker_box
                
                # Bloat prevention
                max_size = self.get_parameter('max_tracker_size').value
                if w > max_size or h > max_size:
                    self.get_logger().warn(f"Tracker Bloat Detected ({w}x{h})! Transitioning to LOST...")
                    self.transition_to_lost()
                    current_bbox = self.tracker_box  # Keep for debug visualization
                else:
                    # Hybrid refinement with MOG2
                    proc_frame = cv_image.copy()
                    fg_mask = self.bg_subtractor.apply(proc_frame)
                    _, thresh = cv2.threshold(fg_mask, 250, 255, cv2.THRESH_BINARY)
                    
                    # Ego mask - use polygon if available, else circle
                    if robot_polygon is not None and len(robot_polygon) >= 3:
                        mask = np.zeros(thresh.shape, dtype=np.uint8)
                        cv2.fillPoly(mask, [robot_polygon], 255)
                        thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))
                    if robot_pos is not None:
                        mask = np.zeros(thresh.shape, dtype=np.uint8)
                        radius = int(self.own_robot_base_radius_px * self.get_parameter('shadow_expansion_factor').value)
                        cv2.circle(mask, robot_pos, radius, 255, -1)
                        thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))
                    
                    # Create mask for tracker area
                    local_mask = np.zeros(thresh.shape, dtype=np.uint8)
                    cv2.rectangle(local_mask, (x, y), (x+w, y+h), 255, -1)
                    refined_thresh = cv2.bitwise_and(thresh, local_mask)
                    
                    # Find best contour within tracker box
                    refined_contours, _ = cv2.findContours(refined_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    best_refined = None
                    max_area = 0
                    for cnt in refined_contours:
                        area = cv2.contourArea(cnt)
                        if area > 100 and area > max_area:
                            max_area = area
                            best_refined = cnt
                    
                    new_pixel_pos = None
                    min_rect = None
                    
                    if best_refined is not None:
                        # Problem 2: Inaccurate Bounding Box - Use Rotated Rect
                        min_rect = cv2.minAreaRect(best_refined) # ((x,y), (w,h), angle)
                        new_pixel_pos = (int(min_rect[0][0]), int(min_rect[0][1]))
                    
                    # SIMPLIFIED: Any motion is significant (restore sensitivity)
                    is_significant_motion = (new_pixel_pos is not None)

                    if new_pixel_pos is not None:
                        # Motion detected - good! 
                        self.no_motion_frames = 0
                        
                        # Problem 3: Tracker-MOG2 Conflict Resolution
                        # Check distance between Tracker center and MOG2 center
                        tracker_center = (x + w//2, y + h//2)
                        dx = new_pixel_pos[0] - tracker_center[0]
                        dy = new_pixel_pos[1] - tracker_center[1]
                        dist = np.sqrt(dx*dx + dy*dy)
                        
                        if dist > 20.0:
                            # Conflict! MOG2 is far from tracker. Trust MOG2 (the physical measurement).
                            # Snap tracker box to MOG2 center.
                            nx = int(new_pixel_pos[0] - w/2)
                            ny = int(new_pixel_pos[1] - h/2)
                            self.tracker_box = (nx, ny, w, h)
                            pixel_pos = new_pixel_pos
                            # Optional: Could re-init tracker here if shape changed significantly
                        else:
                            # Good alignment. Use MOG2 as authoritative position for accuracy.
                            pixel_pos = new_pixel_pos
                            
                        # Problem 2 continued: Smart Padding
                        # If MOG2 shape is significantly different from tracker box, update box size
                        # Calculate bounding box of rotated rect for sizing
                        box_points = cv2.boxPoints(min_rect)
                        bx, by, bw, bh = cv2.boundingRect(box_points)
                        
                        # Apply Smart Padding (20%)
                        pad_x = int(bw * 0.1)
                        pad_y = int(bh * 0.1)
                        # Updates tracker box size only if significantly different (avoid jitter)
                        # (Implementation note: mutating tracker_box width/height blindly causes jitter, 
                        #  so we only re-center as per logic above, or re-init if area mismatch is huge)
                        
                        # Store this valid motion POS
                        self.last_valid_motion_pos = pixel_pos
                        
                        # Increase confidence (motion detected)
                        self.tracking_confidence = min(1.0, self.tracking_confidence + 0.1)
                        
                        # Save rect for visualization
                        self.last_min_rect = min_rect
                    else:
                        # No motion detected
                        self.no_motion_frames += 1
                        
                        # Default to tracker center if no motion measurement
                        pixel_pos = (x + w//2, y + h//2)
                        
                        # OVERSHOOT FIX: Problem 4 - Kalman Tuning
                        # When stationary for > 15 frames (~0.5s), freeze velocity but keep position update
                        if self.no_motion_frames > 15: 
                            self.kalman_filter.damp_velocity(0.0) # Kill momentum completely
                            status_text = "STATIONARY"
                            color = (0, 0, 255) # Red
                        else:
                            self.kalman_filter.damp_velocity(0.5) # Gentle damping
                            status_text = "MOVING"
                            color = (0, 255, 0) # Green
                            
                        # Store status for debug
                        self.debug_status_text = status_text
                        self.debug_status_color = color
                        
                        # Very slow confidence decay - tracker is working, just no motion
                        self.tracking_confidence = max(0.3, self.tracking_confidence - 0.01)
                        
                        # Log occasionally
                        if self.no_motion_frames % 60 == 0:
                            self.get_logger().info(f"Tracking stationary object for {self.no_motion_frames/30:.1f}s")
                    
                    current_bbox = self.tracker_box
                    self.tracking_duration += dt
            else:
                # Tracker failed
                self.get_logger().warn("Tracker Lost! Transitioning to LOST state...")
                self.transition_to_lost()

        # ==========================================================
        # STATE: LOST
        # ==========================================================
        elif self.state == "LOST":
            self.lost_duration += dt
            
            # Try to reacquire using MOG2
            proc_frame = cv_image.copy()
            fg_mask = self.bg_subtractor.apply(proc_frame)
            _, thresh = cv2.threshold(fg_mask, 250, 255, cv2.THRESH_BINARY)
            
            # Ego mask - use polygon if available, else circle
            if robot_polygon is not None and len(robot_polygon) >= 3:
                mask = np.zeros(thresh.shape, dtype=np.uint8)
                cv2.fillPoly(mask, [robot_polygon], 255)
                thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))
            elif robot_pos is not None:
                mask = np.zeros(thresh.shape, dtype=np.uint8)
                radius = int(self.own_robot_base_radius_px * self.get_parameter('shadow_expansion_factor').value)
                cv2.circle(mask, robot_pos, radius, 255, -1)
                thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))

            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            min_area = self.get_parameter('min_contour_area').value
            best_cnt = None
            max_area = 0
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > min_area:
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect = float(w)/h
                    if 0.3 < aspect < 3.0 and area > max_area:
                        max_area = area
                        best_cnt = cnt
            
            if best_cnt is not None:
                # Reacquired!
                target_contour = best_cnt
                pixel_pos = self.get_robot_centroid(best_cnt)
                x, y, w, h = cv2.boundingRect(best_cnt)
                
                # Expand bounding box with padding
                padding = self.get_parameter('bbox_padding').value
                x = max(0, x - padding)
                y = max(0, y - padding)
                w = min(self.image_width - x, w + 2 * padding)
                h = min(self.image_height - y, h + 2 * padding)
                current_bbox = (x, y, w, h)
                
                # Re-lock tracker
                self.tracker = self.create_tracker()
                self.tracker.init(cv_image, current_bbox)
                self.tracker_box = current_bbox
                self.state = "TRACKING"
                self.lost_duration = 0.0
                self.no_motion_frames = 0
                self.get_logger().info(f"Target reacquired at {pixel_pos}! Back to TRACKING.")
            else:
                # Use Kalman prediction
                predicted_pos = self.kalman_filter.predict()
                if predicted_pos is not None:
                    pixel_pos = self.world_to_pixel(predicted_pos[0], predicted_pos[1])
                
                # Check timeout
                lost_timeout = self.get_parameter('lost_state_timeout').value
                if self.lost_duration > lost_timeout:
                    self.get_logger().warn("LOST timeout exceeded. Resetting to SEARCHING...")
                    self.transition_to_searching()

        # ==========================================================
        # STATE: SEARCHING
        # ==========================================================
        elif self.state == "SEARCHING":
            proc_frame = cv_image.copy()
            fg_mask = self.bg_subtractor.apply(proc_frame)
            _, thresh = cv2.threshold(fg_mask, 250, 255, cv2.THRESH_BINARY)
            
            # Ego mask - use polygon if available, else circle
            if robot_polygon is not None and len(robot_polygon) >= 3:
                mask = np.zeros(thresh.shape, dtype=np.uint8)
                cv2.fillPoly(mask, [robot_polygon], 255)
                thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))
            elif robot_pos is not None:
                mask = np.zeros(thresh.shape, dtype=np.uint8)
                radius = int(self.own_robot_base_radius_px * self.get_parameter('shadow_expansion_factor').value)
                cv2.circle(mask, robot_pos, radius, 255, -1)
                thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))

            # ROI mask (ONLY on first acquisition - after that search full arena)
            if self.get_parameter('restrict_to_spawn').value and self.first_acquisition:
                coords = self.get_parameter('spawn_roi_coords').value
                world_corners = [(coords[0], coords[1]), (coords[2], coords[1]), 
                                 (coords[2], coords[3]), (coords[0], coords[3])]
                pixel_corners = [self.world_to_pixel(wx, wy) for wx, wy in world_corners]
                if all(p is not None for p in pixel_corners):
                    roi_polygon = np.array(pixel_corners, dtype=np.int32)
                    roi_mask = np.zeros(thresh.shape, dtype=np.uint8)
                    cv2.fillPoly(roi_mask, [roi_polygon], 255)
                    thresh = cv2.bitwise_and(thresh, thresh, mask=roi_mask)

            # Find biggest moving contour
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            min_area = self.get_parameter('min_contour_area').value
            best_cnt = None
            max_area = 0
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > min_area:
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect = float(w)/h
                    if 0.3 < aspect < 3.0 and area > max_area:
                        max_area = area
                        best_cnt = cnt
            
            if best_cnt is not None:
                target_contour = best_cnt
                pixel_pos = self.get_robot_centroid(best_cnt)
                x, y, w, h = cv2.boundingRect(best_cnt)
                
                # Expand bounding box with padding for better initial coverage
                padding = self.get_parameter('bbox_padding').value
                x = max(0, x - padding)
                y = max(0, y - padding)
                w = min(self.image_width - x, w + 2 * padding)
                h = min(self.image_height - y, h + 2 * padding)
                current_bbox = (x, y, w, h)
                
                # Lock on
                self.tracker = self.create_tracker()
                self.tracker.init(cv_image, current_bbox)
                self.tracker_box = current_bbox
                self.state = "TRACKING"
                self.tracking_duration = 0.0
                self.tracking_confidence = 0.5  # Start with medium confidence
                self.no_motion_frames = 0
                self.first_acquisition = False  # After first lock, search full arena
                self.get_logger().info(f"Target Locked at {pixel_pos} with bbox {current_bbox}! Transitioning to TRACKING.")

        # ==========================================================
        # PROCESS DETECTION & PUBLISH
        # ==========================================================
        if pixel_pos is not None:
            world_pos = self.pixel_to_world(pixel_pos[0], pixel_pos[1])
            
            if world_pos is not None:
                # Update Kalman filter
                # If we are stationary, we already force-set the Kalman state in the TRACKING block above.
                # But if we just fell through without hitting that block (e.g. tracking just started), update normally.
                if not (self.no_motion_frames > 3 and self.state == "TRACKING"):
                    filtered_pos = self.kalman_filter.update(world_pos[0:2])
                else:
                    # Just use current state (which was force-set or predicted)
                    filtered_pos = self.kalman_filter.x[0:2]
                
                # Smooth world position
                alpha = self.get_parameter('smoothing_alpha').value
                if self.smooth_world_pos is None:
                    self.smooth_world_pos = filtered_pos
                else:
                    self.smooth_world_pos = alpha * filtered_pos + (1.0 - alpha) * self.smooth_world_pos
                
                self.last_known_world_pos = self.smooth_world_pos
                
                # Orientation tracking
                if self.get_parameter('track_orientation').value:
                    self.update_orientation(pixel_pos)
                
                # Publish if confidence is high enough and tracking duration met
                min_confidence = self.get_parameter('min_confidence_threshold').value
                min_duration = self.get_parameter('min_tracking_duration').value
                
                if self.tracking_confidence >= min_confidence and self.tracking_duration >= min_duration:
                    orientation = self.smooth_world_orientation if self.smooth_world_orientation is not None else 0.0
                    self.publish_pose(self.smooth_world_pos, orientation)
        else:
            # Predict using Kalman
            if self.state == "LOST":
                predicted_pos = self.kalman_filter.predict()
                if predicted_pos is not None and self.last_known_world_pos is not None:
                    orientation = self.smooth_world_orientation if self.smooth_world_orientation is not None else 0.0
                    self.publish_pose(predicted_pos, orientation)

        if self.get_parameter('debug').value:
            orientation = self.smooth_pixel_orientation if self.smooth_pixel_orientation is not None else 0.0
            self.publish_debug(debug_frame, target_contour, pixel_pos, robot_pos, 
                             orientation, roi_polygon, current_bbox)

    def update_orientation(self, pixel_pos):
        """Update orientation estimates with smoothing."""
        if self.smooth_world_pos is None:
            return
        
        m_thresh = self.get_parameter('motion_threshold').value
        ori_alpha = self.get_parameter('orientation_smoothing_alpha').value
        
        # World frame orientation
        if self.prev_world_pos is not None:
            dx = self.smooth_world_pos[0] - self.prev_world_pos[0]
            dy = self.smooth_world_pos[1] - self.prev_world_pos[1]
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > m_thresh:
                new_orientation = np.arctan2(dy, dx)
                
                if self.smooth_world_orientation is None:
                    self.smooth_world_orientation = new_orientation
                else:
                    # Handle angle wrapping
                    angle_diff = (new_orientation - self.smooth_world_orientation + np.pi) % (2*np.pi) - np.pi
                    self.smooth_world_orientation += ori_alpha * angle_diff
                
                self.prev_world_pos = self.smooth_world_pos.copy()
        else:
            self.prev_world_pos = self.smooth_world_pos.copy()
        
        # Pixel frame orientation (for visualization)
        if self.prev_pixel_pos is not None:
            pdx = pixel_pos[0] - self.prev_pixel_pos[0]
            pdy = pixel_pos[1] - self.prev_pixel_pos[1]
            pdist = np.sqrt(pdx**2 + pdy**2)
            
            if pdist > 10:
                new_pixel_orientation = np.arctan2(pdy, pdx)
                
                if self.smooth_pixel_orientation is None:
                    self.smooth_pixel_orientation = new_pixel_orientation
                else:
                    # Handle angle wrapping
                    angle_diff = (new_pixel_orientation - self.smooth_pixel_orientation + np.pi) % (2*np.pi) - np.pi
                    self.smooth_pixel_orientation += ori_alpha * angle_diff
                
                self.prev_pixel_pos = pixel_pos
        else:
            self.prev_pixel_pos = pixel_pos

    def transition_to_lost(self):
        """Transition to LOST state."""
        self.state = "LOST"
        self.tracker = None
        self.lost_duration = 0.0
        self.tracking_confidence = max(0.0, self.tracking_confidence - 0.3)

    def transition_to_searching(self):
        """Transition to SEARCHING state (full reset)."""
        self.state = "SEARCHING"
        self.tracker = None
        self.tracker_box = None
        self.tracking_confidence = 0.0
        self.tracking_duration = 0.0
        self.lost_duration = 0.0
        self.no_motion_frames = 0
        self.smooth_world_pos = None
        self.smooth_world_orientation = None
        self.smooth_pixel_orientation = None
        self.prev_world_pos = None
        self.prev_pixel_pos = None
        self.kalman_filter = SimpleKalmanFilter2D(
            self.get_parameter('kalman_process_noise').value,
            self.get_parameter('kalman_measurement_noise').value
        )

    def publish_debug(self, frame, contour=None, pixel_pos=None, robot_pos=None, 
                      orientation=0.0, roi_polygon=None, bbox=None):
        """Publish debug visualization."""
        # State and confidence display
        conf_color = (0, 255, 0) if self.tracking_confidence > 0.7 else (0, 165, 255) if self.tracking_confidence > 0.4 else (0, 0, 255)
        cv2.putText(frame, f"STATE: {self.state}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"CONF: {self.tracking_confidence:.2f}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, conf_color, 2)
        
        if self.state == "TRACKING":
            cv2.putText(frame, f"DUR: {self.tracking_duration:.1f}s", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"NO_MOT: {self.no_motion_frames}", (10, 115), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        elif self.state == "LOST":
            cv2.putText(frame, f"LOST: {self.lost_duration:.1f}s", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        if roi_polygon is not None:
            cv2.polylines(frame, [roi_polygon], True, (255, 255, 0), 2)

        if robot_pos is not None:
            radius = int(self.own_robot_base_radius_px * self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(frame, robot_pos, radius, (0, 255, 0), 1)
            cv2.circle(frame, robot_pos, 3, (0, 255, 0), -1)
            
        # Draw robot polygon if exists (Enhanced Debug Visualization)
        with self.lock:
            poly = self.own_robot_polygon_px
            rpos = robot_pos
            
        if poly is not None:
            # Draw shaded overlay for "where no tracking is done"
            overlay = frame.copy()
            cv2.fillPoly(overlay, [poly], (0, 100, 100)) # Dark yellow/olive
            cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)
            
            # Draw polygon edges
            cv2.polylines(frame, [poly], True, (0, 255, 255), 2)
            # Draw numbered points
            for i in range(len(poly)):
                pt = tuple(poly[i][0])
                cv2.circle(frame, pt, 3, (255, 255, 0), -1)
                cv2.putText(frame, str(i), pt, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(frame, "SELF-POLY", tuple(poly[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # Log masking stats periodically
            if self._debug_counter % 100 == 1:
                self.get_logger().info(f"Robot Mask Overlay: center={robot_pos}, points={len(poly)}")
        else:
            if rpos is not None:
                cv2.circle(frame, rpos, 5, (0, 0, 255), -1)
                cv2.putText(frame, "NO POLY", (rpos[0]+10, rpos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                cv2.putText(frame, "NO ROBOT POSE", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Bounding box
        if bbox is not None:
            x, y, w, h = [int(v) for v in bbox]
            if self.state == "SEARCHING":
                color = (0, 0, 255)
            elif self.state == "TRACKING":
                color = (255, 0, 255)
            else:  # LOST
                color = (0, 165, 255)
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            
        # Draw Rotated Rect (Blue) - Problem 5
        if hasattr(self, 'last_min_rect') and self.last_min_rect is not None:
            box_points = cv2.boxPoints(self.last_min_rect)
            box_points = np.int0(box_points)
            cv2.drawContours(frame, [box_points], 0, (255, 0, 0), 2)

        # Draw Velocity Vector (Red) - Problem 5
        if self.kalman_filter.initialized:
            vx, vy = self.kalman_filter.x[2], self.kalman_filter.x[3]
            # Scale velocity for visibility (e.g. * 20 pixels)
            start_pt = (int(self.kalman_filter.x[0]), int(self.kalman_filter.x[1]))
            end_pt = (int(self.kalman_filter.x[0] + vx * 20), int(self.kalman_filter.x[1] + vy * 20))
            cv2.arrowedLine(frame, start_pt, end_pt, (0, 0, 255), 2, tipLength=0.3)
            
        # Draw Status Text
        if hasattr(self, 'debug_status_text'):
            cv2.putText(frame, self.debug_status_text, (200, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.debug_status_color, 2)

        if pixel_pos is not None:
            cv2.circle(frame, pixel_pos, 5, (255, 255, 255), -1)
            if self.get_parameter('track_orientation').value and orientation is not None:
                lx, ly = int(40 * np.cos(orientation)), int(40 * np.sin(orientation))
                cv2.line(frame, pixel_pos, (pixel_pos[0]+lx, pixel_pos[1]+ly), (255, 255, 0), 2)

        # Debug: log frame info before publishing
        if self._debug_counter % 100 == 1:
            self.get_logger().info(f"Debug frame to publish: {frame.shape}, dtype={frame.dtype}, mean={np.mean(frame):.1f}")

        out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        out_msg.header.frame_id = self.camera_frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_publisher.publish(out_msg)

    def get_robot_centroid(self, contour):
        """Calculate centroid of contour."""
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return None
        return (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

    def pixel_to_world(self, u, v):
        """Convert pixel coordinates to world coordinates."""
        try:
            transform = self.tf_buffer.lookup_transform('world', self.get_parameter('camera_optical_frame').value, rclpy.time.Time())
            trans = transform.transform.translation
            cam_pos = np.array([trans.x, trans.y, trans.z])
            rot = transform.transform.rotation
            R = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
            pixel_hom = np.array([u, v, 1.0])
            ray_cam = self.camera_matrix_inv @ pixel_hom
            ray_cam /= np.linalg.norm(ray_cam)
            ray_world = R @ ray_cam
            if abs(ray_world[2]) < 1e-6:
                return None
            t = -cam_pos[2] / ray_world[2]
            return cam_pos + t * ray_world
        except:
            return None
        
    def world_to_pixel(self, x, y):
        """Convert world coordinates to pixel coordinates."""
        try:
            transform = self.tf_buffer.lookup_transform(self.get_parameter('camera_optical_frame').value, 'world', rclpy.time.Time())
            world_pt = PointStamped()
            world_pt.header.frame_id = 'world'
            world_pt.point.x, world_pt.point.y, world_pt.point.z = float(x), float(y), 0.0
            cam_pt = do_transform_point(world_pt, transform)
            pixel_hom = self.camera_matrix @ np.array([cam_pt.point.x, cam_pt.point.y, cam_pt.point.z])
            if pixel_hom[2] > 0:
                return (int(pixel_hom[0] / pixel_hom[2]), int(pixel_hom[1] / pixel_hom[2]))
            return None
        except:
            return None

    def publish_pose(self, pos, orient):
        """Publish opponent pose."""
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(pos[0]), float(pos[1]), 0.0
        q = Rotation.from_euler('z', orient).as_quat() if self.get_parameter('track_orientation').value else [0,0,0,1]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.pose_publisher.publish(msg)

    def cleanup_stale_data(self):
        """Clean up stale own robot position."""
        now = self.get_clock().now()
        with self.lock:
            if self.own_robot_last_update:
                if (now - self.own_robot_last_update).nanoseconds > 1.0e9:
                    self.own_robot_position_px = None


def main(args=None):
    rclpy.init(args=args)
    node = FrameDiffRobotDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()