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


class OpponentDetMOG2Multiple(Node):
    """
    Opponent detector using MOG2 publishing multiple Detection2DArray.
    Detects up to N opponents and assigns persistent IDs to them.
    Uses last known positions from TF when opponents are stationary.
    """

    def __init__(self):
        super().__init__('opponent_det_MOG2multiple')

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
                ('max_opponents', 2),
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
        
        # Tracking variables for persistent IDs
        self.next_id = 0
        # Store: {id: {'last_position': (x,y), 'last_seen': time, 'stationary': bool}}
        self.tracked_opponents = OrderedDict()
        
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

        # Cleanup timer - now cleans up only very old stationary opponents
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)

        self.get_logger().info(f"Multi-Opponent MOG2 Detector initialized - detecting up to {self.get_parameter('max_opponents').value} opponents with persistent IDs")

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

    def assign_persistent_ids(self, detections):
        """
        Assign persistent IDs to detections.
        Uses last known positions when opponents are stationary.
        """
        current_time = self.get_clock().now()
        max_distance = self.get_parameter('match_distance').value
        stationary_timeout = self.get_parameter('stationary_timeout').value
        
        if not detections:
            # No current detections - return empty list
            # Tracked opponents will be kept for stationary_timeout seconds
            return []
        
        # Sort detections by size (largest first)
        detections.sort(key=lambda d: d.bbox.size_x * d.bbox.size_y, reverse=True)
        
        # Create a copy of tracked opponents for matching
        # We'll use OrderedDict to maintain some priority
        available_tracked_ids = list(self.tracked_opponents.keys())
        matched_detections = [None] * len(detections)
        
        # First, try to match detections to recently seen opponents
        for det_idx, detection in enumerate(detections):
            best_match_id = None
            min_distance = float('inf')
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            
            for opp_id in available_tracked_ids[:]:
                opp_data = self.tracked_opponents[opp_id]
                prev_cx, prev_cy = opp_data['last_position']
                
                # Calculate distance
                distance = np.sqrt((cx - prev_cx)**2 + (cy - prev_cy)**2)
                
                # Check if within threshold and better than previous match
                if distance < max_distance and distance < min_distance:
                    # Prefer recently seen opponents
                    time_since_seen = (current_time - opp_data['last_seen']).nanoseconds / 1e9
                    if time_since_seen < stationary_timeout:
                        min_distance = distance
                        best_match_id = opp_id
            
            if best_match_id is not None:
                # Assign existing ID
                detection.results[0].hypothesis.class_id = f"opponent_{best_match_id}"
                matched_detections[det_idx] = detection
                available_tracked_ids.remove(best_match_id)
                # Update tracked position and time
                self.tracked_opponents[best_match_id] = {
                    'last_position': (cx, cy),
                    'last_seen': current_time,
                    'stationary': False
                }
        
        # Assign new IDs to unmatched detections
        final_detections = []
        for i, detection in enumerate(detections):
            if matched_detections[i] is not None:
                final_detections.append(matched_detections[i])
            else:
                # Check if this might be a stationary opponent that just reappeared
                # but wasn't matched due to distance threshold
                matched_stationary = False
                cx = detection.bbox.center.position.x
                cy = detection.bbox.center.position.y
                
                for opp_id in available_tracked_ids[:]:
                    opp_data = self.tracked_opponents[opp_id]
                    prev_cx, prev_cy = opp_data['last_position']
                    
                    # Use a slightly larger threshold for stationary opponents
                    distance = np.sqrt((cx - prev_cx)**2 + (cy - prev_cy)**2)
                    if distance < max_distance * 1.5:  # Larger threshold for stationary
                        # This is likely the same opponent resuming movement
                        detection.results[0].hypothesis.class_id = f"opponent_{opp_id}"
                        final_detections.append(detection)
                        available_tracked_ids.remove(opp_id)
                        self.tracked_opponents[opp_id] = {
                            'last_position': (cx, cy),
                            'last_seen': current_time,
                            'stationary': False
                        }
                        matched_stationary = True
                        self.get_logger().info(f"Reacquired stationary opponent {opp_id}")
                        break
                
                if not matched_stationary:
                    # Need new ID
                    new_id = self.next_id
                    self.next_id += 1
                    
                    detection.results[0].hypothesis.class_id = f"opponent_{new_id}"
                    final_detections.append(detection)
                    
                    self.tracked_opponents[new_id] = {
                        'last_position': (cx, cy),
                        'last_seen': current_time,
                        'stationary': False
                    }
                    self.get_logger().info(f"New opponent detected with persistent ID: {new_id}")
        
        return final_detections

    def image_callback(self, msg):
        """Process image and publish multiple detections"""
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
        
        # Get all valid detections
        valid_detections = []
        min_area = self.get_parameter('min_contour_area').value
        max_area_limit = self.get_parameter('max_contour_area').value
        max_opponents = self.get_parameter('max_opponents').value
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area < area < max_area_limit:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect = float(w)/h
                if 0.3 < aspect < 3.0:
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = float(M['m10'] / M['m00'])
                        cy = float(M['m01'] / M['m00'])
                        
                        detection = Detection2D()
                        detection.bbox.center.position.x = cx
                        detection.bbox.center.position.y = cy
                        detection.bbox.center.theta = 0.0
                        detection.bbox.size_x = float(w)
                        detection.bbox.size_y = float(h)
                        detection.header.frame_id = self.camera_frame_id
                        
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = "opponent"
                        hypothesis.hypothesis.score = 1.0
                        
                        detection.results = [hypothesis]
                        
                        valid_detections.append(detection)

        # Limit to max_opponents
        if len(valid_detections) > max_opponents:
            valid_detections.sort(key=lambda d: d.bbox.size_x * d.bbox.size_y, reverse=True)
            valid_detections = valid_detections[:max_opponents]
        
        # Assign persistent IDs
        valid_detections = self.assign_persistent_ids(valid_detections)

        # Mark undetected opponents as stationary
        current_time = self.get_clock().now()
        for opp_id in self.tracked_opponents:
            # Check if this opponent was detected in this frame
            detected = False
            for det in valid_detections:
                if f"opponent_{opp_id}" == det.results[0].hypothesis.class_id:
                    detected = True
                    break
            
            if not detected:
                # This opponent is stationary (not moving)
                self.tracked_opponents[opp_id]['stationary'] = True
                # Keep last_position unchanged

        # Create detection array
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.camera_frame_id
        detection_array.detections = valid_detections

        # Publish
        self.detections_pub.publish(detection_array)

        # Debug
        if self.get_parameter('debug').value:
            self.publish_debug(debug_frame, valid_detections, robot_pos)

    def publish_debug(self, frame, detections, robot_pos=None):
        debug = frame.copy()
        
        # Draw robot's own position
        if robot_pos is not None:
            radius = int(self.get_parameter('ignore_radius_px').value * 
                        self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(debug, robot_pos, radius, (0, 255, 0), 2)
            cv2.putText(debug, "SELF", (robot_pos[0] - 30, robot_pos[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw current detections
        colors = [(255, 0, 0), (0, 255, 255), (255, 255, 0), (255, 0, 255)]
        
        for i, detection in enumerate(detections):
            color = colors[i % len(colors)]
            
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w = detection.bbox.size_x
            h = detection.bbox.size_y
            x = int(cx - w/2)
            y = int(cy - h/2)
            
            cv2.rectangle(debug, (x, y), (int(x + w), int(y + h)), color, 2)
            cv2.circle(debug, (int(cx), int(cy)), 5, color, -1)
            
            class_id = detection.results[0].hypothesis.class_id
            cv2.putText(debug, f"{class_id} (moving)", (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw stationary opponents (not currently detected)
        current_time = self.get_clock().now()
        stationary_timeout = self.get_parameter('stationary_timeout').value
        
        for opp_id, data in self.tracked_opponents.items():
            if data['stationary']:
                # Check if still within timeout
                time_since_seen = (current_time - data['last_seen']).nanoseconds / 1e9
                if time_since_seen < stationary_timeout:
                    cx, cy = data['last_position']
                    # Draw with dashed line effect or different style
                    color = (128, 128, 128)  # Gray for stationary
                    cv2.circle(debug, (int(cx), int(cy)), 10, color, 2)
                    cv2.putText(debug, f"opponent_{opp_id} (stationary)", 
                               (int(cx) - 40, int(cy) - 15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Header info
        cv2.putText(debug, f"opponent_det_MOG2multiple (max={self.get_parameter('max_opponents').value})", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug, f"Detected: {len(detections)} moving", (10, 55), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        stationary_count = sum(1 for d in self.tracked_opponents.values() if d['stationary'])
        cv2.putText(debug, f"Stationary: {stationary_count}", (10, 75), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 2)

        out_msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
        out_msg.header.frame_id = self.camera_frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_publisher.publish(out_msg)

    def cleanup_stale_data(self):
        """Clean up very old stationary opponents"""
        now = self.get_clock().now()
        
        # Clean up own robot position
        with self.lock:
            if self.own_robot_last_update:
                if (now - self.own_robot_last_update).nanoseconds > 1.0e9:
                    self.own_robot_position_px = None
        
        # Clean up opponents that have been stationary for too long
        stationary_timeout = self.get_parameter('stationary_timeout').value
        expired_ids = []
        
        for opp_id, data in self.tracked_opponents.items():
            if data['stationary']:
                time_since_seen = (now - data['last_seen']).nanoseconds / 1e9
                if time_since_seen > stationary_timeout * 2:  # Double timeout for cleanup
                    expired_ids.append(opp_id)
        
        for opp_id in expired_ids:
            del self.tracked_opponents[opp_id]
            self.get_logger().info(f"Removed very old stationary opponent {opp_id}")


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDetMOG2Multiple()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()