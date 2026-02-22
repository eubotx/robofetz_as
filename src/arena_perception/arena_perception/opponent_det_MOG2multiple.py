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
from std_msgs.msg import Header


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
                ('stationary_timeout', 5.0),  # INCREASED from 2.0 to 5.0 seconds
                ('match_distance', 100),
                ('cleanup_timeout_multiplier', 3.0),  # How many stationary timeouts before cleanup
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
        self.available_ids = set()  # Track IDs that can be reused
        # Store: {id: {'last_position': (x,y), 'last_seen': time, 'stationary': bool, 'stationary_since': time}}
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
        
        # Debug publisher for images
        self.debug_publisher = self.create_publisher(
            Image, 
            '/debug/detection_image', 
            10
        )
        
        # Dictionary to store publishers for each opponent's debug point
        self.debug_point_pubs = {}  # Will be filled dynamically as opponents appear

        # Cleanup timer
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)

        # Status timer
        self.status_timer = self.create_timer(5.0, self.status_callback)

        self.get_logger().info(f"Multi-Opponent MOG2 Detector initialized - detecting up to {self.get_parameter('max_opponents').value} opponents with persistent IDs")

    def status_callback(self):
        """Periodic status check"""
        self.get_logger().debug(f"Status - Tracked opponents: {len(self.tracked_opponents)}, Next ID: {self.next_id}, Available IDs: {self.available_ids}")
        # Log active point publishers
        if self.debug_point_pubs:
            self.get_logger().debug(f"Active point publishers: {list(self.debug_point_pubs.keys())}")

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

    def get_next_available_id(self):
        """Get the next available ID, reusing old ones if possible"""
        max_opponents = self.get_parameter('max_opponents').value
        
        # First, try to reuse an available ID
        if self.available_ids:
            return min(self.available_ids)  # Return smallest available ID
        
        # If no available IDs and we haven't reached max_opponents, create new ID
        if self.next_id < max_opponents:
            new_id = self.next_id
            self.next_id += 1
            return new_id
        
        # If we've reached max_opponents, we need to replace the oldest stationary opponent
        # Find the oldest stationary opponent
        oldest_stationary_id = None
        oldest_stationary_time = None
        
        for opp_id, data in self.tracked_opponents.items():
            if data.get('stationary', False):
                stationary_since = data.get('stationary_since')
                if stationary_since is not None:
                    if oldest_stationary_time is None or stationary_since < oldest_stationary_time:
                        oldest_stationary_time = stationary_since
                        oldest_stationary_id = opp_id
        
        if oldest_stationary_id is not None:
            # Remove the oldest stationary opponent and reuse its ID
            self.get_logger().info(f"Replacing oldest stationary opponent {oldest_stationary_id} with new detection")
            class_id = f"opponent_{oldest_stationary_id}"
            if class_id in self.debug_point_pubs:
                del self.debug_point_pubs[class_id]
            del self.tracked_opponents[oldest_stationary_id]
            return oldest_stationary_id
        
        # If no stationary opponent to replace, we can't add a new one
        self.get_logger().warn("Maximum opponents reached and no stationary opponents to replace")
        return None

    def assign_persistent_ids(self, detections):
        """
        Assign persistent IDs to detections.
        Uses last known positions when opponents are stationary.
        Ensures IDs don't exceed max_opponents.
        """
        current_time = self.get_clock().now()
        max_distance = self.get_parameter('match_distance').value
        stationary_timeout = self.get_parameter('stationary_timeout').value
        max_opponents = self.get_parameter('max_opponents').value
        
        if not detections:
            return []
        
        # Sort detections by size (largest first)
        detections.sort(key=lambda d: d.bbox.size_x * d.bbox.size_y, reverse=True)
        
        # Get currently active tracked IDs (those seen recently)
        active_tracked_ids = []
        for opp_id, data in self.tracked_opponents.items():
            time_since_seen = (current_time - data['last_seen']).nanoseconds / 1e9
            if time_since_seen < stationary_timeout * 2:  # Consider active if seen recently
                active_tracked_ids.append(opp_id)
        
        matched_detections = [None] * len(detections)
        
        # First, try to match detections to recently seen opponents
        for det_idx, detection in enumerate(detections):
            best_match_id = None
            min_distance = float('inf')
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            
            for opp_id in active_tracked_ids[:]:
                opp_data = self.tracked_opponents[opp_id]
                prev_cx, prev_cy = opp_data['last_position']
                
                distance = np.sqrt((cx - prev_cx)**2 + (cy - prev_cy)**2)
                
                if distance < max_distance and distance < min_distance:
                    time_since_seen = (current_time - opp_data['last_seen']).nanoseconds / 1e9
                    if time_since_seen < stationary_timeout:
                        min_distance = distance
                        best_match_id = opp_id
            
            if best_match_id is not None:
                detection.results[0].hypothesis.class_id = f"opponent_{best_match_id}"
                matched_detections[det_idx] = detection
                active_tracked_ids.remove(best_match_id)
                self.tracked_opponents[best_match_id] = {
                    'last_position': (cx, cy),
                    'last_seen': current_time,
                    'stationary': False,
                    'stationary_since': None
                }
        
        # Assign new IDs to unmatched detections
        final_detections = []
        for i, detection in enumerate(detections):
            if matched_detections[i] is not None:
                final_detections.append(matched_detections[i])
            else:
                # Try to match with stationary opponents first
                matched_stationary = False
                cx = detection.bbox.center.position.x
                cy = detection.bbox.center.position.y
                
                for opp_id in active_tracked_ids[:]:
                    opp_data = self.tracked_opponents[opp_id]
                    prev_cx, prev_cy = opp_data['last_position']
                    
                    distance = np.sqrt((cx - prev_cx)**2 + (cy - prev_cy)**2)
                    if distance < max_distance * 1.5:
                        detection.results[0].hypothesis.class_id = f"opponent_{opp_id}"
                        final_detections.append(detection)
                        active_tracked_ids.remove(opp_id)
                        self.tracked_opponents[opp_id] = {
                            'last_position': (cx, cy),
                            'last_seen': current_time,
                            'stationary': False,
                            'stationary_since': None
                        }
                        matched_stationary = True
                        self.get_logger().info(f"Reacquired stationary opponent {opp_id}")
                        break
                
                if not matched_stationary:
                    # Get next available ID (this will handle replacement if needed)
                    new_id = self.get_next_available_id()
                    
                    if new_id is not None:
                        detection.results[0].hypothesis.class_id = f"opponent_{new_id}"
                        final_detections.append(detection)
                        
                        self.tracked_opponents[new_id] = {
                            'last_position': (cx, cy),
                            'last_seen': current_time,
                            'stationary': False,
                            'stationary_since': None
                        }
                        
                        # Remove from available_ids if we're using one
                        if new_id in self.available_ids:
                            self.available_ids.remove(new_id)
                        
                        self.get_logger().info(f"New opponent detected with persistent ID: {new_id}")
                    else:
                        self.get_logger().debug(f"Ignoring detection - already tracking {max_opponents} opponents")
        
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
            try:
                self.publish_debug(debug_frame, [], None)
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug: {e}")
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

        self.get_logger().debug(f"Found {len(valid_detections)} raw detections")

        # Limit raw detections to max_opponents
        if len(valid_detections) > max_opponents:
            valid_detections.sort(key=lambda d: d.bbox.size_x * d.bbox.size_y, reverse=True)
            valid_detections = valid_detections[:max_opponents]
        
        # Assign persistent IDs
        valid_detections = self.assign_persistent_ids(valid_detections)

        # Mark undetected opponents as stationary
        current_time = self.get_clock().now()
        for opp_id in self.tracked_opponents:
            detected = False
            for det in valid_detections:
                if f"opponent_{opp_id}" == det.results[0].hypothesis.class_id:
                    detected = True
                    break
            
            if not detected:
                if not self.tracked_opponents[opp_id]['stationary']:
                    # Just became stationary
                    self.tracked_opponents[opp_id]['stationary'] = True
                    self.tracked_opponents[opp_id]['stationary_since'] = current_time
                    self.get_logger().info(f"Opponent {opp_id} became stationary")

        # Create detection array
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.camera_frame_id
        detection_array.detections = valid_detections

        # Publish
        self.detections_pub.publish(detection_array)

        # Debug
        if self.get_parameter('debug').value:
            try:
                self.publish_debug(debug_frame, valid_detections, robot_pos)
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")

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
            
            # Create or get publisher for this opponent's point
            if class_id not in self.debug_point_pubs:
                topic_name = f'/debug/point_{class_id}'
                self.debug_point_pubs[class_id] = self.create_publisher(
                    PointStamped, 
                    topic_name, 
                    10
                )
                self.get_logger().info(f"Created publisher for {class_id} on {topic_name}")
            
            # Publish point for this specific opponent
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = self.camera_frame_id
            point_msg.point.x = float(cx)
            point_msg.point.y = float(cy)
            point_msg.point.z = 0.0
            self.debug_point_pubs[class_id].publish(point_msg)

        # Draw stationary opponents (not currently detected)
        stationary_timeout = self.get_parameter('stationary_timeout').value
        
        for opp_id, data in self.tracked_opponents.items():
            if data['stationary']:
                time_since_seen = (self.get_clock().now() - data['last_seen']).nanoseconds / 1e9
                if time_since_seen < stationary_timeout:
                    cx, cy = data['last_position']
                    color = (128, 128, 128)  # Gray for stationary
                    
                    # Calculate time stationary for display
                    stationary_duration = 0
                    if data.get('stationary_since'):
                        stationary_duration = (self.get_clock().now() - data['stationary_since']).nanoseconds / 1e9
                    
                    cv2.circle(debug, (int(cx), int(cy)), 10, color, 2)
                    cv2.putText(debug, f"opponent_{opp_id} (stationary {stationary_duration:.1f}s)", 
                               (int(cx) - 60, int(cy) - 15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    
                    # Publish stationary points as well
                    class_id = f"opponent_{opp_id}"
                    if class_id not in self.debug_point_pubs:
                        topic_name = f'/debug/point_{class_id}'
                        self.debug_point_pubs[class_id] = self.create_publisher(
                            PointStamped, 
                            topic_name, 
                            10
                        )
                    
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = self.camera_frame_id
                    point_msg.point.x = float(cx)
                    point_msg.point.y = float(cy)
                    point_msg.point.z = 0.0
                    self.debug_point_pubs[class_id].publish(point_msg)

        # Header info
        cv2.putText(debug, f"opponent_det_MOG2multiple (max={self.get_parameter('max_opponents').value})", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug, f"Detected: {len(detections)} moving", (10, 55), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        stationary_count = sum(1 for d in self.tracked_opponents.values() if d['stationary'])
        cv2.putText(debug, f"Stationary: {stationary_count}", (10, 75), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 2)
        
        # Show available IDs
        cv2.putText(debug, f"Available IDs: {sorted(self.available_ids)}", (10, 95), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        if debug is None or debug.size == 0:
            self.get_logger().error("Debug frame is empty!")
            return
            
        try:
            out_msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
            out_msg.header.frame_id = self.camera_frame_id
            out_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_publisher.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert/publish debug image: {e}")

    def cleanup_stale_data(self):
        """Clean up very old stationary opponents"""
        now = self.get_clock().now()
        max_opponents = self.get_parameter('max_opponents').value
        stationary_timeout = self.get_parameter('stationary_timeout').value
        cleanup_multiplier = self.get_parameter('cleanup_timeout_multiplier').value
        
        # Clean up own robot position
        with self.lock:
            if self.own_robot_last_update:
                if (now - self.own_robot_last_update).nanoseconds > 1.0e9:
                    self.own_robot_position_px = None
        
        # Clean up opponents that have been stationary for too long
        # Use a longer timeout for actual removal (cleanup_multiplier * stationary_timeout)
        expired_ids = []
        
        for opp_id, data in self.tracked_opponents.items():
            if data['stationary']:
                time_since_seen = (now - data['last_seen']).nanoseconds / 1e9
                # Only remove if stationary for much longer than the stationary_timeout
                if time_since_seen > stationary_timeout * cleanup_multiplier:
                    expired_ids.append(opp_id)
        
        for opp_id in expired_ids:
            self.get_logger().info(f"Removing very old stationary opponent {opp_id} (not seen for {time_since_seen:.1f}s)")
            del self.tracked_opponents[opp_id]
            class_id = f"opponent_{opp_id}"
            if class_id in self.debug_point_pubs:
                del self.debug_point_pubs[class_id]
            # Add ID to available pool for reuse
            self.available_ids.add(opp_id)
        
        # Also ensure we don't have more tracked opponents than max_opponents
        # This is a safety measure
        if len(self.tracked_opponents) > max_opponents:
            # Sort by last seen time (oldest first) and remove extras
            sorted_ids = sorted(self.tracked_opponents.keys(), 
                              key=lambda id: self.tracked_opponents[id]['last_seen'])
            ids_to_remove = sorted_ids[:len(self.tracked_opponents) - max_opponents]
            
            for opp_id in ids_to_remove:
                self.get_logger().info(f"Removing excess opponent {opp_id} (max {max_opponents} enforced)")
                del self.tracked_opponents[opp_id]
                class_id = f"opponent_{opp_id}"
                if class_id in self.debug_point_pubs:
                    del self.debug_point_pubs[class_id]
                self.available_ids.add(opp_id)


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