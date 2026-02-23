#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from std_msgs.msg import Header
import numpy as np
from filterpy.kalman import KalmanFilter
from collections import OrderedDict
import threading
import yaml
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
import time


@dataclass
class TrackedOpponent:
    """Represents a single tracked opponent with Kalman filter"""
    opponent_id: int
    kf: KalmanFilter
    last_update_time: rclpy.time.Time
    last_update_source: str
    missed_detections: int = 0
    initialized: bool = True
    
    def get_position(self) -> np.ndarray:
        """Get current position estimate [x, y, z]"""
        return self.kf.x[0:3].flatten()
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate [vx, vy, vz]"""
        return self.kf.x[3:6].flatten()
    
    def get_pose_stamped(self, frame_id: str, timestamp) -> PoseStamped:
        """Get pose for visualization"""
        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(self.kf.x[0])
        pose.pose.position.y = float(self.kf.x[1])
        pose.pose.position.z = float(self.kf.x[2])
        pose.pose.orientation.w = 1.0
        return pose


class OpponentKalmanFusion(Node):
    """
    Fuses MOG2 and color detections using Kalman filtering.
    
    Subscribes to: /detections_3d (with IDs: opponent_MOG2_X, opponent_color_X)
    Publishes: 
        /fused_opponents - Unified tracks with ID opponent_X
        /debug/fused_opponent_poses - PoseArray for RViz
        /primary_opponent - Most confident opponent pose
    """
    
    def __init__(self):
        super().__init__('opponent_kalman_fusion')
        
        # =================== PARAMETERS ===================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('world_frame', 'world'),
                ('process_noise', 0.1),
                ('mog2_measurement_noise', 0.3),
                ('color_measurement_noise', 0.8),
                ('max_missed_detections', 10),
                ('max_track_age_seconds', 2.0),
                ('gating_distance', 1.0),
                ('dt_max', 0.1),
                ('publish_debug_poses', True),
                ('initial_velocity_std', 1.0),
                ('position_std', 1.0),
                ('use_mog2', True),
                ('use_color', True),
            ]
        )
        
        # =================== LOAD CONFIG ===================
        self.load_configuration()
        
        # =================== STATE ===================
        self.tracks: Dict[int, TrackedOpponent] = OrderedDict()
        self.lock = threading.Lock()
        self.last_predict_time = self.get_clock().now()
        
        # =================== COMMS ===================
        # QoS for camera topics (best effort)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber to 3D detections
        self.detections_sub = self.create_subscription(
            Detection3DArray,
            '/detections_3d',
            self.detections_callback,
            qos
        )
        
        # Publishers
        self.fused_tracks_pub = self.create_publisher(
            Detection3DArray,
            '/fused_opponents',
            10
        )
        
        self.primary_track_pub = self.create_publisher(
            PoseStamped,
            '/primary_opponent',
            10
        )
        
        # Debug publishers
        if self.publish_debug:
            self.poses_pub = self.create_publisher(
                PoseArray,
                '/debug/fused_opponent_poses',
                10
            )
        
        # Timer for cleanup and prediction (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(
            f'\n{"="*50}\n'
            f'OpponentKalmanFusion Started\n'
            f'{"="*50}\n'
            f'World frame: {self.world_frame}\n'
            f'Process noise: {self.process_noise}\n'
            f'MOG2 enabled: {self.use_mog2} (noise: {self.mog2_noise})\n'
            f'Color enabled: {self.use_color} (noise: {self.color_noise})\n'
            f'Gating distance: {self.gating_distance}m\n'
            f'Max missed: {self.max_missed}\n'
            f'Max age: {self.max_age}s\n'
            f'Publish debug: {self.publish_debug}\n'
            f'{"="*50}'
        )
    
    def load_configuration(self):
        """Load and validate configuration parameters"""
        # Frames
        self.world_frame = self.get_parameter('world_frame').value
        
        # Process noise
        self.process_noise = self.get_parameter('process_noise').value
        
        # Source-specific settings
        self.use_mog2 = self.get_parameter('use_mog2').value
        self.use_color = self.get_parameter('use_color').value
        
        self.mog2_noise = self.get_parameter('mog2_measurement_noise').value
        self.color_noise = self.get_parameter('color_measurement_noise').value
        
        # Track management
        self.max_missed = self.get_parameter('max_missed_detections').value
        self.max_age = self.get_parameter('max_track_age_seconds').value
        self.gating_distance = self.get_parameter('gating_distance').value
        self.dt_max = self.get_parameter('dt_max').value
        
        # Debug
        self.publish_debug = self.get_parameter('publish_debug_poses').value
        
        # Initial uncertainties
        self.initial_velocity_std = self.get_parameter('initial_velocity_std').value
        self.position_std = self.get_parameter('position_std').value
        
        # Build list of active sources
        self.active_sources = []
        self.source_noise = {}
        
        if self.use_mog2:
            self.active_sources.append('MOG2')
            self.source_noise['MOG2'] = self.mog2_noise
        
        if self.use_color:
            self.active_sources.append('COLOR')
            self.source_noise['COLOR'] = self.color_noise
    
    def create_kalman_filter(self, initial_position: np.ndarray) -> KalmanFilter:
        """Create and initialize a new Kalman filter"""
        kf = KalmanFilter(dim_x=6, dim_z=3)
        
        # State transition matrix (constant velocity)
        kf.F = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=float)
        
        # Measurement matrix
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ], dtype=float)
        
        # Initial state
        kf.x = np.array([
            initial_position[0],
            initial_position[1],
            initial_position[2],
            0.0,  # initial velocity
            0.0,
            0.0
        ]).reshape(6, 1)
        
        # Initial covariance
        kf.P = np.eye(6) * (self.position_std ** 2)
        kf.P[3:6, 3:6] *= (self.initial_velocity_std ** 2) / (self.position_std ** 2)
        
        # Process noise (will be updated with dt in predict)
        kf.Q = np.eye(6) * self.process_noise
        
        return kf
    
    def parse_detection_id(self, class_id: str) -> Tuple[Optional[str], Optional[int]]:
        """
        Parse class_id to extract source and opponent number.
        Expected: "opponent_MOG2_0" or "opponent_color_0"
        """
        try:
            parts = class_id.split('_')
            if len(parts) != 3 or parts[0] != 'opponent':
                return None, None
            
            source = parts[1].upper()  # MOG2 or COLOR
            opponent_id = int(parts[2])
            
            # Check if this source is active
            if source in self.active_sources:
                return source, opponent_id
            else:
                return None, None
                
        except (ValueError, IndexError):
            return None, None
    
    def predict_tracks(self, current_time):
        """Predict all tracks forward to current time"""
        dt = (current_time - self.last_predict_time).nanoseconds / 1e9
        dt = min(dt, self.dt_max)  # Clamp to avoid huge jumps
        
        if dt <= 0:
            return
        
        for track in self.tracks.values():
            # Update F matrix with actual dt
            track.kf.F[0, 3] = dt
            track.kf.F[1, 4] = dt
            track.kf.F[2, 5] = dt
            
            # Scale process noise by dt
            track.kf.Q = np.eye(6) * self.process_noise * dt
            
            # Predict
            track.kf.predict()
            
            # Increment missed detections
            track.missed_detections += 1
        
        self.last_predict_time = current_time
    
    def detections_callback(self, msg):
        """Process incoming 3D detections"""
        if not msg.detections:
            return
        
        current_time = self.get_clock().now()
        
        with self.lock:
            # Predict all tracks to current time
            self.predict_tracks(current_time)
            
            # Process each detection
            for detection in msg.detections:
                if not detection.results:
                    continue
                
                # Parse ID
                class_id = detection.results[0].hypothesis.class_id
                source, opponent_id = self.parse_detection_id(class_id)
                
                if source is None:
                    self.get_logger().debug(f'Invalid or inactive class_id: {class_id}')
                    continue
                
                # Extract measurement
                measurement = np.array([
                    detection.bbox.center.position.x,
                    detection.bbox.center.position.y,
                    detection.bbox.center.position.z
                ])
                
                # Validate measurement
                if not np.all(np.isfinite(measurement)):
                    self.get_logger().warn(f'Invalid measurement from {source}: {measurement}')
                    continue
                
                # Track management
                if opponent_id in self.tracks:
                    # Update existing track
                    track = self.tracks[opponent_id]
                    
                    # Gating check
                    predicted_pos = track.get_position()
                    distance = np.linalg.norm(measurement - predicted_pos)
                    
                    if distance <= self.gating_distance:
                        # Update with appropriate noise
                        measurement_noise = self.source_noise[source]
                        track.kf.update(measurement, measurement_noise * np.eye(3))
                        
                        # Update metadata
                        track.last_update_time = current_time
                        track.last_update_source = source
                        track.missed_detections = 0
                        
                        self.get_logger().debug(f'Updated opponent_{opponent_id} from {source}, dist: {distance:.3f}m')
                    else:
                        self.get_logger().debug(f'Rejected measurement for opponent_{opponent_id} from {source}, dist: {distance:.3f}m')
                        track.missed_detections += 1
                
                else:
                    # Create new track
                    self.get_logger().info(f'New opponent detected: opponent_{opponent_id} from {source}')
                    
                    kf = self.create_kalman_filter(measurement)
                    track = TrackedOpponent(
                        opponent_id=opponent_id,
                        kf=kf,
                        last_update_time=current_time,
                        last_update_source=source,
                        missed_detections=0
                    )
                    self.tracks[opponent_id] = track
            
            # Publish fused tracks
            self.publish_fused_tracks(current_time)
    
    def publish_fused_tracks(self, current_time):
        """Publish all active tracks as Detection3DArray"""
        if not self.tracks:
            return
        
        # Create Detection3DArray
        array_msg = Detection3DArray()
        array_msg.header.stamp = current_time.to_msg()
        array_msg.header.frame_id = self.world_frame
        
        # For primary opponent (lowest ID with recent updates)
        primary_pose = None
        
        for opponent_id, track in self.tracks.items():
            # Skip stale tracks
            if track.missed_detections > self.max_missed:
                continue
            
            age = (current_time - track.last_update_time).nanoseconds / 1e9
            if age > self.max_age:
                continue
            
            # Create detection message
            detection = Detection3D()
            detection.header = array_msg.header
            
            # Set position from Kalman filter
            pos = track.get_position()
            detection.bbox.center.position.x = float(pos[0])
            detection.bbox.center.position.y = float(pos[1])
            detection.bbox.center.position.z = float(pos[2])
            detection.bbox.center.orientation.w = 1.0
            
            # Add velocity info as size (for visualization)
            vel = track.get_velocity()
            detection.bbox.size.x = float(vel[0])
            detection.bbox.size.y = float(vel[1])
            detection.bbox.size.z = float(vel[2])
            
            # Create hypothesis with unified ID
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = f"opponent_{opponent_id}"
            
            # Calculate confidence based on recency
            if track.missed_detections == 0:
                confidence = 1.0
            else:
                confidence = 1.0 / (1.0 + track.missed_detections)
            
            hypothesis.hypothesis.score = confidence
            detection.results = [hypothesis]
            
            array_msg.detections.append(detection)
            
            # Track primary opponent (lowest ID with good confidence)
            if primary_pose is None and confidence > 0.5:
                primary_pose = track.get_pose_stamped(self.world_frame, current_time.to_msg())
        
        # Publish
        if array_msg.detections:
            self.fused_tracks_pub.publish(array_msg)
            self.get_logger().debug(f'Published {len(array_msg.detections)} fused tracks')
        
        # Publish primary opponent
        if primary_pose is not None:
            self.primary_track_pub.publish(primary_pose)
        
        # Publish debug poses
        if self.publish_debug and array_msg.detections:
            pose_array = PoseArray()
            pose_array.header = array_msg.header
            
            for detection in array_msg.detections:
                pose = PoseStamped()
                pose.header = array_msg.header
                pose.pose = detection.bbox.center
                pose_array.poses.append(pose.pose)
            
            self.poses_pub.publish(pose_array)
    
    def timer_callback(self):
        """Periodic cleanup and prediction"""
        current_time = self.get_clock().now()
        
        with self.lock:
            # Predict all tracks
            self.predict_tracks(current_time)
            
            # Remove stale tracks
            stale_ids = []
            for opponent_id, track in self.tracks.items():
                age = (current_time - track.last_update_time).nanoseconds / 1e9
                
                if (track.missed_detections > self.max_missed or 
                    age > self.max_age):
                    stale_ids.append(opponent_id)
            
            for opponent_id in stale_ids:
                self.get_logger().info(f'Removing stale track opponent_{opponent_id}')
                del self.tracks[opponent_id]
            
            # Publish tracks
            if self.tracks:
                self.publish_fused_tracks(current_time)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentKalmanFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down OpponentKalmanFusion')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()