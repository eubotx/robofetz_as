#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Pose, PointStamped, PoseStamped, PoseArray
from std_msgs.msg import Header, String
import numpy as np
from filterpy.kalman import KalmanFilter
from collections import OrderedDict, defaultdict
import threading
from enum import Enum
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class TrackState(Enum):
    """Enum for track states"""
    NEW = 1
    ACTIVE = 2
    STATIONARY = 3
    LOST = 4
    COASTING = 5


class SensorConfig:
    """Configuration for a sensor"""
    def __init__(self, name, topic, weight=1.0, noise_scale=1.0, 
                 measurement_covariance=None, exclude_stationary=False):
        self.name = name
        self.topic = topic
        self.weight = weight
        self.noise_scale = noise_scale
        self.measurement_covariance = measurement_covariance or [0.05, 0.05, 0.05]
        self.exclude_stationary = exclude_stationary
        self.subscriber = None


class TrackedOpponent:
    """Class representing a tracked opponent with Kalman filter"""
    
    def __init__(self, track_id, initial_position, timestamp, config):
        self.track_id = track_id
        self.state = TrackState.NEW
        
        # State vector: [x, y, z, vx, vy, vz, ax, ay, az] (9D)
        self.kf = KalmanFilter(dim_x=9, dim_z=3)
        
        # State transition matrix (constant acceleration model)
        dt = config['process_noise']['dt']  # Will be updated each prediction
        self.kf.F = np.array([
            [1, 0, 0, dt, 0, 0, 0.5*dt**2, 0, 0],
            [0, 1, 0, 0, dt, 0, 0, 0.5*dt**2, 0],
            [0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt**2],
            [0, 0, 0, 1, 0, 0, dt, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, dt, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, dt],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement matrix (we measure position only)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0]
        ])
        
        # Process noise covariance
        self.kf.Q = np.eye(9)
        # Position noise
        self.kf.Q[0:3, 0:3] *= config['process_noise']['position']
        # Velocity noise
        self.kf.Q[3:6, 3:6] *= config['process_noise']['velocity']
        # Acceleration noise
        self.kf.Q[6:9, 6:9] *= config['process_noise']['acceleration']
        
        # Initial state
        self.kf.x = np.array([
            initial_position[0],
            initial_position[1],
            initial_position[2],
            0.0, 0.0, 0.0,  # Initial velocity
            0.0, 0.0, 0.0   # Initial acceleration
        ])
        
        # Initial covariance
        self.kf.P = np.eye(9) * config['initial_covariance']
        
        # Timestamps
        self.last_update_time = timestamp
        self.last_prediction_time = timestamp
        self.creation_time = timestamp
        
        # Detection sources that contribute to this track
        self.sensor_updates = defaultdict(list)  # sensor_name -> list of (timestamp, measurement)
        self.sensor_weights = {}  # sensor_name -> weight
        
        # Statistics
        self.update_count = 0
        self.coast_count = 0
        self.position_history = []
        self.max_history = config['track_history']
        
        # Stationary detection
        self.stationary_threshold = config['stationary']['velocity_threshold']
        self.stationary_time = config['stationary']['min_duration']
        self.stationary_start_time = None
        self.is_stationary = False
        
        # Configuration reference
        self.config = config
    
    def predict(self, dt):
        """Predict step of Kalman filter"""
        # Update state transition matrix with current dt
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        self.kf.F[0, 6] = 0.5 * dt**2
        self.kf.F[1, 7] = 0.5 * dt**2
        self.kf.F[2, 8] = 0.5 * dt**2
        self.kf.F[3, 6] = dt
        self.kf.F[4, 7] = dt
        self.kf.F[5, 8] = dt
        
        self.kf.predict()
        self.last_prediction_time = self.last_prediction_time + Duration(seconds=dt)
    
    def update(self, measurement, covariance, timestamp, sensor_name, sensor_weight, sensor_config):
        """Update step of Kalman filter"""
        # Set measurement noise for this update
        self.kf.R = np.array(covariance) * (1.0 / sensor_weight)
        
        # Perform update
        self.kf.update(measurement)
        
        # Update timestamps and metadata
        self.last_update_time = timestamp
        self.update_count += 1
        
        # Store sensor update
        self.sensor_updates[sensor_name].append((timestamp, measurement.copy()))
        if len(self.sensor_updates[sensor_name]) > 10:
            self.sensor_updates[sensor_name].pop(0)
        
        self.sensor_weights[sensor_name] = sensor_weight
        
        # Check stationary status
        self.check_stationary(timestamp)
        
        # Store in history
        self.position_history.append({
            'time': timestamp,
            'position': self.get_position().copy(),
            'sensor': sensor_name
        })
        
        # Trim history
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
    
    def check_stationary(self, current_time):
        """Check if opponent is stationary"""
        velocity = self.get_velocity()
        speed = np.linalg.norm(velocity)
        
        # Also check position variation in history
        position_variation = 0.0
        if len(self.position_history) >= 5:
            positions = [p['position'] for p in self.position_history[-5:]]
            mean_pos = np.mean(positions, axis=0)
            position_variation = np.mean([np.linalg.norm(p - mean_pos) for p in positions])
        
        is_now_stationary = (speed < self.stationary_threshold) or (position_variation < self.stationary_threshold)
        
        # Update stationary timing
        if is_now_stationary:
            if self.stationary_start_time is None:
                self.stationary_start_time = current_time
            else:
                stationary_duration = (current_time - self.stationary_start_time).nanoseconds / 1e9
                if stationary_duration >= self.stationary_time:
                    self.is_stationary = True
                    if self.state == TrackState.ACTIVE:
                        self.state = TrackState.STATIONARY
        else:
            self.stationary_start_time = None
            self.is_stationary = False
            if self.state == TrackState.STATIONARY:
                self.state = TrackState.ACTIVE
    
    def get_position(self):
        """Get current estimated position"""
        return self.kf.x[0:3].copy()
    
    def get_velocity(self):
        """Get current estimated velocity"""
        return self.kf.x[3:6].copy()
    
    def get_acceleration(self):
        """Get current estimated acceleration"""
        return self.kf.x[6:9].copy()
    
    def get_covariance(self):
        """Get position covariance"""
        return self.kf.P[0:3, 0:3].copy()
    
    def get_full_covariance(self):
        """Get full state covariance"""
        return self.kf.P.copy()
    
    def get_confidence(self):
        """Calculate track confidence based on update history and time since last update"""
        # Base confidence on number of updates (max 1.0 at 20 updates)
        base_confidence = min(1.0, self.update_count / 20.0)
        
        # Decay based on time since last update
        time_since_update = (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9
        decay_factor = max(0.5, 1.0 - time_since_update / self.config['track_management']['timeout'])
        
        # Adjust based on sensor diversity
        sensor_factor = min(1.0, len(self.sensor_weights) / 2.0)
        
        return base_confidence * decay_factor * sensor_factor
    
    def get_pose_stamped(self, frame_id, stamp):
        """Get pose for visualization"""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pos = self.get_position()
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = 1.0
        return pose


class MultiSensorKalmanFilter(Node):
    """
    Kalman filter node for fusing multiple 3D detection sources.
    Uses YAML configuration similar to robot_localization EKF.
    """
    
    def __init__(self):
        super().__init__('multi_sensor_kalman_filter')
        
        # Declare config file parameter
        self.declare_parameter('config_file', '')
        
        # Load configuration
        self.load_config()
        
        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Tracks storage
        self.tracks = OrderedDict()  # track_id -> TrackedOpponent
        self.next_track_id = 0
        self.track_lock = threading.Lock()
        
        # Initialize sensors
        self.sensors = {}  # sensor_name -> SensorConfig
        self.init_sensors()
        
        # Publishers
        self.init_publishers()
        
        # Timers
        self.init_timers()
        
        self.get_logger().info(
            f'Multi-Sensor Kalman Filter initialized with config:\n'
            f'  Output frame: {self.config["output_frame"]}\n'
            f'  Sensors: {list(self.sensors.keys())}\n'
            f'  Max tracks: {self.config["track_management"]["max_tracks"]}'
        )
    
    def load_config(self):
        """Load configuration from YAML file"""
        config_file = self.get_parameter('config_file').value
        
        if not config_file:
            # Try default location
            try:
                pkg_share = get_package_share_directory('your_package_name')
                config_file = os.path.join(pkg_share, 'config', 'kalman_config.yaml')
            except:
                config_file = 'kalman_config.yaml'
        
        self.get_logger().info(f'Loading config from: {config_file}')
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
    
    def init_sensors(self):
        """Initialize sensors from config"""
        for sensor_config in self.config['sensors']:
            name = sensor_config['name']
            topic = sensor_config['topic']
            weight = sensor_config.get('weight', 1.0)
            noise_scale = sensor_config.get('noise_scale', 1.0)
            measurement_covariance = sensor_config.get('measurement_covariance', [0.05, 0.05, 0.05])
            exclude_stationary = sensor_config.get('exclude_stationary', False)
            
            sensor = SensorConfig(
                name=name,
                topic=topic,
                weight=weight,
                noise_scale=noise_scale,
                measurement_covariance=measurement_covariance,
                exclude_stationary=exclude_stationary
            )
            
            # Create subscriber
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            sensor.subscriber = self.create_subscription(
                Detection3DArray,
                topic,
                lambda msg, s=sensor: self.sensor_callback(msg, s),
                qos
            )
            
            self.sensors[name] = sensor
            self.get_logger().info(f'Subscribed to sensor {name} on {topic}')
    
    def init_publishers(self):
        """Initialize publishers"""
        # Main track output
        self.tracks_pub = self.create_publisher(
            Detection3DArray,
            self.config['output_topics']['fused_tracks'],
            10
        )
        
        # Visualization publishers
        if self.config['visualization']['publish_poses']:
            self.track_poses_pub = self.create_publisher(
                PoseArray,
                self.config['output_topics']['track_poses'],
                10
            )
        
        if self.config['visualization']['publish_markers']:
            self.track_markers_pub = self.create_publisher(
                MarkerArray,
                self.config['output_topics']['track_markers'],
                10
            )
        
        # Debug publishers
        if self.config['debug']['enabled']:
            self.debug_info_pub = self.create_publisher(
                String,
                self.config['output_topics']['debug_info'],
                10
            )
            
            self.debug_tracks_pub = self.create_publisher(
                Detection3DArray,
                self.config['output_topics']['debug_tracks'],
                10
            )
    
    def init_timers(self):
        """Initialize timers"""
        # Prediction timer
        dt = self.config['process_noise']['dt']
        self.prediction_timer = self.create_timer(dt, self.prediction_step)
        
        # Cleanup timer
        self.cleanup_timer = self.create_timer(
            self.config['track_management']['cleanup_interval'],
            self.cleanup_tracks
        )
        
        # Debug timer
        if self.config['debug']['enabled']:
            self.debug_timer = self.create_timer(
                self.config['debug']['publish_interval'],
                self.publish_debug_info
            )
    
    def sensor_callback(self, msg, sensor):
        """Callback for sensor detections"""
        if not msg.detections:
            return
        
        # Filter stationary opponents if configured
        if sensor.exclude_stationary:
            # This would require knowledge of which detections are stationary
            # Could be implemented based on detection class ID or other metadata
            pass
        
        # Transform detections to output frame
        detections_in_output = []
        for detection in msg.detections:
            if detection.header.frame_id != self.config['output_frame']:
                detection = self.transform_detection(detection, self.config['output_frame'])
                if detection is None:
                    continue
            detections_in_output.append(detection)
        
        if not detections_in_output:
            return
        
        with self.track_lock:
            # Associate detections to tracks
            associations = self.associate_detections(detections_in_output, sensor)
            
            # Update associated tracks
            for track_id, detection in associations['matched'].items():
                self.update_track(track_id, detection, sensor)
            
            # Create new tracks for unmatched detections
            for detection in associations['unmatched']:
                self.create_track(detection, sensor)
    
    def transform_detection(self, detection, target_frame):
        """Transform detection to target frame"""
        try:
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                detection.header.frame_id,
                rclpy.time.Time(),
                Duration(seconds=self.config['tf_timeout'])
            )
            
            # Transform position
            point = PointStamped()
            point.header = detection.header
            point.point = detection.bbox.center.position
            
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)
            
            # Create transformed detection
            transformed = Detection3D()
            transformed.header = detection.header
            transformed.header.frame_id = target_frame
            transformed.bbox.center.position = transformed_point.point
            transformed.bbox.center.orientation = detection.bbox.center.orientation
            transformed.bbox.size = detection.bbox.size
            transformed.results = detection.results
            transformed.id = detection.id
            
            return transformed
            
        except TransformException as e:
            self.get_logger().warn(f'Transform failed: {e}', throttle_duration_sec=2.0)
            return None
    
    def associate_detections(self, detections, sensor):
        """Associate detections to existing tracks"""
        result = {
            'matched': {},
            'unmatched': list(range(len(detections))),
            'unmatched_tracks': list(self.tracks.keys())
        }
        
        if not self.tracks or not detections:
            return result
        
        # Build cost matrix using Mahalanobis distance
        cost_matrix = np.zeros((len(self.tracks), len(detections)))
        
        for i, (track_id, track) in enumerate(self.tracks.items()):
            track_pos = track.get_position()
            track_cov = track.get_covariance()
            
            for j, detection in enumerate(detections):
                det_pos = np.array([
                    detection.bbox.center.position.x,
                    detection.bbox.center.position.y,
                    detection.bbox.center.position.z
                ])
                
                # Calculate Mahalanobis distance
                diff = det_pos - track_pos
                try:
                    inv_cov = np.linalg.inv(track_cov)
                    distance = np.sqrt(diff.T @ inv_cov @ diff)
                except:
                    distance = np.linalg.norm(diff) / 0.1
                
                cost_matrix[i, j] = distance
        
        # Gating
        gating_threshold = self.config['data_association']['gating_threshold']
        
        # Simple greedy association
        for j in range(len(detections)):
            best_i = -1
            best_cost = float('inf')
            
            for i, track_id in enumerate(self.tracks.keys()):
                if track_id not in result['unmatched_tracks']:
                    continue
                    
                if cost_matrix[i, j] < gating_threshold and cost_matrix[i, j] < best_cost:
                    best_cost = cost_matrix[i, j]
                    best_i = i
            
            if best_i >= 0:
                track_id = list(self.tracks.keys())[best_i]
                result['matched'][track_id] = detections[j]
                result['unmatched_tracks'].remove(track_id)
                result['unmatched'].remove(j)
        
        return result
    
    def create_track(self, detection, sensor):
        """Create new track from detection"""
        if len(self.tracks) >= self.config['track_management']['max_tracks']:
            # Remove oldest track
            oldest_id = next(iter(self.tracks))
            self.get_logger().info(f'Max tracks reached, removing oldest: {oldest_id}')
            del self.tracks[oldest_id]
        
        # Get initial position
        initial_pos = np.array([
            detection.bbox.center.position.x,
            detection.bbox.center.position.y,
            detection.bbox.center.position.z
        ])
        
        # Create track with config
        track_id = f"{self.config['track_management']['track_id_prefix']}{self.next_track_id:03d}"
        self.next_track_id += 1
        
        timestamp = self.get_clock().now()
        track = TrackedOpponent(track_id, initial_pos, timestamp, self.config)
        
        # Calculate measurement covariance
        measurement_cov = np.diag(sensor.measurement_covariance) * sensor.noise_scale
        
        # Initial update
        track.update(
            initial_pos,
            measurement_cov,
            timestamp,
            sensor.name,
            sensor.weight,
            self.config
        )
        
        self.tracks[track_id] = track
        self.get_logger().info(f'Created new track: {track_id} from {sensor.name}')
    
    def update_track(self, track_id, detection, sensor):
        """Update existing track with detection"""
        track = self.tracks.get(track_id)
        if not track:
            return
        
        # Get measurement
        measurement = np.array([
            detection.bbox.center.position.x,
            detection.bbox.center.position.y,
            detection.bbox.center.position.z
        ])
        
        # Calculate measurement covariance
        # Use detection size to estimate uncertainty if available
        base_cov = sensor.measurement_covariance
        if hasattr(detection.bbox, 'size'):
            size = detection.bbox.size
            # Larger objects might have more position uncertainty
            scale = max(size.x, size.y, 0.1)
            base_cov = [c * scale for c in base_cov]
        
        measurement_cov = np.diag(base_cov) * sensor.noise_scale
        
        # Update track
        timestamp = self.get_clock().now()
        track.update(
            measurement,
            measurement_cov,
            timestamp,
            sensor.name,
            sensor.weight,
            self.config
        )
        
        # Update state if needed
        if track.state == TrackState.NEW:
            min_updates = self.config['track_management']['min_updates_to_confirm']
            if track.update_count >= min_updates:
                track.state = TrackState.ACTIVE
    
    def prediction_step(self):
        """Prediction step for all tracks"""
        if not self.tracks:
            return
        
        current_time = self.get_clock().now()
        
        with self.track_lock:
            tracks_to_remove = []
            
            for track_id, track in self.tracks.items():
                # Calculate dt since last prediction
                dt = (current_time - track.last_prediction_time).nanoseconds / 1e9
                
                if dt > 0:
                    track.predict(min(dt, self.config['process_noise']['max_dt']))
                    
                    # Check if coasting
                    time_since_update = (current_time - track.last_update_time).nanoseconds / 1e9
                    coast_time = self.config['track_management']['coast_time']
                    
                    if time_since_update > coast_time and track.state == TrackState.ACTIVE:
                        track.state = TrackState.COASTING
                        track.coast_count += 1
                    
                    # Mark as lost if timeout
                    timeout = self.config['track_management']['timeout']
                    if time_since_update > timeout:
                        track.state = TrackState.LOST
                        tracks_to_remove.append(track_id)
            
            # Remove lost tracks
            for track_id in tracks_to_remove:
                self.get_logger().info(f'Removing lost track: {track_id}')
                del self.tracks[track_id]
            
            # Publish tracks
            self.publish_tracks()
    
    def publish_tracks(self):
        """Publish fused tracks"""
        if not self.tracks:
            return
        
        current_time = self.get_clock().now()
        
        # Create main track array
        track_array = Detection3DArray()
        track_array.header.stamp = current_time.to_msg()
        track_array.header.frame_id = self.config['output_frame']
        
        # Create pose array for visualization
        if self.config['visualization']['publish_poses']:
            pose_array = PoseArray()
            pose_array.header = track_array.header
        
        for track_id, track in self.tracks.items():
            # Skip tracks that are too new if configured
            if track.state == TrackState.NEW and not self.config['debug']['publish_new_tracks']:
                continue
            
            # Create Detection3D for this track
            detection = Detection3D()
            detection.header = track_array.header
            
            # Position
            pos = track.get_position()
            detection.bbox.center.position.x = pos[0]
            detection.bbox.center.position.y = pos[1]
            detection.bbox.center.position.z = pos[2]
            
            # Orientation (identity)
            detection.bbox.center.orientation.w = 1.0
            
            # Velocity (if requested)
            if self.config['output']['publish_velocity']:
                vel = track.get_velocity()
                # Could store in a custom field or results
                # For now, we'll store in the ID field as a hack
                detection.id = f"{track_id}|v:{vel[0]:.2f},{vel[1]:.2f},{vel[2]:.2f}"
            
            # Add hypothesis with track info
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = track_id
            
            # Calculate confidence
            confidence = track.get_confidence()
            hypothesis.hypothesis.score = confidence
            
            # Add state information
            hypothesis.hypothesis.class_id += f"|state:{track.state.name}"
            
            if track.is_stationary:
                hypothesis.hypothesis.class_id += "|stationary"
            
            detection.results = [hypothesis]
            
            # Add covariance if requested
            if self.config['output']['publish_covariance']:
                cov = track.get_covariance().flatten()
                # Could add to a custom field
            
            track_array.detections.append(detection)
            
            # Add to pose array
            if self.config['visualization']['publish_poses']:
                pose = Pose()
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)
        
        # Publish
        if track_array.detections:
            self.tracks_pub.publish(track_array)
            
            if self.config['visualization']['publish_poses']:
                self.track_poses_pub.publish(pose_array)
            
            # Publish debug tracks if enabled
            if self.config['debug']['enabled'] and self.config['debug']['publish_raw']:
                self.debug_tracks_pub.publish(track_array)
    
    def cleanup_tracks(self):
        """Clean up old tracks"""
        current_time = self.get_clock().now()
        timeout = self.config['track_management']['timeout']
        
        with self.track_lock:
            tracks_to_remove = []
            
            for track_id, track in self.tracks.items():
                time_since_update = (current_time - track.last_update_time).nanoseconds / 1e9
                if time_since_update > timeout:
                    tracks_to_remove.append(track_id)
            
            for track_id in tracks_to_remove:
                self.get_logger().debug(f'Cleanup removing track: {track_id}')
                del self.tracks[track_id]
    
    def publish_debug_info(self):
        """Publish debug information"""
        if not self.tracks:
            return
        
        import json
        
        info = {
            'timestamp': self.get_clock().now().nanoseconds,
            'num_tracks': len(self.tracks),
            'tracks': []
        }
        
        for track_id, track in self.tracks.items():
            track_info = {
                'id': track_id,
                'state': track.state.name,
                'position': track.get_position().tolist(),
                'velocity': track.get_velocity().tolist(),
                'acceleration': track.get_acceleration().tolist(),
                'update_count': track.update_count,
                'coast_count': track.coast_count,
                'is_stationary': track.is_stationary,
                'confidence': track.get_confidence(),
                'sensors': list(track.sensor_weights.keys())
            }
            info['tracks'].append(track_info)
        
        msg = String()
        msg.data = json.dumps(info, indent=2)
        self.debug_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorKalmanFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()