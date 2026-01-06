#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, TransformException, TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.signal import butter, filtfilt
from collections import deque
import threading
from enum import Enum
import yaml
import os

class FilterType(Enum):
    MOVING_AVERAGE = "moving_average"
    EWMA = "ewma" 
    KALMAN = "kalman"
    COMPLEMENTARY = "complementary"
    MEDIAN = "median"
    BUTTERWORTH = "butterworth"
    NO_FILTER = "no_filter"
    
    @classmethod
    def has_value(cls, value):
        return value in cls._value2member_map_

class FilterTransformNode(Node):
    def __init__(self):
        super().__init__('filter_transform_node')
        
        self.get_logger().info("Initializing Filter Node...")
        
        # Declare parameters
        self.declare_parameter('config_file', '', 
                              ParameterDescriptor(
                                  description='Path to YAML configuration file',
                                  type=ParameterType.PARAMETER_STRING))
        
        # Load configuration
        config_file = self.get_parameter('config_file').value
        if not config_file:
            self.get_logger().error("No configuration file specified. Use '--ros-args -p config_file:=/path/to/config.yaml'")
            raise ValueError("Configuration file path is required")
        
        self.config = self.load_config(config_file)
        
        # Initialize from configuration
        self.initialize_from_config()
        
        # TF2 setup
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=1.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Filter state
        self.data_buffers = {}
        self.last_poses = {}
        self.kalman_states = {}
        self.lock = threading.RLock()
        
        # Butterworth filter state
        self.butterworth_states = {}
        
        # Complementary filter state
        self.complementary_states = {}
        
        # Statistics
        self.frame_count = 0
        self.missed_frames = 0
        
        # Initialize for each tag mapping
        for tag_mapping in self.tag_mappings:
            input_frame = tag_mapping['input']
            self.initialize_filter_state(input_frame)
        
        # Timer for processing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.process_and_publish)
        
        # Diagnostics timer
        self.diagnostics_timer = self.create_timer(5.0, self.publish_diagnostics)
        
        self.get_logger().info(f"Filter initialized with {self.filter_type.value} filter")
        self.get_logger().info(f"Tracking {len(self.tag_mappings)} tag(s) at {self.publish_rate}Hz")
    
    def destroy_node(self):
        """Override destroy_node to properly clean up TF listener"""
        with self.lock:
            # Destroy the listener before the node
            if hasattr(self, 'tf_listener'):
                try:
                    self.tf_listener.unregister()
                except:
                    pass
                self.tf_listener = None
            
            # Call parent destroy
            super().destroy_node()
        
    def load_config(self, config_path):
        """Load YAML configuration file"""
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found: {config_path}")
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.get_logger().info(f"Loaded configuration from {config_path}")
        return config
    
    def initialize_from_config(self):
        """Initialize from configuration"""
        # Filter configuration
        filter_config = self.config.get('filter', {})
        
        # Set filter type with validation
        filter_type_str = filter_config.get('type', 'ewma')
        if not FilterType.has_value(filter_type_str):
            self.get_logger().warning(f"Unknown filter type '{filter_type_str}', defaulting to 'ewma'")
            filter_type_str = 'ewma'
        self.filter_type = FilterType(filter_type_str)
        
        # General parameters
        self.window_size = int(filter_config.get('window_size', 10))
        self.publish_rate = float(filter_config.get('publish_rate', 30.0))
        
        # Filter-specific parameters
        self.alpha = float(filter_config.get('alpha', 0.3))  # EWMA
        self.complementary_alpha = float(filter_config.get('complementary_alpha', 0.98))
        self.cutoff_freq = float(filter_config.get('cutoff_freq', 5.0))
        self.order = int(filter_config.get('order', 2))
        self.process_noise = float(filter_config.get('process_noise', 0.01))
        self.measurement_noise = float(filter_config.get('measurement_noise', 0.1))
        self.median_window = int(filter_config.get('median_window', 5))
        
        # Frame configuration
        frames_config = self.config.get('frames', {})
        
        # Parent frame (usually camera)
        self.parent_frame = frames_config.get('parent_frame', 'arena_camera_optical')
        
        # Tag mappings - list of input->output pairs
        self.tag_mappings = frames_config.get('tag_mappings', [])
        
        if not self.tag_mappings:
            # Default mappings for backward compatibility
            self.tag_mappings = [
                {'input': 'robot/top_apriltag_link', 'output': 'robot/top_apriltag_link_filtered'},
                {'input': 'robot/bottom_apriltag_link', 'output': 'robot/bottom_apriltag_link_filtered'}
            ]
            self.get_logger().warning("No tag_mappings specified, using defaults")
        
        # Validate tag mappings
        for mapping in self.tag_mappings:
            if 'input' not in mapping or 'output' not in mapping:
                self.get_logger().error(f"Invalid tag mapping: {mapping}")
                raise ValueError("Each tag mapping must have 'input' and 'output' keys")
        
        self.get_logger().info(f"Parent frame: {self.parent_frame}")
        for mapping in self.tag_mappings:
            self.get_logger().info(f"  {mapping['input']} -> {mapping['output']}")
    
    def initialize_filter_state(self, input_frame):
        """Initialize filter state for a specific input frame"""
        with self.lock:
            # Data buffers
            self.data_buffers[input_frame] = {
                'positions': deque(maxlen=self.window_size),
                'rotations': deque(maxlen=self.window_size),
                'timestamps': deque(maxlen=self.window_size),
                'raw_positions': deque(maxlen=self.window_size),
                'raw_rotations': deque(maxlen=self.window_size)
            }
            
            # Last poses for EWMA/Complementary
            self.last_poses[input_frame] = None
            
            # Kalman filter state
            self.kalman_states[input_frame] = {
                'x': np.zeros(6),  # [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
                'P': np.eye(6) * 10.0,
                'last_time': None,
                'initialized': False
            }
            
            # Butterworth filter state
            if self.filter_type == FilterType.BUTTERWORTH:
                nyquist = 0.5 * self.publish_rate
                normal_cutoff = self.cutoff_freq / nyquist
                b, a = butter(self.order, normal_cutoff, btype='low', analog=False)
                
                # Initial conditions for each dimension
                self.butterworth_states[input_frame] = {
                    'b': b,
                    'a': a,
                    'position_zi': [np.zeros(max(len(b), len(a)) - 1) for _ in range(3)],
                    'rotation_zi': [np.zeros(max(len(b), len(a)) - 1) for _ in range(4)]
                }
            
            # Complementary filter state
            self.complementary_states[input_frame] = {
                'position': None,
                'rotation': None,
                'velocity': np.zeros(3),
                'last_time': None
            }
    
    def get_latest_transform(self, target_frame, source_frame):
        """Get the latest transform between frames"""
        try:
            # Try to get the most recent transform
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            return transform
        except TransformException as e:
            self.missed_frames += 1
            # Only log occasionally to avoid spam
            if self.missed_frames % 100 == 0:
                self.get_logger().debug(f"TF lookup failed for {source_frame}: {e}")
            return None
    
    def process_and_publish(self):
        """Main processing loop"""
        with self.lock:
            current_time = self.get_clock().now()
            transforms_published = 0
            
            for tag_mapping in self.tag_mappings:
                input_frame = tag_mapping['input']
                output_frame = tag_mapping['output']
                
                # Get transform from parent frame to input frame
                transform = self.get_latest_transform(self.parent_frame, input_frame)
                
                if transform is None:
                    continue
                
                self.frame_count += 1
                
                # Extract pose from transform
                position = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                
                rotation = np.array([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])
                
                timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                
                # Store raw data
                buffer = self.data_buffers[input_frame]
                buffer['raw_positions'].append(position.copy())
                buffer['raw_rotations'].append(rotation.copy())
                buffer['timestamps'].append(timestamp)
                
                # Apply filter
                filtered_position, filtered_rotation = self.apply_filter(
                    input_frame, position, rotation, timestamp
                )
                
                if filtered_position is not None and filtered_rotation is not None:
                    # Publish filtered transform
                    self.publish_transform(
                        output_frame, 
                        filtered_position, 
                        filtered_rotation, 
                        current_time
                    )
                    transforms_published += 1
                    
                    # Store filtered data
                    buffer['positions'].append(filtered_position.copy())
                    buffer['rotations'].append(filtered_rotation.copy())
    
    def apply_filter(self, input_frame, position, rotation, timestamp):
        """Apply the selected filter to the pose"""
        buffer = self.data_buffers[input_frame]
        
        try:
            if self.filter_type == FilterType.MOVING_AVERAGE:
                return self.moving_average_filter(buffer, position, rotation)
            elif self.filter_type == FilterType.EWMA:
                return self.ewma_filter(input_frame, buffer, position, rotation)
            elif self.filter_type == FilterType.KALMAN:
                return self.kalman_filter(input_frame, buffer, position, rotation, timestamp)
            elif self.filter_type == FilterType.COMPLEMENTARY:
                return self.complementary_filter(input_frame, buffer, position, rotation, timestamp)
            elif self.filter_type == FilterType.MEDIAN:
                return self.median_filter(buffer, position, rotation)
            elif self.filter_type == FilterType.BUTTERWORTH:
                return self.butterworth_filter(input_frame, buffer, position, rotation)
            elif self.filter_type == FilterType.NO_FILTER:
                return position, rotation
            else:
                return self.ewma_filter(input_frame, buffer, position, rotation)
        except Exception as e:
            self.get_logger().error(f"Filter error for {input_frame}: {str(e)}", throttle_duration_sec=1.0)
            return position, rotation  # Fall back to raw data
    
    def moving_average_filter(self, buffer, position, rotation):
        """Simple moving average filter"""
        if len(buffer['raw_positions']) == 0:
            return position, rotation
        
        # Average positions
        positions = np.array(list(buffer['raw_positions']))
        filtered_position = np.mean(positions, axis=0)
        
        # Average rotations using quaternion averaging
        rotations = np.array(list(buffer['raw_rotations']))
        if len(rotations) == 1:
            filtered_rotation = rotations[0]
        else:
            filtered_rotation = self.average_quaternions(rotations)
        
        return filtered_position, filtered_rotation
    
    def ewma_filter(self, input_frame, buffer, position, rotation):
        """Exponentially Weighted Moving Average"""
        if self.last_poses[input_frame] is None:
            self.last_poses[input_frame] = (position.copy(), rotation.copy())
            return position, rotation
        
        last_position, last_rotation = self.last_poses[input_frame]
        
        # Apply EWMA to position
        filtered_position = self.alpha * position + (1 - self.alpha) * last_position
        
        # Apply spherical interpolation to rotation
        new_rot = Rotation.from_quat(rotation)
        last_rot = Rotation.from_quat(last_rotation)
        
        # Interpolate with alpha weight - FIXED: using Slerp class
        slerp = Slerp([0, 1], Rotation.concatenate([last_rot, new_rot]))
        filtered_rot = slerp(self.alpha)
        filtered_rotation = filtered_rot.as_quat()
        
        # Update last pose
        self.last_poses[input_frame] = (filtered_position.copy(), filtered_rotation.copy())
        
        return filtered_position, filtered_rotation
    
    def kalman_filter(self, input_frame, buffer, position, rotation, timestamp):
        """Kalman filter with constant velocity model"""
        state = self.kalman_states[input_frame]
        
        # Initialize if first measurement
        if not state['initialized']:
            state['x'][:3] = position
            state['last_time'] = timestamp
            state['initialized'] = True
            return position, rotation
        
        # Time delta
        dt = timestamp - state['last_time']
        if dt <= 0:
            dt = 0.001
        
        # State transition matrix (constant velocity model)
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        
        # Process noise covariance
        Q = np.eye(6)
        Q[:3, :3] *= self.process_noise * dt**2
        Q[3:, 3:] *= self.process_noise
        
        # Measurement matrix (we measure position only)
        H = np.zeros((3, 6))
        H[:3, :3] = np.eye(3)
        
        # Measurement noise
        R = np.eye(3) * self.measurement_noise
        
        # Prediction step
        x_pred = F @ state['x']
        P_pred = F @ state['P'] @ F.T + Q
        
        # Update step
        z = position
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # State update
        state['x'] = x_pred + K @ y
        state['P'] = (np.eye(6) - K @ H) @ P_pred
        state['last_time'] = timestamp
        
        # For rotation, use EWMA
        if self.last_poses[input_frame] is not None:
            last_position, last_rotation = self.last_poses[input_frame]
            new_rot = Rotation.from_quat(rotation)
            last_rot = Rotation.from_quat(last_rotation)
            slerp = Slerp([0, 1], Rotation.concatenate([last_rot, new_rot]))
            filtered_rot = slerp(0.3)  # FIXED: using Slerp class
            filtered_rotation = filtered_rot.as_quat()
        else:
            filtered_rotation = rotation
        
        # Update last pose
        filtered_position = state['x'][:3]
        self.last_poses[input_frame] = (filtered_position.copy(), filtered_rotation.copy())
        
        return filtered_position, filtered_rotation
    
    def complementary_filter(self, input_frame, buffer, position, rotation, timestamp):
        """Complementary filter - fuses prediction with measurement"""
        state = self.complementary_states[input_frame]
        
        # Initialize if first measurement
        if state['position'] is None:
            state['position'] = position.copy()
            state['rotation'] = rotation.copy()
            state['last_time'] = timestamp
            return position, rotation
        
        # Time delta
        dt = timestamp - state['last_time']
        if dt <= 0:
            dt = 0.001
        
        # Predict position using velocity
        predicted_position = state['position'] + state['velocity'] * dt
        
        # Update velocity estimate
        if len(buffer['raw_positions']) >= 2:
            positions = list(buffer['raw_positions'])
            if len(positions) >= 2:
                dt_hist = buffer['timestamps'][-1] - buffer['timestamps'][-2]
                if dt_hist > 0:
                    state['velocity'] = (positions[-1] - positions[-2]) / dt_hist
        
        # Blend prediction with measurement
        filtered_position = (self.complementary_alpha * position + 
                           (1 - self.complementary_alpha) * predicted_position)
        
        # For rotation, use EWMA
        new_rot = Rotation.from_quat(rotation)
        last_rot = Rotation.from_quat(state['rotation'])
        slerp = Slerp([0, 1], Rotation.concatenate([last_rot, new_rot]))
        filtered_rot = slerp(1 - self.complementary_alpha)  # FIXED: using Slerp class
        filtered_rotation = filtered_rot.as_quat()
        
        # Update state
        state['position'] = filtered_position.copy()
        state['rotation'] = filtered_rotation.copy()
        state['last_time'] = timestamp
        
        return filtered_position, filtered_rotation
    
    def median_filter(self, buffer, position, rotation):
        """Median filter - removes outliers"""
        if len(buffer['raw_positions']) < self.median_window:
            return position, rotation
        
        # Median of positions
        positions = np.array(list(buffer['raw_positions'])[-self.median_window:])
        filtered_position = np.median(positions, axis=0)
        
        # For rotation, use median of Euler angles
        rotations = np.array(list(buffer['raw_rotations'])[-self.median_window:])
        eulers = np.array([Rotation.from_quat(r).as_euler('xyz', degrees=False) for r in rotations])
        median_euler = np.median(eulers, axis=0)
        filtered_rotation = Rotation.from_euler('xyz', median_euler).as_quat()
        
        return filtered_position, filtered_rotation
    
    def butterworth_filter(self, input_frame, buffer, position, rotation):
        """Butterworth low-pass filter"""
        state = self.butterworth_states[input_frame]
        
        if len(buffer['raw_positions']) < len(state['b']):
            return position, rotation
        
        # Apply filter to position components
        filtered_position = np.zeros(3)
        for i in range(3):
            # Get historical data for this component
            hist_data = [p[i] for p in buffer['raw_positions']]
            
            # Apply filter
            filtered_comp, state['position_zi'][i] = filtfilt(
                state['b'], state['a'], hist_data[-len(state['b']):],
                zi=state['position_zi'][i]
            )
            filtered_position[i] = filtered_comp[-1]
        
        # Apply filter to rotation (via Euler angles for simplicity)
        if len(buffer['raw_rotations']) >= len(state['b']):
            # Convert to Euler angles
            eulers = [Rotation.from_quat(r).as_euler('xyz', degrees=False) 
                     for r in buffer['raw_rotations']]
            
            filtered_euler = np.zeros(3)
            for i in range(3):
                hist_data = [e[i] for e in eulers]
                filtered_comp, state['rotation_zi'][i] = filtfilt(
                    state['b'], state['a'], hist_data[-len(state['b']):],
                    zi=state['rotation_zi'][i]
                )
                filtered_euler[i] = filtered_comp[-1]
            
            filtered_rotation = Rotation.from_euler('xyz', filtered_euler).as_quat()
        else:
            filtered_rotation = rotation
        
        return filtered_position, filtered_rotation
    
    def average_quaternions(self, quaternions):
        """Average multiple quaternions using simple normalization"""
        if len(quaternions) == 0:
            return np.array([0, 0, 0, 1])
        
        avg_q = np.mean(quaternions, axis=0)
        norm = np.linalg.norm(avg_q)
        
        if norm > 0:
            return avg_q / norm
        else:
            return np.array([0, 0, 0, 1])
    
    def publish_transform(self, child_frame, position, rotation, stamp):
        """Publish a transform"""
        transform = TransformStamped()
        
        # Header
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = child_frame
        
        # Position
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        
        # Rotation
        transform.transform.rotation.x = float(rotation[0])
        transform.transform.rotation.y = float(rotation[1])
        transform.transform.rotation.z = float(rotation[2])
        transform.transform.rotation.w = float(rotation[3])
        
        # Publish
        self.tf_broadcaster.sendTransform(transform)
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        if self.frame_count > 0:
            # Ensure success rate is between 0-100%
            success_frames = max(0, self.frame_count - self.missed_frames)
            success_rate = 100.0 * success_frames / self.frame_count
            self.get_logger().info(
                f"Filter diagnostics: "
                f"Frames={self.frame_count}, "
                f"Missed={self.missed_frames}, "
                f"Success={success_rate:.1f}%, "
                f"Filter={self.filter_type.value}"
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = FilterTransformNode()
        rclpy.spin(node)
    except ValueError as e:
        if node:
            node.get_logger().error(f"Configuration error: {e}")
        else:
            print(f"Configuration error: {e}")
        return 1
    except Exception as e:
        if node:
            node.get_logger().error(f"Node failed with error: {e}")
        else:
            print(f"Node failed with error: {e}")
        return 1
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()