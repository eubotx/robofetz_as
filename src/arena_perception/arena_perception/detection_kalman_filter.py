#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from filterpy.kalman import KalmanFilter
import threading

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TrackedObject:

    def __init__(self, position, time, kf_params):

        self.last_prediction_time = time
        self.last_update_time = time

        self.missed_detections = 0

        pos = np.array(position)

        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        self.kf.x = np.array([
            pos[0],
            pos[1],
            pos[2],
            0.0,
            0.0,
            0.0
        ]).reshape(6, 1)

        self.kf.F = np.eye(6)

        self.kf.H = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]
        ])

        # Initialize covariance from parameters
        pos_cov = kf_params.get('initial_covariance', {}).get('position', 0.2)
        vel_cov = kf_params.get('initial_covariance', {}).get('velocity', 50.0)
        
        self.kf.P = np.eye(6)
        self.kf.P[0:3,0:3] *= pos_cov
        self.kf.P[3:6,3:6] *= vel_cov

        # Default measurement noise (will be updated per measurement)
        self.kf.R = np.eye(3) * 0.05

        # Process noise parameters
        self.pos_noise = kf_params.get('process_noise', {}).get('position', 0.05)
        self.vel_noise = kf_params.get('process_noise', {}).get('velocity', 0.5)


    def predict(self, now):

        dt = (now - self.last_prediction_time).nanoseconds / 1e9

        if dt <= 0:
            return

        dt = min(dt, 0.2)

        self.kf.F = np.array([
            [1,0,0,dt,0,0],
            [0,1,0,0,dt,0],
            [0,0,1,0,0,dt],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ])

        # Process noise scales with dt
        self.kf.Q = np.diag([
            self.pos_noise * dt,
            self.pos_noise * dt,
            self.pos_noise * dt,
            self.vel_noise * dt,
            self.vel_noise * dt,
            self.vel_noise * dt
        ])

        self.kf.predict()

        self.last_prediction_time = now

        self.missed_detections += 1


    def update(self, measurement, now, noise):

        self.kf.R = np.eye(3) * noise

        z = measurement.reshape(3,1)

        self.kf.update(z)

        self.last_update_time = now
        self.missed_detections = 0


    def state(self):

        pos = self.kf.x[0:3].flatten()
        vel = self.kf.x[3:6].flatten()

        return pos, vel
    
    def get_velocity_variance(self):
        """Get the variance of velocity estimates from covariance matrix"""
        # Return the diagonal elements for x and y velocity
        return self.kf.P[3,3], self.kf.P[4,4]


class SingleOpponentKalmanFilter(Node):

    def __init__(self):

        super().__init__('detection_kalman_filter')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_frame', 'world'),
                ('opponent_frame', 'opponent'),
                ('update_rate', 20.0),
                ('input_topics', ['']),
                ('measurement_noises', [0.0]),
                ('process_noise_position', 0.05),
                ('process_noise_velocity', 0.5),
                ('initial_covariance_position', 0.2),
                ('initial_covariance_velocity', 50.0),
                ('max_gate_distance', 1.5),
                ('max_missed_detections', 15),
                ('vel_threshold_low', 0.02),
                ('vel_threshold_high', 0.08),
                ('smoothing_factor', 0.5),
                ('min_confidence_to_update', 0.1),
                ('default_orientation', [0.0, 0.0, 0.0, 1.0])
            ]
        )

        self.target_frame = self.get_parameter('target_frame').value
        self.child_frame = self.get_parameter('opponent_frame').value
        
        # Build kf_params dict to match what TrackedObject expects
        self.kf_params = {
            'process_noise': {
                'position': self.get_parameter('process_noise_position').value,
                'velocity': self.get_parameter('process_noise_velocity').value
            },
            'initial_covariance': {
                'position': self.get_parameter('initial_covariance_position').value,
                'velocity': self.get_parameter('initial_covariance_velocity').value
            },
            'orientation': {
                'default_orientation': self.get_parameter('default_orientation').value
            }
        }

        # Association parameters
        self.max_gate = self.get_parameter('max_gate_distance').value
        self.max_missed = self.get_parameter('max_missed_detections').value

        # Orientation parameters
        self.vel_threshold_low = self.get_parameter('vel_threshold_low').value
        self.vel_threshold_high = self.get_parameter('vel_threshold_high').value
        self.orientation_smoothing_factor = self.get_parameter('smoothing_factor').value
        self.min_confidence_to_update = self.get_parameter('min_confidence_to_update').value
        
        default_orient = self.get_parameter('default_orientation').value
        self.last_valid_orientation = tuple(default_orient)
        self.current_orientation = tuple(default_orient)
        
        # For debugging
        self.last_confidence = 0.0

        # Update rate
        update_rate = self.get_parameter('update_rate').value
        timer_period = 1.0 / update_rate

        self.track = None

        self.lock = threading.Lock()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscribers = []
        self.measurement_noise = {}

        input_topics = self.get_parameter('input_topics').value
        measurement_noises = self.get_parameter('measurement_noises').value

        for i, topic in enumerate(input_topics):
            src_name = f"source_{i}"
            
            if i < len(measurement_noises):
                self.measurement_noise[src_name] = measurement_noises[i]
            else:
                self.measurement_noise[src_name] = 0.05

            def make_cb(name):
                return lambda msg: self.detection_callback(msg, name)

            self.subscribers.append(
                self.create_subscription(
                    Detection3DArray,
                    topic,
                    make_cb(src_name),
                    10
                )
            )
            self.get_logger().info(f"Subscribed to {topic} as {src_name} with noise {self.measurement_noise[src_name]}")

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Kalman filter node started with orientation smoothing")


    def velocity_to_quaternion(self, vel):
        """
        Convert velocity vector to quaternion with confidence-based smoothing
        Simplified for better responsiveness
        """
        vx, vy, vz = vel
        
        # Calculate speed in XY plane
        speed = np.sqrt(vx*vx + vy*vy)
        
        # If speed is very low, just return last valid orientation
        if speed < self.vel_threshold_low:
            return self.last_valid_orientation
        
        # Calculate target yaw from velocity
        target_yaw = np.arctan2(vy, vx)
        target_qz = float(np.sin(target_yaw/2.0))
        target_qw = float(np.cos(target_yaw/2.0))
        
        # Calculate confidence based on speed
        if speed >= self.vel_threshold_high:
            # High confidence - use target directly with smoothing
            confidence = 1.0
        else:
            # Linear confidence between thresholds
            confidence = (speed - self.vel_threshold_low) / (self.vel_threshold_high - self.vel_threshold_low)
            confidence = max(0.0, min(1.0, confidence))
        
        self.last_confidence = confidence
        
        # Apply smoothing based on confidence
        if confidence >= 0.99:
            # Very confident - use target directly
            self.current_orientation = (0.0, 0.0, target_qz, target_qw)
        else:
            # Blend with last valid orientation
            last_qz, last_qw = self.last_valid_orientation[2], self.last_valid_orientation[3]
            
            # Adaptive smoothing: higher confidence = faster update
            smooth_factor = self.orientation_smoothing_factor * (1.0 - confidence * 0.5)
            
            blended_qz = last_qz * (1 - smooth_factor) + target_qz * smooth_factor
            blended_qw = last_qw * (1 - smooth_factor) + target_qw * smooth_factor
            
            # Normalize
            norm = np.sqrt(blended_qz**2 + blended_qw**2)
            if norm > 1e-6:
                blended_qz /= norm
                blended_qw /= norm
            else:
                blended_qz, blended_qw = 0.0, 1.0
            
            self.current_orientation = (0.0, 0.0, blended_qz, blended_qw)
        
        # Update last valid orientation if we have reasonable confidence
        if confidence > self.min_confidence_to_update:
            self.last_valid_orientation = self.current_orientation
        
        return self.current_orientation


    def extract_position(self, detection):

        try:
            c = detection.bbox.center

            if hasattr(c, "pose"):
                p = c.pose.position
            else:
                p = c.position

            return np.array([p.x, p.y, p.z])

        except Exception as e:
            self.get_logger().debug(f"Failed to extract position: {e}")
            return None


    def detection_callback(self, msg, source):

        if not msg.detections:
            return

        now = self.get_clock().now()

        with self.lock:

            best = None
            best_dist = float('inf')

            for det in msg.detections:

                m = self.extract_position(det)

                if m is None:
                    continue

                if self.track:

                    pos, _ = self.track.state()

                    dist = np.linalg.norm(m - pos)

                    if dist < self.max_gate and dist < best_dist:
                        best_dist = dist
                        best = m

                else:
                    # No track yet, take first valid detection
                    best = m
                    break

            if best is None:
                return

            # Get measurement noise for this source
            noise = self.measurement_noise.get(
                source,
                self.measurement_noise.get("default", 0.05)
            )

            if self.track:
                self.track.update(best, now, noise)
                self.get_logger().debug(f"Updated track with {source} measurement")
            else:
                self.track = TrackedObject(best, now, self.kf_params)
                self.get_logger().info(f"Created new track from {source} detection")


    def timer_callback(self):

        now = self.get_clock().now()

        with self.lock:

            if not self.track:
                return

            # Check if track is too old
            time_since_update = (now - self.track.last_update_time).nanoseconds / 1e9
            if time_since_update > self.max_missed * 0.1:  # Rough conversion to seconds
                self.get_logger().info("Track lost, removing")
                self.track = None
                # Reset orientation to default when track is lost
                default_orient = self.kf_params.get("orientation", {}).get("default_orientation", [0.0, 0.0, 0.0, 1.0])
                self.last_valid_orientation = tuple(default_orient)
                self.current_orientation = tuple(default_orient)
                return

            self.track.predict(now)

            pos, vel = self.track.state()

            # Get orientation from velocity
            qx, qy, qz, qw = self.velocity_to_quaternion(vel)

            # Create and publish transform
            t = TransformStamped()

            t.header.stamp = now.to_msg()
            t.header.frame_id = self.target_frame
            t.child_frame_id = self.child_frame

            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])

            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)

            self.tf_broadcaster.sendTransform(t)
            
            # Log occasionally for debugging (every 20 iterations)
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
                
            if self._debug_counter % 20 == 0:
                speed = np.sqrt(vel[0]**2 + vel[1]**2)
                self.get_logger().info(f"Speed: {speed:.3f}, Confidence: {self.last_confidence:.2f}, Yaw: {np.arctan2(vel[1], vel[0]):.2f} rad")


def main(args=None):

    rclpy.init(args=args)

    node = SingleOpponentKalmanFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()