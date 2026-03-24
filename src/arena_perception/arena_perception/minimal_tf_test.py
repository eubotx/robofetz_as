#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseArray
from vision_msgs.msg import Detection3DArray
from tf2_ros import TransformBroadcaster
import numpy as np
import threading

class MinimalTfTest(Node):
    def __init__(self):
        super().__init__('minimal_tf_test')
        
        # Match your node's structure
        self.tracks = {}
        self.lock = threading.Lock()
        self.target_frame = 'world'
        self.next_track_id = 0
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers (match your node)
        self.publisher = self.create_publisher(Detection3DArray, '/fused_tracks', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/filtered_poses', 10)
        
        # Timer at same rate as your node
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        # Create some dummy tracks that move
        self.create_dummy_tracks()
        
        self.get_logger().info('🚀 Minimal TF Test Node Started')
        self.get_logger().info(f'Publishing transforms to /tf topic')
        
    def create_dummy_tracks(self):
        """Create 3 dummy tracks that move in a circle"""
        with self.lock:
            for i in range(3):
                track = DummyTrack(i, i * 2.0)  # Different starting angles
                self.tracks[i] = track
                self.next_track_id = i + 1
        
        self.get_logger().info(f'Created {len(self.tracks)} dummy tracks')
    
    def timer_callback(self):
        """Timer callback that matches your node's structure"""
        current_time = self.get_clock().now()
        current_time_sec = current_time.nanoseconds / 1e9
        
        with self.lock:
            # Update all dummy tracks (simulating prediction)
            transforms = []
            
            for track_id, track in self.tracks.items():
                # Update track position (moving in circle)
                track.update(current_time_sec)
                pos = track.get_position()
                
                # Create transform - EXACTLY like your node
                t = TransformStamped()
                t.header.stamp = current_time.to_msg()
                t.header.frame_id = self.target_frame
                t.child_frame_id = f"opponent_{track_id}"
                
                # Set translation
                t.transform.translation.x = float(pos[0])
                t.transform.translation.y = float(pos[1])
                t.transform.translation.z = float(pos[2])
                
                # Simple rotation (always identity for now)
                t.transform.rotation.w = 1.0
                
                transforms.append(t)
                
                # Log each track's position occasionally
                if track_id == 0 and int(current_time_sec * 2) % 2 == 0:
                    self.get_logger().info(f'Track 0: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')
            
            # PUBLISH TRANSFORMS - exactly like your node
            if transforms:
                self.tf_broadcaster.sendTransform(transforms)
                
                # Log once per second
                if int(current_time_sec) > getattr(self, '_last_log_sec', 0):
                    self.get_logger().info(f'📍 Published {len(transforms)} transforms at {current_time_sec:.1f}s')
                    self._last_log_sec = int(current_time_sec)


class DummyTrack:
    """Simple dummy track that moves in a circle"""
    def __init__(self, track_id, start_angle=0):
        self.track_id = track_id
        self.start_angle = start_angle
        self.radius = 3.0
        self.speed = 0.5
        
    def update(self, time_sec):
        # Store time for position calculation
        self.current_time = time_sec
        
    def get_position(self):
        # Move in a circle
        angle = self.start_angle + self.current_time * self.speed
        x = self.radius * np.cos(angle)
        y = self.radius * np.sin(angle)
        z = 1.0
        return np.array([x, y, z])


def main(args=None):
    rclpy.init(args=args)
    node = MinimalTfTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()