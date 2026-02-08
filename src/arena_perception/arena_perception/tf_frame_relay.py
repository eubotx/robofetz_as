#!/usr/bin/env python3
"""
TF Frame Relay Node
Relays/republishes transform from source frame to target frame with optional prefix.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import sys

class TFFrameRelay(Node):
    def __init__(self):
        super().__init__('tf_frame_relay')
        
        # Declare parameters
        self.declare_parameter('source_frame', 'robot/base_footprint_sim')
        self.declare_parameter('target_frame', 'arena_perception/robot/base_footprint')
        self.declare_parameter('rate', 60.0)  # Hz
        
        # Get parameters
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.rate = self.get_parameter('rate').value
        
        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(f'Relaying transform from {self.source_frame} to {self.target_frame}')
        self.get_logger().info(f'Broadcast rate: {self.rate} Hz')
        
        # Create timer
        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)
        
    def timer_callback(self):
        try:
            # Look up transform from source frame
            transform = self.tf_buffer.lookup_transform(
                'world',  # Reference frame
                self.source_frame,  # Child frame
                rclpy.time.Time()
            )
            
            # Create new transform message with target frame
            new_transform = TransformStamped()
            new_transform.header.stamp = self.get_clock().now().to_msg()
            new_transform.header.frame_id = transform.header.frame_id
            new_transform.child_frame_id = self.target_frame
            new_transform.transform = transform.transform
            
            # Broadcast the new transform
            self.tf_broadcaster.sendTransform(new_transform)
            
        except Exception as e:
            self.get_logger().debug(f'Could not transform {self.source_frame} to world: {str(e)}', 
                                   throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = TFFrameRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()