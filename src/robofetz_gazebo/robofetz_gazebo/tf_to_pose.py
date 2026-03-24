#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped

class TFToPoseStamped(Node):
    def __init__(self):
        super().__init__('tf_to_pose_stamped')
        
        # Declare required parameters
        self.declare_parameter('parent_frame')  # Source frame
        self.declare_parameter('child_frame')   # Target frame
        self.declare_parameter('pose_topic')    # Output topic
        self.declare_parameter('publish_rate', 10.0)  # Publishing rate in Hz
        
        # Get parameter values
        try:
            self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
            self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
            self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
            self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        except Exception as e:
            self.get_logger().error(
                "Required parameters not set! You must provide 'parent_frame', "
                "'child_frame', and 'pose_topic' parameters."
            )
            raise
        
        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher
        self.pub = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            10)
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info(f"Looking up transform from '{self.parent_frame}' to '{self.child_frame}'")
        self.get_logger().info(f"Publishing PoseStamped messages on '{self.pose_topic}' at {self.publish_rate} Hz")
    
    def timer_callback(self):
        """Timer callback to lookup and publish the transform as PoseStamped"""
        try:
            # Look up the latest transform
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame=self.parent_frame,
                source_frame=self.child_frame,
                time=rclpy.time.Time(),  # Get latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)  # Wait up to 0.1s for transform
            )
            
            # Convert TransformStamped to PoseStamped
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            # Publish the pose
            self.pub.publish(pose)
            
        except Exception as e:
            # Log warning if transform lookup fails
            self.get_logger().warning(
                f"Could not get transform from '{self.parent_frame}' to '{self.child_frame}': {str(e)}",
                throttle_duration_sec=5.0  # Throttle warnings to avoid spam
            )

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TFToPoseStamped()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()