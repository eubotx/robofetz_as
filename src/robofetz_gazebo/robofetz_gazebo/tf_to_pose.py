#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

class TFToPoseStamped(Node):
    def __init__(self):
        super().__init__('tf_to_pose_stamped')
        
        # Declare parameters without default values
        self.declare_parameter('tf_topic')
        self.declare_parameter('pose_topic')
        
        # Get parameter values - these will raise if parameters aren't set
        try:
            tf_topic = self.get_parameter('tf_topic').get_parameter_value().string_value
            pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        except Exception as e:
            self.get_logger().error("Required parameters not set! You must provide both 'tf_topic' and 'pose_topic' parameters.")
            raise
        
        self.sub = self.create_subscription(
            TFMessage,
            tf_topic,
            self.tf_callback,
            10)
            
        self.pub = self.create_publisher(
            PoseStamped,
            pose_topic,
            10)
        
        self.get_logger().info(f"Listening to TF messages on '{tf_topic}'")
        self.get_logger().info(f"Publishing PoseStamped messages on '{pose_topic}'")
    
    def tf_callback(self, msg):
        if not msg.transforms:
            return
            
        # Take the first transform (assuming single robot pose)
        tf = msg.transforms[0]
        
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        
        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TFToPoseStamped()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()