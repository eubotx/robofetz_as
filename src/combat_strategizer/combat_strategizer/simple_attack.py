import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SimpleAttack(Node):
    def __init__(self):
        super().__init__('simple_attack')  # Node name is simple_attack

        # Create a subscriber to the /camera/opponent/pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/opponent/pose_sim',  # Original topic
            self.listener_callback,
            10)

        # Create a publisher to the /goal_pose topic
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info('Simple Attack Node Started!')

    def listener_callback(self, msg):
        # Log received PoseStamped message
        #self.get_logger().info(f'Received PoseStamped message: position ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})')

        # Remap the PoseStamped message to the new topic
        self.publisher.publish(msg)
        #self.get_logger().info(f'Published PoseStamped message to /goal_pose')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAttack()  # Node name is SimpleAttack
    rclpy.spin(node)
    rclpy.shutdown()
