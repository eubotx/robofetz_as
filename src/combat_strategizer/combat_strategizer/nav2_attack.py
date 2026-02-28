import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

# reuse the high‑level navigator implementation for orientation + goals
from robot_navigation.nav2_navigator import Nav2Navigator


class Nav2Attack(Nav2Navigator):
    def __init__(self):
        super().__init__()

        # subscription from the original implementation
        self.subscription = self.create_subscription(
            PoseStamped,
            '/arena_perception_opponent_base_footprint_pose',
            self.listener_callback,
            10)

        self.last_goal_pose = None
        self.min_distance_change = 0.1  # Only update goal if target moved 0.1m

        self.get_logger().info('Nav2 Attack Node Started!')

    def listener_callback(self, msg):
        # Check if we should send a new goal and use the helper method
        if self.should_send_new_goal():
            self.get_logger().info('Received new target pose, sending attack command')
            # Nav2Navigator.attack handles orientation plus sending
            self.attack(msg)

    def should_send_new_goal(self):
        # if self.last_goal_pose is None:
        #     return True

        # # Calculate distance between last goal and current target pose
        # dx = current_pose_msg.pose.position.x - self.last_goal_pose.pose.position.x
        # dy = current_pose_msg.pose.position.y - self.last_goal_pose.pose.position.y
        # dist = math.sqrt(dx*dx + dy*dy)

        # # If target moved significantly, update goal
        # if dist > self.min_distance_change:
        #     return True

        # return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Attack()
    rclpy.spin(node)
    rclpy.shutdown()
