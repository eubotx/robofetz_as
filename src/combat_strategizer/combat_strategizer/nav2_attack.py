import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math

class Nav2Attack(Node):
    def __init__(self):
        super().__init__('nav2_attack')

        # Create a subscriber to the /opponent/pose_sim topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/opponent/pose_sim',
            self.listener_callback,
            10)

        # Create an action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.last_goal_pose = None
        self.goal_in_progress = False
        self.min_distance_change = 0.1  # Only update goal if target moved 0.5m
        
        self.get_logger().info('Nav2 Attack Node Started!')

    def listener_callback(self, msg):
        # Check if we should send a new goal
        # if self.should_send_new_goal(msg):
            # self.send_goal(msg)
        self.send_goal(msg)

    def should_send_new_goal(self, current_pose_msg):
        if self.last_goal_pose is None:
            return True
            
        # Calculate distance between last goal and current target pose
        dx = current_pose_msg.pose.position.x - self.last_goal_pose.pose.position.x
        dy = current_pose_msg.pose.position.y - self.last_goal_pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        # If target moved significantly, update goal
        if dist > self.min_distance_change:
            return True
            
        return False

    def send_goal(self, pose_msg):
        # Check if action server is ready without blocking forever
        if not self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 Action Server not available yet! Is Nav2 running?')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        
        # Ensure the frame is correct (optional, but good practice)
        # goal_msg.pose.header.frame_id = 'map' 
        
        self.get_logger().info(f'Sending goal: ({pose_msg.pose.position.x}, {pose_msg.pose.position.y})')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.last_goal_pose = pose_msg
        self.goal_in_progress = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_in_progress = False
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal finished')
        self.goal_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Attack()
    rclpy.spin(node)
    rclpy.shutdown()
