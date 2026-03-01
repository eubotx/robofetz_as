#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import math

class Nav2Navigator(Node):
    """
    High-level navigation helper that wraps the Nav2 action client.
    Uses a non-blocking state machine to avoid deadlocks.
    """

    def __init__(self):
        super().__init__('nav2_navigator')

        # Navigation parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_kp', 0.5),
                ('angular_kp', 1.5),
                ('max_angular', 1.0),
                ('orientation_tolerance', 0.1),
                ('goal_frame', 'map'),
                ('angular_ki', 0.05),
                ('angular_kd', 0.1),
            ],
        )

        # Internal state
        self.state = "IDLE"  # IDLE, ALIGNING, NAVIGATING
        self.current_pose = None
        self.target_pose = None
        self.goal_to_send = None # The final destination after orienting
        self.last_goal_pose = None

        # PID state
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        # Pubs/Subs
        self.pose_sub = self.create_subscription(
            PoseStamped, '/arena_perception_robot_base_footprint_pose', self._pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Control Timer - This replaces the blocking while-loop
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info("Nav2 Navigator initialized and ready.")

    # ------------------------------------------------------------------
    # State Logic (The "Attack" and "Escape" you asked for)
    # ------------------------------------------------------------------
    
    def attack(self, opponent_pose: PoseStamped) -> None:
        """Face the opponent and then drive towards them."""
        self.get_logger().info("Orienting first...")
        self.target_pose = opponent_pose
        self.goal_to_send = opponent_pose # After turning, go to opponent
        self.last_goal_pose = opponent_pose
        self._start_alignment()

    def escape(self, opponent_pose: PoseStamped, safe_pose: PoseStamped) -> None:
        """Face the opponent, then drive to a safe location."""
        self.get_logger().info("State: ESCAPE. Orienting towards threat...")
        self.target_pose = opponent_pose
        self.goal_to_send = safe_pose # After turning, go to safety
        self._start_alignment()

    def _start_alignment(self):
        """Resets PID and triggers the control loop to start turning."""
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()
        self.state = "ALIGNING"

    # ------------------------------------------------------------------
    # Background Control Loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        """Executes one step of PID if in ALIGNING state. Non-blocking."""
        if self.state != "ALIGNING" or self.current_pose is None or self.target_pose is None:
            return

        # 1. Heading Math
        dx = self.target_pose.pose.position.x - self.current_pose.position.x
        dy = self.target_pose.pose.position.y - self.current_pose.position.y
        desired_yaw = math.atan2(dy, dx)

        q = self.current_pose.orientation
        current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Normalized error between -pi and pi
        error = math.atan2(math.sin(desired_yaw - current_yaw), math.cos(desired_yaw - current_yaw))

        # 2. Check if we are facing the right way
        if abs(error) < self.get_parameter('orientation_tolerance').value:
            self.get_logger().info("Aligned! Sending Nav2 goal.")
            self.cmd_pub.publish(Twist()) # Stop the motors
            self.state = "NAVIGATING"
            self._send_nav_goal(self.goal_to_send)
            return

        # 3. PID calculation
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: dt = 0.05

        p = self.get_parameter('angular_kp').value * error
        self.error_integral += error * dt
        i = self.get_parameter('angular_ki').value * max(min(self.error_integral, 1.0), -1.0)
        d = self.get_parameter('angular_kd').value * (error - self.last_error) / dt

        angular_vel = max(min(p + i + d, self.get_parameter('max_angular').value), -self.get_parameter('max_angular').value)

        cmd = Twist()
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

        self.last_error = error
        self.last_time = now

    # ------------------------------------------------------------------
    # Nav2 Goal & Callbacks
    # ------------------------------------------------------------------

    def _send_nav_goal(self, pose: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 Action Server not available!")
            self.state = "IDLE"
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.frame_id = self.get_parameter('goal_frame').value
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self._action_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.state = "IDLE"
            return
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.get_logger().info("Navigation finished.")
        self.state = "IDLE"

    def _pose_callback(self, msg):
        self.current_pose = msg.pose

def main(args=None):
    rclpy.init(args=args)
    navigator = Nav2Navigator()

    # Subscription wrappers for the demo
    def attack_cb(msg):
        navigator.attack(msg)

    def escape_cb(msg):
        safe = PoseStamped()
        safe.header.frame_id = navigator.get_parameter('goal_frame').value
        safe.pose.position.x = 0.0
        safe.pose.position.y = 0.0
        navigator.escape(msg, safe)

    navigator.create_subscription(PoseStamped, '/attack_command', attack_cb, 10)
    navigator.create_subscription(PoseStamped, '/escape_command', escape_cb, 10)

    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()