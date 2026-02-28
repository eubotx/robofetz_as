#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import math


class Nav2Navigator(Node):
    """High‑level navigation helper that wraps the Nav2 action client and
    provides convenience routines such as orienting towards a target before
    sending a goal.  It can be used by other nodes (for example a
    strategist) to issue ``attack`` and ``escape`` commands without having
    to manage the low‑level details themselves.

    The behaviour is deliberately similar to :class:`SimpleNavigator` and
    :class:`Nav2Attack` so that existing code can be refactored to use it
    without changing how the robot is driven.
    """

    def __init__(self):
        super().__init__('nav2_navigator')

        # navigation parameters (shared with simple_navigator)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_kp', 0.5),
                ('angular_kp', 1.0),
                ('max_linear', 0.2),
                ('max_angular', 1.0),
                ('orientation_tolerance', 0.1),
                ('goal_frame', 'map'),
            ],
        )

        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.goal_frame = self.get_parameter('goal_frame').value

        # current robot pose coming from camera or odom
        self.current_pose = None
        self.current_pose_header = None

        # keep track of last goal sent; helpers can use this when deciding
        # whether a new goal should be issued.
        self.last_goal_pose = None

        # subscribers / publishers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/arena_perception_robot_base_footprint_pose', self._pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_in_progress = False

    # ------------------------------------------------------------------
    # callbacks and helpers
    # ------------------------------------------------------------------
    def _pose_callback(self, msg: PoseStamped) -> None:
        """Keep the last received robot pose for orientation calculations."""
        self.current_pose = msg.pose
        self.current_pose_header = msg.header

    @staticmethod
    def quaternion_to_yaw(q):
        # same implementation as SimpleNavigator
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ------------------------------------------------------------------
    # orientation logic
    # ------------------------------------------------------------------
    def orient_towards(self, target: PoseStamped) -> None:
        """Rotate the base until it faces *target*.

        The routine will block until the yaw error is within
        ``orientation_tolerance``.  It uses the same simple proportional
        controller as :class:`SimpleNavigator` so the behaviour is familiar.

        This method calls ``rclpy.spin_once`` internally to allow the
        subscription to the current pose to be serviced while spinning.
        """

        if self.current_pose is None or self.current_pose_header is None:
            self.get_logger().warn('No current pose available, cannot orient')
            return

        # make sure the frame of the pose is set correctly
        if target.header.frame_id == '':
            target = PoseStamped(header=target.header, pose=target.pose)
            target.header.frame_id = self.goal_frame

        # loop until oriented
        rate = self.create_rate(10)
        while rclpy.ok():
            # compute yaw error
            dx = target.pose.position.x - self.current_pose.position.x
            dy = target.pose.position.y - self.current_pose.position.y
            desired_yaw = math.atan2(dy, dx)
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            yaw_error = self.normalize_angle(desired_yaw - current_yaw)

            if abs(yaw_error) <= self.orientation_tolerance:
                break

            cmd = Twist()
            cmd.angular.z = max(min(self.angular_kp * yaw_error,
                                     self.max_angular), -self.max_angular)
            self.cmd_pub.publish(cmd)

            # allow callbacks to run so current_pose is updated
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()

        # stop rotation
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Nav2 goal wrappers
    # ------------------------------------------------------------------
    def _send_nav_goal(self, pose: PoseStamped) -> None:
        """Send a goal to the Nav2 ``navigate_to_pose`` action server."""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Sending Nav2 goal ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)
        self.goal_in_progress = True
        self._last_goal_handle = None

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Nav2 goal rejected')
            self.goal_in_progress = False
            return
        self.get_logger().info('Nav2 goal accepted')
        self._last_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Nav2 navigation finished')
        self.goal_in_progress = False

    # ------------------------------------------------------------------
    # high‑level behaviours
    # ------------------------------------------------------------------
    def attack(self, opponent_pose: PoseStamped) -> None:
        """Face the opponent and then drive towards their last known pose.

        ``last_goal_pose`` is updated so that callers can decide whether a new
        request should be made (for example only when the opponent has moved).
        """
        self.orient_towards(opponent_pose)
        self._send_nav_goal(opponent_pose)
        self.last_goal_pose = opponent_pose

    def escape(self, opponent_pose: PoseStamped, safe_pose: PoseStamped) -> None:
        """Face the opponent then drive to a predefined *safe_pose*.

        ``safe_pose`` can be any PoseStamped (for example a fixed corner of the
        arena).  ``opponent_pose`` is used only to compute the orientation.
        """
        self.orient_towards(opponent_pose)
        self._send_nav_goal(safe_pose)


# Stand‑alone node for demonstration / testing purposes.  It listens on two
# topics, ``/attack_command`` and ``/escape_command`` which carry
# :class:`geometry_msgs.msg.PoseStamped`.  When a message arrives the
# corresponding behaviour is triggered.

def main(args=None):
    rclpy.init(args=args)
    navigator = Nav2Navigator()

    # subscribers for demo
    def attack_cb(msg):
        navigator.attack(msg)

    def escape_cb(msg):
        # for escape we expect the goal to be stored in the message's pose
        # fields.  it is easier for the sender to pack both poses in the
        # orientation part of the message if necessary, but we keep it simple
        # here and assume the goal is conveyed separately by setting the
        # message's frame_id to "safe".
        #
        # for the purposes of this node we treat the received message as an
        # opponent pose and drive to the origin of the map afterwards.
        safe = PoseStamped()
        safe.header.frame_id = navigator.goal_frame
        safe.header.stamp = navigator.get_clock().now().to_msg()
        safe.pose.position.x = 0.0
        safe.pose.position.y = 0.0
        safe.pose.orientation = msg.pose.orientation
        navigator.escape(msg, safe)

    navigator.create_subscription(PoseStamped, '/attack_command', attack_cb, 10)
    navigator.create_subscription(PoseStamped, '/escape_command', escape_cb, 10)

    rclpy.spin(navigator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
