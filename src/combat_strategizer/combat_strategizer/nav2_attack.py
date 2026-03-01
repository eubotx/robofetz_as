import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
import math
import time

from robot_navigation.nav2_navigator import Nav2Navigator
from combat_strategizer.states import CombatState
from combat_strategizer.conditions import (
    ProximityCondition,
    NavDoneCondition,
    TimeoutCondition,
)


class Nav2Attack(Nav2Navigator):
    def __init__(self):
        super().__init__()

        self.subscription = self.create_subscription(
            PoseStamped,
            '/arena_perception_opponent_base_footprint_pose',
            self.listener_callback,
            1)

        self.defense_sub = self.create_subscription(
            PointStamped,
            '/defense_position',
            self.defense_callback,
            1)

        self.state = CombatState.ATTACK
        self.opponent_pose = None
        self.latest_defense_position = None
        self.defense_pose = None
        self.defence_start_time = None

        self.last_update_time = time.time()
        self.update_interval = 1.0

        self.retreat_conditions = [
            ProximityCondition(threshold=0.3, duration=2.0),
        ]

        self.defence_exit_conditions = [
            NavDoneCondition(),
            TimeoutCondition(duration=3.0),
        ]

        self.check_timer = self.create_timer(self.update_interval, self.check_loop)
        self.status_timer = self.create_timer(1.0, self.log_status)

        self._robot_pose_available = False
        self._log_counter = 0
        self._pose_received = False
        self.get_logger().info('Nav2 Attack Node Started with State Machine!')

    def _pose_callback(self, msg: PoseStamped) -> None:
        super()._pose_callback(msg)
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info(
                f'Robot pose received! Position: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )

    def defense_callback(self, msg: PointStamped):
        self.latest_defense_position = msg

    def log_status(self):
        self.get_logger().info(f"Current state: {self.state.name}")
        self._log_counter += 1
        if self.state == CombatState.ATTACK:
            if self.current_pose is None:
                self.get_logger().info(
                    f'[{self._log_counter}] Robot pose not available - cannot evaluate retreat conditions! '
                    f'Is /arena_perception_robot_base_footprint_pose publishing?'
                )
            else:
                for condition in self.retreat_conditions:
                    status = condition.status
                    self.get_logger().info(
                        f"[{self._log_counter}][ATTACK] {condition.__class__.__name__}: "
                        f"dist={status.get('distance', 'N/A'):.3f}m, "
                        f"accumulated={status.get('time_accumulated', 0):.1f}s / {status.get('duration', 0):.1f}s"
                    )

    def listener_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Opponent pose received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        # self.opponent_pose = msg

        if self.state == CombatState.ATTACK:
            self.get_logger().info('In ATTACK state - evaluating retreat conditions')
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            self.get_logger().debug(f"Time since last update: {dt:.2f}s")

            if self.current_pose is None:
                if not self._robot_pose_available:
                    self._robot_pose_available = True
            else:
                self.get_logger().info(f'Executing attack behavior with opponent pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
                self.attack(msg)
                for condition in self.retreat_conditions:
                    condition.update(
                        robot_pose=self.current_pose,
                        opponent_pose=msg,
                        dt=dt
                    )

            triggered_conditions = [c for c in self.retreat_conditions if c.is_triggered()]
            if triggered_conditions:
                self.get_logger().info(
                    f"Retreat conditions triggered: {[c.__class__.__name__ for c in triggered_conditions]}"
                )
                self._enter_defence()

    def check_loop(self):
        self.get_logger().debug('Running check loop')
        if self.state == CombatState.DEFENCE:
            if self.defence_start_time is None:
                return

            elapsed = (self.get_clock().now() - self.defence_start_time).nanoseconds / 1e9

            for condition in self.defence_exit_conditions:
                condition.update(
                    elapsed=elapsed,
                    goal_in_progress=self.goal_in_progress
                )
                status = condition.status
                self.get_logger().debug(
                    f"Exit condition {condition.__class__.__name__}: {status}"
                )

            triggered_conditions = [c for c in self.defence_exit_conditions if c.is_triggered()]
            if triggered_conditions:
                self.get_logger().info(
                    f"Defence exit conditions triggered: {[c.__class__.__name__ for c in triggered_conditions]}"
                )
                self._enter_standby()

    def _enter_defence(self):
        self.state = CombatState.DEFENCE
        self.get_logger().info('Entering DEFENCE state')

        if self.latest_defense_position is None:
            self.get_logger().warn('No defense position available, staying in attack')
            self.state = CombatState.ATTACK
            return

        self.defense_pose = PoseStamped()
        self.defense_pose.header = self.latest_defense_position.header
        self.defense_pose.pose.position.x = self.latest_defense_position.point.x
        self.defense_pose.pose.position.y = self.latest_defense_position.point.y
        self.defense_pose.pose.position.z = self.latest_defense_position.point.z
        self.defense_pose.pose.orientation.w = 1.0

        self.defence_start_time = self.get_clock().now()

        for condition in self.defence_exit_conditions:
            condition.reset()

        if self.opponent_pose is not None:
            self.escape(self.opponent_pose, self.defense_pose)
        else:
            self._send_nav_goal(self.defense_pose)

    def _enter_standby(self):
        self.state = CombatState.STANDBY
        self.get_logger().info('Entering STANDBY state - returning to ATTACK')

        for condition in self.retreat_conditions:
            condition.reset()

        self.state = CombatState.ATTACK
        self.get_logger().info('Back to ATTACK state')


def main(args=None):
    rclpy.init(args=args)
    
    node = Nav2Attack()
    
    # Use MultiThreadedExecutor for better timer handling
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
