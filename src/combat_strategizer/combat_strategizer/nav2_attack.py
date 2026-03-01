import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from enum import Enum
import math
import time

from robot_navigation.nav2_navigator import Nav2Navigator
from combat_strategizer.states import CombatState
from combat_strategizer.conditions import (
    ProximityCondition,
    NavDoneCondition,
    TimeoutCondition,
)


class State(Enum):
    ATTACK = 1
    DEFENCE = 2
    STANDBY = 3


class ProximityCondition:
    def __init__(self, threshold: float = 0.15, duration: float = 5.0):
        self.threshold = threshold
        self.duration = duration
        self.proximity_start: float | None = None

    def check(self, robot_pose, opponent_pose) -> bool:
        if robot_pose is None or opponent_pose is None:
            self.proximity_start = None
            return False

        dx = opponent_pose.pose.position.x - robot_pose.position.x
        dy = opponent_pose.pose.position.y - robot_pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        now = time.time()

        if distance <= self.threshold:
            if self.proximity_start is None:
                self.proximity_start = now
            elif now - self.proximity_start >= self.duration:
                self.proximity_start = None
                return True
        else:
            self.proximity_start = None

        return False

    def reset(self):
        self.proximity_start = None


class Nav2Attack(Nav2Navigator):
    def __init__(self):
        super().__init__()

        self.combat_state = State.ATTACK

        self.condition = ProximityCondition(threshold=0.23, duration=2.0)

        self.opponent_subscription = self.create_subscription(
            PoseStamped,
            '/arena_perception_opponent_base_footprint_pose',
            self.opponent_callback,
            10)

        self.defense_subscription = self.create_subscription(
            PointStamped,
            '/defense_position',
            self.defense_callback,
            10)

        self.opponent_pose: PoseStamped | None = None
        self.defense_position: PointStamped | None = None

        self.defence_timer_start: float | None = None
        self.defence_duration: float = 3.0

        self.min_distance_change = 0.1
        self.last_goal_pose = None

        self.create_timer(0.1, self.state_machine_loop)

        self.state = CombatState.ATTACK
        self.opponent_pose = None
        self.latest_defense_position = None
        self.defense_pose = None
        self.defence_start_time = None

    def opponent_callback(self, msg: PoseStamped):
        self.opponent_pose = msg
        if self.combat_state == State.ATTACK:
            self._handle_attack_goal(msg)

    def defense_callback(self, msg: PointStamped):
        self.defense_position = msg

    def _handle_attack_goal(self, msg: PoseStamped):
        if not self.should_send_new_goal(msg):
            return
        self.get_logger().info('Sending attack command')
        self.attack(msg)

    def should_send_new_goal(self, goal_pose: PoseStamped) -> bool:
        if self.last_goal_pose is None:
            return True
        dx = goal_pose.pose.position.x - self.last_goal_pose.pose.position.x
        dy = goal_pose.pose.position.y - self.last_goal_pose.pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)
        return dist >= self.min_distance_change

    def state_machine_loop(self):
        if self.combat_state == State.ATTACK:
            self._attack_state()
        elif self.combat_state == State.DEFENCE:
            self._defence_state()
        elif self.combat_state == State.STANDBY:
            self._standby_state()

    def _attack_state(self):
        if self.condition.check(self.current_pose, self.opponent_pose):
            self.get_logger().info('Proximity condition met! Transitioning to DEFENCE')
            self.condition.reset()
            self._transition_to_defence()

    def _transition_to_defence(self):
        self.combat_state = State.DEFENCE
        self.defence_timer_start = None

        if self.defense_position is not None and self.opponent_pose is not None:
            safe_pose = PoseStamped()
            safe_pose.header = self.defense_position.header
            safe_pose.pose.position.x = self.defense_position.point.x
            safe_pose.pose.position.y = self.defense_position.point.y
            safe_pose.pose.position.z = self.defense_position.point.z
            safe_pose.pose.orientation.w = 1.0
            self.escape(self.opponent_pose, safe_pose)
            self.get_logger().info('Navigating to defense position')
        else:
            self.get_logger().warn('No defense position available, waiting...')

    def _defence_state(self):
        if self.defense_position is None:
            return

        if self.defence_timer_start is None:
            if self.state == "IDLE":
                self.defence_timer_start = time.time()
                self.get_logger().info('Reached defense position, starting 3s timer')
        else:
            elapsed = time.time() - self.defence_timer_start
            if elapsed >= self.defence_duration:
                self.get_logger().info('Defence timer complete, transitioning to STANDBY')
                self.combat_state = State.STANDBY

    def _standby_state(self):
        self.get_logger().info('In STANDBY, returning to ATTACK')
        self.combat_state = State.ATTACK
        if self.opponent_pose is not None:
            self.attack(self.opponent_pose)


def main(args=None):
    rclpy.init(args=args)
    
    node = Nav2Attack()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
