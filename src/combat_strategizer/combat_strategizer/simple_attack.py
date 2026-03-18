import math
import time

from combat_strategizer.conditions import ProximityCondition
from combat_strategizer.states import CombatState
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node


class SimpleAttack(Node):
    def __init__(self):
        super().__init__('simple_attack')

        self.combat_state = CombatState.ATTACK
        self.condition = ProximityCondition(threshold=0.23, duration=5.0)

        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            '/arena_perception_robot_base_footprint_pose',
            self.robot_pose_callback,
            10)

        self.opponent_sub = self.create_subscription(
            PoseStamped,
            '/arena_perception_opponent_base_footprint_pose',
            self.opponent_callback,
            10)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.escape_opponent_pub = self.create_publisher(
            PoseStamped, '/escape_opponent', 10
        )
        self.escape_safe_pub = self.create_publisher(
            PoseStamped, '/escape_safe', 10
        )

        self.robot_pose: PoseStamped | None = None
        self.opponent_pose: PoseStamped | None = None
        self.defense_position: PointStamped | None = None

        self.defence_timer_start: float | None = None
        self.defence_duration: float = 10.0

        self.escape_published = False

        self.create_timer(0.1, self.state_machine_loop)
        self.create_timer(0.5, self.calculate_defense_position)

        self.get_logger().info('Simple Attack Node with State Machine Started!')

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    def opponent_callback(self, msg: PoseStamped):
        self.opponent_pose = msg
        if self.combat_state == CombatState.ATTACK:
            self.goal_pub.publish(msg)

    def calculate_defense_position(self):
        center_x = 0.75
        center_y = 0.75
        delta = 0.3

        corners = [
            (center_x - delta, center_y),
            (center_x + delta, center_y),
            (center_x, center_y - delta),
            (center_x, center_y + delta),
        ]

        if self.opponent_pose is None:
            return

        ox = self.opponent_pose.pose.position.x
        oy = self.opponent_pose.pose.position.y

        farthest_corner = max(
            corners, key=lambda c: math.sqrt((c[0] - ox) ** 2 + (c[1] - oy) ** 2)
        )

        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.point.x = farthest_corner[0]
        msg.point.y = farthest_corner[1]
        msg.point.z = 0.0
        self.defense_position = msg

    def state_machine_loop(self):
        self.get_logger().debug(f'Current state: {self.combat_state.name}')
        if self.combat_state == CombatState.ATTACK:
            self._attack_state()
        elif self.combat_state == CombatState.DEFENCE:
            self._defence_state()
        elif self.combat_state == CombatState.STANDBY:
            self._standby_state()

    def _attack_state(self):
        if self.condition.is_triggered():
            self.get_logger().info(
                'Proximity condition met! Transitioning to DEFENCE'
            )
            self.condition.reset()
            self._transition_to_defence()
        else:
            dt = 0.1
            robot_pose_msg = self.robot_pose.pose if self.robot_pose else None
            self.condition.update(
                robot_pose=robot_pose_msg,
                opponent_pose=self.opponent_pose,
                dt=dt
            )

    def _transition_to_defence(self):
        self.combat_state = CombatState.DEFENCE
        self.defence_timer_start = None
        self.escape_published = False

        if self.defense_position is not None and self.opponent_pose is not None:
            safe_pose = PoseStamped()
            safe_pose.header = self.defense_position.header
            safe_pose.pose.position.x = self.defense_position.point.x
            safe_pose.pose.position.y = self.defense_position.point.y
            safe_pose.pose.position.z = self.defense_position.point.z
            safe_pose.pose.orientation.w = 1.0

            self.escape_opponent_pub.publish(self.opponent_pose)
            self.escape_safe_pub.publish(safe_pose)
            self.escape_published = True
            self.get_logger().info(
                f'Published escape command: safe pose at '
                f'({safe_pose.pose.position.x:.2f}, '
                f'{safe_pose.pose.position.y:.2f})'
            )
        else:
            self.get_logger().warn('No defense position available, waiting...')

    def _defence_state(self):
        if self.defence_timer_start is None:
            self.defence_timer_start = time.time()
            self.get_logger().info(
                f'Started defence timer ({self.defence_duration}s)'
            )
        else:
            elapsed = time.time() - self.defence_timer_start
            if elapsed >= self.defence_duration:
                self.get_logger().info(
                    'Defence timer complete, transitioning to STANDBY'
                )
                self.combat_state = CombatState.STANDBY

    def _standby_state(self):
        self.get_logger().info('In STANDBY, returning to ATTACK')
        self.combat_state = CombatState.ATTACK
        if self.opponent_pose is not None:
            self.goal_pub.publish(self.opponent_pose)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAttack()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
