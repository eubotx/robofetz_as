#!/usr/bin/env python3
import math
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ElfState(Enum):
    IDLE = 0
    ATTACK = 1
    DEFENSE = 2
    WALLRECOVERY = 3


class ElfCombatStrategizer(Node):

    def __init__(self):
        super().__init__('elf_combat_strategizer')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('arena_width', 1.5),
                ('arena_height', 1.5),
                ('wall_threshold', 0.1),
                ('center_x', 0.75),
                ('center_y', 0.75),
                ('escape_offset', 0.2),
                ('proximity_radius', 0.2),
                ('proximity_duration', 5.0),
                ('attack_angle_threshold', 10.0),
                ('defense_angle_threshold', 10.0),
                ('robot_pose_timeout', 2.0),
                ('forward_speed', 0.3),
                ('backward_speed', 0.3),
                ('turn_speed', 1.0),
                ('escape_reached_threshold', 0.1),
                ('wall_recovery_backward_distance', 0.1),
                ('turn_tolerance', 0.05),
            ]
        )

        self.arena_width = self.get_parameter('arena_width').value
        self.arena_height = self.get_parameter('arena_height').value
        self.wall_threshold = self.get_parameter('wall_threshold').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.escape_offset = self.get_parameter('escape_offset').value
        self.proximity_radius = self.get_parameter('proximity_radius').value
        self.proximity_duration = self.get_parameter('proximity_duration').value
        attack_angle_deg = self.get_parameter('attack_angle_threshold').value
        self.attack_angle_threshold = math.radians(attack_angle_deg)
        defense_angle_deg = self.get_parameter('defense_angle_threshold').value
        self.defense_angle_threshold = math.radians(defense_angle_deg)
        self.robot_pose_timeout = self.get_parameter('robot_pose_timeout').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        escape_threshold = self.get_parameter('escape_reached_threshold').value
        self.escape_reached_threshold = escape_threshold
        wall_backward = self.get_parameter('wall_recovery_backward_distance').value
        self.wall_recovery_backward_distance = wall_backward
        self.turn_tolerance = self.get_parameter('turn_tolerance').value

        self.state = ElfState.IDLE
        self.autonomy = False
        self.robot_pose: PoseStamped | None = None
        self.robot_pose_last_time: float | None = None
        self.opponent_pose: PoseStamped | None = None

        self.proximity_start_time: float | None = None
        self.escape_position: tuple[float, float] | None = None
        self.wall_recovery_start_time: float | None = None
        self.wall_recovery_phase = 0
        self.wall_recovery_target_yaw: float | None = None

        self.last_command_time = 0.0
        self.command_interval = 0.05

        self.autonomy_sub = self.create_subscription(
            Bool,
            '/autonomy',
            self.autonomy_callback,
            10
        )

        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.robot_pose_callback,
            10
        )

        self.opponent_pose_sub = self.create_subscription(
            PoseStamped,
            '/arena_perception_opponent_base_footprint_pose',
            self.opponent_pose_callback,
            10
        )

        self.navigator_cmd_pub = self.create_publisher(String, '/elf_navigator_cmd', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.create_timer(self.command_interval, self.state_machine_loop)

        self.get_logger().info('Elf Combat Strategizer initialized!')
        self.get_logger().info(f'Arena: {self.arena_width}m x {self.arena_height}m')
        self.get_logger().info(f'Center: ({self.center_x}, {self.center_y})')
        attack_deg = math.degrees(self.attack_angle_threshold)
        self.get_logger().info(f'Attack angle threshold: {attack_deg:.1f} deg')
        defense_deg = math.degrees(self.defense_angle_threshold)
        self.get_logger().info(f'Defense angle threshold: {defense_deg:.1f} deg')

    def autonomy_callback(self, msg: Bool):
        self.autonomy = msg.data
        self.get_logger().info(f'Autonomy: {self.autonomy}')

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg
        self.robot_pose_last_time = time.time()

    def opponent_pose_callback(self, msg: PoseStamped):
        self.opponent_pose = msg

    def state_machine_loop(self):
        robot_pose_valid = self.check_robot_pose_valid()

        if not robot_pose_valid or not self.autonomy:
            if self.state != ElfState.IDLE:
                self.get_logger().info(
                    'Transitioning to IDLE (autonomy=False or pose timeout)'
                )
                self.transition_to_idle()
            self.execute_idle()
            return

        if self.state == ElfState.IDLE:
            if self.autonomy and robot_pose_valid:
                self.get_logger().info('Transitioning to ATTACK')
                self.transition_to_attack()

        elif self.state == ElfState.ATTACK:
            if self.check_proximity_condition():
                self.get_logger().info(
                    'Proximity condition met! Transitioning to DEFENSE'
                )
                self.transition_to_defense()
            elif self.check_wall_condition():
                self.get_logger().info(
                    'Wall condition met! Transitioning to WALLRECOVERY'
                )
                self.transition_to_wall_recovery()
            else:
                self.execute_attack()

        elif self.state == ElfState.DEFENSE:
            if self.check_escape_reached():
                self.get_logger().info(
                    'Escape position reached! Transitioning to ATTACK'
                )
                self.transition_to_attack()
            else:
                self.execute_defense()

        elif self.state == ElfState.WALLRECOVERY:
            self.execute_wall_recovery()

    def check_robot_pose_valid(self) -> bool:
        if self.robot_pose is None or self.robot_pose_last_time is None:
            return False
        elapsed = time.time() - self.robot_pose_last_time
        return elapsed < self.robot_pose_timeout

    def check_proximity_condition(self) -> bool:
        if self.robot_pose is None or self.opponent_pose is None:
            self.proximity_start_time = None
            return False

        rx = self.robot_pose.pose.position.x
        ry = self.robot_pose.pose.position.y
        ox = self.opponent_pose.pose.position.x
        oy = self.opponent_pose.pose.position.y

        distance = math.sqrt((ox - rx) ** 2 + (oy - ry) ** 2)

        self.get_logger().info(f'Proximity check: distance={distance:.2f}m')
        self.get_logger().info( f'Coordinates: {rx} , {ry}')
        current_yaw = self.quaternion_to_yaw(self.robot_pose.pose.orientation)
        
        self.get_logger().info(f'yaw: {math.degrees(current_yaw):.2f}')
        self.get_logger().info

        now = time.time()


        if distance <= self.proximity_radius:
            if self.proximity_start_time is None:
                self.proximity_start_time = now
                self.get_logger().info(
                    f'PROXIMITY: Entered {self.proximity_radius}m radius, '
                    f'distance={distance:.2f}m, starting timer'
                )
            else:
                elapsed = now - self.proximity_start_time
                remaining = self.proximity_duration - elapsed
                if remaining > 0:
                    self.get_logger().info(
                        f'PROXIMITY: In range, {remaining:.1f}s until DEFENSE'
                    )
            if now - self.proximity_start_time >= self.proximity_duration:
                self.proximity_start_time = None
                return True
        else:
            self.proximity_start_time = None

        return False

    def check_wall_condition(self) -> bool:
        if self.robot_pose is None:
            return False

        rx = self.robot_pose.pose.position.x
        ry = self.robot_pose.pose.position.y

        near_wall = (rx < self.wall_threshold or
                     rx > self.arena_width - self.wall_threshold or
                     ry < self.wall_threshold or
                     ry > self.arena_height - self.wall_threshold)

        if not near_wall:
            return False

        current_yaw = self.quaternion_to_yaw(self.robot_pose.pose.orientation)
        
        self.get_logger().info(f'yaw: {current_yaw:.2f}')

        facing_out = False
        if rx < self.wall_threshold and abs(current_yaw) > math.pi *3/4:
            facing_out = True
        elif (rx > self.arena_width - self.wall_threshold and
              abs(current_yaw) < math.pi / 4):
            facing_out = True
        elif (ry < self.wall_threshold and
            #   ( current_yaw < -math.pi / 2 or current_yaw > math.pi / 2)):
              ( current_yaw  < -math.pi /4  and current_yaw > -math.pi * 3/4)):
            facing_out = True
        elif (ry > self.arena_height - self.wall_threshold and
              current_yaw > math.pi / 4 and  current_yaw < math.pi * 3/4):
            facing_out = True

        return facing_out

    def check_escape_reached(self) -> bool:
        if self.robot_pose is None or self.escape_position is None:
            return False

        rx = self.robot_pose.pose.position.x
        ry = self.robot_pose.pose.position.y
        ex, ey = self.escape_position

        distance = math.sqrt((ex - rx) ** 2 + (ey - ry) ** 2)
        return distance < self.escape_reached_threshold

    def transition_to_idle(self):
        self.state = ElfState.IDLE
        self.send_command('STOP')
        self.proximity_start_time = None

    def transition_to_attack(self):
        self.state = ElfState.ATTACK
        self.escape_position = None
        self.proximity_start_time = None

    def transition_to_defense(self):
        self.state = ElfState.DEFENSE
        self.escape_position = self.calculate_escape_position()
        self.send_command('STOP')
        self.get_logger().info(f'Escape position: {self.escape_position}')

    def transition_to_wall_recovery(self):
        self.state = ElfState.WALLRECOVERY
        self.wall_recovery_start_time = time.time()
        self.wall_recovery_phase = 0
        self.wall_recovery_target_yaw = None
        self.send_command('STOP')

    def execute_idle(self):
        if self.robot_pose is not None:
            goal = PoseStamped()
            goal.header = self.robot_pose.header
            goal.pose = self.robot_pose.pose
            self.goal_pose_pub.publish(goal)
        self.send_command('STOP')

    def execute_attack(self):
        if self.robot_pose is None or self.opponent_pose is None:
            self.send_command('STOP')
            return

        angle_to_opponent = self.calculate_angle_to_target(
            self.opponent_pose.pose.position.x,
            self.opponent_pose.pose.position.y
        )

        angle_deg = math.degrees(angle_to_opponent)
        threshold_deg = math.degrees(self.attack_angle_threshold)

        if abs(angle_to_opponent) > self.attack_angle_threshold:
            turn_cmd = self.turn_speed if angle_to_opponent > 0 else -self.turn_speed
            self.get_logger().info(
                f'ATTACK: Angle {angle_deg:.1f} deg > {threshold_deg:.1f} deg '
                f'threshold, TURNING'
            )
            self.send_command(f'TURN:{turn_cmd}')
        else:
            self.get_logger().info(
                f'ATTACK: Angle {angle_deg:.1f} deg OK, DRIVING FORWARD'
            )
            self.send_command(f'FORWARD:{self.forward_speed}')

    def execute_defense(self):
        if self.robot_pose is None or self.escape_position is None:
            self.send_command('STOP')
            return

        ex, ey = self.escape_position
        angle_to_escape = self.calculate_angle_to_target(ex, ey)

        backward_angle = self.normalize_angle(angle_to_escape + math.pi)

        rx = self.robot_pose.pose.position.x
        ry = self.robot_pose.pose.position.y
        dist_to_escape = math.sqrt((ex - rx) ** 2 + (ey - ry) ** 2)

        backward_deg = math.degrees(backward_angle)
        threshold_deg = math.degrees(self.defense_angle_threshold)

        if abs(backward_angle) > self.defense_angle_threshold:
            turn_cmd = self.turn_speed if backward_angle > 0 else -self.turn_speed
            self.get_logger().info(
                f'DEFENSE: Backward angle {backward_deg:.1f} deg > '
                f'{threshold_deg:.1f} deg threshold, TURNING'
            )
            self.send_command(f'TURN:{turn_cmd}')
        else:
            self.get_logger().info(
                f'DEFENSE: Angle OK ({backward_deg:.1f} deg), '
                f'DRIVING BACKWARD (dist to escape: {dist_to_escape:.2f}m)'
            )
            self.send_command(f'BACKWARD:{self.backward_speed}')

    def execute_wall_recovery(self):
        if self.robot_pose is None:
            self.send_command('STOP')
            return

        if self.wall_recovery_phase == 0:
            self.send_command(f'BACKWARD:{self.backward_speed}')
            self.wall_recovery_phase = 1
            self.wall_recovery_start_time = time.time()
            self.get_logger().info('WALLRECOVERY: Phase 1 - Backward')

        elif self.wall_recovery_phase == 1:
            elapsed = time.time() - self.wall_recovery_start_time
            backward_time = (self.wall_recovery_backward_distance /
                             self.backward_speed)

            if elapsed >= backward_time:
                self.send_command('STOP')
                self.wall_recovery_phase = 2
                current_yaw = self.quaternion_to_yaw(
                    self.robot_pose.pose.orientation
                )
                self.wall_recovery_target_yaw = self.normalize_angle(
                    current_yaw + math.pi
                )
                target_deg = math.degrees(self.wall_recovery_target_yaw)
                self.get_logger().info(
                    f'WALLRECOVERY: Phase 2 - Turn 180 deg to {target_deg:.1f}'
                )

        elif self.wall_recovery_phase == 2:
            current_yaw = self.quaternion_to_yaw(self.robot_pose.pose.orientation)
            yaw_error = self.normalize_angle(
                self.wall_recovery_target_yaw - current_yaw
            )

            if abs(yaw_error) < self.turn_tolerance:
                self.send_command('STOP')
                self.wall_recovery_phase = 3
                self.get_logger().info('WALLRECOVERY: Phase 3 - Turn complete')
            else:
                turn_cmd = self.turn_speed if yaw_error > 0 else -self.turn_speed
                error_deg = math.degrees(yaw_error)
                self.get_logger().info(
                    f'WALLRECOVERY: Turning, error={error_deg:.1f} deg'
                )
                self.send_command(f'TURN:{turn_cmd}')

        elif self.wall_recovery_phase == 3:
            self.send_command('STOP')
            self.get_logger().info(
                'WALLRECOVERY complete! Transitioning to ATTACK'
            )
            self.transition_to_attack()

    def calculate_escape_position(self) -> tuple[float, float]:
        escape_positions = [
            (self.center_x - self.escape_offset, self.center_y),
            (self.center_x + self.escape_offset, self.center_y),
            (self.center_x, self.center_y - self.escape_offset),
            (self.center_x, self.center_y + self.escape_offset),
        ]

        if self.opponent_pose is None:
            return escape_positions[0]

        ox = self.opponent_pose.pose.position.x
        oy = self.opponent_pose.pose.position.y

        def dist(pos):
            return math.sqrt((pos[0] - ox) ** 2 + (pos[1] - oy) ** 2)

        farthest = max(escape_positions, key=dist)
        return farthest

    def calculate_angle_to_target(self, target_x: float, target_y: float) -> float:
        if self.robot_pose is None:
            return 0.0

        rx = self.robot_pose.pose.position.x
        ry = self.robot_pose.pose.position.y

        desired_yaw = math.atan2(target_y - ry, target_x - rx)
        current_yaw = self.quaternion_to_yaw(self.robot_pose.pose.orientation)

        angle_error = self.normalize_angle(desired_yaw - current_yaw)
        return angle_error

    @staticmethod
    def quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.navigator_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    strategizer = ElfCombatStrategizer()

    try:
        rclpy.spin(strategizer)
    except KeyboardInterrupt:
        pass
    finally:
        strategizer.send_command('STOP')
        strategizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
