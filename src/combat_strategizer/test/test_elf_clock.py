from unittest.mock import patch

from combat_strategizer.elf_combat_strategizer import ElfCombatStrategizer, ElfState
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.clock import ClockType
from rclpy.parameter import Parameter
from rclpy.time import Time


@pytest.fixture
def strategizer():
    rclpy.init(args=[])
    node = ElfCombatStrategizer()
    node.set_parameters([Parameter('use_sim_time', value=True)])
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def set_sim_time(strategizer, seconds):
    strategizer.get_clock().set_ros_time_override(
        Time(seconds=seconds, clock_type=ClockType.ROS_TIME)
    )


def test_wall_recovery_waits_for_simulation_time(strategizer):
    set_sim_time(strategizer, 10.0)
    strategizer.robot_pose_callback(PoseStamped())
    strategizer.autonomy = True

    with patch.object(strategizer, 'send_command') as send:
        strategizer.transition_to_wall_recovery()
        strategizer.state_machine_loop()
        send.assert_called_with('BACKWARD:0.5')

        set_sim_time(strategizer, 11.0)
        strategizer.state_machine_loop()
        strategizer.state_machine_loop()
        assert strategizer.state == ElfState.WALLRECOVERY
        assert send.call_count == 2

        set_sim_time(strategizer, 12.0)
        strategizer.robot_pose_callback(PoseStamped())
        strategizer.state_machine_loop()
        assert strategizer.state == ElfState.ATTACK
        send.assert_called_with('STOP')


def test_rewinding_clock_clears_poses_and_stops_recovery(strategizer):
    set_sim_time(strategizer, 10.0)
    strategizer.robot_pose_callback(PoseStamped())
    strategizer.opponent_pose_callback(PoseStamped())
    strategizer.transition_to_wall_recovery()

    with patch.object(strategizer, 'send_command') as send:
        set_sim_time(strategizer, 2.0)
        assert strategizer.state == ElfState.IDLE
        assert not strategizer.check_robot_pose_valid()
        assert strategizer.opponent_pose is None
        send.assert_called_once_with('STOP')
