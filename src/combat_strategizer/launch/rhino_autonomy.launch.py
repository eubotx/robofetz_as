from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    simple_navigator = Node(
        package='robot_navigation',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
    )

    simple_attack = Node(
        package='combat_strategizer',
        executable='simple_attack',
        name='simple_attack',
    )

    weapon_control = Node(
        package='combat_strategizer',
        executable='weapon_control',
        name='weapon_control',
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(simple_navigator)
    ld.add_action(simple_attack)
    ld.add_action(weapon_control)

    return ld