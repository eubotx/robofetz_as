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

    simple_combat_strategizer = Node(
        package='combat_strategizer',
        executable='simple_combat_strategizer',
        name='simple_combat_strategizer',
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(simple_navigator)
    ld.add_action(simple_combat_strategizer)

    return ld
