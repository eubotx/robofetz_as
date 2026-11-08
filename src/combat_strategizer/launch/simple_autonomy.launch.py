from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigator_config = DeclareLaunchArgument(
        'simple_navigator_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_navigation'), 'config', 'simple_navigator_config.yaml',
        ]),
        description='Path to the Simple navigator parameter file',
    )

    simple_navigator = Node(
        package='robot_navigation',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
        parameters=[LaunchConfiguration('simple_navigator_config')],
    )

    simple_combat_strategizer = Node(
        package='combat_strategizer',
        executable='simple_combat_strategizer',
        name='simple_combat_strategizer',
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(navigator_config)
    ld.add_action(simple_navigator)
    ld.add_action(simple_combat_strategizer)

    return ld
