from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    navigator_config = DeclareLaunchArgument(
        'elf_navigator_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_navigation'), 'config', 'elf_navigator_config.yaml',
        ]),
        description='Path to the Elf navigator parameter file',
    )
    strategizer_config = DeclareLaunchArgument(
        'elf_strategizer_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('combat_strategizer'), 'config', 'elf_strategizer_config.yaml',
        ]),
        description='Path to the Elf strategizer parameter file',
    )

    elf_navigator = Node(
        package='robot_navigation',
        executable='elf_navigator',
        name='elf_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('elf_navigator_config'),
            {'use_sim_time': use_sim_time},
        ],
    )

    elf_combat_strategizer = Node(
        package='combat_strategizer',
        executable='elf_combat_strategizer',
        name='elf_combat_strategizer',
        output='screen',
        parameters=[
            LaunchConfiguration('elf_strategizer_config'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true',
    ))
    ld.add_action(navigator_config)
    ld.add_action(strategizer_config)
    ld.add_action(elf_navigator)
    ld.add_action(elf_combat_strategizer)

    return ld
