from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    elf_navigator = Node(
        package='robot_navigation',
        executable='elf_navigator',
        name='elf_navigator',
        output='screen',
    )

    elf_combat_strategizer = Node(
        package='combat_strategizer',
        executable='elf_combat_strategizer',
        name='elf_combat_strategizer',
        output='screen',
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(elf_navigator)
    ld.add_action(elf_combat_strategizer)

    return ld