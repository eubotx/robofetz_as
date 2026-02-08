# navigation_fight.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    simple_navigator = Node(
        package='robot_navigation',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('robot_navigation'),
            'config',
            'simple_navigator_config.yaml'
        )]
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(simple_navigator)

    return ld