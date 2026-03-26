import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    simple_navigator = Node(
        package='robot_navigation',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
    )

    combat_strategizer = Node(
        package='combat_strategizer',
        executable='simple_attack',
        name='simple_attack',
    )

    opponent_tf_to_pose = Node(
        package='combat_strategizer',
        executable='tf_to_pose',
        name='opponent_tf_to_pose',
        parameters=[{
            'source_frame': 'opponent',
            'reference_frame': 'world',
            'pose_topic': '/opponent/pose',
        }],
        output='screen'
    )



    # Build launch description
    ld = LaunchDescription()
    ld.add_action(simple_navigator)
    ld.add_action(combat_strategizer)
    ld.add_action(opponent_tf_to_pose)
    return ld