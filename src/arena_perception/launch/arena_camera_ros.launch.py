#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare config file argument
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='arena_camera_ros_config.yaml',
        description='Config file name of .yaml)'
    )
    
    # Build full path to config file
    config_path = PathJoinSubstitution([
        get_package_share_directory('arena_perception'),
        'config',
        LaunchConfiguration('config_file')
    ])
    
    return LaunchDescription([
        config_arg,
        Node(
            package='camera_ros',
            executable='camera_node',
            namespace='arena_camera',
            remappings=[
                # Remap all camera topics to remove the "camera" prefix
                ('camera/image_raw', 'image'),
                ('camera/camera_info', 'camera_info'),
                ('camera/image_raw/compressed', 'image_raw/compressed'),

            ],
            parameters=[config_path]
        )
    ])