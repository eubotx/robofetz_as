#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Config file override argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=EnvironmentVariable('ARENA_CAMERA_CONFIG', default_value='arena_camera_params.yaml'),
        description='Camera config file name (e.g., arena_camera_params.yaml, laptop_camera_params.yaml)'
    )
    
    # Build config path using PathJoinSubstitution (CORRECT WAY)
    config_path = PathJoinSubstitution([
        FindPackageShare('arena_perception'),
        'config',
        LaunchConfiguration('config_file')
    ])
    
    return LaunchDescription([
        config_file_arg,
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='arena_camera',
            parameters=[config_path]
        )
    ])