#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare video_device as a launch argument with environment variable fallback
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value=EnvironmentVariable('ARENA_CAMERA_DEVICE', default_value='/dev/video0'),
        description='Camera device path (e.g., /dev/video0, /dev/video2)'
    )
    
    # Config file override argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=EnvironmentVariable('ARENA_CAMERA_CONFIG', default_value='arena_camera_params.yaml'),
        description='Camera config file name (e.g., arena_camera_params.yaml, laptop_camera_params.yaml)'
    )
    
    # Build config path using PathJoinSubstitution (CORRECT WAY)
    config_path = PathJoinSubstitution([
        FindPackageShare('detection'),
        'config',
        LaunchConfiguration('config_file')
    ])
    
    return LaunchDescription([
        video_device_arg,
        config_file_arg,
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='arena_camera',
            parameters=[config_path, {
                'video_device': LaunchConfiguration('video_device')
            }]
        )
    ])