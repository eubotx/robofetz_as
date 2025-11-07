#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare video_device as a launch argument with environment variable fallback
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value=EnvironmentVariable('ARENA_CAMERA_DEVICE', default_value='/dev/video0'),
        description='Camera device path (e.g., /dev/video0, /dev/video2)'
    )
    
    config = os.path.join(
        get_package_share_directory('detection'),
        'config',
        'arena_camera_params.yaml'
    )
    
    camera_info = os.path.join(
        get_package_share_directory('detection'),
        'config',
        'arena_camera.info.yaml'
    )
    
    return LaunchDescription([
        video_device_arg,
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='arena_camera',
            parameters=[config, {
                'video_device': LaunchConfiguration('video_device'),
                'camera_info_url': f"file://{camera_info}"
            }]
        )
    ])