#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Use the EXACT camera name from the log (with backslashes)
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='\\_SB_.PCI0.XHC_.RHUB.HS03-3:1.0-32e4:0144',  # Add backslash
        description='Camera ID or name'
    )
    
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='YUYV',
        description='Pixel format'
    )

    return LaunchDescription([
        camera_arg,
        format_arg,
        Node(
            package='camera_ros',
            executable='camera_node',
            namespace='arena_camera',
            parameters=[{
                "camera": LaunchConfiguration('camera'),
                "format": LaunchConfiguration('format'),
                "width": 640,
                "height": 480,
                "role": "video",
                "frame_id": "arena_camera_frame",
                "camera_info_url": "package://detection/config/t490s_camera.yaml"
            }]
        )
    ])