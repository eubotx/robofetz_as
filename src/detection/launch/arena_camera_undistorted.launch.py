#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Include the original camera launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('detection'),
                'launch',
                'arena_camera.launch.py'
            ])
        ])
    )
    
    # # Add image_proc node
    # rectify_node = Node(
    #     package='image_proc',
    #     executable='rectify_node',  # This exists in Jazzy
    #     namespace='arena_camera',
    #     remappings=[
    #         ('image', 'image_raw')
    #     ]
    # )

       # Add image_proc node
    rectify_node = Node(
        package='detection',
        executable='rectify_fisheye',  # This exists in Jazzy
        namespace='arena_camera'
    )
    
    return LaunchDescription([
        camera_launch,
        rectify_node
    ])