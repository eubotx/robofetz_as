#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detection',
            executable='arena_detection',  # Corrected executable name
            name='arena_calibration_service'
        ),
        Node(
            package='detection',
            executable='robot_detection',  # Corrected executable name
            name='robot_detection_node',
            parameters=[{'debug_image': True}]  # Debug images enabled
        ),
        # Node(
        #     package='detection',
        #     executable='enemy_detection',
        #     name='enemy_detection_node',
        #     parameters=[{'debug_image': True}]  # Add if enemy detection has debug too
        # ),
    ])