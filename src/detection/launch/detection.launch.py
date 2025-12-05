#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detection',
            executable='camera_rectification_node',  # This exists in Jazzy
            namespace='arena_camera'
        ),
        Node(
            package='detection',
            executable='arena_calibration_service',
            name='arena_calibration_service'
        ),
        # Node(
        #    package='detection',
        #    executable='robot_detection_node',  # Corrected executable name
        #    name='robot_detection_node',
        #    parameters=[{'debug_image': True}]  # Debug images enabled
        # ),
        # Node(
        #     package='detection',
        #     executable='opponent_detection',
        #     name='opponent_detection_node',
        #     parameters=[{'debug_image': True}]  # Add if opponent detection has debug too
        # ),
    ])