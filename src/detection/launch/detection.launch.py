#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detection',
            executable='arena_calibration_service',
            name='arena_calibration_service'
        ),
        Node(
            package='detection',
            executable='robot_detection_node',
            name='robot_detection_node'
        ),
        # Node(
        #     package='detection',
        #     executable='enemy_detection_node',
        #     name='enemy_detection_node'
        # ),
    ])