#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to your package's share directory
    pkg_dir = get_package_share_directory('detection')
    
    # Path to your config file
    config_file = os.path.join(pkg_dir, 'config', 'arena.yaml')
    
    # Verify the file exists (helpful for debugging)
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    return LaunchDescription([
        Node(
            package='detection',
            executable='camera_rectification_node',
            namespace='arena_camera'
        ),
        Node(
            package='detection',
            executable='arena_calibration_service',
            name='arena_calibration_service',
            parameters=[{'config_file': config_file}]
        ),
    ])