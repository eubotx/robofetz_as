#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to your package's share directory
    pkg_dir = get_package_share_directory('arena_perception')
    
    # Path to your config files
    arena_config_file = os.path.join(pkg_dir, 'config', 'arena_detection_config.yaml')
    apriltag_config_file = os.path.join(pkg_dir, 'config', 'apriltag_detection_config.yaml')
    filter_config_file = os.path.join(pkg_dir, 'config', 'robot_detection_filter_config.yaml')
    
    # Verify the files exist (helpful for debugging)
    if not os.path.exists(arena_config_file):
        raise FileNotFoundError(f"Arena config file not found: {arena_config_file}")
    
    if not os.path.exists(apriltag_config_file):
        raise FileNotFoundError(f"Apriltag config file not found: {apriltag_config_file}")
    
    if not os.path.exists(filter_config_file):
        raise FileNotFoundError(f"Filter config file not found: {filter_config_file}")
    
    return LaunchDescription([
        # Camera rectification node
        Node(
            package='arena_perception',
            executable='camera_rectification_node',
            namespace='arena_camera',
            name='camera_rectification'
        ),
        
        # Arena calibration service
        Node(
            package='arena_perception',
            executable='arena_calibration_service',
            name='arena_calibration_service',
            parameters=[
                {'config_file': arena_config_file},
            ]
        ),
        
        # Apriltag detection node, swap with premade ros one for efficiency
        Node(
            package='arena_perception',
            executable='apriltag_detection_node',
            name='apriltag_detection_node',
            parameters=[
                {'config_file': apriltag_config_file}
            ]
        ),
        
        # Filter transform node for TOP tag
        Node(
            package='arena_perception',
            executable='filter_transform_node',
            name='top_tag_filter_node',
            parameters=[
                {'config_file': filter_config_file},
                {'input_frame': 'robot/top_apriltag_link'},
                {'output_frame': 'robot/top_apriltag_link_filtered'}
            ]
        ),
        
        # Filter transform node for BOTTOM tag
        Node(
            package='arena_perception',
            executable='filter_transform_node',
            name='bottom_tag_filter_node',
            parameters=[
                {'config_file': filter_config_file},
                {'input_frame': 'robot/bottom_apriltag_link'},
                {'output_frame': 'robot/bottom_apriltag_link_filtered'}
            ]
        ),

        # Filter transform node for BOTTOM tag
        # Works but causes lags
        Node(
            package='arena_perception',
            executable='robot_detection_node1',
            name='robot_detection_node1',
            parameters=[
            ]
        ),

        # # Filter transform node for BOTTOM tag
        # # Not working but at least doesnt crash
        # Node(
        #     package='arena_perception',
        #     executable='robot_detection_node2',
        #     name='robot_detection_node2',
        #     parameters=[
        #     ]
        # ),

        Node(
            package='arena_perception',
            executable='odom_drift_correction_node1',
            name='odom_drift_correction_node1',
            parameters=[
            ]
        ),

        # Filter transform node for BOTTOM tag
        # Node(
        #     package='arena_perception',
        #     executable='static_tag_position_publisher',
        #     name='static_tag_position_publisher',
        #     parameters=[
        #     ]
        # ),
    ])