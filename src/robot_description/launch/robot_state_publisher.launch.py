#!/usr/bin/env python3
"""
Working robot_state_publisher launch file - processes xacro in Python.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robot_description')
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    prefix = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Robot namespace prefix (e.g., "robot1/")'
    )
    
    # Process URDF with xacro (same as your working example!)
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    
    # Get the prefix value
    prefix_val = LaunchConfiguration('prefix')
    
    # Process xacro with mappings - EXACTLY like your working example
    robot_description = xacro.process_file(
        urdf_path, 
        mappings={'prefix': ''}  # We'll handle prefix via frame_prefix parameter
    ).toxml()
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0,
            'frame_prefix': LaunchConfiguration('prefix')  # Add prefix here
        }]
    )
    
    return LaunchDescription([
        use_sim_time,
        prefix,
        robot_state_publisher_node
    ])