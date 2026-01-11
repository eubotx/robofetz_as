#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    return LaunchDescription([

        # Static transform world -> map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),

        # Publishes map -> robot/odom tf as amcl like drift correction based on arena perception
        Node(
            package='arena_perception',
            executable='odom_drift_correction_node1',
            name='odom_drift_correction_node1',
            parameters=[
            ]
        ),
        
        # Static transform robot/odom -> /robot_footprint until we use odom from EKF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'robot/odom', 'robot/base_footprint'],
            output='screen'
        ),


        # fake localitation startet odom drift correction mit /robot/base_footprint_sim tf und nicht arena_perception/robot/base_footprint tf 
        # also gives an according logging in the terminal

        
    ])