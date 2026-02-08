#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Build launch description
    ld = LaunchDescription()
    
    # Static transform to skip perception and use ground truth from sim instead
    fake_robot_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'robot/base_footprint_sim', 'robot/base_footprint'],
        output='screen'
    )

    # Static transform to skip perception and use ground truth from sim instead
    fake_opponent_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'opponent/base_footprint_sim', 'opponent/base_footprint'],
        output='screen'
    )

    ld.add_action(fake_robot_tf)
    
    ld.add_action(fake_opponent_tf)

    return ld