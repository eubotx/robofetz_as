#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package name
    perception_pkg = 'arena_perception'
    
    default_robot_localization_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'robot_localization_config',
        'robot_localization_config.yaml'
    ])
    
    robot_localization_config_arg = DeclareLaunchArgument(
        'robot_localization_config',
        default_value=default_robot_localization_config,
        description='Path to robot localization config file'
    )
    
    robot_localization_config_file = LaunchConfiguration('robot_localization_config')
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(robot_localization_config_arg)
    
    # Static transform world -> map (publishes world to map transform)
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )
    
    # Odometry drift correction node (publishes map -> odom tf based on arena perception)
    odom_drift_correction = Node(
        package='arena_perception',
        executable='odom_drift_correction_node',
        name='odom_drift_correction_node',
        parameters=[
            robot_localization_config_file
        ],
        output='screen'
    )
    
    # Static transform odom -> /robot_footprint (temporary until EKF odometry is used)
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'robot/base_footprint'],
        output='screen'
    )

    
    # Add nodes with appropriate timing delays if needed
    ld.add_action(static_map_tf)

    ld.add_action(TimerAction(
        period=1.0,
        actions=[static_odom_tf]
    ))
    
    ld.add_action(TimerAction(
        period=2.0,
        actions=[odom_drift_correction]
    ))
    
    return ld