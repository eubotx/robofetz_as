#!/usr/bin/env python3
"""
Main launch file for robot software operation.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package names
    perception_pkg = 'arena_perception'
    localization_pkg = 'arena_perception'
    robot_description_pkg = 'robot_description'
    
    # ============================================
    # ROBOT STATE PUBLISHER CONFIGURATION
    # ============================================
    
    # Define launch arguments for robot state publisher
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='robot/',  # Default to 'robot/' prefix
        description='Robot namespace prefix (e.g., "robot1/")'
    )
    
    # ============================================
    # ARENA PERCEPTION CONFIGURATION
    # ============================================
        
    default_arena_perception_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config', 
        'arena_perception_config.yaml'
    ])
        
    arena_perception_config_arg = DeclareLaunchArgument(
        'arena_perception_config',
        default_value=default_arena_perception_config,
        description='Path to arena_perception_config file'
    )

    # ============================================
    # ROBOT LOCALIZATION CONFIGURATION
    # ============================================

    default_robot_localization_config = PathJoinSubstitution([
        FindPackageShare(localization_pkg),
        'robot_localization_config',
        'robot_localization_config.yaml'
    ])
    
    robot_localization_config_arg = DeclareLaunchArgument(
        'robot_localization_config',
        default_value=default_robot_localization_config,
        description='Path to robot localization config file'
    )
    
    # Get config files from launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    arena_perception_config_file = LaunchConfiguration('arena_perception_config')
    robot_localization_config_file = LaunchConfiguration('robot_localization_config')
    
    # ============================================
    # BUILD LAUNCH DESCRIPTION
    # ============================================
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(prefix_arg)
    ld.add_action(arena_perception_config_arg)
    ld.add_action(robot_localization_config_arg)
    
    # ============================================
    # 1. ROBOT STATE PUBLISHER
    # ============================================
    
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(robot_description_pkg),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'prefix': prefix
        }.items()
    )
    
    # Start robot state publisher immediately
    ld.add_action(robot_state_publisher_launch)
    
    # ============================================
    # 2. ARENA PERCEPTION SYSTEM
    # ============================================
    
    # Create arena perception launch with parameters
    arena_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(perception_pkg),
                'launch',
                'arena_perception.launch.py'
            ])
        ]),
        launch_arguments={
            'arena_perception_config': arena_perception_config_file,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Add arena perception
    ld.add_action(TimerAction(
        period=3.0,  # Give robot_state_publisher time to establish TF tree
        actions=[arena_perception_launch]
    ))
    
    # ============================================
    # 3. ROBOT LOCALIZATION
    # ============================================
    
    # Include robot localization launch with the config file
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(perception_pkg),
                'launch',
                'robot_localization.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_localization_config': robot_localization_config_file,
            'use_sim_time': use_sim_time,
            'prefix': prefix
        }.items()
    )
    
    # Add robot localization
    ld.add_action(TimerAction(
        period=15.0,
        actions=[robot_localization_launch]
    ))
    
    return ld