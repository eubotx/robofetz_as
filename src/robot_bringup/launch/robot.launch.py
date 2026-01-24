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
    
    # Define config paths using PathJoinSubstitution
    default_apriltag_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'apriltag_detection_config.yaml'
    ])
    
    default_camera_finder_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config', 
        'find_camera_in_world_config_sim.yaml'
    ])

    default_calibration = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'world_to_camera_calibration.temp.yaml'
    ])
    
    default_filter_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'robot_detection_filter_config.yaml'
    ])

    default_odom_drift_correction_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'odom_drift_correction_config.yaml'
    ])
    
    # Declare launch arguments for arena perception config files
    apriltag_config_arg = DeclareLaunchArgument(
        'apriltag_config',
        default_value=default_apriltag_config,
        description='Path to apriltag detection config file'
    )
    
    camera_finder_config_arg = DeclareLaunchArgument(
        'camera_finder_config',
        default_value=default_camera_finder_config,
        description='Path to camera finder config file'
    )

    calibration_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=default_calibration,
        description='Path to world to camera calibration file'
    )
    
    odom_drift_correction_config_arg = DeclareLaunchArgument(
        'odom_drift_correction_config',
        default_value=default_odom_drift_correction_config,
        description='Path to odom drift correction config file'
    )

    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value=default_filter_config,
        description='Path to robot detection filter config file'
    )
    
    # Get config files from launch arguments
    apriltag_config_file = LaunchConfiguration('apriltag_config')
    camera_finder_config_file = LaunchConfiguration('camera_finder_config')
    calibration_file = LaunchConfiguration('calibration_file')
    filter_config_file = LaunchConfiguration('filter_config')
    odom_drift_correction_config_file = LaunchConfiguration('odom_drift_correction_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    
    # ============================================
    # BUILD LAUNCH DESCRIPTION
    # ============================================
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(prefix_arg)
    ld.add_action(apriltag_config_arg)
    ld.add_action(camera_finder_config_arg)
    ld.add_action(calibration_arg)
    ld.add_action(filter_config_arg)
    ld.add_action(odom_drift_correction_config_arg)
    
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
            'apriltag_config': apriltag_config_file,
            'arena_config': camera_finder_config_file,
            'filter_config': filter_config_file,
            'calibration_file': calibration_file,
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
            'odom_drift_correction_config': odom_drift_correction_config_file,
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