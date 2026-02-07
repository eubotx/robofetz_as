#!/usr/bin/env python3
"""
Launch file for arena perception system - handles camera rectification, 
apriltag detection, robot detection, and odometry correction.
"""

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package name
    perception_pkg = 'arena_perception'
    
    default_arena_perception_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config', 
        'arena_perception_config.yaml'
    ])

    default_calibration = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'world_to_camera_calibration.temp.yaml'
    ])
    
    arena_perception_config_arg = DeclareLaunchArgument(
        'arena_perception_config',
        default_value=default_arena_perception_config,
        description='Path to arena_perception_config file'
    )
    
    calibration_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=default_calibration,
        description='Path to world to camera calibration file'
    )
    
    # Get config files from launch arguments
    arena_perception_config_file = LaunchConfiguration('arena_perception_config')
    calibration_file = LaunchConfiguration('calibration_file')
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(arena_perception_config_arg)
    ld.add_action(calibration_arg)
    
    # Camera rectification node
    camera_rectification = Node(
        package='arena_perception',
        executable='camera_rectification_node',
        namespace='arena_camera',
        name='camera_rectification',
        output='screen'
    )
    
    ld.add_action(camera_rectification)
    
    # Apriltag detection node 
    # swap with premade ros one for efficiency
    apriltag_detection = Node(
        package='arena_perception',
        executable='apriltag_detection_node',
        name='apriltag_detection_node',
        parameters=[
            arena_perception_config_file
        ],
        output='screen'
    )
    
    # Arena calibration service
    find_camera_service = Node(
        package='arena_perception',
        executable='find_camera_in_world_service',
        name='find_camera_in_world_service',
        parameters=[
            # Load all parameters from config file
            arena_perception_config_file,  # This loads the YAML with all the parameters
            # Then override/add specific parameters
            #{'calibration_file': calibration_file},  # Optional: path to calibration results
        ],
        output='screen'
    )
    
    # Robot detection node, publishes base_footprint, visible tag and robot pose
    robot_detection = Node(
        package='arena_perception',
        executable='robot_detection_node',
        name='robot_detection_node',
        parameters=[
            arena_perception_config_file  # Load parameters from config file
        ],
        output='screen'
    )

    ld.add_action(TimerAction(
        period=3.0,
        actions=[find_camera_service]
    ))

    ld.add_action(TimerAction(
        period=1.0,
        actions=[apriltag_detection]
    ))

    ld.add_action(TimerAction(
        period=5.0,
        actions=[robot_detection]
    ))

    return ld