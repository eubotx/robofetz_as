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
    
    # Define config paths using PathJoinSubstitution (Jazzy best practice)
    default_apriltag_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'apriltag_detection_config.yaml'
    ])
    
    default_arena_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config', 
        'arena_detection_config.yaml'
    ])
    
    default_filter_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'robot_detection_filter_config.yaml'
    ])
    
    default_calibration = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'world_to_camera_calibration.temp.yaml'
    ])
    
    # Declare launch arguments for config files (optional, can use defaults)
    apriltag_config_arg = DeclareLaunchArgument(
        'apriltag_config',
        default_value=default_apriltag_config,
        description='Path to apriltag detection config file'
    )
    
    arena_config_arg = DeclareLaunchArgument(
        'arena_config',
        default_value=default_arena_config,
        description='Path to arena detection config file'
    )
    
    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value=default_filter_config,
        description='Path to robot detection filter config file'
    )
    
    calibration_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=default_calibration,
        description='Path to world to camera calibration file'
    )
    
    # Get config files from launch arguments
    apriltag_config_file = LaunchConfiguration('apriltag_config')
    camera_finder_config_file = LaunchConfiguration('arena_config')
    filter_config_file = LaunchConfiguration('filter_config')
    calibration_file = LaunchConfiguration('calibration_file')
    
    # Verify the files exist (helpful for debugging) - keep your original checks
    # Note: These checks would need to be done differently with LaunchConfiguration
    # We'll keep the structure but note they're not directly usable with LaunchConfiguration
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(apriltag_config_arg)
    ld.add_action(arena_config_arg)
    ld.add_action(filter_config_arg)
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
            {'config_file': apriltag_config_file}
        ],
        output='screen'
    )
    
    # Arena calibration service
    find_camera_service = Node(
        package='arena_perception',
        executable='find_camera_in_world_service',
        name='find_camera_in_world_service',
        parameters=[
            {'config_file': camera_finder_config_file},
            {'calibration_file': calibration_file},  # Optional, if not given it attempts initial calibration
            {'calibration_attempt_rate': 1.0},       # Optional: default is 1.0
            {'dynamic_publish_rate': 60.0},          # Optional: default is 30.0
        ],
        output='screen'
    )
    
    # Filter transform nodes
    top_tag_filter = Node(
        package='arena_perception',
        executable='filter_transform_node',
        name='top_tag_filter_node',
        parameters=[
            {'config_file': filter_config_file},
            {'input_frame': 'robot/top_apriltag_link'},
            {'output_frame': 'robot/top_apriltag_link_filtered'}
        ],
        output='screen'
    )
    
    bottom_tag_filter = Node(
        package='arena_perception',
        executable='filter_transform_node',
        name='bottom_tag_filter_node',
        parameters=[
            {'config_file': filter_config_file},
            {'input_frame': 'robot/bottom_apriltag_link'},
            {'output_frame': 'robot/bottom_apriltag_link_filtered'}
        ],
        output='screen'
    )
    
    # Robot detection node, publishes base_footprint, visible tag and robot pose
    robot_detection_1 = Node(
        package='arena_perception',
        executable='robot_detection_node',
        name='robot_detection_node',
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

    # ld.add_action(TimerAction(
    #     period=4.0,
    #     actions=[top_tag_filter, bottom_tag_filter]
    # ))

    ld.add_action(TimerAction(
        period=5.0,
        actions=[robot_detection_1]
    ))
    
    return ld