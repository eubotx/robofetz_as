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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
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
    
    # Get config files from launch arguments
    arena_perception_config_file = LaunchConfiguration('arena_perception_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(arena_perception_config_arg)
    
    # Camera rectification node
    camera_rectification = Node(
        package='arena_perception',
        executable='camera_rectification_node',
        namespace='arena_camera',
        name='camera_rectification',
        parameters=[{'use_sim_time': use_sim_time}],
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
            arena_perception_config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Arena calibration service
    find_camera_service = Node(
        package='arena_perception',
        executable='find_camera_in_world_service',
        name='find_camera_in_world_service',
        parameters=[
            arena_perception_config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Robot detection node, publishes base_footprint, visible tag and robot pose
    robot_detection = Node(
        package='arena_perception',
        executable='robot_detection_node',
        name='robot_detection_node',
        parameters=[
            arena_perception_config_file,
            {'use_sim_time': use_sim_time}
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