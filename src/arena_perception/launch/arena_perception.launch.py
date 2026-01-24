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
    
    default_robot_detection_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config',
        'robot_detection_config.yaml'
    ])
    
    # Declare launch arguments for config files (optional, can use defaults)
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

    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value=default_filter_config,
        description='Path to robot detection filter config file'
    )
    
    robot_detection_config_arg = DeclareLaunchArgument(
        'robot_detection_config',
        default_value=default_robot_detection_config,
        description='Path to robot detection config file'
    )
    
    # Get config files from launch arguments
    apriltag_config_file = LaunchConfiguration('apriltag_config')
    camera_finder_config_file = LaunchConfiguration('camera_finder_config')
    calibration_file = LaunchConfiguration('calibration_file')
    filter_config_file = LaunchConfiguration('filter_config')
    robot_detection_config_file = LaunchConfiguration('robot_detection_config')
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(apriltag_config_arg)
    ld.add_action(camera_finder_config_arg)
    ld.add_action(calibration_arg)
    ld.add_action(filter_config_arg)
    ld.add_action(robot_detection_config_arg)
    
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
            # Load all parameters from config file
            camera_finder_config_file,  # This loads the YAML with all the parameters
            # Then override/add specific parameters
            #{'calibration_file': calibration_file},  # Optional: path to calibration results
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
    robot_detection = Node(
        package='arena_perception',
        executable='robot_detection_node',
        name='robot_detection_node',
        parameters=[
            robot_detection_config_file  # Load parameters from config file
        ],
        output='screen'
    )

    robot_tf_to_pose = Node(
        package='robofetz_gazebo',
        executable='tf_to_pose',
        name='tf_to_pose_robot',
        parameters=[{
            'tf_topic': '/robot/base_footprint',
            'pose_topic': '/robot/pose'
        }]
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
        actions=[robot_detection]
    ))
    
    # ld.add_action(TimerAction(
    #     period=7.0,
    #     actions=[robot_tf_to_pose]
    # ))

    return ld