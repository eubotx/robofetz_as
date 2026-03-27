#!/usr/bin/env python3
"""
Launch file for arena perception system - handles camera rectification, 
apriltag detection, robot detection, and odometry correction.
"""

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package name
    perception_pkg = 'arena_perception'
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    run_rectification_arg = DeclareLaunchArgument(
        'run_rectification',
        default_value='true',
        description='Whether to run the camera rectification node. Set to false for pinhole cameras (like in Gazebo simulation)'
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
    
    # Arguments for opponent detection
    opponent_camera_topic_arg = DeclareLaunchArgument(
        'opponent_camera_topic',
        default_value='/arena_camera/image_rect_masked',
        description='Camera image topic for opponent detection (use masked image by default)'
    )
    
    opponent_camera_info_topic_arg = DeclareLaunchArgument(
        'opponent_camera_info_topic',
        default_value='/arena_camera/camera_info',
        description='Camera info topic for opponent detection'
    )
    
    # Default opponent detection config path
    default_opponent_detection_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config', 
        'opponent_detection_config.yaml'
    ])
    
    opponent_detection_config_arg = DeclareLaunchArgument(
        'opponent_detection_config',
        default_value=default_opponent_detection_config,
        description='Path to opponent_detection configuration file'
    )
    
    enable_opponent_detection_arg = DeclareLaunchArgument(
        'enable_opponent_detection',
        default_value='true',
        description='Enable opponent detection nodes'
    )
    
    # Get config files from launch arguments
    arena_perception_config_file = LaunchConfiguration('arena_perception_config')
    opponent_detection_config_file = LaunchConfiguration('opponent_detection_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rectification = LaunchConfiguration('run_rectification')
    opponent_camera_topic = LaunchConfiguration('opponent_camera_topic')
    opponent_camera_info_topic = LaunchConfiguration('opponent_camera_info_topic')
    enable_opponent_detection = LaunchConfiguration('enable_opponent_detection')
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(run_rectification_arg)
    ld.add_action(arena_perception_config_arg)
    ld.add_action(opponent_camera_topic_arg)
    ld.add_action(opponent_camera_info_topic_arg)
    ld.add_action(opponent_detection_config_arg)
    ld.add_action(enable_opponent_detection_arg)
    
    # Camera rectification node - only runs if run_rectification is true
    camera_rectification = Node(
        package='image_proc',
        executable='rectify_node',
        namespace='arena_camera',
        name='rectify_node',
        condition=IfCondition(run_rectification),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[
            ('image', 'image'),
            ('image_rect', 'image_rect')
        ],
    )
    
    ld.add_action(camera_rectification)

    # IMAGE REMAPPING NODE (for pinhole camera case) - only runs when run_rectifaction is false
    # republishes image to image_rect so downstream nodes work and we safe computing power
    image_remap_node = Node(
        package='topic_tools',
        executable='relay',
        name='image_to_rect_relay',
        namespace='arena_camera',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=['image', 'image_rect'],
        condition=UnlessCondition(run_rectification),
        output='screen'
    )
    
    ld.add_action(image_remap_node)
    
    # Self written Apriltag detection node 
    # swap with premade ros one for efficiency
    apriltag_detection_diy = Node(
        package='arena_perception',
        executable='apriltag_detection_node',
        name='apriltag_detection_node',
        parameters=[
            arena_perception_config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Apriltag detection node (apriltag_ros version)
    apriltag_detection_ros = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[
            arena_perception_config_file,
            {
                'use_sim_time': use_sim_time,
                'approx_sync': True,  # Enable approximate sync
                'queue_size': 20,      # Larger queue for matching
                'slop': 0.5            # 0.5 second tolerance
            }
        ],
        remappings=[
            ('image_rect', '/arena_camera/image_rect'),
            ('camera_info', '/arena_camera/camera_info')
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

    robot_tf_to_pose = Node(
        package='combat_strategizer',
        executable='tf_to_pose',
        name='robot_tf_to_pose',
        parameters=[{
            'source_frame': 'robot/base_footprint',
            'reference_frame': 'world',
            'pose_topic': '/robot/pose',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    roi_mask_node = Node(
        package='arena_perception',
        executable='roi_mask_node',
        name='roi_mask_node',
        parameters=[
            arena_perception_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('image', '/arena_camera/image_rect'),
            ('camera_info', '/arena_camera/camera_info'),
            ('masked_image', '/arena_camera/image_rect_masked')
        ],
        output='screen'
    )

    # Get the path to opponent detection launch file
    opponent_detection_launch = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'launch',
        'opponent_Fusiondetection.launch.py'
    ])
    
    # Include opponent detection launch file
    opponent_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(opponent_detection_launch),
        condition=IfCondition(enable_opponent_detection),
        launch_arguments={
            'camera_topic': opponent_camera_topic,
            'camera_info_topic': opponent_camera_info_topic,
            'use_sim_time': use_sim_time,
            'opponent_detection_config': opponent_detection_config_file
        }.items()
    )
    
    # Add timer actions for main nodes
    ld.add_action(TimerAction(
        period=1.0,  # Slightly offset to avoid congestion
        actions=[apriltag_detection_diy]
    ))

    # ld.add_action(TimerAction(
    #     period=1.0,  # Slightly offset to avoid congestion
    #     actions=[apriltag_detection_ros]
    # ))

    ld.add_action(TimerAction(
        period=3.0,
        actions=[find_camera_service]
    ))

    ld.add_action(TimerAction(
        period=5.0,
        actions=[robot_detection]
    ))

    ld.add_action(TimerAction(
        period=6.0,
        actions=[roi_mask_node]
    ))

    ld.add_action(TimerAction(
        period=7.0,
        actions=[robot_tf_to_pose]
    ))
    
    # Add opponent detection with appropriate timing (after ROI mask is ready)
    # ld.add_action(TimerAction(
    #     period=9.0,  # Start after ROI mask node (6.0s) to ensure masked image is available
    #     actions=[opponent_detection]
    # ))

    return ld