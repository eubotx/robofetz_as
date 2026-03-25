# launch/opponent_detection_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg = 'arena_perception'
    
    # Get package share directory for config files
    pkg_share = FindPackageShare(pkg)
    
    # Path to opponent_detection config file (single source of truth)
    opponent_detection_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'opponent_detection_config.yaml'
    ])
    
    # =================== LAUNCH ARGUMENTS ===================
    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/arena_camera/image_rect',
        description='Camera image topic'
    )
    
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/arena_camera/camera_info',
        description='Camera info topic'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    declare_opponent_detection_config = DeclareLaunchArgument(
        'opponent_detection_config',
        default_value=opponent_detection_config_path,
        description='Path to opponent_detection configuration file'
    )
    
    # =================== DETECTOR NODES ===================
    # Color detector
    color_detector_node = Node(
        package=pkg,
        executable='opponent_det_ColorSingle',
        name='opponent_det_ColorSingle',
        output='screen',
        parameters=[LaunchConfiguration('opponent_detection_config')],
        remappings=[
            ('/arena_camera/image_rect', LaunchConfiguration('camera_topic')),
            ('/arena_camera/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/robot/pose', '/robot/pose'),
        ]
    )
    
    # MOG2 detector
    mog2_detector_node = Node(
        package=pkg,
        executable='opponent_det_MOG2single',
        name='opponent_det_MOG2single',
        output='screen',
        parameters=[LaunchConfiguration('opponent_detection_config')],
        remappings=[
            ('/arena_camera/image_rect', LaunchConfiguration('camera_topic')),
            ('/arena_camera/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/robot/pose', '/robot/pose'),
        ]
    )
    
    # =================== CONVERTER NODES ===================
    # Color detector 3D converter
    color_converter_node = Node(
        package=pkg,
        executable='detection_transformation_2D_3D_node',
        name='detection_2d_to_3d_color',
        output='screen',
        parameters=[LaunchConfiguration('opponent_detection_config')],
        remappings=[
            # Inputs
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d/color'),
            ('detections3d_array', '/detections_3d/color_array'),
            ('detections3d_point', '/detections_3d/color_point'),
        ]
    )
    
    # MOG2 converter
    mog2_converter_node = Node(
        package=pkg,
        executable='detection_transformation_2D_3D_node',
        name='detection_2d_to_3d_mog2',
        output='screen',
        parameters=[LaunchConfiguration('opponent_detection_config')],
        remappings=[
            # Inputs
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d/mog2'),
            ('detections3d_array', '/detections_3d/mog2_array'),
            ('detections3d_point', '/detections_3d/mog2_point'),
        ]
    )
    
    # =================== KALMAN FILTER NODE ===================
    kalman_filter_node = Node(
        package=pkg,
        executable='detection_kalman_filter',
        name='detection_kalman_filter',
        output='screen',
        parameters=[LaunchConfiguration('opponent_detection_config')]
        # Remappings removed; topics are directly defined in config.
    )
    
    # =================== TIMING AND DELAYS ===================
    # Start ROI masking immediately with detectors
    delayed_converters = TimerAction(
        period=1.0,
        actions=[color_converter_node, mog2_converter_node]
    )
    
    delayed_kalman = TimerAction(
        period=1.5,
        actions=[kalman_filter_node]
    )
    
    # =================== RETURN ===================
    return LaunchDescription([
        declare_camera_topic,
        declare_camera_info_topic,
        declare_use_sim_time,
        declare_opponent_detection_config,
        color_detector_node,
        mog2_detector_node,
        delayed_converters,
        delayed_kalman,
    ])