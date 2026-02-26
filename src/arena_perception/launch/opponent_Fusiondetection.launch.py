# launch/opponent_detection_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # =================== PACKAGE ===================
    pkg = 'arena_perception'
    
    # Get package share directory for config files
    pkg_share = FindPackageShare(pkg)
    
    # Path to Kalman filter config file (renamed to match our new config)
    kalman_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'kalman_config.yaml'
    ])
    
    # =================== LAUNCH ARGUMENTS ===================
    # Common arguments
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
    
    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug visualization'
    )
    
    # Detector-specific arguments
    declare_min_contour_area = DeclareLaunchArgument(
        'min_contour_area',
        default_value='300',
        description='Minimum contour area for detection'
    )
    
    declare_max_contour_area = DeclareLaunchArgument(
        'max_contour_area',
        default_value='10000',
        description='Maximum contour area for detection'
    )
    
    declare_ignore_radius = DeclareLaunchArgument(
        'ignore_radius_px',
        default_value='60',
        description='Radius to ignore around robot position (pixels)'
    )
    
    declare_robot_base_frame = DeclareLaunchArgument(
        'robot_base_frame',
        default_value='robot_base',
        description='Robot base frame for self-filtering'
    )
    
    declare_camera_optical_frame = DeclareLaunchArgument(
        'camera_optical_frame',
        default_value='arena_camera_optical',
        description='Camera optical frame'
    )
    
    # Converter-specific arguments
    declare_target_frame = DeclareLaunchArgument(
        'target_frame',
        default_value='world',
        description='Target frame for 3D points'
    )
    
    declare_z_plane = DeclareLaunchArgument(
        'z_plane',
        default_value='0.0',
        description='Z-plane height for intersection'
    )
    
    declare_confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence to process detection'
    )
    
    declare_publish_visualization = DeclareLaunchArgument(
        'publish_visualization',
        default_value='true',
        description='Publish visualization topics for RViz'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # =================== KALMAN FILTER ARGUMENTS ===================
    declare_kalman_config = DeclareLaunchArgument(
        'kalman_config',
        default_value=kalman_config_path,
        description='Path to Kalman filter configuration file'
    )
    
    # =================== TF TO POSE NODE ===================
    tf_to_pose_node = Node(
        package='robofetz_gazebo',
        executable='tf_to_pose',
        name='tf_to_pose_converter',
        output='screen',
        parameters=[{
            'parent_frame': 'map',
            'child_frame': 'robot/base_footprint',
            'pose_topic': '/bot/pose',
            'publish_rate': 60.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[],
    )
    
    # =================== DETECTOR NODES ===================
    # Color detector (single opponent with color)
    color_detector_node = Node(
        package=pkg,
        executable='opponent_det_ColorSingle',
        name='opponent_det_ColorSingle',
        output='screen',
        parameters=[{
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'max_contour_area': LaunchConfiguration('max_contour_area'),
            'ignore_radius_px': LaunchConfiguration('ignore_radius_px'),
            'shadow_expansion_factor': 1.2,
            'debug': LaunchConfiguration('debug'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # Color detector specific parameters
            'interactive_selection': False,  # Use parameters, not interactive
            'hue_min': 0,
            'hue_max': 179,
            'sat_min': 50,
            'sat_max': 255,
            'val_min': 50,
            'val_max': 255,
        }],
        remappings=[
            ('/arena_camera/image_rect', LaunchConfiguration('camera_topic')),
            ('/arena_camera/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/bot/pose', '/bot/pose'),
            ('/detections_2d', '/detections_2d/color'),
            ('/debug/detection_image', '/debug/color_detection_image'),
        ]
    )
    
    # MOG2 detector (single opponent with MOG2)
    mog2_detector_node = Node(
        package=pkg,
        executable='opponent_det_MOG2single',
        name='opponent_det_MOG2single',
        output='screen',
        parameters=[{
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'max_contour_area': LaunchConfiguration('max_contour_area'),
            'background_history': 500,
            'var_threshold': 16,
            'ignore_radius_px': LaunchConfiguration('ignore_radius_px'),
            'shadow_expansion_factor': 1.2,
            'debug': LaunchConfiguration('debug'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/arena_camera/image_rect', LaunchConfiguration('camera_topic')),
            ('/arena_camera/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/bot/pose', '/bot/pose'),
            ('/detections_2d', '/detections_2d/mog2'),
            ('/debug/detection_image', '/debug/mog2_detection_image'),
        ]
    )
    
    # =================== CONVERTER NODES (UPDATED EXECUTABLE NAME) ===================
    # Color detector 3D converter
    color_converter_node = Node(
        package=pkg,
        executable='detection_2d_to_3d_node',  # Updated to match your actual executable
        name='detection_2d_to_3d_color',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'camera_frame': LaunchConfiguration('camera_optical_frame'),
            'z_plane': LaunchConfiguration('z_plane'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'tf_timeout_sec': 0.1,
            'publish_visualization': LaunchConfiguration('publish_visualization'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d/color'),
            ('detections_3d', '/detections_3d/color'),
            ('detections_3d_viz_point', '/debug/color_3d_point'),
            ('detections_3d_viz_poses', '/debug/color_3d_poses'),
        ]
    )
    
    # MOG2 detector 3D converter
    mog2_converter_node = Node(
        package=pkg,
        executable='detection_2d_to_3d_node',  # Updated to match your actual executable
        name='detection_2d_to_3d_mog2',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'camera_frame': LaunchConfiguration('camera_optical_frame'),
            'z_plane': LaunchConfiguration('z_plane'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'tf_timeout_sec': 0.1,
            'publish_visualization': LaunchConfiguration('publish_visualization'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d/mog2'),
            ('detections_3d', '/detections_3d/mog2'),
            ('detections_3d_viz_point', '/debug/mog2_3d_point'),
            ('detections_3d_viz_poses', '/debug/mog2_3d_poses'),
        ]
    )
    
    # =================== KALMAN FILTER NODE (UPDATED) ===================
    kalman_filter_node = Node(
        package=pkg,
        executable='multi_sensor_kalman_filter',  # Updated to match our new node
        name='multi_sensor_kalman_filter',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('kalman_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            # The Kalman filter now subscribes directly to individual sensor topics
            # No need for a merged topic - it handles multiple subscriptions internally
        ]
    )
    
    # =================== TOPIC MERGERS (REMOVED - NOT NEEDED) ===================
    # The Kalman filter now subscribes directly to each sensor topic,
    # so we don't need the relay nodes anymore
    
    # =================== VISUALIZATION NODE (OPTIONAL) ===================
    # Add a simple visualization node to display fused tracks in RViz
    viz_marker_node = Node(
        package='visualization_msgs',
        executable='marker_array_publisher',  # This might not exist - you may need a custom node
        name='track_visualizer',
        condition=LaunchConfiguration('debug'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    # =================== TIMING AND DELAYS ===================
    # Group nodes by dependency to ensure proper startup order
    # First group: TF and detectors
    delayed_detectors = TimerAction(
        period=1.0,
        actions=[color_detector_node, mog2_detector_node]
    )
    
    # Second group: Converters (depend on detectors and TF)
    delayed_converters = TimerAction(
        period=1.5,
        actions=[color_converter_node, mog2_converter_node]
    )
    
    # Third group: Kalman filter (depends on converters)
    delayed_kalman = TimerAction(
        period=2.0,
        actions=[kalman_filter_node]
    )
    
    # =================== RETURN ===================
    return LaunchDescription([
        # Declare all launch arguments
        declare_camera_topic,
        declare_camera_info_topic,
        declare_debug,
        declare_min_contour_area,
        declare_max_contour_area,
        declare_ignore_radius,
        declare_robot_base_frame,
        declare_camera_optical_frame,
        declare_target_frame,
        declare_z_plane,
        declare_confidence_threshold,
        declare_publish_visualization,
        declare_use_sim_time,
        
        # Kalman filter arguments
        declare_kalman_config,
        
        # Launch TF node first (no delay needed)
        tf_to_pose_node,
        LogInfo(msg="[1/4] TF to PoseStamped node launched"),
        
        # Launch detectors after TF is ready
        delayed_detectors,
        LogInfo(msg="[2/4] Color and MOG2 detectors launched"),
        
        # Launch converters after detectors are publishing
        delayed_converters,
        LogInfo(msg="[3/4] 2D to 3D converters launched"),
        
        # Launch Kalman filter after converters are ready
        delayed_kalman,
        LogInfo(msg="[4/4] Multi-sensor Kalman filter launched with config from kalman_config.yaml"),
        
        # Optional: Add a final log to indicate system is ready
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="=== OPPONENT DETECTION SYSTEM FULLY INITIALIZED ==="),
                LogInfo(msg="Published topics:"),
                LogInfo(msg="  /detections_2d/color - Raw color detections (2D)"),
                LogInfo(msg="  /detections_2d/mog2 - Raw MOG2 detections (2D)"),
                LogInfo(msg="  /detections_3d/color - Transformed color detections (3D)"),
                LogInfo(msg="  /detections_3d/mog2 - Transformed MOG2 detections (3D)"),
                LogInfo(msg="  /fused_tracks - Kalman filtered opponent tracks"),
                LogInfo(msg="  /fused_tracks_poses - Track poses for visualization"),
                LogInfo(msg="  /kalman_debug_info - Debug information"),
            ]
        ),
    ])