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
    
    # Path to Kalman filter config file
    kalman_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'opponent_det_pipeline_config.yaml'
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
    )
    
    # =================== DETECTOR NODES ===================
    # Color detector
    color_detector_node = Node(
        package=pkg,
        executable='opponent_det_ColorSingle',
        name='opponent_det_ColorSingle',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'min_contour_area': 300,
            'max_contour_area': 10000,
            'ignore_radius_px': 60,
            'shadow_expansion_factor': 1.2,
            'debug': False,
            'robot_base_frame': 'robot_base',
            'camera_optical_frame': 'arena_camera_optical',
            'interactive_selection': True,
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
        ]
    )
    
    # MOG2 detector
    mog2_detector_node = Node(
        package=pkg,
        executable='opponent_det_MOG2single',
        name='opponent_det_MOG2single',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'min_contour_area': 300,
            'max_contour_area': 10000,
            'background_history': 500,
            'var_threshold': 16,
            'ignore_radius_px': 60,
            'shadow_expansion_factor': 1.2,
            'debug': False,
            'robot_base_frame': 'robot_base',
            'camera_optical_frame': 'arena_camera_optical',
        }],
        remappings=[
            ('/arena_camera/image_rect', LaunchConfiguration('camera_topic')),
            ('/arena_camera/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/bot/pose', '/bot/pose'),
            ('/detections_2d', '/detections_2d/mog2'),
        ]
    )
    
    # =================== CONVERTER NODES ===================
    # Color detector 3D converter - ONLY array and point topics
    color_converter_node = Node(
        package=pkg,
        executable='detection_transformation_2D_3D_node',
        name='detection_2d_to_3d_color',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': 'world',
            'camera_frame': 'arena_camera_optical',
            'z_plane': 0.0,
            'confidence_threshold': 0.5,
            'tf_timeout_sec': 0.1,
            'publish_visualization': True,  # Still needed for point topics
            'debug_mode': False,
        }],
        remappings=[
            # Inputs
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d/color'),

            # Outputs
            ('detections3d_array', '/detections_3d/color_array'),   # Detection3DArray for Kalman filter
            ('detections3d_point', '/detections_3d/color_point'),   # PointStamped for debugging
        ]
    )
    
    # MOG2 detector 3D converter - ONLY array and point topics
    mog2_converter_node = Node(
        package=pkg,
        executable='detection_transformation_2D_3D_node',
        name='detection_2d_to_3d_mog2',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': 'world',
            'camera_frame': 'arena_camera_optical',
            'z_plane': 0.0,
            'confidence_threshold': 0.5,
            'tf_timeout_sec': 0.1,
            'publish_visualization': True,  # Still needed for point topics
            'debug_mode': False,
        }],
        remappings=[
            # Inputs
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d/mog2'),

            # Outputs
            ('detections3d_array', '/detections_3d/mog2_array'),   # Detection3DArray for Kalman filter
            ('detections3d_point', '/detections_3d/mog2_point'),   # PointStamped for debugging
        ]
    )
    
    # =================== KALMAN FILTER NODE ===================
    kalman_filter_node = Node(
        package=pkg,
        executable='detection_kalman_filter',
        name='detection_kalman_filter',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('kalman_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('detections_3d/color', '/detections_3d/color_array'),
            ('detections_3d/mog2', '/detections_3d/mog2_array'),
        ]
    )
    
    # =================== TIMING AND DELAYS ===================
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
        declare_kalman_config,
        
        tf_to_pose_node,
        color_detector_node,
        mog2_detector_node,
        delayed_converters,
        delayed_kalman,
    ])