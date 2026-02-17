# launch/opponent_detection_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =================== PACKAGE PATHS ===================
    pkg = 'arena_perception'
    
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
        default_value='100',
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
    
    declare_publish_as_pose_array = DeclareLaunchArgument(
        'publish_as_pose_array',
        default_value='false',
        description='Publish as PoseArray instead of individual points'
    )
    
    declare_confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence to process detection'
    )
    
    # Simulation time argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # =================== NODES ===================
    
    # Node 1: Frame Difference Detector (2D Detections)
    detector_node = Node(
        package=pkg,
        executable='frame_diff_detector',
        name='opponent_detector_2d',
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
            ('/detections_2d', '/detections_2d'),
            ('/debug/detection_image', '/debug/opponent_detector_2d'),
        ]
    )
    
    # Node 2: 2D to 3D Converter
    converter_node = Node(
        package=pkg,
        executable='detection_2d_to_3d_node',
        name='detection_to_3d_converter',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'camera_frame': LaunchConfiguration('camera_optical_frame'),
            'z_plane': LaunchConfiguration('z_plane'),
            'publish_as_pose_array': LaunchConfiguration('publish_as_pose_array'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d'),
            ('detections_3d_points', '/detected_opponent/point'),
            ('detections_3d_poses', '/detected_opponent/poses'),
        ]
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
        declare_publish_as_pose_array,
        declare_confidence_threshold,
        declare_use_sim_time,
        
        # Launch both nodes
        detector_node,
        converter_node,
    ])