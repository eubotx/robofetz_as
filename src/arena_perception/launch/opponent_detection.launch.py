# launch/opponent_detection_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # =================== PACKAGE ===================
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
    
    # =================== NEW: TF to PoseStamped node ===================
    # This node publishes robot pose as PoseStamped for self-filtering
    tf_to_pose_node = Node(
        package='robofetz_gazebo',  # Assuming the node is in the same package
        executable='tf_to_pose',
        name='tf_to_pose_converter',
        output='screen',
        parameters=[{
            'parent_frame': 'map',
            'child_frame': 'robot/base_footprint',
            'pose_topic': '/bot/pose',  # Matches the remapping in detector_node
            'publish_rate': 60.0,  # Higher rate for better self-filtering
        }],
        remappings=[
            # No remappings needed for this node
        ],
        # This node will output robot pose on /bot/pose for the detector node
    )
    
    # =================== NODE 1: DETECTOR ===================
    # This node publishes 2D detections
    detector_node = Node(
        package=pkg,
        executable='opponent_det_ContourSingle',
        name='opponent_det_ContourSingle',
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
            ('/bot/pose', '/bot/pose'),  # Receives pose from tf_to_pose_node
            ('/detections_2d', '/detections_2d'),
            ('/debug/detection_image', '/debug/detection_image'),
        ]
    )
    
    # =================== NODE 2: CONVERTER ===================
    # This node converts 2D detections to 3D positions
    converter_node = Node(
        package=pkg,
        executable='detection_transformation_2D_3D',
        name='detection_transformation_2D_3D',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'camera_frame': LaunchConfiguration('camera_optical_frame'),
            'z_plane': LaunchConfiguration('z_plane'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_visualization': LaunchConfiguration('publish_visualization'),
        }],
        remappings=[
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d'),
            ('detections_3d', '/detected_opponent/detections_3d'),
            ('detections_3d_viz_point', '/detected_opponent/viz_point'),
            ('detections_3d_viz_poses', '/detected_opponent/viz_poses'),
        ]
    )
    
    # Optional: Add a small delay for the detector node to ensure TF is available
    # This ensures tf_to_pose_node has time to start publishing before detector needs it
    delayed_detector = TimerAction(
        period=1.0,  # 1 second delay
        actions=[detector_node]
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
        
        # Launch TF node first (it's listed first, so it starts first)
        tf_to_pose_node,
        
        # Log message to indicate TF node is running
        LogInfo(msg="TF to PoseStamped node launched with map -> robot/base_footprint"),
        
        # Launch detector and converter
        # Option A: Launch detector immediately (will work if TF is already being broadcast)
        detector_node,
        
        # Option B: Use delayed detector if you want to ensure TF is available
        # delayed_detector,
        
        converter_node,
    ])