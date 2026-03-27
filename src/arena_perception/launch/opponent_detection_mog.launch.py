from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg = 'arena_perception'
    
    # Package share
    pkg_share = FindPackageShare(pkg)
    
    opponent_detection_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'opponent_detection_config.yaml'
    ])
    
    # =================== LAUNCH ARGUMENTS ===================
    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/arena_camera/image_rect'
    )
    
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/arena_camera/camera_info'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )
    
    declare_opponent_detection_config = DeclareLaunchArgument(
        'opponent_detection_config',
        default_value=opponent_detection_config_path
    )

    # 🔹 Remap node parameters
    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/detections_3d/mog2_point',  # 👈 switched to MOG2 by default
    )

    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/opponent/pose',
    )
    
    # =================== DETECTORS ===================
    
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
    
    # =================== CONVERTERS ===================
    
    mog2_converter_node = Node(
        package=pkg,
        executable='detection_transformation_2D_3D_node',
        name='detection_2d_to_3d_mog2',
        output='screen',
        parameters=[LaunchConfiguration('opponent_detection_config')],
        remappings=[
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('detections_2d', '/detections_2d/mog2'),
            ('detections3d_array', '/detections_3d/mog2_array'),
            ('detections3d_point', '/detections_3d/mog2_point'),
        ]
    )
    
    # =================== STRATEGIZER ===================
    point_to_pose_remap_node = Node(
        package='combat_strategizer',
        executable='point_to_pose_remap',
        name='point_to_pose_remap',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
        }]
    )
    
    # =================== DELAYS ===================
    delayed_converters = TimerAction(
        period=1.0,
        actions=[mog2_converter_node]
    )

    delayed_strategizer = TimerAction(
        period=1.5,
        actions=[point_to_pose_remap_node]
    )
    
    # =================== RETURN ===================
    return LaunchDescription([
        declare_camera_topic,
        declare_camera_info_topic,
        declare_use_sim_time,
        declare_opponent_detection_config,
        declare_input_topic,
        declare_output_topic,
        mog2_detector_node,
        delayed_converters,
        delayed_strategizer,
    ])