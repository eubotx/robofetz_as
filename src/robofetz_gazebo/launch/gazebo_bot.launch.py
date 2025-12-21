import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # Package and file paths
    pkg_name = 'robofetz_gazebo'
    pkg_share = get_package_share_directory(pkg_name)
    worlds_dir = os.path.join(pkg_share, 'worlds')
    
    # Declare launch argument for world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='robofetz_arena_wideangle.world',  # Default world
        description='Name of the world file to load (without path)',
        choices=['robofetz_arena_wideangle.world', 'robofetz_arena_pinhole.world']
    )
    
    # Get the world file from launch argument
    world_file = LaunchConfiguration('world')
    full_world_path = PathJoinSubstitution([worlds_dir, world_file])
    
    models_path = os.path.join(pkg_share, 'models')
    
    # **CRITICAL FOR GAZEBO HARMONIC**: Use both GZ_SIM_RESOURCE_PATH and GAZEBO_MODEL_PATH
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_gazebo_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # Build new paths
    new_gz_path = f"{models_path}"
    new_gazebo_path = f"{models_path}"
    
    if existing_gz_path:
        new_gz_path += f":{existing_gz_path}"
    if existing_gazebo_path:
        new_gazebo_path += f":{existing_gazebo_path}"
    
    print(f"Setting GZ_SIM_RESOURCE_PATH to: {new_gz_path}")
    print(f"Setting GAZEBO_MODEL_PATH to: {new_gazebo_path}")
    
    # Robot (Main)
    robot_name = 'robot'
    robot_xacro = os.path.join(pkg_share, 'models/simple_diff_drive_robot/simple_diff_drive_robot.xacro')
    robot_description = xacro.process_file(robot_xacro).toxml()
    
    # Use ExecuteProcess for Gazebo with explicit environment
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', full_world_path],
        output='screen',
        # Pass ALL environment variables explicitly
        additional_env={
            'GZ_SIM_RESOURCE_PATH': new_gz_path,
            'GAZEBO_MODEL_PATH': new_gazebo_path,
            'IGN_GAZEBO_RESOURCE_PATH': new_gz_path,  # For compatibility
        }
    )
    
    # Robot State Publishers
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': f'{robot_name}/'
        }]
    )
    
    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-name', robot_name,
            '-topic', '/robot_description',
            '-x', '0.3', '-y', '0.3', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        output='screen',
    )
    
    # Bridge
    bridge_params = os.path.join(pkg_share, 'parameters/bridge_parameters.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )
    
    # My robot tf to pose instance
    robot_tf_to_pose = Node(
        package='robofetz_gazebo',
        executable='tf_to_pose',
        name='tf_to_pose_robot',
        parameters=[{
            'tf_topic': '/pose_tf_sim',
            'pose_topic': '/camera/pose'
        }]
    )

    # Grayscale Image Republisher Node
    grayscale_republisher = Node(
        package='robofetz_gazebo',
        executable='grayscale_republisher',
        name='grayscale_republisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Camera info publisher for fisheye model
    camera_info_publisher = Node(
        package='robofetz_gazebo',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        namespace='arena_camera',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Build launch description with proper timing
    ld = LaunchDescription()
    
    # Add the world argument first
    ld.add_action(world_arg)
    
    # 1. Start Gazebo first
    ld.add_action(gazebo_process)
    
    # 2. Wait 3 seconds for Gazebo to initialize
    ld.add_action(TimerAction(
        period=3.0,
        actions=[robot_state_pub]
    ))
    
    # 3. Wait 5 seconds then spawn robots
    ld.add_action(TimerAction(
        period=5.0,
        actions=[spawn_robot]
    ))
    
    # 4. Wait 6 seconds then start bridge
    ld.add_action(TimerAction(
        period=6.0,
        actions=[bridge]
    ))
    
    # 5. Wait 7 seconds then start tf_to_pose nodes
    ld.add_action(TimerAction(
        period=7.0,
        actions=[robot_tf_to_pose]
    ))

    # 6. Wait 8 seconds then start image processing nodes
    ld.add_action(TimerAction(
        period=8.0,
        actions=[grayscale_republisher, camera_info_publisher]
    ))
    
    return ld