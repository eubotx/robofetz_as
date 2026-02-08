#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition
import xacro

def generate_launch_description():
    
    # Package and file paths
    gazebo_pkg = 'robofetz_gazebo'
    desc_pkg = 'robot_description'

    # Robot names also used to build prefix
    robot_name = 'robot'
    opponent_name = 'opponent'
    
    gazebo_pkg_share = get_package_share_directory(gazebo_pkg)  #use get_package share, FindPackageShare causes problems here
    
    worlds_dir = os.path.join(gazebo_pkg_share, 'worlds')
    config_dir = os.path.join(gazebo_pkg_share, 'config')
    models_path = os.path.join(gazebo_pkg_share, 'models')
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='robofetz_arena_wideangle.world',
        description='Name of the world file to load',
        choices=['robofetz_arena_wideangle.world', 'robofetz_arena_pinhole.world']
    )
    
    # Declare argument to enable/disable robot state publisher if launhed in other place
    enable_robot_state_pub_arg = DeclareLaunchArgument(
        'enable_robot_state_pub',
        default_value='true',
        description='Enable robot state publisher for the main robot',
        choices=['true', 'false']
    )
    
    # Get the world file from launch argument
    world_file = LaunchConfiguration('world')
    full_world_path = PathJoinSubstitution([worlds_dir, world_file])
    
    # Get robot state publisher flag
    enable_robot_state_pub = LaunchConfiguration('enable_robot_state_pub')
    
    # Setup Gazebo environment paths
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_gazebo_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    new_gz_path = f"{models_path}"
    new_gazebo_path = f"{models_path}"
    
    if existing_gz_path:
        new_gz_path += f":{existing_gz_path}"
    if existing_gazebo_path:
        new_gazebo_path += f":{existing_gazebo_path}"
    
    print(f"Setting GZ_SIM_RESOURCE_PATH to: {new_gz_path}")
    print(f"Setting GAZEBO_MODEL_PATH to: {new_gazebo_path}")
    
    # Start Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', full_world_path],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': new_gz_path,
            'GAZEBO_MODEL_PATH': new_gazebo_path,
            'IGN_GAZEBO_RESOURCE_PATH': new_gz_path,
        }
    )
    
    # Robot description via robot_state_publisher from robot_description package
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(desc_pkg),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'prefix': f'{robot_name}/'
        }.items(),
        condition=UnlessCondition(enable_robot_state_pub)  # Conditionally include
    )
    
    # Spawn Robot (uses robot_description from included launch)
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

    # Opponent description using opponent xacro from this package
    opponent_xacro = os.path.join(gazebo_pkg_share, 'models/simple_diff_drive_opponent/simple_diff_drive_opponent.xacro')
    opponent_description = xacro.process_file(
        opponent_xacro, 
        mappings={'prefix': f'{opponent_name}/'}
    ).toxml()
    
    opponent_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opponent_state_publisher',
        namespace=opponent_name,
        output='screen',
        parameters=[{
            'robot_description': opponent_description,
            'use_sim_time': True,
        }]
    )

    # Spawn Opponent Robot directly from xacro
    spawn_opponent = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_opponent',
        arguments=[
            '-name', opponent_name,
            '-topic', f'/{opponent_name}/robot_description',
            '-x', '1.2', '-y', '1.2', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '3.14'
        ],
        output='screen',
    )
    
    # Gazebo Ros Bridge
    bridge_params = os.path.join(gazebo_pkg_share, 'parameters/bridge_parameters.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # Grayscale Image Republisher Node to get grayscale cam
    grayscale_republisher = Node(
        package='robofetz_gazebo',
        executable='grayscale_republisher',
        name='grayscale_republisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Use PythonExpression to create conditional config file path
    camera_info_file = PythonExpression([
        "'", os.path.join(config_dir, 'camera_pinhole.yaml'), 
        "' if 'pinhole' in '", world_file, 
        "' else '", os.path.join(config_dir, 'camera_wideangle.yaml'), "'"
    ])

    # Camera info publisher using correct camera info
    camera_info_publisher = Node(
        package='robofetz_gazebo',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        namespace='arena_camera',
        parameters=[{
            'use_sim_time': True,
            'config_file': camera_info_file
        }],
        output='screen'
    )
    
    # Build launch description with timing
    ld = LaunchDescription()
    
    ld.add_action(world_arg)
    ld.add_action(enable_robot_state_pub_arg)

    ld.add_action(gazebo_process)
    
    ld.add_action(TimerAction(
        period=2.0,
        actions=[robot_description_launch, opponent_state_pub]
    ))
    
    ld.add_action(TimerAction(
        period=5.0,
        actions=[spawn_robot, spawn_opponent]
    ))
    
    ld.add_action(TimerAction(
        period=6.0,
        actions=[bridge]
    ))

    ld.add_action(TimerAction(
        period=8.0,
        actions=[grayscale_republisher, camera_info_publisher]
    ))
    
    return ld