#ROS2 and Gazebo Harmonic launch file for differential drive robots

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import math

from launch_ros.actions import Node
import xacro
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package and file paths
    pkg_name = 'robofetz_gazebo'
    world_file = os.path.join(get_package_share_directory(pkg_name), 'worlds/arena.world')

    # Robot (Main)
    robot_name = 'robot'
    robot_xacro = os.path.join(get_package_share_directory(pkg_name), 'models/simple_diff_drive_robot/simple_diff_drive_robot.xacro')
    robot_description = xacro.process_file(robot_xacro).toxml()
    robot_pose = ['0.3', '0.3', '0.0', '0.0', '0.0', '0.0']  # x,y,z,R,P,Y

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )),
        launch_arguments={
            'gz_args': f'-r -v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot State Publishers (with namespaces)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #namespace=robot_name,  # Namespace to avoid topic conflicts
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn Robot (Main)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', '/robot_description',  # Namespaced topic
            '-x', robot_pose[0],
            '-y', robot_pose[1],
            '-z', robot_pose[2],
            '-R', robot_pose[3],
            '-P', robot_pose[4],
            '-Y', robot_pose[5]
        ],
        output='screen',
    )

    # Bridge (configured for both robots)
    bridge_params = os.path.join(get_package_share_directory(pkg_name), 'parameters/bridge_parameters.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # My robot tf to pose instance
    robot_tf_to_pose = Node(
        package='robofetz_gazebo',
        executable='tf_to_pose',
        name='tf_to_pose_robot',  # Unique name
        parameters=[
            {'tf_topic': '/sim/pose_tf'},
            {'pose_topic': '/camera/pose'}
        ]
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(robot_state_pub)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(robot_tf_to_pose)

    return ld