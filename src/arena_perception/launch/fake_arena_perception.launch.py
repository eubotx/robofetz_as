#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package name (assuming this file is in arena_perception package)
    pkg = 'arena_perception'

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    ld.add_action(use_sim_time_arg)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # TF relay for robot - reads robot/base_footprint_sim and publishes to arena_perception/robot/base_footprint
    robot_tf_relay = Node(
        package=pkg,
        executable='tf_frame_relay',
        name='robot_tf_relay',
        parameters=[{
            'source_frame': 'robot/base_footprint_sim',
            'target_frame': 'arena_perception/robot/base_footprint',
            'rate': 60.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # TF relay for opponent
    opponent_tf_relay = Node(
        package=pkg,
        executable='tf_frame_relay',
        name='opponent_tf_relay',
        parameters=[{
            'source_frame': 'opponent/base_footprint_sim',
            'target_frame': 'arena_perception/opponent/base_footprint',
            'rate': 60.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    ld.add_action(robot_tf_relay)
    ld.add_action(opponent_tf_relay)

    return ld