import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directories
    robofetz_gazebo_dir = get_package_share_directory('robofetz_gazebo')
    robot_navigation_dir = get_package_share_directory('robot_navigation')

    # Navigation node - Nav2 Stack
    nav2_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_navigation_dir, 'launch', 'nav2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'params_file': os.path.join(robot_navigation_dir, 'config', 'nav2_params.yaml')
                }.items()
            )
        ]
    )

    # Combat strategizer node behaviour for main robot 
    combat_strategizer = Node(
        package='combat_strategizer',
        executable='nav2_attack',
        name='nav2_attack',
        output='screen',
        parameters=[{'log_level': 'warn'}]
    )

    # Weapon control node
    weapon_control = Node(
        package='combat_strategizer',
        executable='weapon_control',
        name='weapon_control',
        output='screen'
    )

    # RViz2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            robot_navigation_dir, 
            'config', 
            'rviz_config.rviz'
        )]
    )

    # Include teleop launch file for joystick control
    teleop_opponent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')
        ),
        launch_arguments={
            'joy_vel': 'opponent/cmd_vel',  # Remap velocity command topic
            'joy_config': 'xbox',   # Define joystick configuration
            #'joy_dev': '0',        # Joystick device (e.g., /dev/input/js0)
            #'publish_stamped_twist': 'false',  # Whether to publish a stamped twist
            #'config_filepath': 'config/bot135smallWheel_teleop.config.yaml'
        }.items()
    )

    return LaunchDescription([
        combat_strategizer,
        weapon_control,
        rviz2,
        teleop_opponent,
        nav2_launch
    ])