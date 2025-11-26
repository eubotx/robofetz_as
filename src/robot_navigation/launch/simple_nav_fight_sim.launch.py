# navigation_fight.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robofetz_gazebo'), 
            'launch', 
            'gazebo_bot_with_opponent.launch.py'
        )),
        launch_arguments={
        }.items()
    )

    simple_navigator = Node(
        package='robot_navigation',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('robot_navigation'),
                'config',
                'params.yaml',
            ),
            {'log_level': 'warn'}]
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

    combat_strategizer = Node(
        package='combat_strategizer',
        executable='simple_attack',
        name='simple_attack',
        parameters=[{'log_level': 'warn'}]  # Set the logging level to 'warn'
    )

    # RViz2 launch node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('robot_navigation'), 
            'config', 
            'rviz_config.rviz'  # Specify your RViz config file if you have one
        )]
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(simple_navigator)
    ld.add_action(teleop_opponent)
    ld.add_action(combat_strategizer)
    ld.add_action(rviz2)



    return ld