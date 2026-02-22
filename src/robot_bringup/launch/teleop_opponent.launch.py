import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

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
        teleop_opponent
    ])