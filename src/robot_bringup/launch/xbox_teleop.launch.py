import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to your config file directly
    config_path = os.path.join(get_package_share_path('robot_bringup') ,'config' ,'xbox_teleop.config.yaml' )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path('teleop_twist_joy') ,'launch' ,'teleop-launch.py')
        ),
        launch_arguments={
            'joy_vel': 'cmd_vel',
            'joy_config': 'xbox',
            'config_filepath': config_path  # Convert Path object to string
        }.items()
    )


    ld = LaunchDescription()
    ld.add_action(teleop)
    return ld