import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

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

    combat_strategizer = Node(
        package='combat_strategizer',
        executable='simple_attack',
        name='simple_attack',
        parameters=[{'log_level': 'warn'}]  # Set the logging level to 'warn'
    )

    # Include the slider_publisher launch file with the parameters
    slider_publisher = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slider_publisher'),
                'examples',
                'example.launch'
            )
        ),
        launch_arguments={'file': './config/Float32.yaml'}.items()
    )

    # Include the slider_publisher launch file with the parameters
    slider_publisher = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'launch',
                'weaponspeed_slider.launch'
            )
        ),
        launch_arguments={'file': 'Float32.yaml'}.items()
    )

    # Add micro-ROS agent as a Node
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888', '--dev', '	192.168.1.103']  #192.168.8.210
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(simple_navigator)
    ld.add_action(combat_strategizer)
    ld.add_action(slider_publisher)
    ld.add_action(micro_ros_agent) 

    return ld