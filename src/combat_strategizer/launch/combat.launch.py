import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition # <--- Added this

def generate_launch_description():
    # ============================================
    # LAUNCH ARGUMENTS
    # ============================================
    strategizer_type = LaunchConfiguration('strategizer_type')

    declare_strategizer_type = DeclareLaunchArgument(
        'strategizer_type',
        default_value='simple',
        description='Type of combat strategizer to use (simple, nav2)'
    )




    # =================== NODE 1: SIMPLE NAVIGATOR ===================
    # These will only run if strategizer_type == 'simple'
    is_simple_mode = IfCondition(
        PythonExpression(["'", strategizer_type, "' == 'simple'"])
    )

    simple_navigator = Node(
        package='robot_navigation',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
        condition=is_simple_mode, # <--- Apply condition here
        parameters=[
            os.path.join(get_package_share_directory('robot_navigation'), 'config', 'params.yaml'),
            {'log_level': 'warn'}
        ]
    )

    combat_strategizer = Node(
        package='combat_strategizer',
        executable='simple_attack',
        name='simple_attack',
        condition=is_simple_mode, # <--- Apply condition here
        parameters=[{'log_level': 'warn'}]
    )

    # =================== RETURN ===================
    ld = LaunchDescription()

    # Add the declaration first
    ld.add_action(declare_strategizer_type)
    
    # Add nodes/includes
    ld.add_action(simple_navigator)
    ld.add_action(combat_strategizer)

    return ld