#!/usr/bin/env python3
"""
Main launch file for robot software operation.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Package names
    perception_pkg = 'arena_perception'
    localization_pkg = 'arena_perception'
    robot_description_pkg = 'robot_description'
    gazebo_pkg = 'robofetz_gazebo'
    
    # ============================================
    # LAUNCH ARGUMENTS
    # ============================================
    
    # Simulation mode argument
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Launch Gazebo simulation (true/false)',
        choices=['true', 'false']
    )
    
    # Fake perception argument - only usable with simulation
    use_fake_perception_arg = DeclareLaunchArgument(
        'use_fake_perception',
        default_value='false',
        description='Use fake arena perception (only valid when use_sim=true)',
        choices=['true', 'false']
    )
    
    # Gazebo world file argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='robofetz_arena_pinhole.world',
        description='Name of the Gazebo world file to load',
        choices=['robofetz_arena_wideangle.world', 'robofetz_arena_pinhole.world']
    )
    
    # Robot prefix argument
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='robot/',  # Default to 'robot/' prefix
        description='Robot namespace prefix (e.g., "robot1/")'
    )
    
    # ============================================
    # ARENA PERCEPTION CONFIGURATION
    # ============================================
        
    default_arena_perception_config = PathJoinSubstitution([
        FindPackageShare(perception_pkg),
        'config', 
        'arena_perception_config.yaml'
    ])
        
    arena_perception_config_arg = DeclareLaunchArgument(
        'arena_perception_config',
        default_value=default_arena_perception_config,
        description='Path to arena_perception_config file'
    )

    # ============================================
    # ROBOT LOCALIZATION CONFIGURATION
    # ============================================

    default_robot_localization_config = PathJoinSubstitution([
        FindPackageShare(localization_pkg),
        'config',
        'robot_localization_config.yaml'
    ])
    
    robot_localization_config_arg = DeclareLaunchArgument(
        'robot_localization_config',
        default_value=default_robot_localization_config,
        description='Path to robot localization config file'
    )
    
    # Get config files from launch arguments
    use_sim = LaunchConfiguration('use_sim')
    use_fake_perception = LaunchConfiguration('use_fake_perception')
    world_file = LaunchConfiguration('world')
    prefix = LaunchConfiguration('prefix')
    arena_perception_config_file = LaunchConfiguration('arena_perception_config')
    robot_localization_config_file = LaunchConfiguration('robot_localization_config')
    
    # ============================================
    # BUILD LAUNCH DESCRIPTION
    # ============================================
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_arg)
    ld.add_action(use_fake_perception_arg)
    ld.add_action(world_arg)
    ld.add_action(prefix_arg)
    ld.add_action(arena_perception_config_arg)
    ld.add_action(robot_localization_config_arg)
    
    
    # ============================================
    # 0. ROBOT STATE PUBLISHER
    # ============================================
    
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(robot_description_pkg),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim,
            'prefix': prefix
        }.items()
    )
    
    ld.add_action(robot_state_publisher_launch)

    # ============================================
    # 1A. GAZEBO SIMULATION (CONDITIONAL)
    # ============================================
    
    # Include Gazebo simulation launch when use_sim is true
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(gazebo_pkg),
                'launch',
                'robofetz_gazebo_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'enable_robot_state_pub': 'false'  # already covered above
        }.items(),
        condition=IfCondition(use_sim)
    )

    # ============================================
    # 1B. REAL HARDWARE (CONDITIONAL)
    # ============================================
    
    #includes to do

    ld.add_action(TimerAction(
        period=3.0,
        actions=[gazebo_sim_launch]
    ))

    # ============================================
    # 2A. ARENA PERCEPTION SYSTEM
    # ============================================
    
    # Create arena perception launch with parameters
    arena_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(perception_pkg),
                'launch',
                'arena_perception.launch.py'
            ])
        ]),
        launch_arguments={
            'arena_perception_config': arena_perception_config_file,
            'use_sim_time': use_sim
        }.items(),
        # Only launch real perception when NOT using fake perception
        condition=UnlessCondition(use_fake_perception)
    )

    # ============================================
    # 2B. FAKE ARENA PERCEPTION SYSTEM
    # ============================================

    fake_arena_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(perception_pkg),
                'launch',
                'fake_arena_perception.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim
        }.items(),
        # Only launch fake perception when use_fake_perception is true
        condition=IfCondition(use_fake_perception)
    )
    
    # Add arena perception with delay (in simulation, wait longer for Gazebo to start)
    # Use PythonExpression to properly evaluate the string condition
    arena_perception_delay = PythonExpression([
        '8.0 if "', use_sim, '" == "true" else 3.0'
    ])
    
    ld.add_action(TimerAction(
        period=arena_perception_delay,
        actions=[arena_perception_launch, fake_arena_perception_launch]
    ))
    
    # ============================================
    # 3. ROBOT LOCALIZATION
    # ============================================
    
    # Include robot localization launch with the config file
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(perception_pkg),
                'launch',
                'robot_localization.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_localization_config': robot_localization_config_file,
            'use_sim_time': use_sim,
            'prefix': prefix
        }.items()
    )
    
    # Add robot localization with longer delay in simulation
    # Use PythonExpression to properly evaluate the string condition
    localization_delay = PythonExpression([
        '20.0 if "', use_sim, '" == "true" else 15.0'
    ])
    
    ld.add_action(TimerAction(
        period=localization_delay,
        actions=[robot_localization_launch]
    ))
    
    return ld