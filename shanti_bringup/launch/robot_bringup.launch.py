#!/usr/bin/env python3
# robot_bringup.launch.py
# Comprehensive launch file that brings up all Shanti robot systems:
# - Base (differential drive)
# - Perception (lidar, cameras)
# - Localization (GPS, IMU, odometry fusion)
# - Navigation (path planning, obstacle avoidance)

import os
import time
import launch
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument, 
    GroupAction,
    LogInfo,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directories
    base_pkg_dir = get_package_share_directory('differential_drive_base')
    localization_pkg_dir = get_package_share_directory('localization_bringup')
    nav_pkg_dir = get_package_share_directory('nav_bringup')
    perception_pkg_dir = get_package_share_directory('perception_bringup')
    robot_pkg_dir = get_package_share_directory('shanti_base')
    shanti_bringup_pkg_dir = get_package_share_directory('shanti_bringup')
    
    # Configuration file path
    config_file = os.path.join(shanti_bringup_pkg_dir, 'config', 'robot_params.yaml')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    enable_perception = LaunchConfiguration('enable_perception')
    enable_perception_arg = DeclareLaunchArgument(
        'enable_perception',
        default_value='true',
        description='Enable perception system'
    )
    
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    enable_teleop = LaunchConfiguration('enable_teleop')
    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Enable joystick teleop'
    )
    
    # Robot description
    robot_description_path = os.path.join(robot_pkg_dir, 'description/shanti_6w_lidar_description.urdf')
    robot_description_arg = DeclareLaunchArgument(
        'robot_description_file',
        default_value=robot_description_path,
        description='Path to robot URDF/xacro file'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('robot_description_file')]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # 1. BASE SYSTEM LAUNCH
    # ---------------------
    # Looking for differential_drive.launch.py in the source directory first
    base_launch_source_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'base/differential_drive_base/launch/differential_drive.launch.py'
    )
    
    # Fall back to the installed package path if not found in source
    if not os.path.exists(base_launch_source_path):
        base_launch_source_path = os.path.join(base_pkg_dir, 'launch', 'differential_drive.launch.py')
    
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_source_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': config_file
        }.items()
    )
    
    # Log message for base startup
    log_base_start = LogInfo(msg="Starting base controllers...")
    
    # 2. PERCEPTION SYSTEM LAUNCH - with delay
    # ---------------------------
    perception_launch_source_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'perception/perception_bringup/launch/dual_perception_system.launch.py'
    )
    
    # Fall back to the installed package path if not found in source
    if not os.path.exists(perception_launch_source_path):
        perception_launch_source_path = os.path.join(perception_pkg_dir, 'launch', 'dual_perception_system.launch.py')
    
    perception_delayed_action = TimerAction(
        period=5.0,  # 5 second delay after base starts
        actions=[
            LogInfo(msg="Starting perception system..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch_source_path),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': config_file,
                    'front_video_source': LaunchConfiguration('front_video_source', default='0'),
                    'front_lidar_port': LaunchConfiguration('front_lidar_port', default='/dev/ttyUSB0'),
                    'rear_video_source': LaunchConfiguration('rear_video_source', default='1'),
                    'rear_lidar_port': LaunchConfiguration('rear_lidar_port', default='/dev/ttyUSB1')
                }.items(),
                condition=IfCondition(enable_perception)
            )
        ]
    )
    
    # 3. LOCALIZATION SYSTEM LAUNCH - with delay
    # -----------------------------
    localization_launch_source_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'localization/localization_bringup/launch/integrated_localization.launch.py'
    )
    
    # Fall back to the installed package path if not found in source
    if not os.path.exists(localization_launch_source_path):
        localization_launch_source_path = os.path.join(localization_pkg_dir, 'launch', 'integrated_localization.launch.py')
    
    localization_delayed_action = TimerAction(
        period=5.0,  # 5 second delay after base starts
        actions=[
            LogInfo(msg="Starting localization system..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch_source_path),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': config_file
                }.items()
            )
        ]
    )
    
    # 4. NAVIGATION SYSTEM LAUNCH - with delay
    # ---------------------------
    navigation_launch_source_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'navigation/nav_bringup/launch/nav_bringup.launch.py'
    )
    
    # Fall back to the installed package path if not found in source
    if not os.path.exists(navigation_launch_source_path):
        navigation_launch_source_path = os.path.join(nav_pkg_dir, 'launch', 'nav_bringup.launch.py')
    
    navigation_delayed_action = TimerAction(
        period=15.0,  # 15 second delay to ensure perception and localization are up
        actions=[
            LogInfo(msg="Starting navigation system..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch_source_path),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': config_file
                }.items()
            )
        ]
    )
    
    # 5. MONITORING AND DIAGNOSTICS
    # -----------------------------
    # RViz for visualization
    rviz_config_path = os.path.join(robot_pkg_dir, 'rviz/robot_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_rviz)
    )
    
    # Rosbridge
    ros_websocket = launch_ros.actions.Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
    )

    # Rosapi
    ros_api = launch_ros.actions.Node(
        package='rosapi',
        executable='rosapi_node',
    )

    # Optional joystick teleop
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file],
        condition=IfCondition(enable_teleop)
    )
    
    teleop_node = Node(
        package='joystick2base',
        executable='joy2twist',
        name='joy2twist',
        parameters=[config_file],
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')],
        condition=IfCondition(enable_teleop)
    )
    
    # Return the launch description
    return launch.LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        robot_description_arg,
        enable_perception_arg,
        enable_rviz_arg,
        enable_teleop_arg,
        
        # Core systems with sequential launch using timers
        log_base_start,
        robot_state_publisher_node,
        base_launch,
        
        # Delayed perception and localization systems
        perception_delayed_action,
        localization_delayed_action,
        
        # More delayed navigation system
        navigation_delayed_action,
        
        # Optional visualization and teleop
        rviz_node,
        ros_websocket,
        ros_api,
        joy_node,
        teleop_node
    ])