#!/usr/bin/env python3

# Copyright 2023-2025 Shanti Team
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
import launch_ros.actions
from launch_ros.actions import Node, LifecycleNode
import os
from pathlib import Path

def generate_launch_description():
    # Get directory paths
    localization_bringup_dir = get_package_share_directory("localization_bringup")
    xsens_dir = get_package_share_directory("xsens_mti_ros2_driver")
    ntrip_dir = get_package_share_directory("ntrip")
    
    # Parameters file paths
    rl_params_file = os.path.join(localization_bringup_dir, "config", "dual_ekf_navsat_params.yaml")
    xsens_params_file = os.path.join(xsens_dir, "param", "xsens_mti_node.yaml")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    gps_topic = LaunchConfiguration('gps_topic', default='/gps/fix')
    imu_topic = LaunchConfiguration('imu_topic', default='/xsens/imu/data')  # Updated to use XSens IMU
    map_odom_topic = LaunchConfiguration('map_odom_topic', default='/odometry/map')
    map_frame = LaunchConfiguration('map_frame', default='map')
    utm_frame = LaunchConfiguration('utm_frame', default='utm')
    log_level = LaunchConfiguration('log_level', default='info')
    
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'gps_topic',
            default_value='/gps/fix',
            description='Topic for GPS data'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/xsens/imu/data',
            description='Topic for XSens IMU data'
        ),
        DeclareLaunchArgument(
            'map_odom_topic',
            default_value='/odometry/map',
            description='Topic for map odometry data'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame name'
        ),
        DeclareLaunchArgument(
            'utm_frame',
            default_value='utm',
            description='UTM frame name'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error, fatal)'
        ),
    ]
    
    # Set environment variables for XSens driver
    set_env_actions = [
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    ]
    
    # 1. XSens IMU Driver Node
    xsens_mti_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[xsens_params_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # 2. NTRIP Client Node for GPS corrections
    ntrip_node = Node(
        package='ntrip',
        executable='ntrip',
        name='ntrip_client',
        output='screen',
        parameters=[{
            # NTRIP Server Configuration
            'host': '3.143.243.81',  # Update this with your NTRIP caster
            'port': 2101,
            'mountpoint': 'WAYNEROBOTICS',
            'username': 'roboticsclub-at-wayne-d-edu',
            'password': 'none',

            # NMEA and Update Rate Configuration
            'nmea_input_rate': 4.0,
            'update_rate': 1.0,

            # Connection Configuration
            'reconnect_delay': 5.0,
            'max_reconnect_attempts': 0,

            # Debug Configuration
            'send_default_gga': False,
            'debug': True,
            'output_rtcm_details': True,
            
            # Simulation time
            'use_sim_time': use_sim_time
        }],
        # Topic Remapping
        remappings=[
            ('nmea', 'nmea'),
            ('rtcm', 'rtcm')
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # 3. NavSat Transform Node (from robot_localization)
    navsat_transform_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/imu/data", imu_topic),  # Use XSens IMU data
            ("/gps/fix", "/gps/fix"),
            ("/odometry/gps", "/odometry/gps"),
            ("/gps/filtered", "/gps/filtered"),
            ('/odometry/filtered', '/odometry/odom'),
        ],
    )
    
    # 4. UTM Map Transform Publisher (from your custom package)
    utm_map_transform_publisher = LifecycleNode(
        package="localization_bringup",
        executable="utm_map_transform_publisher",
        name="utm_map_transform_publisher",
        output="screen",
        namespace='',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': map_frame,
            'utm_frame': utm_frame,
            'gps_topic': gps_topic,
            'imu_topic': imu_topic,
            'map_odom_topic': map_odom_topic
        }],
    )
    
    # 5. Custom Lifecycle Manager
    custom_lifecycle_manager = Node(
        package='localization_bringup',
        executable='custom_lifecycle_manager',
        name='custom_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_odom_topic': map_odom_topic,
            'map_frame': map_frame,
            'utm_frame': utm_frame,
            'transform_check_period': 1.0,
            'utm_map_transform_publisher_node': 'utm_map_transform_publisher',
            'gps_map_transformer_node': 'gps_map_transformer'
        }]
    )
    
    # 6. EKF for Odometry (local frame)
    ekf_odom = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[rl_params_file, {
            "use_sim_time": use_sim_time,
            "imu0": imu_topic  # Update to use XSens IMU
        }],
        remappings=[("odometry/filtered", "/odometry/odom")],
    )
    
    # 7. EKF for Map (global frame)
    ekf_map = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[rl_params_file, {
            "use_sim_time": use_sim_time,
            "imu0": imu_topic  # Update to use XSens IMU
        }],
        remappings=[("odometry/filtered", "/odometry/map")]
    )
    
    # 8. Odometry Rebroadcaster
    odometry_rebroadcaster = launch_ros.actions.Node(
        package="localization_bringup",
        executable="odometry_rebroadcaster",
        name="odometry_rebroadcaster",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/odometry/gps", "/odometry/gps"),
            ("/odometry/gps_map", "/odometry/gps_map")
        ],
    )
    
    # 9. TF for IMU (static transform from base_link to imu_link)
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Assemble launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    for arg in launch_args:
        ld.add_action(arg)
    
    # Add environment variables
    for action in set_env_actions:
        ld.add_action(action)
    
    # Add nodes in order
    ld.add_action(xsens_mti_node)
    ld.add_action(ntrip_node)
    ld.add_action(imu_tf)
    ld.add_action(navsat_transform_node)
    ld.add_action(utm_map_transform_publisher)
    ld.add_action(custom_lifecycle_manager)
    ld.add_action(ekf_odom)
    ld.add_action(ekf_map)
    ld.add_action(odometry_rebroadcaster)
    
    return ld