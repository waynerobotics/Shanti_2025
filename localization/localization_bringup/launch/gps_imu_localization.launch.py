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
    rl_params_file = os.path.join(localization_bringup_dir, "config", "gps_imu_params.yaml")
    xsens_params_file = os.path.join(xsens_dir, "param", "xsens_mti_node.yaml")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    gps_topic = LaunchConfiguration('gps_topic', default='/gnss_pose')
    imu_topic = LaunchConfiguration('imu_topic', default='/imu/data')
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
            default_value='/gnss_pose',
            description='Topic for GPS data'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='Topic for XSens IMU data'
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
    parameters_file_path = Path(get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node.yaml')
    xsens_mti_node = Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[parameters_file_path, {'use_sim_time': use_sim_time}],
            arguments=[]
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
            'mountpoint': 'fahnerfarms',
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

    # 3. TF for IMU (static transform from base_link to imu_link)
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Add static transform from map to odom
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Add a temporary static transform from odom to base_link
    # This will be replaced by the EKF once it starts publishing transforms
    odom_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Add static transform from base_link to gnss
    gnss_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gnss_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gnss_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. NavSat Transform Node (from robot_localization)
    navsat_transform_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/imu", imu_topic),  # Use XSens IMU data
            ("/gps/fix", "/gnss"),  # We need NavSatFix data for navsat_transform
            ("/odometry/filtered", "/odometry/filtered"),  # Use the same odometry topic as EKF output
            ("/odometry/gps", "/odometry/gps"),  # GPS odometry input to EKF
        ],
    )
    
    # 5. EKF Filter Node - only one needed when not using wheel odometry
    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
            ("/imu/data", imu_topic),
            ("/odometry/gps", "/odometry/gps"),  # Explicitly map GPS odometry input
        ],
    )
    
    # GPS pose to NavSatFix converter node
    pose_to_gps_converter_node = Node(
        package='localization_bringup',
        executable='pose_to_gps_converter',
        name='pose_to_gps_converter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/gnss_pose',
            'output_topic': '/gnss',
            # You may need to update these reference coordinates for your location
            'reference_latitude': 42.66791,
            'reference_longitude': -83.21958,
            'reference_altitude': 0.0,
        }]
    )
    
    # 6. GPS Monitor tool (for debugging)
    gps_monitor_node = Node(
        package='localization_bringup',
        executable='gps_monitor',
        name='gps_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/gps/fix', gps_topic),
            ('/odometry/filtered', '/odometry/filtered'),
        ]
    )

    # Assemble launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    for arg in launch_args:
        ld.add_action(arg)
    
    # Add environment variables
    for action in set_env_actions:
        ld.add_action(action)
    
    # Add nodes in order - order matters for initialization
    ld.add_action(imu_tf)
    ld.add_action(gnss_tf)
    ld.add_action(map_odom_tf)
    ld.add_action(odom_base_tf)  # Add the temporary transform
    ld.add_action(xsens_mti_node)
    ld.add_action(pose_to_gps_converter_node)
    ld.add_action(ntrip_node)
    ld.add_action(ekf_node)  # EKF must start before navsat_transform
    ld.add_action(navsat_transform_node)
    ld.add_action(gps_monitor_node)
    
    return ld
