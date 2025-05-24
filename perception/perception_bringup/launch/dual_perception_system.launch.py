#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    perception_bringup_pkg_dir = get_package_share_directory('perception_bringup')
    
    # Define launch arguments for the first instance
    front_video_source_arg = DeclareLaunchArgument(
        'front_video_source',
        default_value='0',
        description='Video source for the front camera (device number or file path)'
    )
    
    front_lidar_port_arg = DeclareLaunchArgument(
        'front_lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the front Unitree LiDAR'
    )

    # Define launch arguments for the second instance
    rear_video_source_arg = DeclareLaunchArgument(
        'rear_video_source',
        default_value='1',
        description='Video source for the rear camera (device number or file path)'
    )
    
    rear_lidar_port_arg = DeclareLaunchArgument(
        'rear_lidar_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the rear Unitree LiDAR'
    )
    
    # Create front perception system group
    front_system = GroupAction([
        # Push namespace for all nodes in this group
        PushRosNamespace('front'),
        
        # Include the perception_bringup launch file with front-specific parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(perception_bringup_pkg_dir, 'launch', 'perception_bringup.launch.py')
            ]),
            launch_arguments={
                'video_source': LaunchConfiguration('front_video_source'),
                'lidar_port': LaunchConfiguration('front_lidar_port'),
                'cloud_frame': 'front_unilidar_lidar',
                'imu_frame': 'front_unilidar_imu'
            }.items()
        )
    ])
    
    # Create rear perception system group
    rear_system = GroupAction([
        # Push namespace for all nodes in this group
        PushRosNamespace('rear'),
        
        # Include the perception_bringup launch file with rear-specific parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(perception_bringup_pkg_dir, 'launch', 'perception_bringup.launch.py')
            ]),
            launch_arguments={
                'video_source': LaunchConfiguration('rear_video_source'),
                'lidar_port': LaunchConfiguration('rear_lidar_port'),
                'cloud_frame': 'rear_unilidar_lidar',
                'imu_frame': 'rear_unilidar_imu'
            }.items()
        )
    ])
    
    return LaunchDescription([
        # Launch arguments
        front_video_source_arg,
        front_lidar_port_arg,
        rear_video_source_arg,
        rear_lidar_port_arg,
        
        # Launch both perception systems
        front_system,
        rear_system
    ])