#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    perception_bringup_pkg_dir = get_package_share_directory('perception_bringup')
    
    # Define launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the ROS bag file to replay'
    )
    
    topics_arg = DeclareLaunchArgument(
        'topics',
        default_value='',
        description='Topics to replay (empty for all topics)'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier (1.0 = normal speed)'
    )
    
    # Launch perception system
    perception_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(perception_bringup_pkg_dir, 'launch', 'perception_bringup.launch.py')
        ]),
        # We're not passing video_source since we're using bag data
    )
    
    # ROS2 bag play command with loop
    bag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--loop',  # This makes the bag replay continuously
            '-r', LaunchConfiguration('rate'),
            LaunchConfiguration('topics')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        bag_path_arg,
        topics_arg,
        rate_arg,
        perception_system,
        bag_play_cmd,
    ])