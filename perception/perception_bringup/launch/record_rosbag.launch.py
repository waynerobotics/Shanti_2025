#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from datetime import datetime

def generate_launch_description():
    # Define launch arguments
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value=f'perception_data_{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}',
        description='Name of the bag to be created'
    )
    
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='~/rosbags',
        description='Path where to save the bag file'
    )
    
    all_topics_arg = DeclareLaunchArgument(
        'all_topics',
        default_value='false',
        description='Record all topics (true) or only specified ones (false)'
    )
    
    include_lidar_arg = DeclareLaunchArgument(
        'include_lidar',
        default_value='true',
        description='Include LiDAR data in recording'
    )
    
    include_camera_arg = DeclareLaunchArgument(
        'include_camera',
        default_value='true',
        description='Include camera data in recording'
    )
    
    include_processed_arg = DeclareLaunchArgument(
        'include_processed',
        default_value='true',
        description='Include processed vision data in recording'
    )
    
    include_pointcloud_arg = DeclareLaunchArgument(
        'include_pointcloud',
        default_value='true',
        description='Include processed point cloud data in recording'
    )
    
    # Prepare topic list based on include flags
    topic_list = ''
    
    # ROS2 bag record command
    record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', PythonExpression(['"', LaunchConfiguration('bag_path'), '/', LaunchConfiguration('bag_name'), '"']),
            '--include-hidden-topics',
            PythonExpression([
                '"-a" if "', LaunchConfiguration('all_topics'), '" == "true" else ',
                '"',
                PythonExpression([
                    '"/unilidar/cloud /unilidar/imu " if "', LaunchConfiguration('include_lidar'), '" == "true" else ""',
                ]), 
                PythonExpression([
                    '"+ /camera/image_raw " if "', LaunchConfiguration('include_camera'), '" == "true" else ""',
                ]),
                PythonExpression([
                    '"+ /lane_detection/image_processed /lane_detection/segmentation_mask " if "',
                    LaunchConfiguration('include_processed'), '" == "true" else ""',
                ]),
                PythonExpression([
                    '"+ /perception/obstacle_pointcloud " if "',
                    LaunchConfiguration('include_pointcloud'), '" == "true" else ""',
                ]),
                '"'
            ])
        ],
        shell=True,
        output='screen'
    )
    
    return LaunchDescription([
        bag_name_arg,
        bag_path_arg,
        all_topics_arg,
        include_lidar_arg,
        include_camera_arg,
        include_processed_arg,
        include_pointcloud_arg,
        record_cmd,
    ])