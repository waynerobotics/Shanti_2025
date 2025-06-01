#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the ROS bag file to replay (required)'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier (1.0 = normal speed)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Whether to loop the bag file playback'
    )
    
    start_arg = DeclareLaunchArgument(
        'start',
        default_value='0',
        description='Start seconds into the bag file'
    )
    
    topics_arg = DeclareLaunchArgument(
        'topics',
        default_value='',
        description='Topics to replay (empty for all topics)'
    )
    
    process_perception_arg = DeclareLaunchArgument(
        'process_perception',
        default_value='false',
        description='Whether to launch the perception processing nodes'
    )
    
    # Prepare options for ros2 bag play command
    loop_option = PythonExpression(['"--loop" if "', LaunchConfiguration('loop'), '" == "true" else ""'])
    
    # ROS2 bag play command
    bag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            loop_option,
            '-r', LaunchConfiguration('rate'),
            '--start', LaunchConfiguration('start'),
            LaunchConfiguration('topics')
        ],
        shell=True,
        output='screen'
    )
    
    # Launch perception processing (mask_to_pointcloud) if requested
    mask_to_pointcloud_node = Node(
        package='omnivision',
        executable='mask_to_pointcloud',
        name='mask_to_pointcloud',
        parameters=[{
            'transformation_matrix': [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0
            ],
            'pointcloud_topic': '/unilidar/cloud',
            'mask_topic': '/lane_detection/segmentation_mask',
            'obstacle_pointcloud': '/perception/obstacle_pointcloud',
            'debug_overlay': '/perception/debug/mask_overlay'
        }],
        output='screen',
        condition=LaunchConfiguration('process_perception')
    )
    
    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        loop_arg,
        start_arg,
        topics_arg,
        process_perception_arg,
        bag_play_cmd,
        mask_to_pointcloud_node
    ])