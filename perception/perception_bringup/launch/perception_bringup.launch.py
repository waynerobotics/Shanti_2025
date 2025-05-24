#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    vision3d_pkg_dir = get_package_share_directory('vision3D')
    
    # Define launch arguments
    video_source_arg = DeclareLaunchArgument(
        'video_source',
        default_value='0',
        description='Video source for the camera (device number or file path)'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Unitree LiDAR'
    )
    
    cloud_frame_arg = DeclareLaunchArgument(
        'cloud_frame',
        default_value='unilidar_lidar',
        description='Frame ID for the point cloud'
    )
    
    imu_frame_arg = DeclareLaunchArgument(
        'imu_frame',
        default_value='unilidar_imu',
        description='Frame ID for the IMU'
    )
    
    # Unitree LiDAR node
    unitree_lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
            {'port': LaunchConfiguration('lidar_port')},
            {'rotate_yaw_bias': 0.0},
            {'range_scale': 0.001},
            {'range_bias': 0.0},
            {'range_max': 50.0},
            {'range_min': 0.0},
            {'cloud_frame': LaunchConfiguration('cloud_frame')},
            {'cloud_topic': "unilidar/cloud"},
            {'cloud_scan_num': 18},
            {'imu_frame': LaunchConfiguration('imu_frame')},
            {'imu_topic': "unilidar/imu"}
        ]
    )
    
    # Vision3D bringup launch
    vision3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(vision3d_pkg_dir, 'launch', 'vision3D_bringup.launch.py')
        ]),
        launch_arguments={
            'video_source': LaunchConfiguration('video_source'),
            'mask_topic': 'lane_detection/segmentation_mask',
        }.items()
    )
    
    # MaskToPointCloud node
    mask_to_pointcloud_node = Node(
        package='omnivision',
        executable='mask_to_pointcloud',
        name='mask_to_pointcloud',
        parameters=[
            {
                'transformation_matrix': [
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0
                ],
                'pointcloud_topic': 'unilidar/cloud',
                'mask_topic': 'lane_detection/segmentation_mask',
                'obstacle_pointcloud': 'perception/obstacle_pointcloud',
                'debug_overlay': 'perception/debug/mask_overlay'
            }
        ],
        output='screen',
    )
    
    return LaunchDescription([
        video_source_arg,
        lidar_port_arg,
        cloud_frame_arg,
        imu_frame_arg,
        unitree_lidar_node,
        vision3d_launch,
        mask_to_pointcloud_node,
    ])