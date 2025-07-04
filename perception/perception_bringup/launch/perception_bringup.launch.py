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

    intensity_min_arg = DeclareLaunchArgument(
        'intensity_min',
        default_value='50.0',
        description='Minimum intensity for filtering the point cloud'
    )

    intensity_max_arg = DeclareLaunchArgument(
        'intensity_max',
        default_value='250.0',
        description='Maximum intensity for filtering the point cloud'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Height for filtering the point cloud (in meters)'
    )
    
    # Unitree LiDAR 
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
                    0.0, -1, 0.0, -0.058,
                    -1, 1.0, 0.0, 0.0,
                    0.0, 0.0, -1.0, 0.09,
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
    
    # PointCloud Intensity Filter Node
    pointcloud_intensity_filter_node = Node(
        package='omnivision',
        executable='pointcloud_filter',
        name='pointcloud_filter',
        parameters=[
            {
                'pointcloud_topic': 'unilidar/cloud',
                'filtered_pointcloud_topic': 'filtered_cloud',
                'intensity_min': LaunchConfiguration('intensity_min'),
                'intensity_max': LaunchConfiguration('intensity_max'),
                'height': LaunchConfiguration('height')
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        video_source_arg,
        lidar_port_arg,
        cloud_frame_arg,
        imu_frame_arg,
        intensity_min_arg,
        intensity_max_arg,
        height_arg,
        unitree_lidar_node,
        vision3d_launch,
        mask_to_pointcloud_node,
        pointcloud_intensity_filter_node,
    ])