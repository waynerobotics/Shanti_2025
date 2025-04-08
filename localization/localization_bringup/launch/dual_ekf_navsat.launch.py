# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.actions import Node, LifecycleNode
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "localization_bringup")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")
    print(rl_params_file)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gps_topic = LaunchConfiguration('gps_topic', default='/gps/fix')
    imu_topic = LaunchConfiguration('imu_topic', default='/demo/imu')
    map_odom_topic = LaunchConfiguration('map_odom_topic', default='/odometry/map')
    map_frame = LaunchConfiguration('map_frame', default='map')
    utm_frame = LaunchConfiguration('utm_frame', default='utm')
    
    # Create the custom lifecycle manager node
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

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation clock if true'
            ),
            DeclareLaunchArgument(
                'gps_topic',
                default_value='/gps/filtered',
                description='Topic for filtered GPS data'
            ),
            DeclareLaunchArgument(
                'imu_topic',
                default_value='/demo/imu',
                description='Topic for IMU data'
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
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/imu/data", "/demo/imu"),#(prefixed name, new name)
                    ("/gps/fix", "/gps/fix"),
                    ("/odometry/gps", "/odometry/gps"),
                    ("/gps/filtered", "/gps/filtered"),
                    ('/odometry/filtered', '/odometry/odom'),
                ],
            ),
            # Use the UTM to map transform publisher as lifecycle node
            LifecycleNode(
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
            ),
            # Add the custom lifecycle manager
            custom_lifecycle_manager,
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odometry/odom")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odometry/map")]
            ),

            launch_ros.actions.Node(
                package="localization_bringup",
                executable="odometry_rebroadcaster",
                name="odometry_rebroadcaster",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/odometry/gps", "/odometry/gps"),
                    ("/odometry/gps_map", "/odometry/gps_map")
                ],
            ),
        ]
    )
