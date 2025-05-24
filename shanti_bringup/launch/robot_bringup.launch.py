#!/usr/bin/env python3
# robot_bringup.launch.py
# Comprehensive launch file that brings up all Shanti robot systems:
# - Base (differential drive)
# - Perception (lidar, cameras)
# - Localization (GPS, IMU, odometry fusion)
# - Navigation (path planning, obstacle avoidance)

import os
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directories
    base_pkg_dir = get_package_share_directory('differential_drive_base')
    localization_pkg_dir = get_package_share_directory('localization_bringup')
    nav_pkg_dir = get_package_share_directory('nav_bringup')
    omnivision_pkg_dir = get_package_share_directory('omnivision')
    lidar_pkg_dir = get_package_share_directory('unitree_lidar_ros2')
    robot_pkg_dir = get_package_share_directory('shanti_base')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    debug_mode = LaunchConfiguration('debug')
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging if true'
    )
    
    # Base configuration arguments
    left_roboclaw_port = LaunchConfiguration('left_roboclaw_port')
    right_roboclaw_port = LaunchConfiguration('right_roboclaw_port')
    baud_rate = LaunchConfiguration('baud_rate')
    left_address = LaunchConfiguration('left_address')
    right_address = LaunchConfiguration('right_address')
    
    left_roboclaw_port_arg = DeclareLaunchArgument(
        'left_roboclaw_port',
        default_value='/dev/ttyACM0',
        description='Serial port for left Roboclaw controller'
    )
    
    right_roboclaw_port_arg = DeclareLaunchArgument(
        'right_roboclaw_port',
        default_value='/dev/ttyACM1',
        description='Serial port for right Roboclaw controller'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='38400',
        description='Baud rate for Roboclaw controllers'
    )
    
    left_address_arg = DeclareLaunchArgument(
        'left_address',
        default_value='128',
        description='Address of left Roboclaw controller'
    )
    
    right_address_arg = DeclareLaunchArgument(
        'right_address',
        default_value='128',
        description='Address of right Roboclaw controller'
    )
    
    # Robot description
    robot_description_path = os.path.join(robot_pkg_dir, 'description/shanti_6w_lidar_description.urdf')
    robot_description_arg = DeclareLaunchArgument(
        'robot_description_file',
        default_value=robot_description_path,
        description='Path to robot URDF/xacro file'
    )
    
    # 1. BASE SYSTEM LAUNCH
    # ---------------------
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_pkg_dir, 'launch', 'differential_drive.launch.py')
        ),
        launch_arguments={
            'left_roboclaw_port': left_roboclaw_port,
            'right_roboclaw_port': right_roboclaw_port,
            'baud_rate': baud_rate,
            'left_address': left_address,
            'right_address': right_address,
            'use_sim_time': use_sim_time
        }.items()
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
    
    # 2. PERCEPTION SYSTEM LAUNCH
    # ---------------------------
    # Lidar node
    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_node',
        name='unitree_lidar_node',
        output='screen',
        parameters=[{
            'frame_id': 'lidar_link',
            'use_sim_time': use_sim_time
        }]
    )
    
    # Omnivision cameras (assuming there's a camera launch)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(omnivision_pkg_dir, 'launch', 'omnivision.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_cameras', default='true'))
    )
    
    # 3. LOCALIZATION SYSTEM LAUNCH
    # -----------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg_dir, 'launch', 'integrated_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 4. NAVIGATION SYSTEM LAUNCH
    # ---------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_dir, 'launch', 'nav_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
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
        condition=IfCondition(LaunchConfiguration('enable_rviz', default='true'))
    )
    
    # Optional joystick teleop
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': LaunchConfiguration('joy_device', default='0'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_teleop', default='false'))
    )
    
    teleop_node = Node(
        package='joystick2base',
        executable='joy2twist',
        name='joy2twist',
        parameters=[{
            'linear_axis': 1,
            'angular_axis': 0,
            'enable_button': 0,
            'linear_scale': 0.5,
            'angular_scale': 1.0
        }],
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')],
        condition=IfCondition(LaunchConfiguration('enable_teleop', default='false'))
    )
    
    # Return the launch description
    return launch.LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        debug_arg,
        left_roboclaw_port_arg,
        right_roboclaw_port_arg,
        baud_rate_arg,
        left_address_arg,
        right_address_arg,
        robot_description_arg,
        
        # Additional optional feature arguments
        DeclareLaunchArgument('enable_cameras', default_value='true',
                             description='Enable camera nodes'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
                             description='Start RViz for visualization'),
        DeclareLaunchArgument('enable_teleop', default_value='false',
                             description='Enable joystick teleop'),
        DeclareLaunchArgument('joy_device', default_value='0',
                             description='Joystick device ID'),
        
        # Core systems
        robot_state_publisher_node,
        base_launch,
        
        # Perception systems
        lidar_node,
        camera_launch,
        
        # Localization system
        localization_launch,
        
        # Navigation system
        navigation_launch,
        
        # Optional systems
        rviz_node,
        joy_node,
        teleop_node
    ])