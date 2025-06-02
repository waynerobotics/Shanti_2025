#!/usr/bin/env python3
# filepath: /home/shanti/ros2_ws/src/Shanti_2025/base/differential_drive_base/launch/diff_drive_please.launch.py
"""
Launch file for the differential drive controller with joystick control.
This launches:
1. Left and right RoboClaw controllers
2. Differential drive controller
3. Joystick node
4. Joystick to twist converter
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )
    
    left_roboclaw_port_arg = DeclareLaunchArgument(
        'left_roboclaw_port',
        default_value='/dev/ttyACM0',
        description='Port for the left RoboClaw controller'
    )

    right_roboclaw_port_arg = DeclareLaunchArgument(
        'right_roboclaw_port',
        default_value='/dev/ttyACM1',
        description='Port for the right RoboClaw controller'
    )

    # Paths
    pkg_share = FindPackageShare('differential_drive_base')
    diff_drive_launch_file = PathJoinSubstitution([pkg_share, 'launch', 'differential_drive_launch.py'])
    
    # Include the differential drive launch file
    diff_drive_please_left = Node(
        package='differential_drive_base',
        executable='diff_drive_please',
        name='diff_drive_please_node_left',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('left_roboclaw_port'),
            'wheel_base': 0.5,  # Example value, adjust as needed
            'wheel_radius': 0.1,  # Example value, adjust as needed
            'max_speed': 1.0,  # Example value, adjust as needed
            'max_angular_speed': 1.5,  # Example value, adjust as needed
            'max_pwm': 32767,
            'min_pwm': 5000,
            'cmd_timeout': 0.5,
            'pwm_deadband': 0.05,
            'debug_level': 1,
        }],
    )


    diff_drive_please_right = Node(
        package='differential_drive_base',
        executable='diff_drive_please',
        name='diff_drive_please_node_right',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('right_roboclaw_port'),
            'wheel_radius': 0.1,  # Example value, adjust as needed
            'max_speed': 1.0,  # Example value, adjust as needed
            'max_angular_speed': 1.5,  # Example value, adjust as needed
            'max_pwm': 32767,
            'min_pwm': 5000,
            'cmd_timeout': 0.5,
            'pwm_deadband': 0.05,
            'debug_level': 1,
        }],
    )

    # Launch joystick node 
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    # Launch joystick axis to twist message conversion 
    joy2twist_node = Node(
        package='joystick2base',
        executable='joy2twist',
        name='joy2twist_node',
        output='screen',
        # Remap to cmd_vel topic
        remappings=[
            ('/turtle1/cmd_vel', '/cmd_vel')
        ],
        parameters=[{
            'linear_axis': 1, 
            'angular_axis': 0, # 0 for yoke, 3 for joystick
            'scale_linear': 1.0,
            'scale_angular': 1.0,
        }]
    )
    
    return LaunchDescription([
        # Arguments
        joy_dev_arg,
        left_roboclaw_port_arg,
        right_roboclaw_port_arg,
        
        # Launch files
        diff_drive_please_left,
        diff_drive_please_right,

        # Nodes
        joy_node,
        joy2twist_node,
    ])