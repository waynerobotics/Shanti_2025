#!/usr/bin/env python3
"""
Launch file for the closed-loop RoboClaw controller.
This provides more precise motor control using encoder feedback.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments
    roboclaw_port_arg = DeclareLaunchArgument(
        'roboclaw_port',
        default_value='/dev/ttyACM0',
        description='Port for the RoboClaw controller'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='38400',
        description='Baud rate for RoboClaw communication'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='128',  # 0x80
        description='Address for the RoboClaw controller'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.5334',
        description='Distance between wheels in meters'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1016',
        description='Wheel radius in meters'
    )
    
    encoder_cpr_arg = DeclareLaunchArgument(
        'encoder_cpr',
        default_value='1024',
        description='Encoder counts per revolution'
    )
    
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='9.0',
        description='Motor gear ratio'
    )
    
    pid_p_arg = DeclareLaunchArgument(
        'pid_p',
        default_value='1.36',
        description='PID Proportional gain'
    )
    
    pid_i_arg = DeclareLaunchArgument(
        'pid_i',
        default_value='0.08',
        description='PID Integral gain'
    )
    
    pid_d_arg = DeclareLaunchArgument(
        'pid_d',
        default_value='0.0',
        description='PID Derivative gain'
    )
    
    pid_qpps_arg = DeclareLaunchArgument(
        'pid_qpps',
        default_value='30000',
        description='PID maximum encoder counts per second'
    )
    
    debug_level_arg = DeclareLaunchArgument(
        'debug_level',
        default_value='1',
        description='Debug level (0=INFO, 1=DEBUG, 2=VERBOSE)'
    )
    
    telemetry_rate_arg = DeclareLaunchArgument(
        'telemetry_rate',
        default_value='5.0',
        description='Telemetry update rate in Hz'
    )
    
    # Nodes
    closed_loop_controller_node = Node(
        package='differential_drive_base',
        executable='closed_loop_controller',
        name='closed_loop_controller',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('roboclaw_port'),
            'baudrate': LaunchConfiguration('baud_rate'),
            'address': LaunchConfiguration('address'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'encoder_cpr': LaunchConfiguration('encoder_cpr'),
            'gear_ratio': LaunchConfiguration('gear_ratio'),
            'pid_p': LaunchConfiguration('pid_p'),
            'pid_i': LaunchConfiguration('pid_i'),
            'pid_d': LaunchConfiguration('pid_d'),
            'pid_qpps': LaunchConfiguration('pid_qpps'),
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 1.5,
            'cmd_timeout': 0.5,
            'invert_left': False,
            'invert_right': True,
            'debug_level': LaunchConfiguration('debug_level'),
            'telemetry_rate': LaunchConfiguration('telemetry_rate')
        }]
    )
    
    # Launch joystick node 
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
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
        roboclaw_port_arg,
        baud_rate_arg,
        address_arg,
        wheel_base_arg,
        wheel_radius_arg,
        encoder_cpr_arg,
        gear_ratio_arg,
        pid_p_arg,
        pid_i_arg,
        pid_d_arg,
        pid_qpps_arg,
        debug_level_arg,
        telemetry_rate_arg,
        
        # Nodes
        closed_loop_controller_node,
        joy_node,
        joy2twist_node
    ])
