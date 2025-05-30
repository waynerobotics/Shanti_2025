from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments
    left_roboclaw_port_arg = DeclareLaunchArgument(
        'left_roboclaw_port',
        default_value='/dev/ttyACM0',
        description='Port for the left RoboClaw controller'
    )
    
    right_roboclaw_port_arg = DeclareLaunchArgument(
        'right_roboclaw_port',
        default_value='/dev/ttyACM2',
        description='Port for the right RoboClaw controller'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='38400',
        description='Baud rate for RoboClaw communication'
    )
    
    left_address_arg = DeclareLaunchArgument(
        'left_address',
        default_value='128',  # 0x80
        description='Address for the left RoboClaw controller'
    )
    
    right_address_arg = DeclareLaunchArgument(
        'right_address',
        default_value='128',  # 0x80
        description='Address for the right RoboClaw controller'
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
    
    debug_level_arg = DeclareLaunchArgument(
        'debug_level',
        default_value='1',
        description='Debug level (0=INFO, 1=DEBUG, 2=VERBOSE)'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='1.',
        description='Control rate in seconds (5Hz = 0.2s)'
    )
    
    # Nodes
    left_motor_controller_node = Node(
        package='differential_drive_base',
        executable='single_roboclaw_controller',
        name='left_motor_controller',
        output='screen',
        parameters=[{
            'roboclaw_port': LaunchConfiguration('left_roboclaw_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'address': LaunchConfiguration('left_address'),
            'debug_level': LaunchConfiguration('debug_level'),
            'pwm_topic_prefix': 'left_motor_controller',
            'motor1_invert': False,
            'motor2_invert': True,
            'control_rate': LaunchConfiguration('control_rate'),
            'heartbeat_interval': LaunchConfiguration('control_rate')
        }]
    )
    
    right_motor_controller_node = Node(
        package='differential_drive_base',
        executable='single_roboclaw_controller',
        name='right_motor_controller',
        output='screen',
        parameters=[{
            'roboclaw_port': LaunchConfiguration('right_roboclaw_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'address': LaunchConfiguration('right_address'),
            'debug_level': LaunchConfiguration('debug_level'),
            'pwm_topic_prefix': 'right_motor_controller',
            'motor1_invert': True,
            'motor2_invert': True,
            'control_rate': LaunchConfiguration('control_rate'),
            'heartbeat_interval': LaunchConfiguration('control_rate')
        }]
    )
    
    differential_drive_node = Node(
        package='differential_drive_base',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        output='screen',
        parameters=[{
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'max_speed': 1.0,
            'max_angular_speed': 1.5,
            'max_pwm': 32767,
            'min_pwm': 5000,
            'cmd_timeout': 0.5,
            'pwm_deadband': 0.05,
            'debug_level': LaunchConfiguration('debug_level'),
            'control_rate': LaunchConfiguration('control_rate'),
            'left_motor_controller_prefix': 'left_motor_controller',
            'right_motor_controller_prefix': 'right_motor_controller',
            'left_front_motor_number': 1,
            'left_rear_motor_number': 2,
            'right_front_motor_number': 1,
            'right_rear_motor_number': 2
        }]
    )

        # Launch joystick node 
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        
    )

    # Launch joystick axis to twist message conversion 
    joy2twist_node = Node(
        package='joystick2base',
        executable='joy2twist',
            
        # Remap to Gazebo diff drive topic
        remappings=[
            ('/turtle1/cmd_vel', '/cmd_vel')
        ],
        parameters=[{
            'linear_axis': 1, 
            'angular_axis': 0, # 0 for yoke, 3 for joystick
        }],
        
    )
    
    return LaunchDescription([
        # Arguments
        left_roboclaw_port_arg,
        right_roboclaw_port_arg,
        baud_rate_arg,
        left_address_arg,
        right_address_arg,
        wheel_base_arg,
        wheel_radius_arg,
        debug_level_arg,
        control_rate_arg,
        
        # Nodes
        left_motor_controller_node,
        right_motor_controller_node,
        differential_drive_node,
        joy_node,
        joy2twist_node,
    ])
