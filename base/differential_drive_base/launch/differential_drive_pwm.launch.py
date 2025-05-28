from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch parameters with defaults
    left_roboclaw_port = LaunchConfiguration('left_roboclaw_port', default='/dev/ttyACM2')
    right_roboclaw_port = LaunchConfiguration('right_roboclaw_port', default='/dev/ttyACM3')
    baud_rate = LaunchConfiguration('baud_rate', default='38400')
    left_address = LaunchConfiguration('left_address', default='128')  # 0x80
    right_address = LaunchConfiguration('right_address', default='128')  # 0x80
    wheel_base = LaunchConfiguration('wheel_base', default='0.5334')  # meters
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.1016')  # meters
    max_speed = LaunchConfiguration('max_speed', default='1.0')  # m/s
    max_angular_speed = LaunchConfiguration('max_angular_speed', default='1.5')  # rad/s
    max_pwm = LaunchConfiguration('max_pwm', default='127')  # maximum PWM value
    min_pwm = LaunchConfiguration('min_pwm', default='20')  # minimum PWM value
    pwm_deadband = LaunchConfiguration('pwm_deadband', default='0.05')  # 5% deadband
    debug_level = LaunchConfiguration('debug_level', default='1')
    
    # Serial port for encoders (separate from Roboclaw controllers)
    encoder_serial_port = LaunchConfiguration('encoder_serial_port', default='/dev/ttyACM2')
    encoder_baud_rate = LaunchConfiguration('encoder_baud_rate', default='115200')
    encoder_resolution = LaunchConfiguration('encoder_resolution', default='4096')

  

    
    # Launch Arguments
    launch_args = [
        # Roboclaw communication parameters
        DeclareLaunchArgument('left_roboclaw_port', 
                              default_value='/dev/ttyACM2',
                              description='Serial port for the left Roboclaw controller'),
        DeclareLaunchArgument('right_roboclaw_port', 
                              default_value='/dev/ttyACM3',
                              description='Serial port for the right Roboclaw controller'),
        DeclareLaunchArgument('baud_rate', 
                              default_value='38400',
                              description='Baud rate for Roboclaw communication'),
        DeclareLaunchArgument('left_address', 
                              default_value='128',
                              description='Address of the left Roboclaw controller (decimal)'),
        DeclareLaunchArgument('right_address', 
                              default_value='128',
                              description='Address of the right Roboclaw controller (decimal)'),
                              
        # Robot physical parameters
        DeclareLaunchArgument('wheel_base', 
                              default_value='0.5334',
                              description='Distance between wheels in meters'),
        DeclareLaunchArgument('wheel_radius', 
                              default_value='0.1016',
                              description='Wheel radius in meters'),
                              
        # Controller parameters
        DeclareLaunchArgument('max_speed', 
                              default_value='2.1',
                              description='Maximum linear speed in m/s'),
        DeclareLaunchArgument('max_angular_speed', 
                              default_value='1.5',
                              description='Maximum angular speed in rad/s'),
        DeclareLaunchArgument('max_pwm', 
                              default_value='127',
                              description='Maximum PWM value (0-127)'),
        DeclareLaunchArgument('min_pwm', 
                              default_value='20',
                              description='Minimum PWM value to overcome motor stiction'),
        DeclareLaunchArgument('pwm_deadband', 
                              default_value='0.05',
                              description='Percentage of max speed below which motors are stopped'),
        DeclareLaunchArgument('debug_level', 
                              default_value='1',
                              description='Debug level: 0=minimal, 1=normal, 2=verbose'),
                              
        # Encoder odometry parameters
        DeclareLaunchArgument('encoder_serial_port', 
                              default_value='/dev/ttyACM2',
                              description='Serial port for encoder readings'),
        DeclareLaunchArgument('encoder_baud_rate', 
                              default_value='115200',
                              description='Baud rate for encoder serial communication'),
        DeclareLaunchArgument('encoder_resolution', 
                              default_value='4096',
                              description='Encoder counts per revolution'),
                
        
    ]
    
    # Define node actions
    roboclaw_pwm_controller_node = Node(
        package='differential_drive_base',
        executable='roboclaw_pwm_controller',
        name='roboclaw_pwm_controller_node',
        output='screen',
        parameters=[{
            'left_roboclaw_port': left_roboclaw_port,
            'right_roboclaw_port': right_roboclaw_port,
            'baud_rate': baud_rate,
            'left_address': left_address,
            'right_address': right_address,
            'wheel_base': wheel_base,
            'wheel_radius': wheel_radius,
            'max_speed': max_speed,
            'max_angular_speed': max_angular_speed,
            'max_pwm': max_pwm,
            'min_pwm': min_pwm,
            'pwm_deadband': pwm_deadband,
            'invert_right_motors': LaunchConfiguration('left_right_direction', default=True),
            'debug_level': debug_level,
        }],
    )
    
    encoder_odometry_node = Node(
        package='differential_drive_base',
        executable='encoder_odom',
        name='encoder_odometry_node',
        output='screen',
        parameters=[{
            'wheel_base': wheel_base,
            'wheel_radius': wheel_radius,
            # Convert string to integer for encoder resolution
            'encoder_resolution': 4096,  # Use integer instead of string
            'serial_port': encoder_serial_port,
            'baud_rate': encoder_baud_rate,
            'debug_level': debug_level,
        }],
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
    
    
    # Return the LaunchDescription object with all entities to launch
    return LaunchDescription(
        launch_args + 
        [
            roboclaw_pwm_controller_node,
            #        encoder_odometry_node,
            joy_node,
            joy2twist_node,
        ]
    )
