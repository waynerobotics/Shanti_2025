from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch parameters with defaults
    left_roboclaw_port = LaunchConfiguration('left_roboclaw_port', default='/dev/ttyACM0')
    right_roboclaw_port = LaunchConfiguration('right_roboclaw_port', default='/dev/ttyACM1')
    baud_rate = LaunchConfiguration('baud_rate', default='38400')
    left_address = LaunchConfiguration('left_address', default='128')  # 0x80
    right_address = LaunchConfiguration('right_address', default='128')  # 0x80
    wheel_base = LaunchConfiguration('wheel_base', default='0.5334')  # meters
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.1016')  # meters
    max_speed = LaunchConfiguration('max_speed', default='1.0')  # m/s
    max_angular_speed = LaunchConfiguration('max_angular_speed', default='1.5')  # rad/s
    encoder_cpr = LaunchConfiguration('encoder_cpr', default='4096')
    debug_level = LaunchConfiguration('debug_level', default='1')
    timeout = LaunchConfiguration('timeout', default='0.1')  # seconds
    retries = LaunchConfiguration('retries', default='3')  # number of retries
    
    # Serial port for encoders (separate from Roboclaw controllers)
    encoder_serial_port = LaunchConfiguration('encoder_serial_port', default='/dev/ttyACM2')
    encoder_baud_rate = LaunchConfiguration('encoder_baud_rate', default='115200')
    
    # PID parameters
    pid_kp = LaunchConfiguration('pid_kp', default='1.0')
    pid_ki = LaunchConfiguration('pid_ki', default='0.0')
    pid_kd = LaunchConfiguration('pid_kd', default='0.0')
    
    # Motor direction configuration
    left_motor_direction = LaunchConfiguration('left_motor_direction', default='1')
    right_motor_direction = LaunchConfiguration('right_motor_direction', default='1')
    
    # Launch Arguments
    launch_args = [
        # Roboclaw communication parameters
        DeclareLaunchArgument('left_roboclaw_port', 
                              default_value='/dev/ttyACM0',
                              description='Serial port for the left Roboclaw controller'),
        DeclareLaunchArgument('right_roboclaw_port', 
                              default_value='/dev/ttyACM1',
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
                              default_value='1.0',
                              description='Maximum linear speed in m/s'),
        DeclareLaunchArgument('max_angular_speed', 
                              default_value='1.5',
                              description='Maximum angular speed in rad/s'),
        DeclareLaunchArgument('encoder_cpr', 
                              default_value='4096',
                              description='Encoder counts per revolution'),
        DeclareLaunchArgument('debug_level', 
                              default_value='1',
                              description='Debug level: 0=minimal, 1=normal, 2=verbose'),
        DeclareLaunchArgument('timeout', 
                              default_value='0.1',
                              description='Timeout for Roboclaw communication in seconds'),
        DeclareLaunchArgument('retries', 
                              default_value='3',
                              description='Number of retries for Roboclaw communication'),
                              
        # Encoder odometry parameters
        DeclareLaunchArgument('encoder_serial_port', 
                              default_value='/dev/ttyACM2',
                              description='Serial port for encoder readings'),
        DeclareLaunchArgument('encoder_baud_rate', 
                              default_value='115200',
                              description='Baud rate for encoder serial communication'),
                              
        # PID parameters
        DeclareLaunchArgument('pid_kp', 
                              default_value='1.0',
                              description='Proportional gain for PID controller'),
        DeclareLaunchArgument('pid_ki', 
                              default_value='0.0',
                              description='Integral gain for PID controller'),
        DeclareLaunchArgument('pid_kd', 
                              default_value='0.0',
                              description='Derivative gain for PID controller'),
                              
        # Motor direction configuration
        DeclareLaunchArgument('left_motor_direction', 
                              default_value='1',
                              description='Direction multiplier for left motor (1 or -1)'),
        DeclareLaunchArgument('right_motor_direction', 
                              default_value='1',
                              description='Direction multiplier for right motor (1 or -1)'),
    ]
    
    # Define node actions
    roboclaw_controller_node = Node(
        package='differential_drive_base',
        executable='roboclaw_controller',
        name='roboclaw_controller_node',
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
            'encoder_cpr': encoder_cpr,
            'debug_level': debug_level,
            'timeout': timeout,
            'retries': retries,
            'pid_kp': pid_kp,
            'pid_ki': pid_ki,
            'pid_kd': pid_kd,
            'left_motor_direction': left_motor_direction,
            'right_motor_direction': right_motor_direction,
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
            'encoder_resolution': encoder_cpr,
            'serial_port': encoder_serial_port,
            'baud_rate': encoder_baud_rate,
            'debug_level': debug_level,
        }],
    )
    
    # Return the LaunchDescription object with all entities to launch
    return LaunchDescription(
        launch_args + 
        [
            roboclaw_controller_node,
            encoder_odometry_node,
        ]
    )