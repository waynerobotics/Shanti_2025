import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('nav_bringup')
    
    # Create the absolute path to the parameters file
    nav2_params_path = os.path.join(pkg_dir, 'params', 'nav2_params.yaml')
    
    # Print the parameters file path for debugging
    print(f"Loading navigation parameters from: {nav2_params_path}")

    # Declare launch arguments - keep for compatibility but use absolute path
    params_file = LaunchConfiguration('params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use')
    
    # Define explicit costmap parameters to override YAML settings
    costmap_params = {
        'local_costmap.static_layer.enabled': False,
        'local_costmap.plugins': ['obstacle_layer', 'inflation_layer'],
        'local_costmap.rolling_window': True,
        'global_costmap.static_layer.enabled': False,
        'global_costmap.plugins': ['obstacle_layer', 'inflation_layer'],
        'global_costmap.rolling_window': True,
    }

    return LaunchDescription([
        declare_params_file_cmd,
        
        # Log info about the parameters file
        LogInfo(msg=['Navigation parameters file: ', nav2_params_path]),

        # Nav2 Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_path, costmap_params],  # Add costmap overrides
            remappings=[
                ('/cmd_vel', '/cmd_vel_relay'),
                ('/odom', '/odometry/map'),
            ]
        ),

        # Nav2 Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_path, costmap_params],  # Add costmap overrides
        ),

        # Nav2 Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_path, {'waypoint_follower.loop_rate': 20.0}],  # Ensure loop_rate is a float
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': [
                            'controller_server',
                            'planner_server',
                            'waypoint_follower'
                        ]}]
        ),

        # Waypoint Publisher
        Node(
            package='nav_bringup',
            executable='waypoint_publisher',
            name='waypoint_publisher',
            output='screen'
        ),

        # Include GPS Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('localization_bringup'), 'launch', 'dual_ekf_navsat.launch.py')
            )
        ),
    ])
