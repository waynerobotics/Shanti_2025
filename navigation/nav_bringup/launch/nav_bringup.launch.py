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
            parameters=[nav2_params_path],
            remappings=[
                ('/cmd_vel', '/demo/cmd_vel'),
                ('/odom', '/odometry/map'),
            ]
        ),

        # Nav2 Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_path],
        ),

        # Nav2 Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_path],
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_path],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/odom', '/odometry/map'),
            ]
        ),

        # Nav2 Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_path],
            remappings=[
                ('/cmd_vel', '/demo/cmd_vel'),
            ]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'behavior_server',  # Changed from recovery_server to behavior_server
                ]},
            ]
        ),

        # Include GPS Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('localization_bringup'), 'launch', 'dual_ekf_navsat.launch.py')
            )
        ),

        # Waypoint Publisher - launched with a delay to ensure waypoint follower is active
        Node(
            package='nav_bringup',
            executable='waypoint_publisher',
            name='waypoint_publisher',
            output='screen',
            # Add a delay before launching to ensure waypoint follower is fully active
            prefix=['bash -c "sleep 10.0 && exec $0 $@"'],
        ),
    ])
