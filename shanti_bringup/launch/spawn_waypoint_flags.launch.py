import os
import yaml
import utm
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction

def gps_to_gazebo(lat, lon, origin_lat, origin_lon, robot_x, robot_y):
    """
    Convert GPS coordinates to Gazebo world coordinates
    
    Args:
        lat (float): Waypoint latitude
        lon (float): Waypoint longitude
        origin_lat (float): World origin latitude
        origin_lon (float): World origin longitude
        robot_x (float): Robot spawn x position in Gazebo
        robot_y (float): Robot spawn y position in Gazebo
        
    Returns:
        tuple: (x, y) coordinates in Gazebo world frame
    """
    # Convert origin to UTM
    origin_easting, origin_northing, origin_zone_number, origin_zone_letter = utm.from_latlon(origin_lat, origin_lon)
    
    # Convert waypoint to UTM (ensure same zone)
    waypoint_easting, waypoint_northing, wp_zone_number, wp_zone_letter = utm.from_latlon(lat, lon)
    
    # Calculate UTM difference in meters
    gazebo_x = waypoint_easting - origin_easting
    gazebo_y = waypoint_northing - origin_northing    
    
    return gazebo_x, gazebo_y

def generate_launch_description():
    # Get package paths
    nav_bringup_pkg = get_package_share_directory('nav_bringup')
    simulation_pkg = get_package_share_directory('simulation')
    
    # Path to waypoints file
    waypoints_file = os.path.join(nav_bringup_pkg, 'params', 'gps_waypoints_competition.yaml')
    
    # Load waypoints
    with open(waypoints_file, 'r') as f:
        waypoints_data = yaml.safe_load(f)
    
    # Robot spawn position in Gazebo from simulation_bringup.launch.py
    # ToDo: pull these from source
    robot_x = -13
    robot_y = 33
    
    # World spherical coordinates from competition.world
    # ToDo: pull these from source
    origin_lat = 42.66791
    origin_lon = -83.21958
    
    # Create a node for fixed positions around the robot instead of using GPS coordinates
    spawn_nodes = []

    # Get waypoints from YAML
    waypoints = waypoints_data.get('waypoints', [])

    # Use the absolute path to the model file instead of relying on the package path
    #get the home directory
    # ToDo: pull this from nav parameters
    home_dir = os.path.expanduser('~/')
    model_path = f'{home_dir}ros2_ws/src/Shanti_2025/simulation/worlds/models/waypoint_flag/model.sdf'
    # model_path = '/home/fire/ros2_ws/src/Shanti_2025/simulation/worlds/models/waypoint_flag/model.sdf'

    # Add a delay between each spawn to give Gazebo time to process
    delay_between_spawns = 1.0  # seconds

    for i, waypoint in enumerate(waypoints):
        # Convert GPS coordinates to Gazebo coordinates
        lat = waypoint[0]
        lon = waypoint[1]
        
        x, y = gps_to_gazebo(lat, lon, origin_lat, origin_lon, robot_x, robot_y)

        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_wp_flag_{i}',
            arguments=[
                '-entity', f'waypoint_wp_flag_{i}',
                '-file', model_path,
                '-x', str(x),
                '-y', str(y),
                '-z', '6.0',
            ],
            output='screen'
        )

        timed_spawn = TimerAction(
            period=i * delay_between_spawns,
            actions=[spawn_node]
        )

        spawn_nodes.append(timed_spawn)
        spawn_nodes.append(LogInfo(msg=f'Spawning waypoint flag {i} at position x: {x}, y: {y} (from GPS: {lat}, {lon})'))

    return LaunchDescription(spawn_nodes)