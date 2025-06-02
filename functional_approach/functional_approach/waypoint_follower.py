#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, NavSatFix
from .pure_pursuit_controller import pure_pursuit
from .obstacle_avoidance import avoid_obstacles
import numpy as np
import pyproj
import math
from std_msgs.msg import Bool

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('safety_distance', 1.0)
        self.declare_parameter('use_gps_waypoints', False)
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.safety_dist = self.get_parameter('safety_distance').value
        self.use_gps_waypoints = self.get_parameter('use_gps_waypoints').value
        
        # Initialize variables
        self.current_pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.waypoints = []  # List of (x, y) tuples in map frame
        self.gps_waypoints = [(42.668201, -83.218060)]  # List of (lat, lon) tuples
        self.has_pose = False
        self.has_waypoints = False
        self.latest_pointcloud = None
        
        # GPS/map frame transformation
        self.map_origin = None  # Will be set by the first GPS reading
        self.proj = None  # Projection for GPS -> local conversion
        self.origin_set = False
        
        # Create subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10)
            
        self.path_sub = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10)
            
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
            
        self.origin_sub = self.create_subscription(
            PoseStamped,
            '/localization/origin',
            self.origin_callback,
            10)
            
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'front/filtered_cloud',
            self.pointcloud_callback,
            10)
            
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        self.transformed_path_pub = self.create_publisher(
            Path,
            '/transformed_path',
            10)
            
        self.origin_ready_pub = self.create_publisher(
            Bool,
            '/localization/origin_ready',
            10)
            
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        self.get_logger().info('Waypoint Follower node initialized with GPS waypoint support')
        
    def pose_callback(self, msg: PoseStamped):
        # Extract pose from message
        self.current_pose[0] = msg.pose.position.x
        self.current_pose[1] = msg.pose.position.y
        
        # Extract yaw from quaternion
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # Calculate yaw from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_pose[2] = np.arctan2(siny_cosp, cosy_cosp)
        
        self.has_pose = True
        
    def path_callback(self, msg: Path):
        """
        Callback for path messages containing waypoints
        
        Args:
            msg: Path message containing a list of poses
        """
        if self.use_gps_waypoints:
            # For GPS waypoints, we'll receive lat/lon in the position.x and position.y fields
            self.gps_waypoints = []
            for pose in msg.poses:
                lat = pose.pose.position.x
                lon = pose.pose.position.y
                self.gps_waypoints.append((lat, lon))
                
            self.get_logger().info(f'Received {len(self.gps_waypoints)} GPS waypoints')
            
            # If we have the origin set, transform the waypoints
            if self.origin_set:
                self.transform_gps_waypoints()
        else:
            # Regular waypoints in map frame
            self.waypoints = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                self.waypoints.append((x, y))
                
            self.has_waypoints = len(self.waypoints) > 0
            self.get_logger().info(f'Received {len(self.waypoints)} map frame waypoints')
    
    def origin_callback(self, msg: PoseStamped):
        """
        Callback for localization origin messages
        
        Args:
            msg: PoseStamped message containing the origin lat/lon in position.x and position.y
        """
        # Extract origin lat/lon
        origin_lat = msg.pose.position.x
        origin_lon = msg.pose.position.y
        
        # Initialize the map origin
        self.map_origin = {
            'lat': origin_lat,
            'lon': origin_lon
        }
        
        # Set up the projection
        self.proj = pyproj.Proj(
            proj='aeqd',
            ellps='WGS84',
            datum='WGS84',
            lat_0=self.map_origin['lat'],
            lon_0=self.map_origin['lon']
        )
        
        self.origin_set = True
        self.get_logger().info(f'Received map origin: Lat={origin_lat}, Lon={origin_lon}')
        
        # If we have GPS waypoints, transform them
        if self.use_gps_waypoints and self.gps_waypoints:
            self.transform_gps_waypoints()
    
    def gps_callback(self, msg: NavSatFix):
        """
        Callback for GPS fix messages
        
        Args:
            msg: NavSatFix message with current GPS coordinates
        """
        # If we're not using GPS waypoints or the origin is already set, ignore
        if not self.use_gps_waypoints or self.origin_set:
            return
            
        # Check if GPS data is valid
        if msg.status.status < 0:
            return
            
        # Set the first GPS reading as the origin
        if self.map_origin is None:
            self.map_origin = {
                'lat': msg.latitude,
                'lon': msg.longitude
            }
            
            # Set up the projection
            self.proj = pyproj.Proj(
                proj='aeqd',
                ellps='WGS84',
                datum='WGS84',
                lat_0=self.map_origin['lat'],
                lon_0=self.map_origin['lon']
            )
            
            self.origin_set = True
            self.get_logger().info(f'Set map origin from GPS: Lat={msg.latitude}, Lon={msg.longitude}')
            
            # Publish that we have the origin
            msg = Bool()
            msg.data = True
            self.origin_ready_pub.publish(msg)
            
            # If we have GPS waypoints, transform them
            if self.gps_waypoints:
                self.transform_gps_waypoints()
    
    def transform_gps_waypoints(self):
        """
        Transform GPS waypoints to local map frame
        """
        if not self.proj or not self.gps_waypoints:
            return
            
        self.waypoints = []
        for lat, lon in self.gps_waypoints:
            # Convert lat/lon to East-North-Up (ENU) coordinates in meters
            east, north = self.proj(lon, lat)
            self.waypoints.append((east, north))
            
        self.has_waypoints = len(self.waypoints) > 0
        self.get_logger().info(f'Transformed {len(self.waypoints)} GPS waypoints to map frame')
        
        # Publish the transformed path for visualization
        self.publish_transformed_path()
    
    def publish_transformed_path(self):
        """
        Publish the transformed waypoints as a Path message
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Default orientation (identity quaternion)
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
            
        self.transformed_path_pub.publish(path_msg)
        
    def pointcloud_callback(self, msg: PointCloud2):
        # Store the latest point cloud message
        self.latest_pointcloud = msg
        
    def control_loop(self):
        if not self.has_pose or not self.has_waypoints:
            return
            
        # Calculate steering angle using pure pursuit
        steering_angle = pure_pursuit(
            self.current_pose,
            self.waypoints,
            self.lookahead
        )
        
        # Adjust steering angle based /on obstacle avoidance if pointcloud data is available
        if self.latest_pointcloud is not None:
            steering_angle = avoid_obstacles(
                self.latest_pointcloud,
                steering_angle,
                self.safety_dist
            )
        
        # Create velocity command
        cmd = Twist()
        
        # Base forward velocity (could be dynamic based on curvature)
        forward_speed = 0.5  # m/s
        
        # Reduce speed when turning
        scaling_factor = 1.0 - 0.5 * abs(steering_angle / 0.5)
        cmd.linear.x = forward_speed * scaling_factor
        
        # Convert steering angle to angular velocity for differential drive
        # For a differential drive robot, angular velocity is proportional to steering angle
        # and inversely proportional to wheelbase
        wheelbase = 1.0  # meters
        cmd.angular.z = cmd.linear.x * np.tan(steering_angle) / wheelbase
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
