#!/usr/bin/env python3

import math
import pyproj
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster

class GPSIMULocalization(Node):
    def __init__(self):
        super().__init__('gps_imu_localization')
        
        # Initialize variables
        self.origin = None
        self.proj = None
        self.current_pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.using_gnss_pose = True
        self.map_origin = None  # First GNSS pose will be set as origin (0,0)
        self.count_publications = 0  # Counter for logging frequency control
        
        # Declare parameters
        self.declare_parameter('use_tf', True)
        self.use_tf = self.get_parameter('use_tf').value
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create subscribers
        # Primary GNSS pose subscriber
        self.gnss_pose_sub = self.create_subscription(
            PoseStamped,
            '/gnss_pose',
            self.gnss_pose_callback,
            10)
            
        # Fallback to separate GPS/IMU if needed
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
            
        # Create publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/localization/pose',
            10)
            
        self.origin_pub = self.create_publisher(
            PoseStamped,
            '/localization/origin',
            10)
            
        self.get_logger().info('GPS/IMU Localization node initialized with GNSS pose support')
        
    def gnss_pose_callback(self, msg: PoseStamped):
        """
        Callback function for GNSS pose messages
        
        Args:
            msg: PoseStamped message with GNSS pose data
        """
        # Extract lat/lon coordinates from the GNSS pose
        lat = msg.pose.position.x
        lon = msg.pose.position.y
        
        # Set the first GNSS pose as the origin if not set
        if self.map_origin is None:
            self.map_origin = {
                'lat': lat,
                'lon': lon,
                'yaw': 0.0  # We'll set the initial heading as the 0-angle
            }
            
            # Set up the projection using PyProj for accurate lat/lon to meters conversion
            # Azimuthal Equidistant projection centered at the origin
            self.proj = pyproj.Proj(
                proj='aeqd',  # Azimuthal Equidistant projection
                ellps='WGS84',  # WGS84 ellipsoid
                datum='WGS84',
                lat_0=self.map_origin['lat'],  # Center latitude
                lon_0=self.map_origin['lon']   # Center longitude
            )
            
            # Extract yaw from the first pose quaternion
            quaternion = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            _, _, initial_yaw = euler_from_quaternion(quaternion)
            self.map_origin['yaw'] = initial_yaw
            
            self.get_logger().info(f'Map origin set to: Lat={self.map_origin["lat"]}, Lon={self.map_origin["lon"]}, Yaw={self.map_origin["yaw"]}')
            
            # Publish the origin for other nodes
            self.publish_origin()
        
        # Convert lat/lon to East-North-Up (ENU) coordinates in meters
        # The pyproj.Proj forward transformation returns (easting, northing) in meters
        east, north = self.proj(lon, lat)
        
        # Extract orientation from quaternion
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, yaw = euler_from_quaternion(quaternion)
        
        # Calculate position in map frame (rotate by initial yaw)
        cos_origin_yaw = math.cos(-self.map_origin['yaw'])
        sin_origin_yaw = math.sin(-self.map_origin['yaw'])
        
        # Rotate the point around the origin
        map_x = east * cos_origin_yaw - north * sin_origin_yaw
        map_y = east * sin_origin_yaw + north * cos_origin_yaw
        
        # Calculate heading relative to the map frame
        map_yaw = yaw - self.map_origin['yaw']
        # Normalize to [-pi, pi]
        map_yaw = (map_yaw + math.pi) % (2 * math.pi) - math.pi
        
        # Update current pose
        self.current_pose[0] = map_x
        self.current_pose[1] = map_y
        self.current_pose[2] = map_yaw
        
        # Publish updated pose
        self.publish_pose()
        
        # Broadcast transform if enabled
        if self.use_tf:
            self.broadcast_transform(msg.header.stamp)
        
        # Publish updated pose
        self.publish_pose()
        
        # Broadcast transform if enabled
        if self.use_tf:
            self.broadcast_transform(msg.header.stamp)
        
    def gps_callback(self, msg: NavSatFix):
        """
        Callback function for GPS messages (fallback if GNSS pose is not available)
        
        Args:
            msg: NavSatFix message with GPS data
        """
        # Skip if we're using GNSS pose and already have a valid pose
        if self.using_gnss_pose and self.map_origin is not None:
            return
            
        # Check if GPS data is valid
        if msg.status.status < 0:  # No fix
            self.get_logger().warn('No GPS fix available')
            return
            
        # Set origin with first valid GPS reading
        if self.map_origin is None:
            self.map_origin = {
                'lat': msg.latitude,
                'lon': msg.longitude,
                'yaw': 0.0  # Initial heading
            }
            
            # Set up the projection using PyProj for accurate lat/lon to meters conversion
            self.proj = pyproj.Proj(
                proj='aeqd',  # Azimuthal Equidistant projection
                ellps='WGS84',  # WGS84 ellipsoid
                datum='WGS84',
                lat_0=self.map_origin['lat'],  # Center latitude
                lon_0=self.map_origin['lon']   # Center longitude
            )
            
            self.get_logger().info(f'Map origin set to: Lat={self.map_origin["lat"]}, Lon={self.map_origin["lon"]}, Yaw=0.0')
        
        # Convert lat/lon to East-North-Up (ENU) coordinates in meters
        east, north = self.proj(msg.longitude, msg.latitude)
        
        # In this case, we're not rotating by initial yaw since we don't have it
        # East and North directly correspond to x and y in the map
        self.current_pose[0] = east
        self.current_pose[1] = north
        
        # Yaw will be set by IMU data or remain 0 if no IMU data is available
        
        # Publish updated pose
        self.publish_pose()
        
        # Broadcast transform if enabled
        if self.use_tf:
            self.broadcast_transform(msg.header.stamp)
        
    def publish_origin(self):
        """
        Publish the map origin as a PoseStamped message
        """
        if self.map_origin is None:
            return
            
        origin_msg = PoseStamped()
        origin_msg.header.stamp = self.get_clock().now().to_msg()
        origin_msg.header.frame_id = 'map'
        
        # Put lat/lon in the position fields
        origin_msg.pose.position.x = self.map_origin['lat']
        origin_msg.pose.position.y = self.map_origin['lon']
        origin_msg.pose.position.z = 0.0
        
        # Put the initial heading in the orientation
        quat = quaternion_from_euler(0.0, 0.0, self.map_origin['yaw'])
        origin_msg.pose.orientation.x = quat[0]
        origin_msg.pose.orientation.y = quat[1]
        origin_msg.pose.orientation.z = quat[2]
        origin_msg.pose.orientation.w = quat[3]
        
        # Publish the origin
        self.origin_pub.publish(origin_msg)
        self.get_logger().info('Published map origin')
        
    def publish_pose(self):
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Set position
        pose_msg.pose.position.x = self.current_pose[0]
        pose_msg.pose.position.y = self.current_pose[1]
        pose_msg.pose.position.z = 0.0
        
        # Set orientation (convert yaw to quaternion)
        quat = quaternion_from_euler(0.0, 0.0, self.current_pose[2])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # Log the current pose at a lower frequency
        if self.count_publications % 10 == 0:
            self.get_logger().info(f'Publishing pose: x={self.current_pose[0]:.2f}, y={self.current_pose[1]:.2f}, yaw={self.current_pose[2]:.2f}')
        self.count_publications += 1
        
        # Publish the pose
        self.pose_pub.publish(pose_msg)
        
    def broadcast_transform(self, timestamp):
        # Create transform message
        t = TransformStamped()
        
        # Set header
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # Set translation
        t.transform.translation.x = self.current_pose[0]
        t.transform.translation.y = self.current_pose[1]
        t.transform.translation.z = 0.0
        
        # Set rotation
        quat = quaternion_from_euler(0.0, 0.0, self.current_pose[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GPSIMULocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
