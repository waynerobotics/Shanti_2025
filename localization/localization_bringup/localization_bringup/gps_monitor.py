#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import math

# Try to import utm, but provide fallback if not available
try:
    import utm
    UTM_AVAILABLE = True
except ImportError:
    UTM_AVAILABLE = False
    print("UTM package not available. GPS coordinates will not be converted to UTM.")

class GPSMonitor(Node):
    """
    Node to monitor GPS data and corresponding odometry data.
    Displays the relationship between GPS coordinates and map coordinates.
    """

    def __init__(self):
        super().__init__('gps_monitor')
        
        # Declare parameters
        self.declare_parameter('gps_topic', '/gnss_pose')
        self.gps_topic = self.get_parameter('gps_topic').value
        
        # Create subscribers
        self.gps_pose_sub = self.create_subscription(
            PoseStamped,
            self.gps_topic,
            self.gps_pose_callback,
            10
        )
        
        # Also subscribe to regular NavSatFix if available
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gnss',
            self.gps_callback,
            10
        )
        
        self.filtered_gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/filtered',
            self.filtered_gps_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Initialize data storage
        self.latest_gps = None
        self.latest_gps_pose = None
        self.latest_filtered_gps = None
        self.latest_odom = None
        
        # Set up tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create timer for status updates
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('GPS Monitor node initialized')
    
    def gps_callback(self, msg):
        """Callback for raw GPS data."""
        self.latest_gps = msg
    
    def filtered_gps_callback(self, msg):
        """Callback for filtered GPS data."""
        self.latest_filtered_gps = msg
    
    def odom_callback(self, msg):
        """Callback for odometry data."""
        self.latest_odom = msg
    
    def gps_pose_callback(self, msg):
        """Callback for GNSS pose data (PoseStamped format)."""
        self.latest_gps_pose = msg
    
    def publish_status(self):
        """Publish status information to console."""
        if self.latest_gps_pose:
            self.get_logger().info(f"GNSS Pose Data:")
            self.get_logger().info(f"  X: {self.latest_gps_pose.pose.position.x:.2f}, Y: {self.latest_gps_pose.pose.position.y:.2f}, Z: {self.latest_gps_pose.pose.position.z:.2f}")
            self.get_logger().info(f"  Frame: {self.latest_gps_pose.header.frame_id}")
        elif self.latest_gps:
            lat = self.latest_gps.latitude
            lon = self.latest_gps.longitude
            
            self.get_logger().info(f"GPS Data:")
            self.get_logger().info(f"  Lat: {lat:.7f}, Lon: {lon:.7f}")
            
            # Convert GPS to UTM for display if the UTM package is available
            if UTM_AVAILABLE:
                try:
                    easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
                    self.get_logger().info(f"  UTM: E: {easting:.2f}, N: {northing:.2f}, Zone: {zone_number}{zone_letter}")
                except Exception as e:
                    self.get_logger().warn(f"Error converting to UTM: {e}")
        else:
            self.get_logger().warn("No GPS data received")
        
        if self.latest_filtered_gps:
            # Convert filtered GPS to UTM for display
            lat = self.latest_filtered_gps.latitude
            lon = self.latest_filtered_gps.longitude
            
            self.get_logger().info(f"Filtered GPS Data:")
            self.get_logger().info(f"  Lat: {lat:.7f}, Lon: {lon:.7f}")
            
            if UTM_AVAILABLE:
                try:
                    easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
                    self.get_logger().info(f"  UTM: E: {easting:.2f}, N: {northing:.2f}, Zone: {zone_number}{zone_letter}")
                except Exception as e:
                    self.get_logger().warn(f"Error converting filtered GPS to UTM: {e}")
        
        if self.latest_odom:
            self.get_logger().info(f"Filtered Odometry Position:")
            self.get_logger().info(f"  X: {self.latest_odom.pose.pose.position.x:.2f}, Y: {self.latest_odom.pose.pose.position.y:.2f}")
        
        # Check for transform availability
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.get_logger().info(f"Transform map->base_link: X: {transform.transform.translation.x:.2f}, Y: {transform.transform.translation.y:.2f}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform map->base_link not available: {e}")
        
        self.get_logger().info("----------------------------------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = GPSMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
