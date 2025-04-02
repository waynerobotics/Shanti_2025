#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import utm
import numpy as np
from tf2_ros import StaticTransformBroadcaster
import math
from std_srvs.srv import Trigger  # Import the Trigger service


class UtmMapTransformPublisher(Node):
    """
    ROS2 node to dynamically create a static transform publisher for UTM to map transform.
    
    This node listens to GPS and IMU data, then calculates the appropriate transform 
    between UTM and map frames based on the robot's initial position and orientation.
    Publishes zeroes until an odometry/map message is received.
    """

    def __init__(self):
        super().__init__('utm_map_transform_publisher')
        
        # Declare parameters - don't declare use_sim_time, let ROS handle it
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('imu_topic', '/demo/imu')
        self.declare_parameter('map_odom_topic', '/odometry/map')  # Changed to map odometry
        self.declare_parameter('transform_timeout', 5.0)  # seconds to wait for data
        self.declare_parameter('datum_latitude', 38.1614789)  # default datum from config
        self.declare_parameter('datum_longitude', -122.454630)  # default datum from config
        self.declare_parameter('datum_altitude', 0.0)  # default datum height
        self.declare_parameter('publish_period', 1.0)  # How often to publish the transform
        
        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_odom_topic = self.get_parameter('map_odom_topic').value
        self.transform_timeout = self.get_parameter('transform_timeout').value
        self.datum_latitude = self.get_parameter('datum_latitude').value
        self.datum_longitude = self.get_parameter('datum_longitude').value
        self.datum_altitude = self.get_parameter('datum_altitude').value
        self.publish_period = self.get_parameter('publish_period').value
        
        # TF broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Initialize data storage
        self.latest_gps = None
        self.latest_imu = None
        self.latest_map_odom = None
        self.utm_origin = None
        self.transform_published = False
        self.received_map_odom = False
        
        # Set up subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            self.gps_topic, 
            self.gps_callback, 
            10)
            
        self.imu_sub = self.create_subscription(
            Imu, 
            self.imu_topic, 
            self.imu_callback, 
            10)
            
        # Subscribe to odometry/map instead of odometry/navsat
        self.map_odom_sub = self.create_subscription(
            Odometry,
            self.map_odom_topic,
            self.map_odom_callback,
            10)
        
        # Create service for resetting the transform
        self.reset_srv = self.create_service(
            Trigger,
            'reset_utm_map_transform',
            self.reset_transform_callback
        )
        
        # Publish initial transform with all zeroes immediately
        self.publish_zero_transform()
        
        # Create timer to check and publish the transform
        self.timer = self.create_timer(self.publish_period, self.publish_transform)
        
        self.get_logger().info(f"UTM to Map Transform Publisher initialized. Publishing zeroes until map odometry is received.")
    
    def publish_zero_transform(self):
        """Publish a zero transform from UTM to map."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.utm_frame
        transform.child_frame_id = self.map_frame
        
        # Set zero transform
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f"Published zero transform from {self.utm_frame} to {self.map_frame}")
        
    def reset_transform_callback(self, request, response):
        """
        Service callback to reset the transform to the current robot position and heading.
        """
        # Reset the transform_published flag to allow republishing
        self.transform_published = False
        
        if not self.latest_map_odom:
            response.success = False
            response.message = "Cannot reset transform: No map odometry data available"
            return response
            
        self.get_logger().info("Resetting UTM to map transform to current position and heading")
        
        # Force immediate republish
        self.publish_transform()
        
        if self.transform_published:
            response.success = True
            response.message = "UTM to map transform has been reset successfully"
        else:
            response.success = False
            response.message = "Failed to reset UTM to map transform"
            
        return response
        
    def gps_callback(self, msg):
        """Process GPS data when received."""
        self.latest_gps = msg
        self.get_logger().debug(f"Received GPS data: lat={msg.latitude}, lon={msg.longitude}")
        
    def imu_callback(self, msg):
        """Process IMU data when received."""
        self.latest_imu = msg
        self.get_logger().debug("Received IMU data")
        
    def map_odom_callback(self, msg):
        """Process odometry/map data when received."""
        self.latest_map_odom = msg
        if not self.received_map_odom:
            self.received_map_odom = True
            self.get_logger().info("Received first map odometry data, will update transform")
            # Reset the published flag to force an update with the new data
            self.transform_published = False
        self.get_logger().debug("Received map odometry data")
    
    def calculate_utm_origin(self):
        """Calculate UTM origin point from datum."""
        easting, northing, zone_number, zone_letter = utm.from_latlon(
            self.datum_latitude, self.datum_longitude)
        self.utm_origin = (easting, northing, zone_number, zone_letter)
        self.get_logger().info(f"UTM origin calculated: E={easting}, N={northing}, "
                              f"Zone={zone_number}{zone_letter}")
        return self.utm_origin
    
    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def publish_transform(self):
        """Create and publish the UTM to map transform."""
        # If we've already published the transform, no need to do it again (static transform)
        if self.transform_published:
            return
            
        # If we haven't received map odometry data yet, continue publishing zeroes
        if not self.received_map_odom:
            self.publish_zero_transform()
            return
            
        # Check if we have all the data we need
        if not self.latest_gps or not self.latest_map_odom:
            self.get_logger().info("Waiting for GPS and map odometry data...")
            return
            
        if self.utm_origin is None:
            self.calculate_utm_origin()
            
        # Create transform from UTM to Map
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.utm_frame
        transform.child_frame_id = self.map_frame
        
        # Calculate the current position in UTM
        current_easting, current_northing, zone_number, zone_letter = utm.from_latlon(
            self.latest_gps.latitude, self.latest_gps.longitude)
            
        # Use the map odometry for the transform calculation
        transform.transform.translation.x = current_easting
        transform.transform.translation.y = current_northing
        transform.transform.translation.z = 0.0
        
        # Set rotation from IMU or map odometry if available
        if self.latest_imu:
            imu_quat = self.latest_imu.orientation
            transform.transform.rotation = imu_quat
            self.get_logger().info("Using IMU orientation for transform rotation")
        elif self.latest_map_odom:
            map_odom_quat = self.latest_map_odom.pose.pose.orientation
            transform.transform.rotation = map_odom_quat
            self.get_logger().info("Using map odometry orientation for transform rotation")
        else:
            # Default to identity quaternion if no orientation data available
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            self.get_logger().info("No orientation data available, using identity quaternion")
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        self.transform_published = True
        
        # Log the transform data
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        roll, pitch, yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        yaw_degrees = math.degrees(yaw)
        
        self.get_logger().info(f"Published UTM to Map transform:")
        self.get_logger().info(f"  Translation: x={transform.transform.translation.x}, "
                              f"y={transform.transform.translation.y}, "
                              f"z={transform.transform.translation.z}")
        self.get_logger().info(f"  Rotation: roll={roll:.2f}, pitch={pitch:.2f}, "
                              f"yaw={yaw:.2f} rad ({yaw_degrees:.2f} deg)")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = UtmMapTransformPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()