#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import utm
import math

class PoseToGpsConverter(Node):
    """
    Converts PoseStamped messages to NavSatFix messages.
    Uses a reference GPS position and UTM conversion to convert local poses back to GPS coordinates.
    """

    def __init__(self):
        super().__init__('pose_to_gps_converter')
        
        # Declare parameters
        self.declare_parameter('reference_latitude', 42.66791)  # Default reference point
        self.declare_parameter('reference_longitude', -83.21958)
        self.declare_parameter('reference_altitude', 0.0)
        self.declare_parameter('input_topic', '/gnss_pose')
        self.declare_parameter('output_topic', '/gnss')
        
        # Get parameters
        self.reference_latitude = self.get_parameter('reference_latitude').value
        self.reference_longitude = self.get_parameter('reference_longitude').value
        self.reference_altitude = self.get_parameter('reference_altitude').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Set up initial UTM reference
        easting, northing, zone_number, zone_letter = utm.from_latlon(
            self.reference_latitude, 
            self.reference_longitude
        )
        self.ref_easting = easting
        self.ref_northing = northing
        self.zone_number = zone_number
        self.zone_letter = zone_letter
        
        # Create publisher
        self.gps_pub = self.create_publisher(
            NavSatFix,
            self.output_topic,
            10
        )
        
        # Create subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            10
        )
        
        self.get_logger().info(f'Converting poses from {self.input_topic} to GPS on {self.output_topic}')
        self.get_logger().info(f'Using reference: Lat {self.reference_latitude}, Lon {self.reference_longitude}')
        self.get_logger().info(f'Reference UTM: E {self.ref_easting}, N {self.ref_northing}, Zone {self.zone_number}{self.zone_letter}')
    
    def pose_callback(self, msg):
        """
        Convert PoseStamped to NavSatFix using UTM conversion.
        The PoseStamped x/y is assumed to be in meters from the reference point.
        """
        # Extract position from pose message
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Convert local x/y to UTM easting/northing
        easting = self.ref_easting + x
        northing = self.ref_northing + y
        
        # Convert UTM back to latitude/longitude
        lat, lon = utm.to_latlon(easting, northing, self.zone_number, self.zone_letter)
        
        # Create NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header = msg.header
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = self.reference_altitude + z
        
        # Set covariance - moderate accuracy (3m horizontal, 5m vertical)
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps_msg.position_covariance = [
            9.0, 0.0, 0.0,  # 3m standard deviation for latitude
            0.0, 9.0, 0.0,  # 3m standard deviation for longitude
            0.0, 0.0, 25.0  # 5m standard deviation for altitude
        ]
        
        # Set status (make sure we indicate this is a valid fix)
        gps_msg.status.status = 0  # STATUS_FIX (valid GPS fix)
        gps_msg.status.service = 1  # SERVICE_GPS
        
        # Publish NavSatFix message
        self.gps_pub.publish(gps_msg)
        
        # Log occasionally (once every 100 messages)
        if hasattr(self, 'msg_count'):
            self.msg_count += 1
            if self.msg_count % 100 == 0:
                self.get_logger().info(f'Converted pose x:{x:.2f}, y:{y:.2f} to GPS lat:{lat:.7f}, lon:{lon:.7f}')
        else:
            self.msg_count = 1
            self.get_logger().info(f'Converted pose x:{x:.2f}, y:{y:.2f} to GPS lat:{lat:.7f}, lon:{lon:.7f}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseToGpsConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
