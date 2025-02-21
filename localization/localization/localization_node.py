#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import pyproj

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        # UTM conversion utility
        self.utm_proj = pyproj.Proj(proj='utm', zone=33, ellps='WGS84', south=False)

        # Initialize variables
        self.current_lat = None
        self.current_lon = None
        self.current_utm_x = None
        self.current_utm_y = None
        self.odom_position = None
        self.imu_orientation = None

        # Subscribers
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Odometry, '/demo/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/demo/imu', self.imu_callback, 10)

        # Publisher for UTM coordinates
        self.utm_publisher = self.create_publisher(Point, '/utm', 10)

    def gps_callback(self, msg):
        """ Callback for GPS data: Convert GPS (lat, lon) to UTM. """
        if msg.status.status < 0:  # GPS status check (no fix)
            self.get_logger().warn("No valid GPS fix!")
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        # Convert lat/lon to UTM coordinates
        self.current_utm_x, self.current_utm_y = self.convert_to_utm(self.current_lat, self.current_lon)
        self.publish_utm_coordinates()

    def odom_callback(self, msg):
        """ Callback for Odometry data: Store position and velocity. """
        self.odom_position = msg.pose.pose.position
        self.get_logger().info(f"Received Odometry Data: x={self.odom_position.x}, y={self.odom_position.y}")

    def imu_callback(self, msg):
        """ Callback for IMU data: Store orientation. """
        self.imu_orientation = msg.orientation
        self.get_logger().info("Received IMU Data")

    def convert_to_utm(self, latitude, longitude):
        """ Convert GPS latitude/longitude to UTM coordinates. """
        utm_x, utm_y = self.utm_proj(longitude, latitude)
        return utm_x, utm_y

    def publish_utm_coordinates(self):
        """ Publish the robot's UTM coordinates. """
        if self.current_utm_x is None or self.current_utm_y is None:
            return

        utm_point = Point()
        utm_point.x = self.current_utm_x
        utm_point.y = self.current_utm_y
        utm_point.z = 0.0  # No altitude info in UTM

        self.utm_publisher.publish(utm_point)
        self.get_logger().info(f"Published UTM Coordinates: X={utm_point.x}, Y={utm_point.y}")

def main():
    rclpy.init()
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
