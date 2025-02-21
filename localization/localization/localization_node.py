#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, TransformStamped
import pyproj
import tf2_ros

class LocalizationNode(Node):
    ENABLE_PRINTING = True  # ✅ Toggle this to enable/disable logging

    def __init__(self):
        super().__init__('localization_node')

        # TF2 Broadcaster for UTM → Map
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # UTM conversion utility
        self.utm_proj = None
        self.utm_zone = None

        # Store the robot's initial UTM position (zero for the /map frame)
        self.initial_utm_x = None
        self.initial_utm_y = None

        # Initialize variables
        self.current_lat = None
        self.current_lon = None
        self.current_utm_x = None
        self.current_utm_y = None

        # Subscribers
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publisher for UTM coordinates
        self.utm_publisher = self.create_publisher(Point, '/robot/utm', 10)

    def gps_callback(self, msg):
        """ Callback for GPS data: Convert GPS (lat, lon) to UTM and publish transformation. """
        if msg.status.status < 0:  # GPS status check (no fix)
            if self.ENABLE_PRINTING:
                self.get_logger().warn("No valid GPS fix!")
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        # Convert lat/lon to UTM coordinates
        self.current_utm_x, self.current_utm_y = self.convert_to_utm(self.current_lat, self.current_lon)

        # Store initial UTM position if not set
        if self.initial_utm_x is None or self.initial_utm_y is None:
            self.initial_utm_x = self.current_utm_x
            self.initial_utm_y = self.current_utm_y
            if self.ENABLE_PRINTING:
                self.get_logger().info(f"Set initial UTM zero at: X={self.initial_utm_x}, Y={self.initial_utm_y}")

        # Publish UTM coordinates
        self.publish_utm_coordinates()

        # Broadcast UTM → Map transformation
        self.broadcast_utm_to_map_tf()

    def odom_callback(self, msg):
        """ Callback for Odometry data. """
        if self.ENABLE_PRINTING:
            self.get_logger().info(f"Received Odometry Data: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")

    def imu_callback(self, msg):
        """ Callback for IMU data. """
        if self.ENABLE_PRINTING:
            self.get_logger().info("Received IMU Data")

    def convert_to_utm(self, latitude, longitude):
        """ Convert GPS latitude/longitude to UTM coordinates and auto-select zone. """
        self.utm_zone = int((longitude + 180) / 6) + 1
        self.utm_proj = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', south=(latitude < 0))

        utm_x, utm_y = self.utm_proj(longitude, latitude)
        
        if self.ENABLE_PRINTING:
            self.get_logger().info(f"Converted to UTM (Zone {self.utm_zone}): X={utm_x}, Y={utm_y}")

        return utm_x, utm_y

    def publish_utm_coordinates(self):
        """ Publish the robot's UTM coordinates relative to the initial position. """
        if self.current_utm_x is None or self.current_utm_y is None:
            return

        utm_point = Point()
        utm_point.x = self.current_utm_x - self.initial_utm_x
        utm_point.y = self.current_utm_y - self.initial_utm_y
        utm_point.z = 0.0  # No altitude info in UTM

        self.utm_publisher.publish(utm_point)

        if self.ENABLE_PRINTING:
            self.get_logger().info(f"Published UTM (relative): X={utm_point.x}, Y={utm_point.y}")

    def broadcast_utm_to_map_tf(self):
        """ Broadcast the UTM → Map transformation using the initial UTM position as the zero. """
        if self.initial_utm_x is None or self.initial_utm_y is None:
            return

        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'utm'    # Global UTM Frame
        t.child_frame_id = 'map'     # Local navigation frame

        # Set translation (use initial UTM position as zero)
        t.transform.translation.x = self.initial_utm_x
        t.transform.translation.y = self.initial_utm_y
        t.transform.translation.z = 0.0

        # Set identity quaternion (no rotation applied)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(t)
        
        if self.ENABLE_PRINTING:
            self.get_logger().info(f"Broadcasting UTM → Map TF: Zero at X={self.initial_utm_x}, Y={self.initial_utm_y}")

def main():
    rclpy.init()
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
