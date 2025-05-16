import rclpy
from rclpy.node import Node
from rclpy.service import Service
from std_srvs.srv import Trigger
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator

import yaml
import os
import sys
import time

from gps_srv.utils.gps_utils import latLonYaw2Geopose
from nav2_simple_commander.robot_navigator import BasicNavigator

class YamlWaypointParser:
    """
    Parses a set of GPS waypoints from a YAML file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)
    

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWaypointService(Node):
    """
    ROS 2 Service Node to publish GPS waypoints as goal poses
    """

    def __init__(self, wps_file_path):
        super().__init__('gps_waypoint_service')
        
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.service = self.create_service(Trigger, 'publish_waypoints', self.publish_waypoints_callback)

    def publish_waypoints_callback(self, request, response):
        """
        Callback function to publish waypoints as goal poses
        """
        waypoints = self.wp_parser.get_wps()
        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Convert GeoPose to Pose
            if isinstance(waypoint, GeoPose):
                pose_stamped.pose = Pose()
                pose_stamped.pose.position.x = waypoint.position.latitude  # Replace with UTM conversion if needed
                pose_stamped.pose.position.y = waypoint.position.longitude  # Replace with UTM conversion if needed
                pose_stamped.pose.position.z = 0.0  # Assuming flat terrain
                pose_stamped.pose.orientation = waypoint.orientation
            else:
                self.get_logger().error(f"Waypoint format mismatch: {waypoint}")
                continue
            
            self.publisher.publish(pose_stamped)
            self.get_logger().info(f'Published waypoint: {pose_stamped}')
            time.sleep(1.0)  # Sleep to simulate delay between waypoints

        response.success = True
        response.message = "Waypoints published successfully."
        return response


def main():
    rclpy.init()

    # Allow passing the waypoints file as an argument
    default_yaml_file_path = os.path.join(r"/home/siva/ros2_ws/src/Shanti_2025/gps_srv/gps_srv/config/waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_waypoint_service = GpsWaypointService(yaml_file_path)

    rclpy.spin(gps_waypoint_service)  # Keep the service node running
    rclpy.shutdown()


if __name__ == "__main__":
    main()