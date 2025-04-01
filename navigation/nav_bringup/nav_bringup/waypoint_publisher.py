import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Publisher for navigation goals
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Load waypoints from the YAML file
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            self.get_logger().error('No waypoints found in the YAML file.')
            return

        # Publish waypoints sequentially
        self.current_index = 0
        self.timer = self.create_timer(5.0, self.publish_next_waypoint)

    def load_waypoints(self):
        # Use ament_index_python to find the correct path to the params directory
        package_share_directory = get_package_share_directory('nav_bringup')
        waypoints_file = os.path.join(package_share_directory, 'params', 'gps_waypoints.yaml')
        try:
            with open(waypoints_file, 'r') as file:
                data = yaml.safe_load(file)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def publish_next_waypoint(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints have been published.')
            self.timer.cancel()
            return

        waypoint = self.waypoints[self.current_index]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.publisher_.publish(pose)
        self.get_logger().info(f'Published waypoint {self.current_index + 1}: {waypoint}')
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()