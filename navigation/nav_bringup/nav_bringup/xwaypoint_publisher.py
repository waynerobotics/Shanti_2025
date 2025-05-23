import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import yaml
import os
import utm
import math
import tf2_ros
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
import tf2_geometry_msgs
# Define your origin
ORIGIN_LAT = 42.66791
ORIGIN_LON = -83.21958

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Create an action client for the waypoint follower
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Declare parameters
        self.declare_parameter('waypoints_file', 'gps_waypoints.yaml')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('map_frame', 'map')
        
        # Get parameters
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.get_logger().info(f'*** Using map coordinates: {self.map_frame}')
        
        # Set up tf listener for utm to map transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize waypoint tracking variables
        self.last_waypoint = -1
        self.current_attempt = 0
        self.max_attempts = 3
        
        # Load waypoints from the YAML file
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            self.get_logger().error('No waypoints found in the YAML file.')
            return
            
        # Wait for the action server to be available
        self.get_logger().info('Waiting for waypoint follower action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Waypoint follower action server is available!')
        
        # Wait for transform to be available
        self.get_logger().info(f'Waiting for transform from {self.utm_frame} to {self.map_frame}...')
        self.create_timer(1.0, self.check_transform_and_send_waypoints)
        
        # Create a watchdog timer to detect if navigation is stuck
        self.watchdog_timer = self.create_timer(10.0, self.watchdog_callback)
        self.watchdog_timer.cancel()  # Start disabled

    def load_waypoints(self):
        # Use ament_index_python to find the correct path to the params directory
        package_share_directory = get_package_share_directory('nav_bringup')
        waypoints_file = os.path.join(package_share_directory, 'params', self.waypoints_file)
        try:
            with open(waypoints_file, 'r') as file:
                data = yaml.safe_load(file)
                waypoints_data = data.get('waypoints', [])
                self.get_logger().info(f'Loaded {len(waypoints_data)} GPS waypoints')
                return waypoints_data
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def check_transform_and_send_waypoints(self):
        # Check if the transform is available
        try:
            if self.tf_buffer.can_transform(self.map_frame, self.utm_frame, rclpy.time.Time()):
                self.get_logger().info(f'Transform from {self.utm_frame} to {self.map_frame} is available')
                self.send_waypoints()
                # Cancel the timer after sending waypoints
                for timer in self.timers:
                    if timer != self.watchdog_timer:
                        timer.cancel()
                # Start the watchdog timer to monitor progress
                self.watchdog_timer.reset()
            else:
                self.get_logger().info(f'Waiting for transform from {self.utm_frame} to {self.map_frame}...')
        except Exception as e:
            self.get_logger().warn(f'Error checking transform: {e}')

    def watchdog_callback(self):
        """Monitor if navigation has stalled and retry if needed"""
        self.get_logger().info(f'Watchdog check: Current waypoint = {self.last_waypoint}, Total waypoints = {len(self.waypoints)}')
        
        # If we've been stuck at the same waypoint for too long, try to move to the next one
        if hasattr(self, 'last_waypoint') and self.last_waypoint >= 0:
            if hasattr(self, 'stall_count'):
                self.stall_count += 1
                if self.stall_count >= 3:  # If stalled for 3 watchdog periods
                    if self.last_waypoint < len(self.waypoints) - 1:
                        self.get_logger().warn(f'Navigation appears to be stalled at waypoint {self.last_waypoint}. Moving to next waypoint.')
                        # Skip to the next waypoint
                        next_waypoint = self.last_waypoint + 1
                        self.send_waypoints(start_from=next_waypoint)
                        self.stall_count = 0
                    else:
                        self.get_logger().info('At final waypoint. Mission complete.')
                        self.watchdog_timer.cancel()
            else:
                self.stall_count = 1
        else:
            # Reset stall count if we don't have a valid last_waypoint yet
            self.stall_count = 0

    def send_waypoints(self, start_from=0):
        # Create goal message with list of waypoints
        goal_msg = FollowWaypoints.Goal()
        poses = []
        
        self.get_logger().info(f'Preparing to send waypoints starting from index {start_from}')
        
        # Process only waypoints from the starting index
        for idx, waypoint in enumerate(self.waypoints[start_from:]):
            # Get the actual index in the overall waypoints list
            actual_idx = idx + start_from
            
            # Check waypoint format - support both [lat, lon] and {'lat': lat, 'lon': lon} formats
            if isinstance(waypoint, list) and len(waypoint) >= 2:
                lat, lon = waypoint[0], waypoint[1]
            elif isinstance(waypoint, dict) and 'lat' in waypoint and 'lon' in waypoint:
                lat, lon = waypoint['lat'], waypoint['lon']
            else:
                self.get_logger().error(f'Invalid waypoint format: {waypoint}')
                continue
                
            self.get_logger().info(f'Processing waypoint {actual_idx}: lat={lat}, lon={lon}')
            
            # Convert lat/lon to UTM
            try:
                # easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
                # self.get_logger().info(f'UTM coordinates: E={easting}, N={northing}, Zone={zone_number}{zone_letter}')
                
                # # Create PoseStamped in UTM frame
                # utm_pose = PoseStamped()
                # utm_pose.header.frame_id = self.utm_frame
                # utm_pose.header.stamp = self.get_clock().now().to_msg()
                # utm_pose.pose.position.x = easting
                # utm_pose.pose.position.y = northing
                # utm_pose.pose.position.z = 0.0
                # utm_pose.pose.orientation.w = 1.0
                
                # # Transform from UTM to map frame
                # try:
                #     map_pose = self.tf_buffer.transform(utm_pose, self.map_frame)
                #     poses.append(map_pose)

                try:
                    origin_e, origin_n, _, _ = utm.from_latlon(ORIGIN_LAT, ORIGIN_LON)
                    wp_e, wp_n, _, _ = utm.from_latlon(lat, lon)

                    pose = PoseStamped()
                    pose.header.frame_id = self.map_frame
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = wp_e - origin_e
                    pose.pose.position.y = wp_n - origin_n
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    poses.append(pose)
                    
                    self.get_logger().info(f'Transformed to map coordinates: x={pose.pose.position.x}, y={pose.pose.position.y}')
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().error(f'Transform failed: {e}')
                    continue
                    
            except Exception as e:
                self.get_logger().error(f'Failed to convert GPS coordinates: {e}')
                continue

        if not poses:
            self.get_logger().error('No valid waypoints to send!')
            return
            
        goal_msg.poses = poses
        
        self.get_logger().info(f'Sending {len(poses)} waypoints to follower')
        
        # Send goal and register callbacks
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            return
            
        self.get_logger().info('Goal accepted by the action server')
        self.goal_handle = goal_handle
        
        # Request for final result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_waypoint
        total_waypoints = len(self.waypoints)
        
        self.get_logger().info(f'Currently executing waypoint: {current_waypoint} of {total_waypoints}')
        
        # Reset the stall count since we're receiving feedback
        self.stall_count = 0
        
        # Track waypoint progress for better monitoring
        if hasattr(self, 'last_waypoint') and current_waypoint > self.last_waypoint:
            self.get_logger().info(f'Transitioned from waypoint {self.last_waypoint} to {current_waypoint}')
        
        self.last_waypoint = current_waypoint
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Successfully completed all waypoints!')
            # Stop the watchdog timer
            self.watchdog_timer.cancel()
        else:
            self.get_logger().error(f'Failed to complete waypoints with status: {status}')
            # Log detailed information about what might have gone wrong
            self.get_logger().error(f'Status code: {status}')
            if hasattr(result, 'missed_waypoints') and result.missed_waypoints:
                self.get_logger().error(f'Missed waypoints: {result.missed_waypoints}')
            
            # If not all waypoints were completed, retry from the last waypoint
            if self.last_waypoint < len(self.waypoints) - 1:
                self.current_attempt += 1
                if self.current_attempt <= self.max_attempts:
                    self.get_logger().info(f'Retrying navigation from waypoint {self.last_waypoint}')
                    self.send_waypoints(start_from=self.last_waypoint)
                else:
                    self.get_logger().error(f'Failed after {self.max_attempts} attempts. Giving up.')
                    self.watchdog_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()