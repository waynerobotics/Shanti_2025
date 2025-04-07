#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from lifecycle_msgs.msg import Transition
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException, Buffer, TransformListener
import utm
import math
import numpy as np
from std_srvs.srv import Trigger


class GpsMapTransformer(LifecycleNode):
    """
    ROS2 lifecycle node to transform GPS data into the map frame.
    
    This node listens to filtered GPS data, transforms it into the map frame 
    using the UTM->map transform, and republishes the data as a PoseStamped message.
    The node only activates when the UTM->map transform is available.
    """

    def __init__(self):
        super().__init__('gps_map_transformer')
        
        # Declare parameters
        self.declare_parameter('gps_topic', '/gps/filtered')
        self.declare_parameter('map_pose_topic', '/gps/pose/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('transform_check_period', 1.0)  # How often to check for transform
        self.declare_parameter('publish_rate', 10.0)  # Rate at which to republish GPS data (Hz)
        
        # Get parameters
        self.gps_topic = self.get_parameter('gps_topic').value
        self.map_pose_topic = self.get_parameter('map_pose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.transform_check_period = self.get_parameter('transform_check_period').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize data storage
        self.latest_gps = None
        self.transform_available = False
        
        # These will be initialized in configure callback
        self.tf_buffer = None
        self.tf_listener = None
        self.gps_sub = None
        self.map_pose_pub = None
        self.transform_check_timer = None
        self.publish_timer = None
        self.reset_srv = None
        
        # Log initialization
        self.get_logger().info('GPS Map Transformer node initialized but not yet configured')

    def on_configure(self, state):
        """Lifecycle configure callback."""
        self.get_logger().info('Configuring...')
        
        try:
            # Initialize TF listener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            
            # Set up transform check timer
            self.transform_check_timer = self.create_timer(
                self.transform_check_period, 
                self.check_transform_availability
            )
            
            # Set up GPS subscription (monitoring only in this state)
            self.gps_sub = self.create_subscription(
                NavSatFix,
                self.gps_topic,
                self.gps_callback_inactive,
                10
            )
            
            self.get_logger().info('Configuration complete, checking for UTM->map transform...')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {str(e)}')
            return TransitionCallbackReturn.ERROR
    
    def check_transform_availability(self):
        """
        Check if the UTM->map transform is available.
        """
        try:
            # Check if transform is available by looking it up with a timeout
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)  # Short timeout
            )
            
            if not self.transform_available:
                self.transform_available = True
                self.get_logger().info('UTM->map transform is now available')
                # Note: We no longer auto-activate here, this will be handled by the lifecycle manager
            
        except TransformException as e:
            if self.transform_available:
                self.transform_available = False
                self.get_logger().warn('UTM->map transform is no longer available')
                # Note: We no longer auto-deactivate here, this will be handled by the lifecycle manager
            else:
                self.get_logger().debug('Waiting for UTM->map transform to become available')
    
    def gps_callback_inactive(self, msg):
        """
        Store GPS data but don't process it when inactive.
        """
        self.latest_gps = msg
    
    def gps_callback_active(self, msg):
        """
        Process GPS data when active.
        """
        self.latest_gps = msg
        self.transform_and_publish_gps()
    
    def on_activate(self, state):
        """Lifecycle activate callback."""
        self.get_logger().info('Activating...')
        
        try:
            # Check if transform is available before activating
            try:
                self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.utm_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1)
                )
                self.transform_available = True
            except TransformException:
                self.get_logger().warn('UTM->map transform not available during activation')
                self.transform_available = False
                # We'll still activate, but won't publish anything until transform is available
            
            # Create the pose publisher
            self.map_pose_pub = self.create_publisher(
                PoseStamped,
                self.map_pose_topic,
                10
            )
            
            # Replace GPS subscription with active version
            self.destroy_subscription(self.gps_sub)
            self.gps_sub = self.create_subscription(
                NavSatFix,
                self.gps_topic,
                self.gps_callback_active,
                10
            )
            
            # Create publish timer
            self.publish_timer = self.create_timer(
                1.0 / self.publish_rate,
                self.transform_and_publish_gps
            )
            
            # Create reset service
            self.reset_srv = self.create_service(
                Trigger,
                'reset_gps_map_transformer',
                self.reset_callback
            )
            
            self.get_logger().info('Node activated successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Activation failed: {str(e)}')
            return TransitionCallbackReturn.ERROR
    
    def on_deactivate(self, state):
        """Lifecycle deactivate callback."""
        self.get_logger().info('Deactivating...')
        
        try:
            # Clean up publisher
            if self.map_pose_pub:
                self.destroy_publisher(self.map_pose_pub)
                self.map_pose_pub = None
            
            # Replace GPS subscription with inactive version
            if self.gps_sub:
                self.destroy_subscription(self.gps_sub)
                self.gps_sub = self.create_subscription(
                    NavSatFix,
                    self.gps_topic,
                    self.gps_callback_inactive,
                    10
                )
            
            # Clean up timers
            if self.publish_timer:
                self.destroy_timer(self.publish_timer)
                self.publish_timer = None
                
            # Clean up service
            if self.reset_srv:
                self.destroy_service(self.reset_srv)
                self.reset_srv = None
                
            self.get_logger().info('Node deactivated successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Deactivation failed: {str(e)}')
            return TransitionCallbackReturn.ERROR
    
    def on_cleanup(self, state):
        """Lifecycle cleanup callback."""
        self.get_logger().info('Cleaning up...')
        
        try:
            # Clean up all resources
            if self.map_pose_pub:
                self.destroy_publisher(self.map_pose_pub)
                self.map_pose_pub = None
                
            if self.gps_sub:
                self.destroy_subscription(self.gps_sub)
                self.gps_sub = None
                
            if self.transform_check_timer:
                self.destroy_timer(self.transform_check_timer)
                self.transform_check_timer = None
                
            if self.publish_timer:
                self.destroy_timer(self.publish_timer)
                self.publish_timer = None
                
            if self.reset_srv:
                self.destroy_service(self.reset_srv)
                self.reset_srv = None
                
            # Reset data
            self.latest_gps = None
            self.transform_available = False
            
            self.get_logger().info('Cleanup complete')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Cleanup failed: {str(e)}')
            return TransitionCallbackReturn.ERROR
    
    def on_shutdown(self, state):
        """Lifecycle shutdown callback."""
        self.get_logger().info('Shutting down...')
        # Similar to cleanup but we're shutting down completely
        self.on_cleanup(state)
        self.get_logger().info('Shutdown complete')
        return TransitionCallbackReturn.SUCCESS
    
    def reset_callback(self, request, response):
        """
        Service callback to reset the node.
        """
        try:
            # Clear cached data
            self.latest_gps = None
            
            self.get_logger().info("GPS Map Transformer reset successfully")
            response.success = True
            response.message = "GPS Map Transformer reset successfully"
        except Exception as e:
            self.get_logger().error(f"Failed to reset GPS Map Transformer: {str(e)}")
            response.success = False
            response.message = f"Failed to reset: {str(e)}"
            
        return response
    
    def transform_and_publish_gps(self):
        """
        Transform the latest GPS data into the map frame and publish it.
        """
        if not self.latest_gps or not self.map_pose_pub:
            return

        try:
            # Convert GPS to UTM
            easting, northing, zone_number, zone_letter = utm.from_latlon(
                self.latest_gps.latitude, self.latest_gps.longitude)

            # Create a PoseStamped in the UTM frame
            utm_pose = PoseStamped()
            utm_pose.header.stamp = self.get_clock().now().to_msg()
            utm_pose.header.frame_id = self.utm_frame

            # Set the position from the UTM coordinates
            utm_pose.pose.position.x = easting
            utm_pose.pose.position.y = northing
            utm_pose.pose.position.z = (
                self.latest_gps.altitude if not math.isnan(self.latest_gps.altitude) else 0.0
            )

            # Set the orientation to identity quaternion (no orientation from GPS)
            utm_pose.pose.orientation.x = 0.0
            utm_pose.pose.orientation.y = 0.0
            utm_pose.pose.orientation.z = 0.0
            utm_pose.pose.orientation.w = 1.0

            # Transform from UTM to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )

            # Apply the transform to the UTM pose
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert quaternion to rotation matrix
            q = [rotation.x, rotation.y, rotation.z, rotation.w]
            rotation_matrix = self.quaternion_to_rotation_matrix(q)

            # Transform position
            position = np.array([utm_pose.pose.position.x, utm_pose.pose.position.y, utm_pose.pose.position.z])
            transformed_position = np.dot(rotation_matrix, position) + np.array([translation.x, translation.y, translation.z])

            # Create transformed PoseStamped
            map_pose = PoseStamped()
            map_pose.header.stamp = utm_pose.header.stamp
            map_pose.header.frame_id = self.map_frame
            map_pose.pose.position.x = transformed_position[0]
            map_pose.pose.position.y = transformed_position[1]
            map_pose.pose.position.z = transformed_position[2]

            # Correctly transform orientation
            map_pose.pose.orientation = rotation

            # Publish the transformed pose
            self.map_pose_pub.publish(map_pose)

        except TransformException as e:
            self.get_logger().warning(f'Failed to transform GPS to map frame: {str(e)}')

        except Exception as e:
            self.get_logger().error(f'Error transforming GPS data: {str(e)}')

    def quaternion_to_rotation_matrix(self, q):
        """
        Convert a quaternion into a 3x3 rotation matrix.
        """
        x, y, z, w = q
        return np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])


def main(args=None):
    """Entry point for the GPS Map Transformer node."""
    rclpy.init(args=args)
    
    # Create the lifecycle node
    node = GpsMapTransformer()
    
    # Create executor and spin
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()