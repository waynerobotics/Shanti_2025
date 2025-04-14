#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from lifecycle_msgs.msg import Transition
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


class UtmMapTransformPublisher(LifecycleNode):
    """
    ROS2 lifecycle node to dynamically create a static transform publisher for UTM to map transform.
    
    This node listens to GPS and IMU data, then calculates the appropriate transform 
    between UTM and map frames based on the robot's initial position and orientation.
    Activates only upon receiving an odometry/map message.
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
        self.declare_parameter('publish_period', 1.0)  # How often to publish the transform
        self.declare_parameter('gps_filtered_topic', '/gps/filtered')  # Use gps/filtered for datum
        
        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_odom_topic = self.get_parameter('map_odom_topic').value
        self.transform_timeout = self.get_parameter('transform_timeout').value
        self.publish_period = self.get_parameter('publish_period').value
        self.gps_filtered_topic = self.get_parameter('gps_filtered_topic').value
        
        # Initialize data storage
        self.latest_gps = None
        self.latest_imu = None
        self.latest_map_odom = None
        self.latest_gps_filtered = None
        self.utm_origin = None
        self.transform_published = False
        self.received_map_odom = False
        
        # These will be initialized in configure callback
        self.tf_broadcaster = None
        self.gps_sub = None
        self.imu_sub = None
        self.map_odom_sub = None
        self.gps_filtered_sub = None
        self.timer = None
        self.reset_srv = None
        
        # Log initialization
        self.get_logger().info('UTM Map Transform Publisher node initialized but not yet configured')

    def on_configure(self, state):
        """Lifecycle configure callback."""
        self.get_logger().info('Configuring...')
        
        try:
            # Initialize TF broadcaster
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            
            # Set up the monitoring subscription for activation
            # This will only monitor for map odometry but won't process it yet
            self.map_odom_sub = self.create_subscription(
                Odometry,
                self.map_odom_topic,
                self.map_odom_monitor_callback,
                10)
                
            # Publish initial zero transform
            self.publish_zero_transform()
            
            self.get_logger().info('Configuration complete, waiting for map odometry to activate')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {str(e)}')
            return TransitionCallbackReturn.ERROR
    
    def map_odom_monitor_callback(self, msg):
        """
        Monitor for map odometry messages and trigger activation if received.
        """
        if not self.received_map_odom:
            self.received_map_odom = True
            self.latest_map_odom = msg
            self.get_logger().info('Received first map odometry message')
        else:
            # Just update the latest message
            self.latest_map_odom = msg
    
    def on_activate(self, state):
        """Lifecycle activate callback."""
        self.get_logger().info('Activating...')
        
        try:
            # Create full subscriptions now that we're active
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
                
            self.gps_filtered_sub = self.create_subscription(
                NavSatFix,
                self.gps_filtered_topic,
                self.gps_filtered_callback,
                10)
                
            # Replace the monitoring subscription with the full version
            self.destroy_subscription(self.map_odom_sub)
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
            
            # Create timer to check and publish the transform
            self.timer = self.create_timer(self.publish_period, self.publish_transform)
            
            # Reset the transform_published flag to force a new calculation
            self.transform_published = False
            
            self.get_logger().info('Node activated successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Activation failed: {str(e)}')
            return TransitionCallbackReturn.ERROR
    
    def on_deactivate(self, state):
        """Lifecycle deactivate callback."""
        self.get_logger().info('Deactivating...')
        
        try:
            # Clean up full subscriptions
            if self.gps_sub:
                self.destroy_subscription(self.gps_sub)
                self.gps_sub = None
                
            if self.imu_sub:
                self.destroy_subscription(self.imu_sub)
                self.imu_sub = None
            
            if self.gps_filtered_sub:
                self.destroy_subscription(self.gps_filtered_sub)
                self.gps_filtered_sub = None
            
            # Destroy and recreate map_odom_sub as monitor only
            if self.map_odom_sub:
                self.destroy_subscription(self.map_odom_sub)
                self.map_odom_sub = self.create_subscription(
                    Odometry,
                    self.map_odom_topic,
                    self.map_odom_monitor_callback,
                    10)
            
            # Clean up timer
            if self.timer:
                self.destroy_timer(self.timer)
                self.timer = None
                
            # Clean up service
            if self.reset_srv:
                self.destroy_service(self.reset_srv)
                self.reset_srv = None
                
            # Reset flags
            self.transform_published = False
            
            self.get_logger().info('Node deactivated successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Deactivation failed: {str(e)}')
            return TransitionCallbackReturn.ERROR
    
    def on_cleanup(self, state):
        """Lifecycle cleanup callback."""
        self.get_logger().info('Cleaning up...')
        
        try:
            # Ensure all subscriptions are destroyed
            if self.gps_sub:
                self.destroy_subscription(self.gps_sub)
                self.gps_sub = None
                
            if self.imu_sub:
                self.destroy_subscription(self.imu_sub)
                self.imu_sub = None
                
            if self.map_odom_sub:
                self.destroy_subscription(self.map_odom_sub)
                self.map_odom_sub = None
                
            if self.gps_filtered_sub:
                self.destroy_subscription(self.gps_filtered_sub)
                self.gps_filtered_sub = None
                
            # Clean up timer
            if self.timer:
                self.destroy_timer(self.timer)
                self.timer = None
                
            # Clean up service
            if self.reset_srv:
                self.destroy_service(self.reset_srv)
                self.reset_srv = None
                
            # Reset all data
            self.latest_gps = None
            self.latest_imu = None
            self.latest_map_odom = None
            self.latest_gps_filtered = None
            self.utm_origin = None
            self.transform_published = False
            self.received_map_odom = False
            
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
    
    def publish_zero_transform(self):
        """Publish a zero transform from UTM to map."""
        if self.tf_broadcaster is None:
            self.get_logger().error("Cannot publish transform: TF broadcaster not initialized")
            return
            
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
        
    def gps_filtered_callback(self, msg):
        """Process filtered GPS data for datum."""
        self.latest_gps_filtered = msg
        self.get_logger().debug(f"Received filtered GPS data: lat={msg.latitude}, lon={msg.longitude}")
    
    def map_odom_callback(self, msg):
        """Process odometry/map data when received."""
        self.latest_map_odom = msg
        self.get_logger().debug("Received map odometry data")
    
    def calculate_utm_origin(self):
        """Calculate UTM origin point from filtered GPS data."""
        if not self.latest_gps_filtered:
            self.get_logger().error("Cannot calculate UTM origin: No filtered GPS data available")
            return None
        
        easting, northing, zone_number, zone_letter = utm.from_latlon(
            self.latest_gps_filtered.latitude, self.latest_gps_filtered.longitude)
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
        if not self.latest_gps or not self.latest_map_odom or not self.latest_gps_filtered:
            self.get_logger().info("Waiting for GPS, filtered GPS, and map odometry data...")
            return
            
        # Ensure the node publishes a static transform only once the datum is located
        if self.utm_origin is None:
            self.calculate_utm_origin()

        if not self.transform_published:
            # Publish the transform only once
            self.publish_static_transform()
            self.transform_published = True

        # After publishing the transform, stop further actions unless the reset service is called
        if self.transform_published:
            self.get_logger().info("Static transform published. Node will remain idle unless reset service is called.")
            return

    def publish_static_transform(self):
        """Helper method to publish the static transform."""
        # Create and publish the UTM to map transform
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
    """Entry point for the UTM Map Transform Publisher node."""
    rclpy.init(args=args)
    
    # Create the lifecycle node
    node = UtmMapTransformPublisher()
    
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