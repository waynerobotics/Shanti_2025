#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion, Vector3
from std_srvs.srv import Empty
import serial
import math
import numpy as np
import time
import threading
import queue

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')
        
        # Create callback groups for better concurrency
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        
        # Create publishers with a larger queue size
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create reset service
        self.reset_service = self.create_service(
            Empty, 'reset_odometry', 
            self.reset_odometry_callback,
            callback_group=self.service_callback_group)
            
        # Declare parameters with default values
        self.declare_parameter('wheel_base', 0.5334)  # Distance between wheels in meters
        self.declare_parameter('wheel_radius', 0.1016)  # Wheel radius in meters
        self.declare_parameter('encoder_resolution', 1024)  # Encoder ticks per revolution
        self.declare_parameter('update_frequency', 50.0)  # Hz - increased for better responsiveness
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.01)  # Short timeout for non-blocking reads
        self.declare_parameter('debug_level', 1)  # 0=minimal, 1=normal, 2=verbose
        
        # Get parameter values
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.update_frequency = self.get_parameter('update_frequency').value
        self.debug_level = self.get_parameter('debug_level').value
        
        # Calculate meters per encoder tick
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.encoder_resolution
        
        # Setup communication with microcontroller
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        
        # Set up serial port with a timeout for non-blocking reads
        try:
            self.serial_port = serial.Serial(port, baud, timeout=timeout)
            time.sleep(0.1)  # Short wait for port to open
            self.serial_port.reset_input_buffer()  # Clear any initial data
            self.get_logger().info(f"Connected to serial port {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise e
        
        # Thread-safe queue for data from serial port
        self.serial_queue = queue.Queue(maxsize=100)
        
        # Initialize state variables with thread locks
        self._lock = threading.RLock()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = self.get_clock().now()
        
        # Initialize encoder variables
        self.prev_left_encoder = None
        self.prev_right_encoder = None
        self.first_reading = True
        
        # Setup serial reading thread
        self.stop_thread = False
        self.serial_thread = threading.Thread(target=self._read_from_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # Performance tracking
        self.last_diagnostic_time = time.time()
        self.callback_count = 0
        self.avg_processing_time = 0.0
        self.max_processing_time = 0.0
        
        # Set log level based on debug parameter
        if self.debug_level <= 0:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        elif self.debug_level == 1:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        self.get_logger().info(f"Odometry node started with parameters:")
        self.get_logger().info(f"  Wheel base: {self.wheel_base}m")
        self.get_logger().info(f"  Wheel radius: {self.wheel_radius}m")
        self.get_logger().info(f"  Encoder resolution: {self.encoder_resolution} ticks/revolution")
        self.get_logger().info(f"  Meters per encoder tick: {self.meters_per_tick}m")
        self.get_logger().info(f"  Update frequency: {self.update_frequency}Hz")
        
        # Create timer for odometry updates at specified frequency
        period_sec = 1.0 / self.update_frequency
        self.update_timer = self.create_timer(
            period_sec, 
            self.update_odometry_callback, 
            callback_group=self.timer_callback_group
        )
        
        # Timer for monitoring node performance
        self.create_timer(5.0, self.publish_diagnostics)
    
    def _read_from_serial(self):
        """Thread function to continuously read from serial port"""
        while not self.stop_thread:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        # Put data in queue with a timeout to avoid blocking if queue is full
                        try:
                            self.serial_queue.put(line, timeout=0.01)
                        except queue.Full:
                            self.get_logger().warning("Serial queue full, dropping data")
                            # Clear the input buffer to avoid buildup
                            self.serial_port.reset_input_buffer()
                else:
                    # Short sleep to avoid CPU spinning when no data
                    time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f"Serial reading error: {e}")
                time.sleep(0.1)  # Prevent rapid error loops
                
    def reset_odometry_callback(self, request, response):
        """Service callback to reset odometry to zero"""
        with self._lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.first_reading = True
            self.prev_left_encoder = None
            self.prev_right_encoder = None
            
            # Reset the serial buffer to start fresh
            self.serial_port.reset_input_buffer()
            # Clear the queue
            while not self.serial_queue.empty():
                try:
                    self.serial_queue.get_nowait()
                except queue.Empty:
                    break
            
        self.get_logger().info("Odometry reset to zero")
        return response
    
    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi"""
        return math.fmod(angle + math.pi, 2 * math.pi) - math.pi
    
    def update_odometry_callback(self):
        """Timer callback to process encoder data and update odometry"""
        callback_start_time = time.time()
        
        # Try to get data from the queue without blocking
        try:
            line = self.serial_queue.get_nowait()
            self.process_encoder_data(line)
            self.callback_count += 1
        except queue.Empty:
            # No data available, just return
            pass
        except Exception as e:
            self.get_logger().error(f"Error in odometry update: {e}")
        
        # Track performance metrics
        processing_time = time.time() - callback_start_time
        self.avg_processing_time = 0.95 * self.avg_processing_time + 0.05 * processing_time
        self.max_processing_time = max(self.max_processing_time, processing_time)
    
    def process_encoder_data(self, line):
        """Process a line of encoder data"""
        try:
            if self.debug_level >= 2:
                self.get_logger().debug(f"Processing data: {line}")
            
            # Parse the raw encoder counts
            # Expected format: "L,R" where L and R are encoder count integers
            parts = line.split(',')
            if len(parts) != 2:
                self.get_logger().warning(f"Invalid data format: {line}. Expected: L,R")
                return
            
            try:
                left_encoder = int(parts[0])
                right_encoder = int(parts[1])
            except ValueError:
                self.get_logger().warning(f"Invalid encoder values: {line}")
                return
                
            with self._lock:
                # First reading - just store the values and return
                if self.first_reading:
                    self.prev_left_encoder = left_encoder
                    self.prev_right_encoder = right_encoder
                    self.prev_time = self.get_clock().now()
                    self.first_reading = False
                    if self.debug_level >= 1:
                        self.get_logger().info(f"Initial encoder values: Left={left_encoder}, Right={right_encoder}")
                    return
                
                # Calculate encoder count differences
                delta_left = left_encoder - self.prev_left_encoder
                delta_right = right_encoder - self.prev_right_encoder
                
                # Handle encoder wrap-around/overflow
                # This depends on your encoder's max count - adjust as needed
                MAX_ENCODER_COUNT = 65535  # Typical for 16-bit encoder counter
                
                # Check for left encoder wrap-around
                if abs(delta_left) > MAX_ENCODER_COUNT/2:
                    if delta_left < 0:
                        delta_left += MAX_ENCODER_COUNT
                    else:
                        delta_left -= MAX_ENCODER_COUNT
                
                # Check for right encoder wrap-around
                if abs(delta_right) > MAX_ENCODER_COUNT/2:
                    if delta_right < 0:
                        delta_right += MAX_ENCODER_COUNT
                    else:
                        delta_right -= MAX_ENCODER_COUNT
                
                # Convert encoder ticks to distance traveled
                left_distance = delta_left * self.meters_per_tick
                right_distance = delta_right * self.meters_per_tick
                
                # Calculate time difference
                current_time = self.get_clock().now()
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                
                if dt <= 0.0 or dt > 1.0:
                    # Unreasonable dt, possibly due to time jumps or very slow processing
                    self.get_logger().warning(f"Suspicious dt value: {dt}s, using default")
                    dt = 1.0 / self.update_frequency
                
                # Calculate linear and angular displacement
                distance = (left_distance + right_distance) / 2.0
                rotation = (right_distance - left_distance) / self.wheel_base
                
                # Calculate velocities
                linear_vel = distance / dt
                angular_vel = rotation / dt
                
                # Sanity check on velocities - filter out physically impossible values
                if abs(linear_vel) > 10.0:  # m/s - unrealistic for most robots
                    self.get_logger().warning(f"Unrealistic linear velocity: {linear_vel} m/s")
                    linear_vel = 0.0
                    distance = 0.0
                
                if abs(angular_vel) > 10.0:  # rad/s - unrealistic for most robots
                    self.get_logger().warning(f"Unrealistic angular velocity: {angular_vel} rad/s")
                    angular_vel = 0.0
                    rotation = 0.0
                
                # Update robot position and orientation
                if abs(rotation) < 0.0001:  # Moving straight
                    # Simple straight-line update
                    self.x += distance * math.cos(self.theta)
                    self.y += distance * math.sin(self.theta)
                else:
                    # Calculate the radius of curvature
                    radius = distance / rotation if rotation != 0 else 0
                    
                    # Update position using arc formulas
                    old_theta = self.theta
                    self.theta = self.normalize_angle(old_theta + rotation)
                    
                    # Avoid division by zero or very small values
                    if abs(radius) > 0.0001:
                        self.x += radius * (math.sin(self.theta) - math.sin(old_theta))
                        self.y += radius * (math.cos(old_theta) - math.cos(self.theta))
                
                # Publish odometry message
                self.publish_odometry(linear_vel, angular_vel, current_time)
                
                # Update previous values for next iteration
                self.prev_left_encoder = left_encoder
                self.prev_right_encoder = right_encoder
                self.prev_time = current_time
                
                # Log debug information if needed
                if self.debug_level >= 2:
                    self.get_logger().debug(
                        f"dL={delta_left}, dR={delta_right}, " +
                        f"dist_L={left_distance:.4f}m, dist_R={right_distance:.4f}m, " +
                        f"dt={dt:.4f}s, " +
                        f"linear_vel={linear_vel:.4f}m/s, angular_vel={angular_vel:.4f}rad/s"
                    )
                    self.get_logger().debug(
                        f"Position: x={self.x:.4f}, y={self.y:.4f}, theta={self.theta:.4f}rad ({math.degrees(self.theta):.1f}°)"
                    )
                
        except Exception as e:
            self.get_logger().error(f"Error processing encoder data: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def publish_odometry(self, linear_vel, angular_vel, current_time):
        """Create and publish odometry message"""
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation as quaternion
        quat = Quaternion()
        quat.w = math.cos(self.theta/2)
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(self.theta/2)
        odom_msg.pose.pose.orientation = quat
        
        # Set velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel
        
        # Set covariance - higher values mean less certainty
        # Simple dynamic covariance based on velocity
        # Pose covariance (x, y, z, roll, pitch, yaw)
        pose_covariance = np.zeros(36)
        pose_covariance[0] = pose_covariance[7] = 0.01 + abs(linear_vel) * 0.1  # x, y
        pose_covariance[35] = 0.01 + abs(angular_vel) * 0.1  # yaw
        
        # Twist covariance (linear x,y,z, angular x,y,z)
        twist_covariance = np.zeros(36)
        twist_covariance[0] = 0.01 + abs(linear_vel) * 0.1  # Linear velocity
        twist_covariance[35] = 0.01 + abs(angular_vel) * 0.1  # Angular velocity
        
        # Set covariances in message
        odom_msg.pose.covariance = list(pose_covariance)
        odom_msg.twist.covariance = list(twist_covariance)
        
        # Publish the message
        self.odom_pub.publish(odom_msg)
    
    def publish_diagnostics(self):
        """Publish diagnostic information about the node's performance"""
        now = time.time()
        elapsed = now - self.last_diagnostic_time
        if elapsed > 0:
            rate = self.callback_count / elapsed
            self.last_diagnostic_time = now
            self.get_logger().info(
                f"Performance: {rate:.1f} updates/s, " +
                f"avg processing: {self.avg_processing_time*1000:.3f}ms, " +
                f"max: {self.max_processing_time*1000:.3f}ms, " +
                f"queue size: {self.serial_queue.qsize()}/{self.serial_queue.maxsize}"
            )
            self.callback_count = 0
            self.max_processing_time = 0.0
    
    def shutdown(self):
        """Clean shutdown of the node"""
        self.stop_thread = True
        if hasattr(self, 'serial_thread'):
            self.serial_thread.join(timeout=1.0)
        if hasattr(self, 'serial_port'):
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    
    # Create and initialize node
    node = EncoderOdometryNode()
    
    # Use a multi-threaded executor for better performance
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        # Spin using the multi-threaded executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()