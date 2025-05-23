import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion, Vector3
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
import serial
from math import sin, cos, pi, atan2, fmod
import numpy as np

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create reset service
        self.reset_service = self.create_service(
            Empty, 'reset_odometry', self.reset_odometry_callback)
            
        # Declare parameters with default values
        self.declare_parameter('wheel_base', 0.5334)  # Distance between wheels in meters
        self.declare_parameter('wheel_radius', 0.1016)  # Wheel radius in meters
        self.declare_parameter('left_wheel_correction', 1.0)  # Correction factor for left wheel
        self.declare_parameter('right_wheel_correction', 1.0)  # Correction factor for right wheel
        self.declare_parameter('encoder_resolution', 360)  # Encoder ticks per revolution
        self.declare_parameter('update_frequency', 20.0)  # Hz
        self.declare_parameter('max_acceleration', 5.0)  # m/s²
        self.declare_parameter('covariance_scale', 0.1)  # Scale factor for covariance
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        
        # Get parameter values
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.left_correction = self.get_parameter('left_wheel_correction').value
        self.right_correction = self.get_parameter('right_wheel_correction').value
        self.update_frequency = self.get_parameter('update_frequency').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.covariance_scale = self.get_parameter('covariance_scale').value
        
        # Setup serial connection
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.serial_port = serial.Serial(port, baud)
        
        # Initialize state variables
        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Initialize velocity tracking for filtering
        self.prev_v_left = 0.0
        self.prev_v_right = 0.0
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0
        
        # Set log level for this node to debug to see all messages
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Wait for serial port to stabilize
        import time
        time.sleep(2.0)
        
        self.get_logger().info(f"Odometry node started with parameters:")
        self.get_logger().info(f"  Wheel base: {self.wheel_base}m")
        self.get_logger().info(f"  Wheel radius: {self.wheel_radius}m")
        self.get_logger().info(f"  Wheel correction factors: L={self.left_correction}, R={self.right_correction}")
        self.get_logger().info(f"  Update frequency: {self.update_frequency}Hz")
        
        # Display a message about the expected data format
        self.get_logger().info("Expecting serial data in format: E1_RPM:value,E1_DIR:value,E2_RPM:value,E2_DIR:value")
        
        # Try to read some initial data to show the actual format
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode().strip()
                self.get_logger().info(f"Example of actual data received: {line}")
                parsed = self.parse_serial_data(line)
                self.get_logger().info(f"Parsed as: {parsed}")
            except Exception as e:
                self.get_logger().error(f"Error reading initial data: {str(e)}")
                
        # Create timer for odometry updates at specified frequency
        period_sec = 1.0 / self.update_frequency
        self.create_timer(period_sec, self.update_odometry)
        
    def reset_odometry_callback(self, request, response):
        """Service callback to reset odometry to zero"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_v_left = 0.0
        self.prev_v_right = 0.0
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0
        self.prev_time = self.get_clock().now()
        self.get_logger().info("Odometry reset to zero")
        return response

    def parse_serial_data(self, line):
        data = {}
        # Split by comma first, then by colon
        parts = line.strip().split(',')
        for pair in parts:
            if ':' in pair:
                key, value = pair.split(':')
                try:
                    # Try to convert to float if possible
                    data[key] = float(value)
                except ValueError:
                    # Keep as string if not a number
                    data[key] = value
            else:
                self.get_logger().warning(f"Invalid data format (missing colon): {pair}")
        return data

    def normalize_angle(self, angle):
        """
        Normalize angle to be between -pi and pi
        """
        return fmod(angle + pi, 2 * pi) - pi

    def apply_low_pass_filter(self, current_value, previous_value, alpha=0.2):
        """
        Apply a simple low-pass filter to smooth values
        alpha = 0 means no filtering (use current value)
        alpha = 1 means ignore current value (use previous value)
        """
        return alpha * previous_value + (1 - alpha) * current_value

    def update_odometry(self):
        try:
            # Check if data is available to read
            if not self.serial_port.in_waiting > 0:
                return  # No data available yet
                
            # Read data from serial port
            line = self.serial_port.readline().decode().strip()
            self.get_logger().debug(f"Received data: {line}")
            data = self.parse_serial_data(line)
            
            # Log the parsed data for debugging
            self.get_logger().debug(f"Parsed data: {data}")
            
            # Check if all required keys are present
            required_keys = ['E1_RPM', 'E1_DIR', 'E2_RPM', 'E2_DIR']
            if not all(key in data for key in required_keys):
                missing_keys = [key for key in required_keys if key not in data]
                self.get_logger().warning(f"Missing required keys: {missing_keys}. Skipping odometry update.")
                return
            
            # Extract RPM and direction for both wheels
            rpm_left = float(data['E1_RPM'])
            dir_left = 1 if data['E1_DIR'] == 'F' else -1
            rpm_right = float(data['E2_RPM'])
            dir_right = 1 if data['E2_DIR'] == 'F' else -1
            
            # Log direction information for debugging
            self.get_logger().debug(f"Wheel directions: Left={data['E1_DIR']}({dir_left}), Right={data['E2_DIR']}({dir_right})")
            
            # Get current time and compute dt
            current_time = self.get_clock().now()
            dt = 1.0 / self.update_frequency  # Use expected dt instead
            
            # Compute wheel velocities (m/s) with correction factors
            v_left = dir_left * rpm_left * (2 * pi * self.wheel_radius) / 60.0 * self.left_correction
            v_right = dir_right * rpm_right * (2 * pi * self.wheel_radius) / 60.0 * self.right_correction
            
            # Apply low-pass filter to reduce noise in wheel velocities
            v_left = self.apply_low_pass_filter(v_left, self.prev_v_left)
            v_right = self.apply_low_pass_filter(v_right, self.prev_v_right)
            
            # Apply acceleration limits to prevent physically impossible jumps
            v_left = self.apply_acceleration_limit(v_left, self.prev_v_left, self.max_acceleration, dt)
            v_right = self.apply_acceleration_limit(v_right, self.prev_v_right, self.max_acceleration, dt)
            
            # Store current velocities for next iteration
            self.prev_v_left = v_left
            self.prev_v_right = v_right
            
            # Differential drive kinematics
            linear_vel = (v_left + v_right) / 2.0
            angular_vel = (v_right - v_left) / self.wheel_base
            
            # Apply filtering to computed velocities
            linear_vel = self.apply_low_pass_filter(linear_vel, self.prev_linear_vel)
            angular_vel = self.apply_low_pass_filter(angular_vel, self.prev_angular_vel)
            
            # Store for next iteration
            self.prev_linear_vel = linear_vel
            self.prev_angular_vel = angular_vel
            
            # More accurate pose update using small-angle approximations when appropriate
            if abs(angular_vel) < 0.0001:  # Going straight
                # Simple integration for straight line motion
                dx = linear_vel * cos(self.theta) * dt
                dy = linear_vel * sin(self.theta) * dt
                self.x += dx
                self.y += dy
            else:  # Following an arc
                # Use arc-based integration for better accuracy during turns
                old_theta = self.theta
                self.theta = self.normalize_angle(self.theta + angular_vel * dt)
                
                # Calculate position change using arc equations
                turning_radius = linear_vel / angular_vel
                self.x += turning_radius * (sin(self.theta) - sin(old_theta))
                self.y += turning_radius * (cos(old_theta) - cos(self.theta))
            
            # Build and publish odometry message
            self.publish_odometry(linear_vel, angular_vel, current_time)
            
            # Update timestamp for next iteration
            self.prev_time = current_time
            
            # Log data for debugging
            self.get_logger().debug(f"v_left={v_left:.3f}, v_right={v_right:.3f}, " +
                                  f"linear={linear_vel:.3f}, angular={angular_vel:.3f}, " +
                                  f"x={self.x:.3f}, y={self.y:.3f}, theta={self.theta*180.0/pi:.1f}°")
                                  
        except Exception as e:
            self.get_logger().error(f"Error in update_odometry: {str(e)}")

    def apply_acceleration_limit(self, current_vel, prev_vel, max_accel, dt):
        """Limit acceleration to prevent physically impossible jumps"""
        max_vel_change = max_accel * dt
        vel_change = current_vel - prev_vel
        if abs(vel_change) > max_vel_change:
            # Limit the velocity change to the maximum allowed
            return prev_vel + max_vel_change * (1 if vel_change > 0 else -1)
        return current_vel

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
        quat.w = cos(self.theta/2)
        quat.x = 0.0
        quat.y = 0.0
        quat.z = sin(self.theta/2)
        odom_msg.pose.pose.orientation = quat
        
        # Set velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel
        
        # Set covariance - higher values mean less certainty
        # Pose covariance (x, y, z, roll, pitch, yaw)
        pose_covariance = np.zeros(36)
        pose_covariance[0] = pose_covariance[7] = self.covariance_scale * abs(linear_vel)  # x, y uncertainty increases with speed
        pose_covariance[35] = self.covariance_scale * abs(angular_vel) + 0.01  # yaw uncertainty increases with rotation speed
        
        # Twist covariance (linear x,y,z, angular x,y,z)
        twist_covariance = np.zeros(36)
        twist_covariance[0] = self.covariance_scale  # Linear velocity uncertainty
        twist_covariance[35] = self.covariance_scale  # Angular velocity uncertainty
        
        # Set covariances in message
        odom_msg.pose.covariance = list(pose_covariance)
        odom_msg.twist.covariance = list(twist_covariance)
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()