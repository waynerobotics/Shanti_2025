#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import threading
import math
import time
import logging

# Import the RoboClaw driver
try:
    from differential_drive_base.roboclaw_3 import Roboclaw
except ImportError as e:
    print(f"Error importing Roboclaw library: {e}")
    print("Make sure the roboclaw_3 library is installed correctly")
    raise


class RoboclawPWMControllerNode(Node):
    """
    ROS2 node for controlling a differential drive base using two Roboclaw controllers with PWM.
    Each Roboclaw controls two motors (one on each side of the robot).
    """
    def __init__(self):
        super().__init__('roboclaw_pwm_controller_node')
        
        # Create callback groups for better concurrency
        self.subscription_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Declare parameters directly instead of through a method
        self.declare_parameter('left_roboclaw_port', '/dev/ttyACM0')
        self.declare_parameter('right_roboclaw_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('left_address', 0x80)
        self.declare_parameter('right_address', 0x80)
        self.declare_parameter('wheel_base', 0.5334)
        self.declare_parameter('wheel_radius', 0.1016)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_pwm', 127)
        self.declare_parameter('min_pwm', 20)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('invert_left_motors', False)
        self.declare_parameter('invert_right_motors', False)
        self.declare_parameter('timeout', 0.5)  # Increased from 0.1 to 0.5 seconds
        self.declare_parameter('retries', 5)  # Increased from 3 to 5 retries
        self.declare_parameter('pwm_deadband', 0.05)
        self.declare_parameter('debug_level', 1)
        
        # Get parameters from ROS
        self.get_parameters_from_ros()
        
        # Create mutex for thread safety
        self._lock = threading.RLock()
        
        # Initialize controllers
        self.init_roboclaw_controllers()
        
        # Create cmd_vel subscription
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        # Create services
        self.stop_service = self.create_service(
            Empty, 
            'stop_motors', 
            self.stop_motors_callback,
            callback_group=self.service_callback_group
        )
        
        self.reset_service = self.create_service(
            Empty, 
            'reset_controllers', 
            self.reset_controllers_callback,
            callback_group=self.service_callback_group
        )
        
        # Set up watchdog timer for safety with more lenient timing
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(
            0.2,  # 5 Hz - less aggressive timing
            self.watchdog_callback,
            callback_group=self.timer_callback_group
        )
        self.consecutive_timeouts = 0  # Track consecutive timeouts
        
        # Heartbeat timer - just logs status
        self.heartbeat_timer = self.create_timer(
            5.0,  # 0.2 Hz 
            self.publish_heartbeat,
            callback_group=self.timer_callback_group
        )
        
        self.get_logger().info('Roboclaw PWM controller node initialized')

    def get_parameters_from_ros(self):
        """Get all parameters from ROS parameter server"""
        # RoboClaw communication parameters
        self.left_roboclaw_port = self.get_parameter('left_roboclaw_port').value
        self.right_roboclaw_port = self.get_parameter('right_roboclaw_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.left_address = self.get_parameter('left_address').value
        self.right_address = self.get_parameter('right_address').value
        self.timeout = self.get_parameter('timeout').value
        self.retries = self.get_parameter('retries').value
        
        # Robot physical parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Controller parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.invert_left_motors = self.get_parameter('invert_left_motors').value
        self.invert_right_motors = self.get_parameter('invert_right_motors').value
        self.pwm_deadband = self.get_parameter('pwm_deadband').value
        self.debug_level = self.get_parameter('debug_level').value
        
        # Set log level based on debug parameter
        if self.debug_level <= 0:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        elif self.debug_level == 1:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            
        self.get_logger().info(f"Robot parameters: wheel_base={self.wheel_base}m, wheel_radius={self.wheel_radius}m")
        self.get_logger().info(f"Max speed: linear={self.max_speed}m/s, angular={self.max_angular_speed}rad/s")
        self.get_logger().info(f"PWM range: min={self.min_pwm}, max={self.max_pwm}")

    def init_roboclaw_controllers(self):
        """Initialize both Roboclaw controllers"""
        try:
            # Initialize left controller with the proper timeout and retries
            self.get_logger().info(f"Connecting to left Roboclaw on {self.left_roboclaw_port}")
            self.left_roboclaw = Roboclaw(self.left_roboclaw_port, self.baud_rate, 
                                         timeout=self.timeout, retries=self.retries)
            left_result = self.left_roboclaw.Open()
            
            # Initialize right controller with the proper timeout and retries
            self.get_logger().info(f"Connecting to right Roboclaw on {self.right_roboclaw_port}")
            self.right_roboclaw = Roboclaw(self.right_roboclaw_port, self.baud_rate,
                                          timeout=self.timeout, retries=self.retries)
            right_result = self.right_roboclaw.Open()
            
            if left_result and right_result:
                self.get_logger().info("Successfully connected to both Roboclaw controllers")
                
                # Get firmware versions for logging
                left_version = self.left_roboclaw.ReadVersion(self.left_address)
                if left_version[0]:
                    self.get_logger().info(f"Left Roboclaw firmware version: {left_version[1]}")
                
                right_version = self.right_roboclaw.ReadVersion(self.right_address)
                if right_version[0]:
                    self.get_logger().info(f"Right Roboclaw firmware version: {right_version[1]}")
                
                # Initialize motor PWM to zero
                self.stop_all_motors()
                return True
            else:
                if not left_result:
                    self.get_logger().error(f"Failed to connect to left Roboclaw on {self.left_roboclaw_port}")
                if not right_result:
                    self.get_logger().error(f"Failed to connect to right Roboclaw on {self.right_roboclaw_port}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error initializing Roboclaw controllers: {e}")
            return False

    def differential_drive_to_wheel_pwm(self, linear_x, angular_z):
        """Convert linear and angular velocities to individual wheel PWM values"""
        # Apply simple differential drive kinematics
        # v_l = linear_x - (angular_z * wheel_base / 2)
        # v_r = linear_x + (angular_z * wheel_base / 2)
        
        # Clamp the maximum/minimum velocities
        linear_x = max(min(linear_x, self.max_speed), -self.max_speed)
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
        
        # Calculate wheel velocities - THIS IS THE KEY PART FOR TURNING
        wheel_distance = self.wheel_base / 2.0
        left_wheel_vel = linear_x - (angular_z * wheel_distance) / self.wheel_radius
        right_wheel_vel = linear_x + (angular_z * wheel_distance) / self.wheel_radius
        

        self.get_logger().debug(
            f"Raw wheel velocities: left={left_wheel_vel:.3f}, right={right_wheel_vel:.3f} for "
            f"linear={linear_x:.2f}, angular={angular_z:.2f}"
        )
        
        # Apply motor inversions if configured
        if self.invert_left_motors:
            left_wheel_vel = -left_wheel_vel
        if self.invert_right_motors:
            right_wheel_vel = -right_wheel_vel
            
        # Convert velocities to PWM values (linear scaling)
        # First, calculate the percentage of max speed for each wheel
        # Use the maximum of max_speed and max_angular_speed*wheel_base/2 for better scaling
        effective_max = max(self.max_speed, self.max_angular_speed * wheel_distance)
        
        left_percent = left_wheel_vel / effective_max
        right_percent = right_wheel_vel / effective_max
        
        # Apply deadband - if percentage is less than deadband, set to 0
        if abs(left_percent) < self.pwm_deadband:
            left_percent = 0.0
        if abs(right_percent) < self.pwm_deadband:
            right_percent = 0.0
        
        # Calculate PWM values with min_pwm offset for non-zero values
        # Directly scale to the full Roboclaw duty range (-32767 to 32767)
        max_duty = 32767
        min_duty = int(max_duty * (self.min_pwm / self.max_pwm))
        
        left_pwm = 0
        if left_percent != 0:
            # Scale between min_duty and max_duty
            left_pwm_magnitude = min_duty + abs(left_percent) * (max_duty - min_duty)
            left_pwm = int(math.copysign(left_pwm_magnitude, left_percent))
            
        right_pwm = 0
        if right_percent != 0:
            # Scale between min_duty and max_duty
            right_pwm_magnitude = min_duty + abs(right_percent) * (max_duty - min_duty)
            right_pwm = int(math.copysign(right_pwm_magnitude, right_percent))
        
        # Log the conversion for debugging
        if self.debug_level >= 2:
            self.get_logger().debug(
                f"Wheel velocities: left={left_wheel_vel:.3f}, right={right_wheel_vel:.3f} | "
                f"Effective max: {effective_max:.3f} | "
                f"Percentages: left={left_percent:.3f}, right={right_percent:.3f} | "
                f"PWM values: left={left_pwm}, right={right_pwm}"
            )
        
        return left_pwm, right_pwm

    def set_left_motors_pwm(self, pwm_value):
        """Set PWM for both motors on left side (controlled by left Roboclaw)"""
        try:
            success = self.send_pwm_with_retry(
                self.left_roboclaw, 
                self.left_address,
                pwm_value,    # M1
                -pwm_value    # M2 - reversed
            )
            
            if success and self.debug_level >= 2:
                self.get_logger().debug(f"Set left motors PWM to M1:{pwm_value}, M2:{-pwm_value}")
            return success
        except Exception as e:
            self.get_logger().error(f"Error setting left motors PWM: {e}")
            self.handle_communication_error()
            return False

    def set_right_motors_pwm(self, pwm_value):
        """Set PWM for both motors on right side (controlled by right Roboclaw)"""
        try:
            success = self.send_pwm_with_retry(
                self.right_roboclaw,
                self.right_address,
                pwm_value,    # M1
                -pwm_value    # M2 - reversed
            )
            
            if success and self.debug_level >= 2:
                self.get_logger().debug(f"Set right motors PWM to M1:{pwm_value}, M2:{-pwm_value}")
            return success
        except Exception as e:
            self.get_logger().error(f"Error setting right motors PWM: {e}")
            self.handle_communication_error()
            return False

    def send_pwm_with_retry(self, roboclaw, address, motor1_pwm, motor2_pwm, retries=3):
        """Send PWM commands with retry mechanism"""
        for attempt in range(retries):
            try:
                success1 = roboclaw.DutyM1(address, motor1_pwm)
                success2 = roboclaw.DutyM2(address, motor2_pwm)
                
                if success1 and success2:
                    return True
                
                if attempt < retries - 1:
                    self.get_logger().warn(f"PWM command failed, retrying... (attempt {attempt + 1}/{retries})")
                    time.sleep(0.1 * (attempt + 1))  # Progressive delay
            except Exception as e:
                if attempt < retries - 1:
                    self.get_logger().error(f"PWM command error: {e}, retrying...")
                    time.sleep(0.1 * (attempt + 1))
                else:
                    self.get_logger().error(f"PWM command failed after {retries} attempts: {e}")
        return False

    def cmd_vel_callback(self, msg):
        """Callback function for cmd_vel topic subscription"""
        with self._lock:
            # Update timestamp for watchdog
            self.last_cmd_time = self.get_clock().now()
            
            # Log the incoming command
            self.get_logger().debug(f"Received cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
            
            # Convert to wheel PWM values
            left_pwm, right_pwm = self.differential_drive_to_wheel_pwm(
                msg.linear.x, msg.angular.z)
            
            # Apply to motors
            left_success = self.set_left_motors_pwm(left_pwm)
            right_success = self.set_right_motors_pwm(right_pwm)
            
            if self.debug_level >= 1:
                self.get_logger().debug(
                    f"CMD_VEL: linear={msg.linear.x:.2f} m/s, angular={msg.angular.z:.2f} rad/s -> "
                    f"left_pwm={left_pwm}, right_pwm={right_pwm}")
            
            if not (left_success and right_success):
                self.get_logger().warning("Failed to set some motor PWM values")

    def watchdog_callback(self):
        """Watchdog callback to stop motors if no cmd_vel received for a while"""
        try:
            now = self.get_clock().now()
            time_diff = (now - self.last_cmd_time).nanoseconds / 1e9
            
            if time_diff > self.cmd_timeout:
                # No recent command, stop motors for safety
                if self.debug_level >= 1:
                    self.get_logger().debug(f"Watchdog timeout after {time_diff:.2f}s, stopping motors")
                self.stop_all_motors()
                self.handle_communication_error()
            else:
                # Reset timeout counter if everything is working
                self.consecutive_timeouts = 0
        except Exception as e:
            self.get_logger().error(f"Error in watchdog: {e}")
            self.handle_communication_error()

    def stop_all_motors(self):
        """Stop all motors on both controllers"""
        success = True
        
        # Try to stop left motors
        try:
            left_success = self.send_pwm_with_retry(
                self.left_roboclaw,
                self.left_address,
                0,  # M1
                0   # M2
            )
            if not left_success:
                self.get_logger().error("Failed to stop left motors")
                success = False
        except Exception as e:
            self.get_logger().error(f"Error stopping left motors: {e}")
            success = False
            
        # Try to stop right motors
        try:
            right_success = self.send_pwm_with_retry(
                self.right_roboclaw,
                self.right_address,
                0,  # M1
                0   # M2
            )
            if not right_success:
                self.get_logger().error("Failed to stop right motors")
                success = False
        except Exception as e:
            self.get_logger().error(f"Error stopping right motors: {e}")
            success = False
            
        if success:
            self.get_logger().info("All motors stopped successfully")
        else:
            self.get_logger().warning("Some motors may not have stopped properly")
            
        return success

    def stop_motors_callback(self, request, response):
        """Service callback to stop all motors"""
        with self._lock:
            self.stop_all_motors()
        return response

    def reset_controllers_callback(self, request, response):
        """Service callback to reset Roboclaw controllers"""
        with self._lock:
            try:
                # Stop all motors first
                self.stop_all_motors()
                
                # Explicitly close the ports
                try:
                    if hasattr(self.left_roboclaw, '_port') and self.left_roboclaw._port.is_open:
                        self.left_roboclaw._port.close()
                    if hasattr(self.right_roboclaw, '_port') and self.right_roboclaw._port.is_open:
                        self.right_roboclaw._port.close()
                except:
                    pass
                
                # Reopen connections
                self.init_roboclaw_controllers()
                
                self.get_logger().info("Controllers reset successfully")
            except Exception as e:
                self.get_logger().error(f"Error resetting controllers: {e}")
                
        return response

    def publish_heartbeat(self):
        """Publish heartbeat information to show the node is running"""
        self.get_logger().info("Roboclaw PWM controller node is running")

    def shutdown(self):
        """Clean shutdown of the node"""
        self.get_logger().info("Shutting down Roboclaw PWM controller node")
        
        # Stop all motors
        self.stop_all_motors()
        
        # Close serial connections
        try:
            if hasattr(self.left_roboclaw, '_port') and self.left_roboclaw._port.is_open:
                self.left_roboclaw._port.close()
            if hasattr(self.right_roboclaw, '_port') and self.right_roboclaw._port.is_open:
                self.right_roboclaw._port.close()
        except:
            pass

    def reconnect_controllers(self):
        """Attempt to reconnect to the Roboclaw controllers"""
        try:
            # Close existing connections if they exist
            if hasattr(self, 'left_roboclaw'):
                try:
                    self.left_roboclaw._port.close()
                except:
                    pass
            if hasattr(self, 'right_roboclaw'):
                try:
                    self.right_roboclaw._port.close()
                except:
                    pass
            
            # Wait a moment before reconnecting
            time.sleep(1.0)
            
            # Attempt to reinitialize
            success = self.init_roboclaw_controllers()
            if success:
                self.get_logger().info("Successfully reconnected to Roboclaw controllers")
                return True
        except Exception as e:
            self.get_logger().error(f"Failed to reconnect to controllers: {e}")
        return False

    def handle_communication_error(self):
        """Handle communication errors with a progressive backoff"""
        self.consecutive_timeouts += 1
        if self.consecutive_timeouts >= 3:
            self.get_logger().warn("Multiple consecutive timeouts, attempting to reconnect...")
            if self.reconnect_controllers():
                self.consecutive_timeouts = 0
            else:
                # If reconnection fails, stop motors for safety
                self.stop_all_motors()
                time.sleep(1.0)  # Wait before next attempt


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RoboclawPWMControllerNode()
        
        # Use a MultiThreadedExecutor for better performance
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            # Clean shutdown
            node.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f"Error in Roboclaw PWM controller node: {e}")
    finally:
        # Ensure ROS is shut down properly
        rclpy.shutdown()


if __name__ == '__main__':
    main()