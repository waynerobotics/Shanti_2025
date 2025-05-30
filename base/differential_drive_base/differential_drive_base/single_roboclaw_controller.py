#!/usr/bin/env python3
"""
Single RoboClaw Controller Node

This node controls a single RoboClaw motor controller.
It subscribes to PWM commands and sends them to the controller.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int32, Bool
from std_srvs.srv import Empty, Trigger
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import math
import time
import logging
import threading
import serial
import sys

# Import the RoboClaw driver
try:
    from differential_drive_base.roboclaw_3 import Roboclaw
except ImportError as e:
    print(f"Error importing Roboclaw library: {e}")
    print("Make sure the roboclaw_3 library is installed correctly")
    raise


class SingleRoboclawControllerNode(Node):
    """
    ROS2 node for controlling a single Roboclaw motor controller.
    """
    def __init__(self):
        super().__init__('single_roboclaw_controller_node')
        
        # Create callback groups for better concurrency
        self.subscription_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Declare parameters directly
        self.declare_parameter('roboclaw_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('address', 0x80)
        self.declare_parameter('timeout', 1.0)  # Increased default timeout
        self.declare_parameter('retries', 5)
        self.declare_parameter('debug_level', 1)
        self.declare_parameter('motor1_invert', False)
        self.declare_parameter('motor2_invert', True)  # Usually inverted by default
        self.declare_parameter('pwm_topic_prefix', 'motor_controller')
        self.declare_parameter('reconnect_interval', 2.0)  # Seconds between reconnection attempts
        self.declare_parameter('heartbeat_interval', 0.2)  # Seconds between heartbeat checks (5Hz)
        self.declare_parameter('control_rate', 0.2)  # 5Hz control rate
        self.declare_parameter('max_consecutive_errors', 3)  # Maximum consecutive errors before backing off
        
        # Get parameters from ROS
        self.get_parameters_from_ros()
        
        # Connection status
        self.connected = False
        self.connection_lock = threading.Lock()  # For connection status
        self.serial_lock = threading.RLock()     # For thread-safe serial access
        self.reconnect_timer = None
        self.consecutive_errors = 0
        self.max_consecutive_errors = self.get_parameter('max_consecutive_errors').value
        
        # Status publisher
        self.status_pub = self.create_publisher(
            Bool,
            f'{self.pwm_topic_prefix}/connected',
            10
        )
        
        # Initialize controller
        self.init_roboclaw_controller()
        
        # Create PWM command subscriptions for each motor
        self.motor1_pwm_sub = self.create_subscription(
            Int32,
            f'{self.pwm_topic_prefix}/motor1_pwm',
            self.motor1_pwm_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        self.motor2_pwm_sub = self.create_subscription(
            Int32,
            f'{self.pwm_topic_prefix}/motor2_pwm',
            self.motor2_pwm_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        # Create services
        self.stop_service = self.create_service(
            Empty, 
            f'{self.pwm_topic_prefix}/stop_motors', 
            self.stop_motors_callback,
            callback_group=self.service_callback_group
        )
        
        self.reset_service = self.create_service(
            Trigger, 
            f'{self.pwm_topic_prefix}/reset_controller', 
            self.reset_controller_callback,
            callback_group=self.service_callback_group
        )
        
        # Heartbeat timer - checks connection and logs status
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_interval,
            self.publish_heartbeat,
            callback_group=self.timer_callback_group
        )
        
        # Safety watchdogs for each motor
        self.motor1_last_cmd_time = self.get_clock().now()
        self.motor2_last_cmd_time = self.get_clock().now()
        self.motor_timeout = 0.5  # seconds
        
        self.watchdog_timer = self.create_timer(
            self.control_rate,  # 5 Hz (0.2s)
            self.watchdog_callback,
            callback_group=self.timer_callback_group
        )
        
        self.get_logger().info('Single Roboclaw controller node initialized')

    def get_parameters_from_ros(self):
        """Get all parameters from ROS parameter server"""
        # RoboClaw communication parameters
        self.roboclaw_port = self.get_parameter('roboclaw_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.address = self.get_parameter('address').value
        self.timeout = self.get_parameter('timeout').value
        self.retries = self.get_parameter('retries').value
        self.debug_level = self.get_parameter('debug_level').value
        self.motor1_invert = self.get_parameter('motor1_invert').value
        self.motor2_invert = self.get_parameter('motor2_invert').value
        self.pwm_topic_prefix = self.get_parameter('pwm_topic_prefix').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.control_rate = self.get_parameter('control_rate').value
        
        # Set log level based on debug parameter
        if self.debug_level <= 0:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        elif self.debug_level == 1:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            
        self.get_logger().info(f"Roboclaw port: {self.roboclaw_port}, address: {self.address:#x}")
        self.get_logger().info(f"Reconnect interval: {self.reconnect_interval}s, Control rate: {self.control_rate}s (5Hz)")

    def init_roboclaw_controller(self):
        """Initialize the Roboclaw controller"""
        with self.serial_lock:
            try:
                # Close the port if it's already open to ensure clean state
                if hasattr(self, 'roboclaw') and hasattr(self.roboclaw, '_port') and self.roboclaw._port.is_open:
                    try:
                        self.roboclaw._port.close()
                        time.sleep(0.1)  # Give the OS a moment to release the port
                        self.get_logger().info("Closed existing port before reconnecting")
                    except Exception as e:
                        self.get_logger().error(f"Error closing existing port: {e}")
                
                # Initialize controller with the proper timeout and retries
                self.get_logger().info(f"Connecting to Roboclaw on {self.roboclaw_port}")
                self.roboclaw = Roboclaw(self.roboclaw_port, self.baud_rate, 
                                        timeout=self.timeout, retries=self.retries)
                result = self.roboclaw.Open()
                
                # Apply Linux-specific serial optimizations
                if sys.platform == 'linux' and hasattr(self.roboclaw, '_port'):
                    try:
                        # Set low latency flag for better responsiveness
                        self.roboclaw._port.low_latency = True
                        
                        # Disable exclusive access to allow reconnection
                        self.roboclaw._port.exclusive = False
                        
                        # Adjust serial parameters
                        self.roboclaw._port.inter_byte_timeout = 0.01  # Short inter-byte timeout
                        
                        self.get_logger().info("Applied serial port optimizations")
                    except Exception as e:
                        self.get_logger().warn(f"Could not set serial port optimizations: {e}")
                
                if result:
                    self.get_logger().info("Successfully connected to Roboclaw controller")
                    
                    # Get firmware version for logging
                    version = self.roboclaw.ReadVersion(self.address)
                    if version[0]:
                        version_data = version[1]
                        try:
                            # Handle both string and bytes version formats
                            if hasattr(version_data, 'decode'):
                                version_str = version_data.decode('utf-8').strip()
                            else:
                                version_str = str(version_data).strip()
                            self.get_logger().info(f"Roboclaw firmware version: {version_str}")
                        except Exception as e:
                            self.get_logger().warn(f"Error decoding version: {e}, raw: {version_data}")
                    
                    # Initialize motor PWM to zero
                    self.stop_all_motors()
                    self.connected = True
                    self.consecutive_errors = 0  # Reset error counter on successful connection
                    
                    # Publish initial status
                    self.publish_connection_status()
                    return True
                else:
                    self.get_logger().error(f"Failed to connect to Roboclaw on {self.roboclaw_port}")
                    self.connected = False
                    self.schedule_reconnect()
                    return False
                    
            except Exception as e:
                self.get_logger().error(f"Error initializing Roboclaw controller: {e}")
                self.connected = False
                self.schedule_reconnect()
                return False

    def handle_serial_error(self, error, operation="serial operation"):
        """Handle serial communication errors in a centralized way"""
        self.get_logger().error(f"Serial error during {operation}: {error}")
        
        # Increment error counter
        self.consecutive_errors += 1
        
        # Mark connection as broken if too many errors
        if self.consecutive_errors >= self.max_consecutive_errors:
            with self.connection_lock:
                if self.connected:
                    self.connected = False
                    self.publish_connection_status()
                    self.get_logger().error(f"Connection marked as broken after {self.consecutive_errors} consecutive errors")
            
            # Schedule reconnection
            self.schedule_reconnect()
        
        return False

    def set_motor1_pwm(self, pwm_value):
        """Set PWM for motor 1"""
        if not self.connected:
            return False
            
        try:
            # Apply inversion if configured
            if self.motor1_invert:
                pwm_value = -pwm_value
            
            with self.serial_lock:
                success = self.send_pwm_with_retry(
                    self.roboclaw, 
                    self.address,
                    pwm_value,  # M1
                    None        # Don't change M2
                )
            
            if success and self.debug_level >= 2:
                self.get_logger().debug(f"Set motor 1 PWM to {pwm_value}")
            return success
        except (serial.SerialException, OSError) as e:
            return self.handle_serial_error(e, "setting motor 1 PWM")
        except Exception as e:
            self.get_logger().error(f"Error setting motor 1 PWM: {e}")
            return False

    def set_motor2_pwm(self, pwm_value):
        """Set PWM for motor 2"""
        if not self.connected:
            return False
            
        try:
            # Apply inversion if configured
            if self.motor2_invert:
                pwm_value = -pwm_value
            
            with self.serial_lock:
                success = self.send_pwm_with_retry(
                    self.roboclaw,
                    self.address,
                    None,        # Don't change M1
                    pwm_value    # M2
                )
            
            if success and self.debug_level >= 2:
                self.get_logger().debug(f"Set motor 2 PWM to {pwm_value}")
            return success
        except (serial.SerialException, OSError) as e:
            return self.handle_serial_error(e, "setting motor 2 PWM")
        except Exception as e:
            self.get_logger().error(f"Error setting motor 2 PWM: {e}")
            return False

    def send_pwm_with_retry(self, roboclaw, address, motor1_pwm=None, motor2_pwm=None, retries=3):
        """Send PWM commands without retry mechanism (name kept for compatibility)"""
        # Must be called with serial_lock already acquired
        try:
            success1 = True
            success2 = True
            
            if motor1_pwm is not None:
                success1 = roboclaw.DutyM1(address, motor1_pwm)
                
                # Check for errors if command failed
                if not success1:
                    try:
                        error_status = roboclaw.ReadError(address)[1]
                        if error_status != 0:
                            error_msg = self.decode_roboclaw_error(error_status)
                            self.get_logger().error(f"Motor 1 PWM error: {error_msg} (code: {error_status:#x})")
                    except Exception as err:
                        self.get_logger().error(f"Failed to read error status after Motor 1 PWM failure: {err}")
            
            if motor2_pwm is not None:
                success2 = roboclaw.DutyM2(address, motor2_pwm)
                
                # Check for errors if command failed
                if not success2:
                    try:
                        error_status = roboclaw.ReadError(address)[1]
                        if error_status != 0:
                            error_msg = self.decode_roboclaw_error(error_status)
                            self.get_logger().error(f"Motor 2 PWM error: {error_msg} (code: {error_status:#x})")
                    except Exception as err:
                        self.get_logger().error(f"Failed to read error status after Motor 2 PWM failure: {err}")
            
            if success1 and success2:
                # Successful operation - reset error counter
                self.consecutive_errors = 0
                return True
            else:
                # Increment error counter on command failures
                self.consecutive_errors += 1
                self.get_logger().warn(f"PWM command failed: motor1={not success1 if motor1_pwm is not None else 'not set'}, motor2={not success2 if motor2_pwm is not None else 'not set'}, error count: {self.consecutive_errors}")
                
                # If too many failures, mark the connection as broken
                if self.consecutive_errors >= self.max_consecutive_errors:
                    with self.connection_lock:
                        if self.connected:
                            self.connected = False
                            self.publish_connection_status()
                            self.get_logger().error(f"Connection marked as broken after {self.consecutive_errors} PWM failures")
                            self.schedule_reconnect()
                
                return False
        except Exception as e:
            self.consecutive_errors += 1
            self.get_logger().error(f"PWM command error: {e}, error count: {self.consecutive_errors}")
            
            # If too many failures, mark the connection as broken
            if self.consecutive_errors >= self.max_consecutive_errors:
                with self.connection_lock:
                    if self.connected:
                        self.connected = False
                        self.publish_connection_status()
                        self.get_logger().error(f"Connection marked as broken after {self.consecutive_errors} errors")
                        self.schedule_reconnect()
            
            return False

    def motor1_pwm_callback(self, msg):
        """Callback function for motor1_pwm topic"""
        self.motor1_last_cmd_time = self.get_clock().now()
        
        # Log the incoming command
        if self.debug_level >= 1:
            self.get_logger().debug(f"Received motor1_pwm: {msg.data}")
        
        # Apply to motor
        success = self.set_motor1_pwm(msg.data)
        
        if not success:
            self.get_logger().warning("Failed to set motor 1 PWM value")

    def motor2_pwm_callback(self, msg):
        """Callback function for motor2_pwm topic"""
        self.motor2_last_cmd_time = self.get_clock().now()
        
        # Log the incoming command
        if self.debug_level >= 1:
            self.get_logger().debug(f"Received motor2_pwm: {msg.data}")
        
        # Apply to motor
        success = self.set_motor2_pwm(msg.data)
        
        if not success:
            self.get_logger().warning("Failed to set motor 2 PWM value")

    def watchdog_callback(self):
        """Watchdog to stop motors if no commands received for a while"""
        if not self.connected:
            return
            
        try:
            now = self.get_clock().now()
            
            # Check motor 1
            time_diff_m1 = (now - self.motor1_last_cmd_time).nanoseconds / 1e9
            if time_diff_m1 > self.motor_timeout:
                # No recent command, stop motor 1
                if self.debug_level >= 1:
                    self.get_logger().debug(f"Motor 1 watchdog timeout after {time_diff_m1:.2f}s, stopping")
                
                # Use the same lock as in set_motor1_pwm
                try:
                    with self.serial_lock:
                        self.set_motor1_pwm(0)
                except Exception as e:
                    self.get_logger().error(f"Error in watchdog stopping motor 1: {e}")
            
            # Check motor 2
            time_diff_m2 = (now - self.motor2_last_cmd_time).nanoseconds / 1e9
            if time_diff_m2 > self.motor_timeout:
                # No recent command, stop motor 2
                if self.debug_level >= 1:
                    self.get_logger().debug(f"Motor 2 watchdog timeout after {time_diff_m2:.2f}s, stopping")
                
                # Use the same lock as in set_motor2_pwm
                try:
                    with self.serial_lock:
                        self.set_motor2_pwm(0)
                except Exception as e:
                    self.get_logger().error(f"Error in watchdog stopping motor 2: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in watchdog callback: {e}")
            # Don't stop motors or mark connection as broken here to avoid cascading failures

    def stop_all_motors(self):
        """Stop all motors on the controller"""
        if not self.connected:
            return False
            
        try:
            with self.serial_lock:
                success = self.send_pwm_with_retry(
                    self.roboclaw,
                    self.address,
                    0,  # M1
                    0   # M2
                )
            
            if success:
                self.get_logger().info("All motors stopped successfully")
            else:
                self.get_logger().warning("Failed to stop motors")
                
            return success
        except (serial.SerialException, OSError) as e:
            return self.handle_serial_error(e, "stopping all motors")
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {e}")
            return False

    def stop_motors_callback(self, request, response):
        """Service callback to stop all motors"""
        self.stop_all_motors()
        return response

    def reset_controller_callback(self, request, response):
        """Service callback to reset Roboclaw controller"""
        try:
            # Stop all motors first
            self.stop_all_motors()
            
            with self.serial_lock:
                # Explicitly close the port
                try:
                    if hasattr(self.roboclaw, '_port') and self.roboclaw._port.is_open:
                        self.roboclaw._port.close()
                        self.get_logger().info("Closed port for reset")
                except Exception as e:
                    self.get_logger().error(f"Error closing port: {e}")
            
            # Reopen connection
            success = self.init_roboclaw_controller()
            
            if success:
                response.success = True
                response.message = "Controller reset successfully"
                self.get_logger().info("Controller reset successfully")
            else:
                response.success = False
                response.message = "Failed to reset controller"
                self.get_logger().error("Failed to reset controller")
        except Exception as e:
            response.success = False
            response.message = f"Error resetting controller: {e}"
            self.get_logger().error(f"Error resetting controller: {e}")
            
        return response

    def publish_heartbeat(self):
        """Publish heartbeat information to show the node is running"""
        if not self.connected:
            self.get_logger().warn("Roboclaw controller node is not connected")
            return
        
        self.get_logger().info("Roboclaw controller node is running")

    def publish_connection_status(self):
        """Publish the current connection status"""
        if self.connected:
            self.status_pub.publish(Bool(data=True))
            self.get_logger().info("Published connection status: OK")
        else:
            self.status_pub.publish(Bool(data=False))
            self.get_logger().warn("Published connection status: ERROR")

    def schedule_reconnect(self):
        """Schedule a reconnection attempt after a delay"""
        if self.reconnect_timer is not None:
            self.reconnect_timer.cancel()
        
        def reconnect_callback():
            self.get_logger().info("Attempting to reconnect to Roboclaw...")
            
            # Use serial_lock to ensure thread safety during reconnection
            try:
                with self.serial_lock:
                    success = self.init_roboclaw_controller()
                
                if success:
                    self.get_logger().info("Reconnected to Roboclaw successfully")
                    # consecutive_errors is reset in init_roboclaw_controller
                else:
                    self.consecutive_errors += 1
                    self.get_logger().warn(f"Reconnect attempt failed, error count: {self.consecutive_errors}")
                    
                    # If too many consecutive errors, back off exponentially
                    if self.consecutive_errors >= self.max_consecutive_errors:
                        backoff_time = min(30, 2 ** (self.consecutive_errors - self.max_consecutive_errors))  # Max 30 seconds
                        self.get_logger().warn(f"Too many consecutive errors, backing off for {backoff_time} seconds")
                        time.sleep(backoff_time)
                    
                    # Reschedule reconnection
                    self.schedule_reconnect()
            except Exception as e:
                self.get_logger().error(f"Error during reconnection attempt: {e}")
                self.consecutive_errors += 1
                self.schedule_reconnect()  # Try again later
        
        # Schedule the reconnection attempt
        self.reconnect_timer = threading.Timer(self.reconnect_interval, reconnect_callback)
        self.reconnect_timer.start()
        
    def decode_roboclaw_error(self, error_code):
        """Decode RoboClaw error code into a human-readable message"""
        error_descriptions = {
            0x0000: "Normal",
            0x0001: "M1 overcurrent warning",
            0x0002: "M2 overcurrent warning",
            0x0004: "Emergency stop",
            0x0008: "Temperature error",
            0x0010: "Temperature warning",
            0x0020: "Main voltage high warning",
            0x0040: "Main voltage low warning",
            0x0080: "Logic voltage high warning",
            0x0100: "Logic voltage low warning",
            0x0200: "M1 driver fault",
            0x0400: "M2 driver fault",
            0x0800: "Main voltage high fault",
            0x1000: "Main voltage low fault",
            0x2000: "Temperature fault",
            0x4000: "M1 overcurrent fault",
            0x8000: "M2 overcurrent fault"
        }
        
        if error_code in error_descriptions:
            return error_descriptions[error_code]
        else:
            return f"Unknown error code: {error_code:#x}"

    def shutdown(self):
        """Clean shutdown of the node"""
        self.get_logger().info("Shutting down Roboclaw controller node")
        
        # Stop all motors
        self.stop_all_motors()
        
        # Close serial connection
        with self.serial_lock:
            try:
                if hasattr(self, 'roboclaw') and hasattr(self.roboclaw, '_port') and self.roboclaw._port.is_open:
                    self.roboclaw._port.close()
                    self.get_logger().info("Closed serial port on shutdown")
            except Exception as e:
                self.get_logger().error(f"Error closing port during shutdown: {e}")
        
        # Cancel any pending reconnection
        if self.reconnect_timer is not None:
            self.reconnect_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SingleRoboclawControllerNode()
        
        # Use a MultiThreadedExecutor for better performance
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor(num_threads=2)
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
        print(f"Error in Single Roboclaw controller node: {e}")
    finally:
        # Ensure ROS is shut down properly
        rclpy.shutdown()


if __name__ == '__main__':
    main()
