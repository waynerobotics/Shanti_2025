#!/usr/bin/env python3
"""
Closed Loop RoboClaw Controller

This node controls a RoboClaw motor controller using closed-loop velocity control
with quadrature encoders and PID. It provides more precise control than
open-loop PWM control by using encoder feedback.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, Bool
from differential_drive_base.roboclaw_3 import Roboclaw
import sys
import time
import threading
import serial
import math

# Configuration parameters
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUDRATE = 38400
DEFAULT_ADDRESS = 0x80
MAX_ENCODER_SPEED = 20000  # Maximum encoder count per second (adjust based on your encoders)
DEFAULT_WHEEL_RADIUS = 0.1016  # meters (4 inches)
DEFAULT_WHEEL_BASE = 0.5334  # meters (21 inches)
DEFAULT_MAX_LINEAR_VEL = 1.0  # m/s
DEFAULT_MAX_ANGULAR_VEL = 1.5  # rad/s
DEFAULT_TIMEOUT = 0.5  # seconds
DEFAULT_ENCODER_CPR = 1024  # Counts per revolution for quadrature encoder (adjust for your encoders)
DEFAULT_GEAR_RATIO = 25.0  # Motor gear ratio (adjust for your motors)

# PID parameters - tune these for your specific setup
DEFAULT_P = 8.0
DEFAULT_I = 2.0
DEFAULT_D = 0.0
DEFAULT_QPPS = 10000  # Max encoder counts per second at full speed

class ClosedLoopMotorController(Node):
    def __init__(self):
        super().__init__('closed_loop_motor_controller')
        
        # Create locks for thread safety
        self.serial_lock = threading.RLock()
        self.connection_lock = threading.Lock()
        
        # Status tracking
        self.connected = False
        self.consecutive_errors = 0
        self.max_consecutive_errors = 3
        
        # Declare parameters
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('address', DEFAULT_ADDRESS)
        self.declare_parameter('wheel_radius', DEFAULT_WHEEL_RADIUS)
        self.declare_parameter('wheel_base', DEFAULT_WHEEL_BASE)
        self.declare_parameter('max_linear_velocity', DEFAULT_MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_velocity', DEFAULT_MAX_ANGULAR_VEL)
        self.declare_parameter('cmd_timeout', DEFAULT_TIMEOUT)
        self.declare_parameter('encoder_cpr', DEFAULT_ENCODER_CPR)
        self.declare_parameter('gear_ratio', DEFAULT_GEAR_RATIO)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)
        self.declare_parameter('telemetry_rate', 1.0)  # Hz
        self.declare_parameter('debug_level', 1)
        
        # PID parameters
        self.declare_parameter('pid_p', DEFAULT_P)
        self.declare_parameter('pid_i', DEFAULT_I)
        self.declare_parameter('pid_d', DEFAULT_D)
        self.declare_parameter('pid_qpps', DEFAULT_QPPS)
        
        # Get parameters
        self.get_parameters_from_ros()
        
        # Publishers for telemetry
        self.status_pub = self.create_publisher(Bool, 'controller/connected', 10)
        self.left_speed_pub = self.create_publisher(Float32, 'controller/left_speed', 10)
        self.right_speed_pub = self.create_publisher(Float32, 'controller/right_speed', 10)
        self.battery_pub = self.create_publisher(Float32, 'controller/battery_voltage', 10)
        self.left_current_pub = self.create_publisher(Float32, 'controller/left_current', 10)
        self.right_current_pub = self.create_publisher(Float32, 'controller/right_current', 10)
        self.error_pub = self.create_publisher(Int32, 'controller/error_status', 10)
        
        # Connect to RoboClaw
        self.init_roboclaw()
        
        # Motor control variables
        self.last_cmd_time = self.get_clock().now()
        self.left_speed = 0
        self.right_speed = 0
        
        # Create cmd_vel subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create timers
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        self.telemetry_timer = self.create_timer(1.0/self.telemetry_rate, self.telemetry_callback)
        
        self.get_logger().info("Closed loop motor controller initialized")

    def get_parameters_from_ros(self):
        """Get all parameters from ROS parameter server"""
        # Communication parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.address = self.get_parameter('address').value
        
        # Robot physical parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        
        # Control parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.timeout = self.get_parameter('cmd_timeout').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        self.telemetry_rate = self.get_parameter('telemetry_rate').value
        self.debug_level = self.get_parameter('debug_level').value
        
        # PID parameters
        self.pid_p = self.get_parameter('pid_p').value
        self.pid_i = self.get_parameter('pid_i').value
        self.pid_d = self.get_parameter('pid_d').value
        self.pid_qpps = self.get_parameter('pid_qpps').value
        
        # Set log level based on debug parameter
        if self.debug_level <= 0:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        elif self.debug_level == 1:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            
        self.get_logger().info(f"RoboClaw port: {self.port}, address: {self.address:#x}")
        self.get_logger().info(f"Wheel parameters: radius={self.wheel_radius}m, base={self.wheel_base}m")
        self.get_logger().info(f"Encoder CPR: {self.encoder_cpr}, Gear ratio: {self.gear_ratio}")
        self.get_logger().info(f"PID settings: P={self.pid_p}, I={self.pid_i}, D={self.pid_d}, QPPS={self.pid_qpps}")

    def init_roboclaw(self):
        """Initialize connection to the RoboClaw controller"""
        with self.serial_lock:
            try:
                # Close any existing connection
                if hasattr(self, 'roboclaw') and hasattr(self.roboclaw, '_port') and self.roboclaw._port.is_open:
                    try:
                        self.roboclaw._port.close()
                        time.sleep(0.1)  # Give the OS time to release the port
                        self.get_logger().info("Closed existing port before reconnecting")
                    except Exception as e:
                        self.get_logger().error(f"Error closing existing port: {e}")
                
                # Connect to RoboClaw
                self.roboclaw = Roboclaw(self.port, self.baudrate, timeout=1.0, retries=3)
                success = self.roboclaw.Open()
                
                if not success:
                    self.get_logger().error(f"Failed to open port {self.port}")
                    self.connected = False
                    return False
                
                # Linux-specific serial optimizations
                if sys.platform == 'linux' and hasattr(self.roboclaw, '_port'):
                    try:
                        # Set low latency flag for better responsiveness
                        self.roboclaw._port.low_latency = True
                        
                        # Disable exclusive access to allow reconnection
                        self.roboclaw._port.exclusive = False
                        
                        # Adjust serial parameters
                        self.roboclaw._port.inter_byte_timeout = 0.01
                        
                        self.get_logger().info("Applied serial port optimizations")
                    except Exception as e:
                        self.get_logger().warn(f"Could not set serial port optimizations: {e}")
                
                # Get firmware version
                version = self.roboclaw.ReadVersion(self.address)
                if version[0]:
                    version_data = version[1]
                    try:
                        if hasattr(version_data, 'decode'):
                            version_str = version_data.decode('utf-8').strip()
                        else:
                            version_str = str(version_data).strip()
                        self.get_logger().info(f"Connected to RoboClaw: {version_str}")
                    except Exception as e:
                        self.get_logger().warn(f"Error decoding version: {e}, raw: {version_data}")
                else:
                    self.get_logger().error("Connected but failed to read version")
                    self.connected = False
                    return False
                
                # Configure encoders - reset counters
                self.roboclaw.ResetEncoders(self.address)
                
                # Configure PID for both motors
                self.roboclaw.SetM1VelocityPID(self.address, self.pid_p, self.pid_i, self.pid_d, self.pid_qpps)
                self.roboclaw.SetM2VelocityPID(self.address, self.pid_p, self.pid_i, self.pid_d, self.pid_qpps)
                
                # Read and verify PID settings
                pid1 = self.roboclaw.ReadM1VelocityPID(self.address)
                pid2 = self.roboclaw.ReadM2VelocityPID(self.address)
                
                if pid1[0] and pid2[0]:
                    self.get_logger().info(f"M1 PID: P={pid1[1]}, I={pid1[2]}, D={pid1[3]}, QPPS={pid1[4]}")
                    self.get_logger().info(f"M2 PID: P={pid2[1]}, I={pid2[2]}, D={pid2[3]}, QPPS={pid2[4]}")
                else:
                    self.get_logger().warn("Could not read PID settings")
                
                # Stop all motors
                self.stop_motors()
                
                # Mark as connected
                self.connected = True
                self.consecutive_errors = 0
                self.publish_connection_status()
                
                return True
                
            except Exception as e:
                self.get_logger().error(f"Error initializing RoboClaw: {e}")
                self.connected = False
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
            
            # Try to reconnect
            self.init_roboclaw()
        
        return False

    def linear_velocity_to_encoder_speed(self, linear_vel):
        """
        Convert linear velocity (m/s) to encoder speed (counts/second)
        
        Formula:
        - Angular velocity (rad/s) = linear_vel / wheel_radius
        - RPM = Angular velocity * 60 / (2*pi)
        - Encoder counts per second = RPM * gear_ratio * encoder_cpr / 60
        """
        angular_vel_rad_s = linear_vel / self.wheel_radius
        rpm = angular_vel_rad_s * 60.0 / (2.0 * math.pi)
        encoder_speed = int(rpm * self.gear_ratio * self.encoder_cpr / 60.0)
        return encoder_speed

    def convert_velocity_to_speed(self, linear, angular):
        """
        Convert linear and angular velocities to wheel speeds (encoder counts per second)
        """
        # Calculate wheel velocities in m/s using differential drive kinematics
        left_vel = linear - (angular * self.wheel_base / 2.0)
        right_vel = linear + (angular * self.wheel_base / 2.0)
        
        # Convert to encoder speeds
        left_speed = self.linear_velocity_to_encoder_speed(left_vel)
        right_speed = self.linear_velocity_to_encoder_speed(right_vel)
        
        # Apply inversion if needed
        if self.invert_left:
            left_speed = -left_speed
        if self.invert_right:
            right_speed = -right_speed
            
        # Clamp values to valid range (based on QPPS)
        left_speed = max(min(left_speed, self.pid_qpps), -self.pid_qpps)
        right_speed = max(min(right_speed, self.pid_qpps), -self.pid_qpps)
        
        return left_speed, right_speed

    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds using closed-loop velocity control"""
        if not self.connected:
            return False
            
        try:
            with self.serial_lock:
                # Using SpeedM1M2 to set both speeds in one command
                success = self.roboclaw.SpeedM1M2(self.address, left_speed, right_speed)
                
                if success:
                    self.left_speed = left_speed
                    self.right_speed = right_speed
                    if self.debug_level >= 2:
                        self.get_logger().debug(f"Set speeds: left={left_speed}, right={right_speed}")
                    self.consecutive_errors = 0
                    return True
                else:
                    self.consecutive_errors += 1
                    self.get_logger().warn(f"Failed to set motor speeds: error count {self.consecutive_errors}")
                    
                    # Check for controller errors
                    try:
                        err = self.roboclaw.ReadError(self.address)[1]
                        if err != 0:
                            error_msg = self.decode_roboclaw_error(err)
                            self.get_logger().error(f"RoboClaw error: {error_msg} (code: {err:#x})")
                    except Exception as err:
                        self.get_logger().error(f"Failed to read error status: {err}")
                    
                    return False
                
        except (serial.SerialException, OSError) as e:
            return self.handle_serial_error(e, "setting motor speeds")
        except Exception as e:
            self.get_logger().error(f"Error setting motor speeds: {e}")
            return False

    def stop_motors(self):
        """Stop both motors using closed-loop control"""
        if not self.connected:
            return False
            
        try:
            with self.serial_lock:
                success = self.roboclaw.SpeedM1M2(self.address, 0, 0)
                
                if success:
                    self.left_speed = 0
                    self.right_speed = 0
                    self.get_logger().info("Motors stopped")
                    return True
                else:
                    self.get_logger().warn("Failed to stop motors")
                    return False
                    
        except (serial.SerialException, OSError) as e:
            return self.handle_serial_error(e, "stopping motors")
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {e}")
            return False

    def cmd_vel_callback(self, msg):
        """Handle incoming Twist messages"""
        self.last_cmd_time = self.get_clock().now()
        
        # Convert velocities to motor speeds
        left_speed, right_speed = self.convert_velocity_to_speed(
            msg.linear.x,
            msg.angular.z
        )
        
        # Apply motor commands
        success = self.set_motor_speeds(left_speed, right_speed)
        
        if not success and self.connected:
            self.get_logger().warn("Failed to set motor speeds from cmd_vel")

    def watchdog_callback(self):
        """Stop motors if no commands received recently"""
        if not self.connected:
            return
            
        try:
            now = self.get_clock().now()
            elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
            
            if elapsed > self.timeout and (self.left_speed != 0 or self.right_speed != 0):
                self.get_logger().warn(f"No cmd_vel for {elapsed:.1f}s - stopping motors")
                self.stop_motors()
                
        except Exception as e:
            self.get_logger().error(f"Error in watchdog: {e}")

    def telemetry_callback(self):
        """Read and publish diagnostic information"""
        if not self.connected:
            self.publish_connection_status()
            return
            
        try:
            with self.serial_lock:
                # Read encoder speeds
                speed1 = self.roboclaw.ReadSpeedM1(self.address)
                speed2 = self.roboclaw.ReadSpeedM2(self.address)
                
                # Read currents
                currents = self.roboclaw.ReadCurrents(self.address)
                
                # Read battery voltage
                voltage = self.roboclaw.ReadMainBatteryVoltage(self.address)
                
                # Read error status
                error = self.roboclaw.ReadError(self.address)
                
                # Read temperatures
                temp1 = self.roboclaw.ReadTemp(self.address)
                temp2 = self.roboclaw.ReadTemp2(self.address)
                
            # Publish data if reads were successful
            if speed1[0] and speed2[0]:
                # Convert encoder speed to m/s
                left_mps = self.encoder_speed_to_linear_velocity(speed1[1])
                right_mps = self.encoder_speed_to_linear_velocity(speed2[1])
                
                self.left_speed_pub.publish(Float32(data=left_mps))
                self.right_speed_pub.publish(Float32(data=right_mps))
                
                if self.debug_level >= 1:
                    self.get_logger().debug(f"Speeds: left={left_mps:.2f} m/s, right={right_mps:.2f} m/s")
            
            if currents[0]:
                left_current = currents[1] / 100.0  # Convert to amps
                right_current = currents[2] / 100.0  # Convert to amps
                
                self.left_current_pub.publish(Float32(data=left_current))
                self.right_current_pub.publish(Float32(data=right_current))
                
                if self.debug_level >= 1:
                    self.get_logger().debug(f"Currents: left={left_current:.2f}A, right={right_current:.2f}A")
            
            if voltage[0]:
                batt_voltage = voltage[1] / 10.0  # Convert to volts
                self.battery_pub.publish(Float32(data=batt_voltage))
                
                if self.debug_level >= 1:
                    self.get_logger().debug(f"Battery: {batt_voltage:.1f}V")
            
            if error[0] and error[1] != 0:
                error_msg = self.decode_roboclaw_error(error[1])
                self.error_pub.publish(Int32(data=error[1]))
                self.get_logger().warn(f"RoboClaw error: {error_msg} (code: {error[1]:#x})")
            
            if temp1[0] and temp2[0]:
                temp1_c = temp1[1] / 10.0  # Convert to Celsius
                temp2_c = temp2[1] / 10.0  # Convert to Celsius
                
                if self.debug_level >= 1:
                    self.get_logger().debug(f"Temperatures: {temp1_c:.1f}°C, {temp2_c:.1f}°C")
                
                # Check if overheating
                if temp1_c > 70.0 or temp2_c > 70.0:
                    self.get_logger().warn(f"Controller overheating! Temps: {temp1_c:.1f}°C, {temp2_c:.1f}°C")
                
        except (serial.SerialException, OSError) as e:
            self.handle_serial_error(e, "reading telemetry")
        except Exception as e:
            self.get_logger().error(f"Error reading telemetry: {e}")

    def encoder_speed_to_linear_velocity(self, encoder_speed):
        """Convert encoder speed (counts/second) to linear velocity (m/s)"""
        # Encoder counts per second to RPM
        rpm = encoder_speed * 60.0 / (self.encoder_cpr * self.gear_ratio)
        
        # RPM to angular velocity (rad/s)
        angular_vel = rpm * 2.0 * math.pi / 60.0
        
        # Angular velocity to linear velocity
        linear_vel = angular_vel * self.wheel_radius
        
        return linear_vel

    def publish_connection_status(self):
        """Publish the current connection status"""
        status_msg = Bool(data=self.connected)
        self.status_pub.publish(status_msg)
        
        if self.debug_level >= 1:
            if self.connected:
                self.get_logger().debug("Connection status: CONNECTED")
            else:
                self.get_logger().debug("Connection status: DISCONNECTED")

    def decode_roboclaw_error(self, error_code):
        """Decode RoboClaw error code to human-readable message"""
        messages = []
        
        if error_code & 0x01:
            messages.append("ERR_NORMAL")
        if error_code & 0x02:
            messages.append("ERR_M1_OVERCURRENT")
        if error_code & 0x04:
            messages.append("ERR_M2_OVERCURRENT")
        if error_code & 0x08:
            messages.append("ERR_ESTOP")
        if error_code & 0x10:
            messages.append("ERR_TEMPERATURE")
        if error_code & 0x20:
            messages.append("ERR_MAIN_BATT_HIGH")
        if error_code & 0x40:
            messages.append("ERR_MAIN_BATT_LOW")
        if error_code & 0x80:
            messages.append("ERR_LOGIC_BATT_HIGH")
        if error_code & 0x100:
            messages.append("ERR_LOGIC_BATT_LOW")
        if error_code & 0x200:
            messages.append("ERR_M1_DRIVER_FAULT")
        if error_code & 0x400:
            messages.append("ERR_M2_DRIVER_FAULT")
        if error_code & 0x800:
            messages.append("ERR_M1_SPEED_LIMIT")
        if error_code & 0x1000:
            messages.append("ERR_M2_SPEED_LIMIT")
        if error_code & 0x2000:
            messages.append("ERR_MAIN_BATT_HIGH_LIMIT")
        if error_code & 0x4000:
            messages.append("ERR_MAIN_BATT_LOW_LIMIT")
        if error_code & 0x8000:
            messages.append("ERR_TEMPERATURE_LIMIT")
        
        if not messages:
            return "UNKNOWN_ERROR"
            
        return ", ".join(messages)

    def shutdown(self):
        """Clean up resources on shutdown"""
        self.get_logger().info("Shutting down motor controller")
        
        # Stop the motors
        if hasattr(self, 'roboclaw'):
            self.stop_motors()
            
            # Close the serial port
            try:
                if hasattr(self.roboclaw, '_port') and self.roboclaw._port.is_open:
                    self.roboclaw._port.close()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    controller = ClosedLoopMotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
