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


class RoboclawControllerNode(Node):
    """
    ROS2 node for controlling a differential drive base using two Roboclaw controllers.
    Each Roboclaw controls two motors (one on each side of the robot).
    """
    def __init__(self):
        # Initialize the node BEFORE declaring parameters
        super().__init__('roboclaw_controller_node')
        
        # Create callback groups for better concurrency
        self.subscription_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Instead of using declare_parameters(), declare parameters directly
        # RoboClaw communication parameters
        self.declare_parameter('left_roboclaw_port', '/dev/ttyACM0')
        self.declare_parameter('right_roboclaw_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('left_address', 0x80)
        self.declare_parameter('right_address', 0x80)
        
        # Robot physical parameters
        self.declare_parameter('wheel_base', 0.5334)
        self.declare_parameter('wheel_radius', 0.1016)
        
        # Controller parameters
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('encoder_cpr', 4096)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('invert_left_motors', False)
        self.declare_parameter('invert_right_motors', False)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('retries', 3)
        self.declare_parameter('debug_level', 1)
        
        # PID parameters
        self.declare_parameter('pid_p', 1.36)
        self.declare_parameter('pid_i', 0.08)
        self.declare_parameter('pid_d', 0.0)
        self.declare_parameter('pid_qpps', 45540)
        
        # Motor direction parameters - specify which motors need to be reversed
        self.declare_parameter('reverse_left_m2', True)
        self.declare_parameter('reverse_right_m2', True)
        
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
        
        # Set up watchdog timer for safety
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(
            0.1,  # 10 Hz
            self.watchdog_callback,
            callback_group=self.timer_callback_group
        )
        
        # Heartbeat timer - just logs status
        self.heartbeat_timer = self.create_timer(
            5.0,  # 0.2 Hz 
            self.publish_heartbeat,
            callback_group=self.timer_callback_group
        )
        
        self.get_logger().info('Roboclaw controller node initialized')

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
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.invert_left_motors = self.get_parameter('invert_left_motors').value
        self.invert_right_motors = self.get_parameter('invert_right_motors').value
        self.debug_level = self.get_parameter('debug_level').value
        
        # PID parameters
        self.pid_p = self.get_parameter('pid_p').value
        self.pid_i = self.get_parameter('pid_i').value
        self.pid_d = self.get_parameter('pid_d').value
        self.pid_qpps = self.get_parameter('pid_qpps').value
        
        # Motor direction parameters
        self.reverse_left_m2 = self.get_parameter('reverse_left_m2').value
        self.reverse_right_m2 = self.get_parameter('reverse_right_m2').value
        
        # Calculate derived parameters
        # Conversion factor from m/s to encoder counts/s
        self.velocity_to_qpps = self.encoder_cpr / (2.0 * math.pi * self.wheel_radius)
        
        # Set log level based on debug parameter
        if self.debug_level <= 0:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        elif self.debug_level == 1:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            
        self.get_logger().info(f"Robot parameters: wheel_base={self.wheel_base}m, wheel_radius={self.wheel_radius}m")
        self.get_logger().info(f"Max speed: linear={self.max_speed}m/s, angular={self.max_angular_speed}rad/s")
        self.get_logger().info(f"Velocity conversion factor: {self.velocity_to_qpps} counts per m/s")

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
                
                # Initialize motor speeds to zero
                self.stop_all_motors()
                
                # Configure PID and motor directions
                self.configure_pid(self.left_roboclaw, self.left_address)
                self.configure_pid(self.right_roboclaw, self.right_address)
                self.configure_motor_directions()
                
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

    def configure_pid(self, roboclaw, address, motor_num=None):
        """Configure PID values for velocity control
        
        Args:
            roboclaw: The Roboclaw instance to configure
            address: The address of the Roboclaw
            motor_num: Which motor to configure (1, 2, or None for both)
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Convert the PID values to integers after multiplying by 65536
            p_val = int(self.pid_p * 65536)
            i_val = int(self.pid_i * 65536)
            d_val = int(self.pid_d * 65536)
            qpps = int(self.pid_qpps)
            
            results = []
            
            # Configure PID for motor 1 if requested
            if motor_num is None or motor_num == 1:
                result = roboclaw._write4444(address, roboclaw.Cmd.SETM1PID, 
                                            p_val, i_val, d_val, qpps)
                results.append(result)
                
                # Read back and log the values to verify
                pid_values = roboclaw.ReadM1VelocityPID(address)
                if pid_values[0]:
                    self.get_logger().info(
                        f"M1 PID values set to: P={pid_values[1]/65536.0:.4f}, "
                        f"I={pid_values[2]/65536.0:.4f}, D={pid_values[3]/65536.0:.4f}, "
                        f"QPPS={pid_values[4]}"
                    )
            
            # Configure PID for motor 2 if requested
            if motor_num is None or motor_num == 2:
                result = roboclaw._write4444(address, roboclaw.Cmd.SETM2PID, 
                                            p_val, i_val, d_val, qpps)
                results.append(result)
                
                # Read back and log the values to verify
                pid_values = roboclaw.ReadM2VelocityPID(address)
                if pid_values[0]:
                    self.get_logger().info(
                        f"M2 PID values set to: P={pid_values[1]/65536.0:.4f}, "
                        f"I={pid_values[2]/65536.0:.4f}, D={pid_values[3]/65536.0:.4f}, "
                        f"QPPS={pid_values[4]}"
                    )
            
            if all(results):
                self.get_logger().info(f"PID parameters configured successfully for motor{' ' + str(motor_num) if motor_num else 's'}")
                return True
            else:
                self.get_logger().error("Failed to configure some PID parameters")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error configuring PID: {e}")
            return False
    
    def configure_motor_directions(self):
        """Configure motor directions - reverse M2 motors as needed using firmware commands
        
        Instead of using SetM2Orientation (which doesn't exist), we'll modify the 
        differential_drive_to_wheel_velocities method to handle reversals
        """
        try:
            # Log that we're configuring motor directions
            if self.reverse_left_m2:
                self.get_logger().info("Left M2 motor will be reversed through velocity calculations")
            
            if self.reverse_right_m2:
                self.get_logger().info("Right M2 motor will be reversed through velocity calculations")
            
            # Store motor reversal settings to be used when setting velocities
            self._reverse_left_m2 = self.reverse_left_m2
            self._reverse_right_m2 = self.reverse_right_m2
            
            return True
        except Exception as e:
            self.get_logger().error(f"Error configuring motor directions: {e}")
            return False

    def differential_drive_to_wheel_velocities(self, linear_x, angular_z):
        """Convert linear and angular velocities to individual wheel velocities"""
        # Apply simple differential drive kinematics
        # v_l = linear_x - (angular_z * wheel_base / 2)
        # v_r = linear_x + (angular_z * wheel_base / 2)
        
        # Clamp the maximum/minimum velocities
        linear_x = max(min(linear_x, self.max_speed), -self.max_speed)
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
        
        # Calculate wheel velocities
        left_wheel_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        right_wheel_vel = linear_x + (angular_z * self.wheel_base / 2.0)
        
        # Apply motor inversions if configured
        if self.invert_left_motors:
            left_wheel_vel = -left_wheel_vel
        if self.invert_right_motors:
            right_wheel_vel = -right_wheel_vel
            
        # Convert m/s to encoder counts/s
        left_qpps = int(left_wheel_vel * self.velocity_to_qpps)
        right_qpps = int(right_wheel_vel * self.velocity_to_qpps)
        
        # Handle M2 motor reversals with different speed values rather than trying to use non-existent orientation commands
        left_m1_qpps = left_qpps
        left_m2_qpps = left_qpps
        if hasattr(self, '_reverse_left_m2') and self._reverse_left_m2:
            left_m2_qpps = -left_m2_qpps
            
        right_m1_qpps = right_qpps
        right_m2_qpps = right_qpps
        if hasattr(self, '_reverse_right_m2') and self._reverse_right_m2:
            right_m2_qpps = -right_m2_qpps
        
        return left_m1_qpps, left_m2_qpps, right_m1_qpps, right_m2_qpps

    def set_left_motors_velocity(self, velocity_qpps):
        """Set velocity for both motors on left side (controlled by left Roboclaw)"""
        try:
            # Set motor 1 velocity directly
            self.left_roboclaw.SpeedM1(self.left_address, velocity_qpps)
            
            # Reverse direction for motor 2
            self.left_roboclaw.SpeedM2(self.left_address, -velocity_qpps)
            
            if self.debug_level >= 2:
                self.get_logger().debug(f"Set left motors velocity to M1:{velocity_qpps}, M2:{-velocity_qpps} qpps")
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting left motors velocity: {e}")
            return False

    def set_right_motors_velocity(self, velocity_qpps):
        """Set velocity for both motors on right side (controlled by right Roboclaw)"""
        try:
            # Set motor 1 velocity directly
            self.right_roboclaw.SpeedM1(self.right_address, velocity_qpps)
            
            # Reverse direction for motor 2
            self.right_roboclaw.SpeedM2(self.right_address, -velocity_qpps)
            
            if self.debug_level >= 2:
                self.get_logger().debug(f"Set right motors velocity to M1:{velocity_qpps}, M2:{-velocity_qpps} qpps")
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting right motors velocity: {e}")
            return False

    def cmd_vel_callback(self, msg):
        """Callback function for cmd_vel topic subscription"""
        with self._lock:
            # Update timestamp for watchdog
            self.last_cmd_time = self.get_clock().now()
            
            # Convert to wheel velocities - now returns separate speeds for M1 and M2 motors
            left_m1_qpps, left_m2_qpps, right_m1_qpps, right_m2_qpps = self.differential_drive_to_wheel_velocities(
                msg.linear.x, msg.angular.z)
            
            # Apply to motors with the individual motor speeds
            left_success = self.set_left_motors_velocity(left_m1_qpps)
            right_success = self.set_right_motors_velocity(right_m1_qpps)
            
            if self.debug_level >= 1:
                self.get_logger().debug(
                    f"CMD_VEL: linear={msg.linear.x:.2f} m/s, angular={msg.angular.z:.2f} rad/s -> "
                    f"left_m1={left_m1_qpps} qpps, left_m2={left_m2_qpps} qpps, "
                    f"right_m1={right_m1_qpps} qpps, right_m2={right_m2_qpps} qpps")
            
            if not (left_success and right_success):
                self.get_logger().warning("Failed to set some motor velocities")

    def watchdog_callback(self):
        """Watchdog callback to stop motors if no cmd_vel received for a while"""
        now = self.get_clock().now()
        time_diff = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if time_diff > self.cmd_timeout:
            # No recent command, stop motors for safety
            if self.debug_level >= 1:
                self.get_logger().debug(f"Watchdog timeout after {time_diff:.2f}s, stopping motors")
            self.stop_all_motors()

    def stop_all_motors(self):
        """Stop all motors on both controllers"""
        try:
            # Stop left motors - using SpeedM1/M2 with zero speed
            self.left_roboclaw.SpeedM1(self.left_address, 0)
            self.left_roboclaw.SpeedM2(self.left_address, 0)
            
            # Stop right motors
            self.right_roboclaw.SpeedM1(self.right_address, 0)
            self.right_roboclaw.SpeedM2(self.right_address, 0)
            
            self.get_logger().info("All motors stopped")
            return True
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {e}")
            return False

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
        self.get_logger().info("Roboclaw controller node is running")

    def shutdown(self):
        """Clean shutdown of the node"""
        self.get_logger().info("Shutting down Roboclaw controller node")
        
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


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RoboclawControllerNode()
        
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
        print(f"Error in Roboclaw controller node: {e}")
    finally:
        # Ensure ROS is shut down properly
        rclpy.shutdown()


if __name__ == '__main__':
    main()