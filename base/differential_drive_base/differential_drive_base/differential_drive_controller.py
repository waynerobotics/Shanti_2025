#!/usr/bin/env python3
"""
Differential Drive Controller Node

This node handles differential drive kinematics and sends PWM commands to the individual
motor controller nodes. It subscribes to cmd_vel and publishes PWM values to control
the robot's movement.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import Empty
import math
import time


class DifferentialDriveNode(Node):
    """
    ROS2 node for handling differential drive kinematics and sending commands 
    to individual motor controller nodes.
    """
    def __init__(self):
        super().__init__('differential_drive_node')
        
        # Create callback groups for better concurrency
        self.subscription_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Declare parameters directly
        self.declare_parameter('wheel_base', 0.5334)
        self.declare_parameter('wheel_radius', 0.1016)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('max_pwm', 32767)
        self.declare_parameter('min_pwm', 5000)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('pwm_deadband', 0.05)
        self.declare_parameter('debug_level', 1)
        self.declare_parameter('control_rate', 0.2)  # 5Hz control rate
        self.declare_parameter('left_motor_controller_prefix', 'left_motor_controller')
        self.declare_parameter('right_motor_controller_prefix', 'right_motor_controller')
        self.declare_parameter('left_front_motor_number', 1)
        self.declare_parameter('left_rear_motor_number', 2)
        self.declare_parameter('right_front_motor_number', 1)
        self.declare_parameter('right_rear_motor_number', 2)
        
        # Get parameters from ROS
        self.get_parameters_from_ros()
        
        # Create cmd_vel subscription
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        # Create publishers for motor PWM commands
        self.left_front_motor_pub = self.create_publisher(
            Int32,
            f'{self.left_motor_controller_prefix}/motor{self.left_front_motor_number}_pwm',
            10
        )
        
        self.left_rear_motor_pub = self.create_publisher(
            Int32,
            f'{self.left_motor_controller_prefix}/motor{self.left_rear_motor_number}_pwm',
            10
        )
        
        self.right_front_motor_pub = self.create_publisher(
            Int32,
            f'{self.right_motor_controller_prefix}/motor{self.right_front_motor_number}_pwm',
            10
        )
        
        self.right_rear_motor_pub = self.create_publisher(
            Int32,
            f'{self.right_motor_controller_prefix}/motor{self.right_rear_motor_number}_pwm',
            10
        )
        
        # Create emergency stop service
        self.stop_service = self.create_service(
            Empty, 
            'stop_robot', 
            self.stop_robot_callback
        )
        
        # Set up watchdog timer for safety
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(
            self.control_rate,  # 5 Hz (0.2s)
            self.watchdog_callback,
            callback_group=self.timer_callback_group
        )
        
        # Heartbeat timer - just logs status
        self.heartbeat_timer = self.create_timer(
            5.0,  # 0.2 Hz 
            self.publish_heartbeat,
            callback_group=self.timer_callback_group
        )
        
        self.get_logger().info('Differential drive controller node initialized')

    def get_parameters_from_ros(self):
        """Get all parameters from ROS parameter server"""
        # Robot physical parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Controller parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.pwm_deadband = self.get_parameter('pwm_deadband').value
        self.debug_level = self.get_parameter('debug_level').value
        self.control_rate = self.get_parameter('control_rate').value
        
        # Motor controller configuration
        self.left_motor_controller_prefix = self.get_parameter('left_motor_controller_prefix').value
        self.right_motor_controller_prefix = self.get_parameter('right_motor_controller_prefix').value
        self.left_front_motor_number = self.get_parameter('left_front_motor_number').value
        self.left_rear_motor_number = self.get_parameter('left_rear_motor_number').value
        self.right_front_motor_number = self.get_parameter('right_front_motor_number').value
        self.right_rear_motor_number = self.get_parameter('right_rear_motor_number').value
        
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
        self.get_logger().info(f"Control rate: {self.control_rate}s (5Hz)")

    def differential_drive_to_wheel_pwm(self, linear_x, angular_z):
        """Convert linear and angular velocities to individual wheel PWM values"""
        # Clamp the maximum/minimum velocities
        linear_x = max(min(linear_x, self.max_speed), -self.max_speed)
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
        
        # Calculate wheel velocities
        wheel_distance = self.wheel_base / 2.0
        left_wheel_vel = linear_x - (angular_z * wheel_distance)
        right_wheel_vel = linear_x + (angular_z * wheel_distance)
        
        self.get_logger().debug(
            f"Raw wheel velocities: left={left_wheel_vel:.3f}, right={right_wheel_vel:.3f} for "
            f"linear={linear_x:.2f}, angular={angular_z:.2f}"
        )
            
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
        left_pwm = 0
        if left_percent != 0:
            # Scale between min_pwm and max_pwm
            left_pwm_magnitude = self.min_pwm + abs(left_percent) * (self.max_pwm - self.min_pwm)
            left_pwm = int(math.copysign(left_pwm_magnitude, left_percent))
            
        right_pwm = 0
        if right_percent != 0:
            # Scale between min_pwm and max_pwm
            right_pwm_magnitude = self.min_pwm + abs(right_percent) * (self.max_pwm - self.min_pwm)
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

    def cmd_vel_callback(self, msg):
        """Callback function for cmd_vel topic subscription"""
        # Update timestamp for watchdog
        self.last_cmd_time = self.get_clock().now()
        
        # Log the incoming command
        self.get_logger().debug(f"Received cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
        
        # Convert to wheel PWM values
        left_pwm, right_pwm = self.differential_drive_to_wheel_pwm(
            msg.linear.x, msg.angular.z)
        
        # Publish PWM values to all motors
        self.publish_motor_commands(left_pwm, right_pwm)
        
        if self.debug_level >= 1:
            self.get_logger().debug(
                f"CMD_VEL: linear={msg.linear.x:.2f} m/s, angular={msg.angular.z:.2f} rad/s -> "
                f"left_pwm={left_pwm}, right_pwm={right_pwm}")

    def publish_motor_commands(self, left_pwm, right_pwm):
        """Publish PWM commands to all motors"""
        # Create messages
        left_front_msg = Int32()
        left_front_msg.data = left_pwm
        
        left_rear_msg = Int32()
        left_rear_msg.data = left_pwm
        
        right_front_msg = Int32()
        right_front_msg.data = right_pwm
        
        right_rear_msg = Int32()
        right_rear_msg.data = right_pwm
        
        # Publish messages
        self.left_front_motor_pub.publish(left_front_msg)
        self.left_rear_motor_pub.publish(left_rear_msg)
        self.right_front_motor_pub.publish(right_front_msg)
        self.right_rear_motor_pub.publish(right_rear_msg)

    def stop_robot(self):
        """Stop the robot by sending zero PWM to all motors"""
        self.publish_motor_commands(0, 0)
        self.get_logger().info("Robot stopped")

    def stop_robot_callback(self, request, response):
        """Service callback to stop the robot"""
        self.stop_robot()
        return response

    def watchdog_callback(self):
        """Watchdog callback to stop motors if no cmd_vel received for a while"""
        try:
            now = self.get_clock().now()
            time_diff = (now - self.last_cmd_time).nanoseconds / 1e9
            
            if time_diff > self.cmd_timeout:
                # No recent command, stop motors for safety
                if self.debug_level >= 1:
                    self.get_logger().debug(f"Watchdog timeout after {time_diff:.2f}s, stopping robot")
                self.stop_robot()
        except Exception as e:
            self.get_logger().error(f"Error in watchdog: {e}")
            self.stop_robot()

    def publish_heartbeat(self):
        """Publish heartbeat information to show the node is running"""
        self.get_logger().info("Differential drive controller node is running")

    def shutdown(self):
        """Clean shutdown of the node"""
        self.get_logger().info("Shutting down differential drive node")
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DifferentialDriveNode()
        
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
        print(f"Error in differential drive node: {e}")
    finally:
        # Ensure ROS is shut down properly
        rclpy.shutdown()


if __name__ == '__main__':
    main()
