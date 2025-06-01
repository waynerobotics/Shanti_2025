#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from differential_drive_base.roboclaw_3 import Roboclaw
import sys
import time

# Configuration parameters
DEFAULT_PORT = '/dev/ttyACM1'
DEFAULT_BAUDRATE = 38400
DEFAULT_ADDRESS = 0x80
MAX_SPEED = 32767
DEFAULT_WHEEL_RADIUS = 0.1  # meters
DEFAULT_WHEEL_BASE = 0.5    # meters
DEFAULT_MAX_LINEAR_VEL = 1.0  # m/s
DEFAULT_TIMEOUT = 0.5       # seconds

class RoboclawMotorController(Node):
    def __init__(self):
        super().__init__('roboclaw_motor_controller')
        
        # Declare parameters
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('address', DEFAULT_ADDRESS)
        self.declare_parameter('wheel_radius', DEFAULT_WHEEL_RADIUS)
        self.declare_parameter('wheel_base', DEFAULT_WHEEL_BASE)
        self.declare_parameter('max_linear_velocity', DEFAULT_MAX_LINEAR_VEL)
        self.declare_parameter('cmd_vel_timeout', DEFAULT_TIMEOUT)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.address = self.get_parameter('address').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.timeout = self.get_parameter('cmd_vel_timeout').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        
        # Saftey light publisher 
        self.safety_light_pub = self.create_publisher(
            Int32,
            '/safety_light',
            10)
       
        self.publish_safety_light()
        # Connect to RoboClaw
        self.roboclaw = Roboclaw(self.port, self.baudrate)
        success = self.roboclaw.Open()
        if not success:
            self.get_logger().error(f"Failed to open port {self.port}")
            sys.exit(1)
            
        version = self.roboclaw.ReadVersion(self.address)
        if version[0]:
            self.get_logger().info(f"Connected to RoboClaw: {version[1]}")
        else:
            self.get_logger().error("Connected but failed to read version")
            sys.exit(1)
        
        # Motor control variables
        self.last_cmd_time = self.get_clock().now()
        self.m1_speed = 0
        self.m2_speed = 0
        
        # Create cmd_vel subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
 
        # Create watchdog timer
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        
        self.get_logger().info("RoboClaw motor controller initialized")

    def convert_velocity_to_duty(self, linear, angular):
        """
        Convert linear and angular velocities to wheel duties using differential drive kinematics
        """
        # Calculate wheel velocities in m/s
        left_vel = linear - angular * self.wheel_base / 2.0
        right_vel = linear + angular * self.wheel_base / 2.0
        
        # Convert to rad/s
        left_rads = left_vel / self.wheel_radius
        right_rads = right_vel / self.wheel_radius
        
        # Scale to duty cycle (-32767 to 32767)
        max_rads = self.max_linear_vel / self.wheel_radius
        left_duty = int((left_rads / max_rads) * MAX_SPEED)
        right_duty = int((right_rads / max_rads) * MAX_SPEED)
        
        # Apply inversion if needed
        if self.invert_left:
            left_duty = -left_duty
        if self.invert_right:
            right_duty = -right_duty
            
        # Clamp values to valid range
        left_duty = max(min(left_duty, MAX_SPEED), -MAX_SPEED)
        right_duty = max(min(right_duty, MAX_SPEED), -MAX_SPEED)
        
        return left_duty, right_duty

    def set_motor_speeds(self, left_duty, right_duty):
        """Set motor speeds with error handling"""
        try:
            self.roboclaw.DutyM1(self.address, left_duty)
            self.roboclaw.DutyM2(self.address, right_duty)
            self.m1_speed = left_duty
            self.m2_speed = right_duty
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting motor speeds: {e}")
            return False

    def stop_motors(self):
        """Stop both motors"""
        try:
            self.roboclaw.DutyM1(self.address, 0)
            self.roboclaw.DutyM2(self.address, 0)
            self.m1_speed = 0
            self.m2_speed = 0
            self.get_logger().info("Motors stopped")
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {e}")

    def cmd_vel_callback(self, msg):
        """Handle incoming Twist messages"""
        self.last_cmd_time = self.get_clock().now()
        
        # Convert velocities to motor duties
        left_duty, right_duty = self.convert_velocity_to_duty(
            msg.linear.x,
            msg.angular.z
        )
        
        # Apply motor commands
        self.set_motor_speeds(left_duty, right_duty)
        # SAFTEY LIGHT : CALL WHERE IT IS RUN THE MOST 
        self.publish_safety_light()
        

    def watchdog_callback(self):
        """Stop motors if no commands received recently"""
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if elapsed > self.timeout and (self.m1_speed != 0 or self.m2_speed != 0):
            self.get_logger().warn(f"No cmd_vel for {elapsed:.1f}s - stopping motors")
            self.stop_motors()

    def shutdown(self):
        """Clean up resources on shutdown"""
        self.get_logger().info("Shutting down motor controller")
        self.stop_motors()
        try:
            self.roboclaw._port.close()
        except:
            pass
    
    def publish_safety_light(self):
        msg = Int32()
        msg.data = 2
        self.safety_light_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RoboclawMotorController()
    
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