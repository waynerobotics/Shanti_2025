#!/usr/bin/env python3
# safety_light.py - ROS2 node to control Arduino safety light via serial
# Subscribes to integer topic for better type safety than the original aisha.py
# Compatible with safety_light Arduino program
# Date: May 27, 2025

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

class SafetyLightController(Node):
    def __init__(self):
        super().__init__('safety_light_controller')
        
        # Configure serial connection to Arduino
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        # Setup serial connection
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for the connection to initialize
            self.get_logger().info(f'Serial connection established on {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Create subscription to Int32 topic
        self.create_subscription(
            Int32, 
            'safety_light/command',  # New topic name with more descriptive path
            self.command_callback, 
            10
        )
        
        # Timer for reading from serial
        self.create_timer(0.5, self.read_from_serial)
        
        self.get_logger().info('Safety light controller initialized')

    def command_callback(self, msg):
        """Handle incoming light control commands (0=off, 1=on, 2=blink)"""
        command = msg.data
        
        if command in [0, 1, 2]:
            self.send_data(str(command))
            modes = {0: 'OFF', 1: 'ON', 2: 'BLINKING'}
            self.get_logger().info(f'Setting light mode: {modes[command]}')
        else:
            self.get_logger().warn(f'Invalid input: {command}. Please use 0 (off), 1 (on), or 2 (blink)')

    def send_data(self, data):
        """Send command to Arduino"""
        try:
            self.ser.write(f'{data}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def read_from_serial(self):
        """Read feedback from Arduino"""
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()
                if line:  # Only log non-empty lines
                    self.get_logger().info(f'Arduino: {line}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyLightController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down safety light controller...')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Clean up
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info('Serial connection closed')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()