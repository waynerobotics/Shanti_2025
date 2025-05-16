import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)  # Wait for the connection to initialize
        self.get_logger().info('Serial connection established')
        self.create_subscription(String, 'serial_data', self.listener_callback, 10)
        self.create_timer(1.0, self.read_from_serial)

    def send_data(self, data):
        self.ser.write(f'{data}\n'.encode())

    def listener_callback(self, msg):
        data = msg.data
        if data in ['0', '1', '2']:
            self.send_data(data)
            self.get_logger().info(f'Sent: {data}')
        else:
            self.get_logger().warn('Invalid input. Please enter 0, 1, or 2.')

    def read_from_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info(f'Received: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Exiting...')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
