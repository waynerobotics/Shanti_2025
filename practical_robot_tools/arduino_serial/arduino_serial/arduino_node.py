import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading  # Import threading for non-blocking keyboard input

class SerialNode(Node):
    def __init__(self):
        super().__init__('arduino_node')

        # Set up Serial Connection
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)
            self.get_logger().info("Serial connection established")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            return

        # ROS 2 Subscriber
        self.subscription = self.create_subscription(
            String, 'serial_data', self.listener_callback, 10
        )

        # Timer for reading serial data
        self.create_timer(1.0, self.read_from_serial)

        # Start a separate thread for keyboard input
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.keyboard_thread.start()

    def listener_callback(self, msg):
        """Handles incoming messages from the topic"""
        data = msg.data
        self.get_logger().info(f"Received ROS message: {data}")

        if data in ['0', '1', '2']:
            self.send_data(data)
            self.get_logger().info(f"Sent to Arduino: {data}")
        else:
            self.get_logger().warn(f"Invalid input: {data}")

    def send_data(self, data):
        """Sends data to the Arduino over serial"""
        self.ser.write(f'{data}\n'.encode())

    def read_from_serial(self):
        """Reads data from Arduino"""
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info(f"Arduino says: {line}")

    def keyboard_input_loop(self):
        """Handles user keyboard input without blocking ROS"""
        while rclpy.ok():
            user_input = input("Enter 0, 1, or 2 to send (q to quit): ").strip()
            if user_input in ['0', '1', '2']:
                self.send_data(user_input)
                self.get_logger().info(f"Sent from keyboard: {user_input}")
            elif user_input.lower() == 'q':
                self.get_logger().info("Exiting keyboard input...")
                break
            else:
                self.get_logger().warn("Invalid input! Please enter 0, 1, or 2.")

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()

    try:
        rclpy.spin(node)  # Keeps ROS 2 running
    except KeyboardInterrupt:
        node.get_logger().info("Exiting...")
    finally:
        if hasattr(node, 'ser'):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
