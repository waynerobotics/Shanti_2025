import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('publisher1_node')
        #declare both publisher nodes
        self.publisher_1 = self.create_publisher(Float32, 'temp1', 10)
        self.publisher_2 = self.create_publisher(Float32, 'temp2', 10)
        #set up the serial connection 
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)

        self.timer = self.create_timer(1.0, self.timer_callback)
    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            try:
                # Skip lines that don't have 2 comma-separated values
                parts = line.split(',')
                if len(parts) != 2:
                    self.get_logger().error(f'Invalid data line: {line}')
                    return
                
                temp1 = float(parts[0])
                temp2 = float(parts[1])
                msg1 = Float32()
                msg2 = Float32()
                msg1.data = temp1
                msg2.data = temp2
                self.publisher_1.publish(msg1)
                self.publisher_2.publish(msg2)
                self.get_logger().info(f'Published: temp1={temp1}, temp2={temp2}')
            except Exception as e:
                self.get_logger().error(f'Failed to parse line: {line} | Error: {e}')



def main(args=None):
        rclpy.init(args=args)

        temp_publisher = MinimalPublisher()

        rclpy.spin(temp_publisher)


        # Destroy the nodes
        temp_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()