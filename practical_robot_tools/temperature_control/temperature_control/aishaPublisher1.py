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
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()

                if "Sensor 1 Temperature" in line:
                    self.last_temp1 = float(line.split('=')[1].split('°')[0].strip())
                elif "Sensor 2 Temperature" in line:
                    self.last_temp2 = float(line.split('=')[1].split('°')[0].strip())

                # If both temps have been read, publish them
                if hasattr(self, 'last_temp1') and hasattr(self, 'last_temp2'):
                    msg1 = Float32()
                    msg2 = Float32()
                    msg1.data = self.last_temp1
                    msg2.data = self.last_temp2
                    self.publisher_1.publish(msg1)
                    self.publisher_2.publish(msg2)
                    self.get_logger().info(f"{msg1.data},{msg2.data}")
                    if msg1.data > 60 or msg2.data > 60:
                        self.ser.write(b'ON\n')  # Send ON command to Arduino
                    else:
                        self.ser.write(b'OFF\n')  # Send OFF command to Arduino
                        
                    # delete stored values to avoid repetition
                    del self.last_temp1
                    del self.last_temp2

        except Exception as e:
            self.get_logger().error(f"Failed to parse line: {line} | Error: {e}")

                


def main(args=None):
        rclpy.init(args=args)

        temp_publisher = MinimalPublisher()

        rclpy.spin(temp_publisher)


        # Destroy the nodes
        temp_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

