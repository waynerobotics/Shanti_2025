#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.publisher_ = self.create_publisher(String, 'simulation_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
    
    def publish_message(self):
        msg = String()
        msg.data = "Simulation is running..."
        self.get_logger().info(msg.data)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = SimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
