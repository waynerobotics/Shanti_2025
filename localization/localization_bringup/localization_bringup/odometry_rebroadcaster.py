#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryRebroadcaster(Node):
    def __init__(self):
        super().__init__('odometry_rebroadcaster')

        # Declare parameters
        self.declare_parameter('input_topic', '/odometry/gps')
        self.declare_parameter('output_topic', '/odometry/gps_map')
        self.declare_parameter('new_frame_id', 'map')

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.new_frame_id = self.get_parameter('new_frame_id').value

        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self.odometry_callback,
            10
        )
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)

        self.get_logger().info(f'Rebroadcasting {self.input_topic} to {self.output_topic} with frame_id {self.new_frame_id}')

    def odometry_callback(self, msg):
        # Modify the header frame_id
        msg.header.frame_id = self.new_frame_id

        # Publish the modified message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryRebroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()