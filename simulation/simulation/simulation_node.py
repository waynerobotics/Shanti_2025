#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.subscription = self.create_subscription(
            Twist, '/demo/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_front = self.create_publisher(Twist, '/diff_drive/front/cmd_vel', 10)
        self.pub_back = self.create_publisher(Twist, '/diff_drive/back/cmd_vel', 10)
        self.pub_center = self.create_publisher(Twist, '/diff_drive/center/cmd_vel', 10)

    def cmd_vel_callback(self, msg):
        self.pub_front.publish(msg)
        self.pub_back.publish(msg)
        self.pub_center.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
