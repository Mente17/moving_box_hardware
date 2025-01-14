#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelListener(Node):

    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav', #/cmd_vel
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f"\nTimestamp: {self.get_clock().now().to_msg()} \nLinear: {msg.linear} \nAngular: {msg.angular}\n")

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_listener = CmdVelListener()
    rclpy.spin(cmd_vel_listener)
    cmd_vel_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
