#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

class WheelChangeDetector(Node):
    def __init__(self):
        super().__init__('wheel_change_detector')

        # Subscriptions to topicals with wheel positions
        self.sub_pos_left = self.create_subscription(Int32, '/pos_left', self.left_callback, 10)
        self.sub_pos_right = self.create_subscription(Int32, '/pos_right', self.right_callback, 10)

        # Publishing a change signal
        self.pub_change_detected = self.create_publisher(Bool, '/wheel_movement_detected', 10)

        # Storing previous values
        self.prev_left = None
        self.prev_right = None

    def left_callback(self, msg):
        """ Processes changes in the left wheel """
        if self.prev_left is None or self.prev_left != msg.data:
            self.prev_left = msg.data
            self.publish_signal()

    def right_callback(self, msg):
        """ Processes changes in the right wheel """
        if self.prev_right is None or self.prev_right != msg.data:
            self.prev_right = msg.data
            self.publish_signal()

    def publish_signal(self):
        """ Publishes an alarm if a change is detected in any wheel """
        msg = Bool()
        msg.data = True
        self.pub_change_detected.publish(msg)
        self.get_logger().info('Wheel movement detected!')

def main(args=None):
    rclpy.init(args=args)
    node = WheelChangeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
