#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import sys


class LatencyChecker(Node):
    def __init__(self, topic1, type1, topic2, type2):
        super().__init__('latency_checker')

        self.topic1 = topic1
        self.topic2 = topic2

        # Временные метки для первого сообщения каждого топика
        self.timestamp1 = None
        self.timestamp2 = None

        # Типы сообщений
        type1 = self.get_msg_type(type1)
        type2 = self.get_msg_type(type2)

        # Подписки на топики
        self.sub1 = self.create_subscription(type1, topic1, self.callback1, 10)
        self.sub2 = self.create_subscription(type2, topic2, self.callback2, 10)

        # Часы для получения текущего времени
        self.clock = self.get_clock()

    def get_msg_type(self, msg_type_str):
        if '/' in msg_type_str:
            msg_type_str = msg_type_str.replace('/', '.msg.')
        else:
            raise ValueError(f"Invalid message type format: {msg_type_str}. Expected format: 'pkg/Message'")

        try:
            msg_type = self.import_msg_type(msg_type_str)
            return msg_type
        except Exception as e:
            self.get_logger().error(f"Failed to load message type {msg_type_str}: {e}")
            raise

    def import_msg_type(self, msg_type_str):
        try:
            module_name, class_name = msg_type_str.rsplit('.', 1)
            module = __import__(module_name, fromlist=[class_name])
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            raise ValueError(f"Failed to import message type {msg_type_str}: {e}")

    def extract_timestamp(self):
        now = self.clock.now()
        return now.nanoseconds * 1e-9

    def format_latency(self, latency_seconds):
        if latency_seconds < 0.001:
            return f"{latency_seconds * 1_000_000:.2f} µs"
        elif latency_seconds < 1:
            return f"{latency_seconds * 1000:.2f} ms"
        else:
            return f"{latency_seconds:.2f} s"

    def callback1(self, msg):
        if self.timestamp1 is None:  # Запоминаем только первое сообщение
            self.timestamp1 = self.extract_timestamp()
            self.get_logger().info(f"Received first message on {self.topic1} at {self.timestamp1:.9f}")
            self.check_latency()

    def callback2(self, msg):
        if self.timestamp2 is None:  # Запоминаем только первое сообщение
            self.timestamp2 = self.extract_timestamp()
            self.get_logger().info(f"Received first message on {self.topic2} at {self.timestamp2:.9f}")
            self.check_latency()

    def check_latency(self):
        # Если оба сообщения получены, вычисляем задержку
        if self.timestamp1 is not None and self.timestamp2 is not None:
            latency = self.timestamp2 - self.timestamp1  # Используем абсолютное значение задержки
            formatted_latency = self.format_latency(latency)
            self.get_logger().info(f"Latency between {self.topic1} and {self.topic2}: {formatted_latency}\n")
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 5:
        print("Usage: ros2 run <package_name> latency.py <topic1> <type1> <topic2> <type2>")
        sys.exit(1)

    topic1 = sys.argv[1]
    type1 = sys.argv[2]
    topic2 = sys.argv[3]
    type2 = sys.argv[4]

    node = LatencyChecker(topic1, type1, topic2, type2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
