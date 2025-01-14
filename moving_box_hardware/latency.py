#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock
import sys

class LatencyChecker(Node):
    def __init__(self, topic1, type1, topic2, type2):
        super().__init__('latency_checker')

        self.topic1 = topic1
        self.topic2 = topic2

        # Инициализация переменных времени
        self.timestamp1 = None
        self.timestamp2 = None

        # Преобразуем типы сообщений в правильный формат и загружаем их
        type1 = self.get_msg_type(type1)
        type2 = self.get_msg_type(type2)

        # Создаем подписчиков
        self.sub1 = self.create_subscription(type1, topic1, self.callback1, 10)
        self.sub2 = self.create_subscription(type2, topic2, self.callback2, 10)

        # Инициализация времени узла
        self.clock = self.get_clock()

    def get_msg_type(self, msg_type_str):
        """
        Преобразуем тип сообщения из формата 'pkg/Message' в 'pkg.msg.Message' и загружаем его.
        """
        if '/' in msg_type_str:
            msg_type_str = msg_type_str.replace('/', '.msg.')
        else:
            raise ValueError(f"Invalid message type format: {msg_type_str}. Expected format: 'pkg/Message'")

        try:
            # Динамическая загрузка типа сообщения
            msg_type = self.import_msg_type(msg_type_str)
            return msg_type
        except Exception as e:
            self.get_logger().error(f"Failed to load message type {msg_type_str}: {e}")
            raise

    def import_msg_type(self, msg_type_str):
        """
        Импортируем тип сообщения по его строковому представлению.
        """
        try:
            # Разделяем строку на пакет и класс
            module_name, class_name = msg_type_str.rsplit('.', 1)
            module = __import__(module_name, fromlist=[class_name])
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            raise ValueError(f"Failed to import message type {msg_type_str}: {e}")

    def extract_timestamp(self):
        """
        Получаем текущее системное время узла (в секундах).
        """
        now = self.clock.now()
        return now.nanoseconds * 1e-9

    def callback1(self, msg):
        """
        Обработчик сообщений для первого топика.
        """
        self.timestamp1 = self.extract_timestamp()
        self.get_logger().info(f"Received message on {self.topic1} at {self.timestamp1:.9f}")
        self.check_latency()

    def callback2(self, msg):
        """
        Обработчик сообщений для второго топика.
        """
        self.timestamp2 = self.extract_timestamp()
        self.get_logger().info(f"Received message on {self.topic2} at {self.timestamp2:.9f}")
        self.check_latency()

    def check_latency(self):
        """
        Проверяем задержку между двумя топиками.
        """
        if self.timestamp1 is not None and self.timestamp2 is not None:
            latency = abs(self.timestamp2 - self.timestamp1)
            self.get_logger().info(f"Latency between {self.topic1} and {self.topic2}: {latency:.6f} seconds")

def main(args=None):
    rclpy.init(args=args)

    # Парсим аргументы из командной строки
    if len(sys.argv) != 5:
        print("Usage: ros2 run <package_name> latency.py <topic1> <type1> <topic2> <type2>")
        sys.exit(1)

    topic1 = sys.argv[1]
    type1 = sys.argv[2]
    topic2 = sys.argv[3]
    type2 = sys.argv[4]

    # Запуск узла
    node = LatencyChecker(topic1, type1, topic2, type2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
