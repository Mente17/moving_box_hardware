#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class SensorDataCombiner(Node):
    def __init__(self):
        super().__init__('sensor_data_combiner')

        # Create a publisher for the combined laser scan data
        self.combined_laser_scan_pub = self.create_publisher(LaserScan, '/combined_laser_scan', 10)

        # Create subscribers for each sensor topic
        self.sub_fl = self.create_subscription(LaserScan, '/laser_scan_fl', self.fl_callback, 10)
        self.sub_fr = self.create_subscription(LaserScan, '/laser_scan_fr', self.fr_callback, 10)
        self.sub_rl = self.create_subscription(LaserScan, '/laser_scan_rl', self.rl_callback, 10)
        self.sub_rr = self.create_subscription(LaserScan, '/laser_scan_rr', self.rr_callback, 10)

        # Initialize variables to store data from each sensor
        self.laser_scan_fl = LaserScan()
        self.laser_scan_fr = LaserScan()
        self.laser_scan_rl = LaserScan()
        self.laser_scan_rr = LaserScan()

    def fl_callback(self, msg):
        self.laser_scan_fl = msg
        self.combine_and_publish()

    def fr_callback(self, msg):
        self.laser_scan_fr = msg
        self.combine_and_publish()

    def rl_callback(self, msg):
        self.laser_scan_rl = msg
        self.combine_and_publish()

    def rr_callback(self, msg):
        self.laser_scan_rr = msg
        self.combine_and_publish()

    def combine_and_publish(self):
        # Combine data from all sensors by taking the average of measurements for each angle
        combined_ranges = []

        num_measurements = len(self.laser_scan_fl.ranges)

        for i in range(num_measurements):
            # Initialize total and count for averaging
            total_range = 0.0
            num_valid_measurements = 0

            # Add ranges from each sensor if they are valid (not NaN or Inf)
            if not math.isnan(self.laser_scan_fl.ranges[i]) and not math.isinf(self.laser_scan_fl.ranges[i]):
                total_range += self.laser_scan_fl.ranges[i]
                num_valid_measurements += 1
            if not math.isnan(self.laser_scan_fr.ranges[i]) and not math.isinf(self.laser_scan_fr.ranges[i]):
                total_range += self.laser_scan_fr.ranges[i]
                num_valid_measurements += 1
            if not math.isnan(self.laser_scan_rl.ranges[i]) and not math.isinf(self.laser_scan_rl.ranges[i]):
                total_range += self.laser_scan_rl.ranges[i]
                num_valid_measurements += 1
            if not math.isnan(self.laser_scan_rr.ranges[i]) and not math.isinf(self.laser_scan_rr.ranges[i]):
                total_range += self.laser_scan_rr.ranges[i]
                num_valid_measurements += 1

            # Calculate the average range for this angle
            if num_valid_measurements > 0:
                average_range = total_range / num_valid_measurements
            else:
                average_range = float('nan')

            combined_ranges.append(average_range)

        # Create a new LaserScan message
        combined_laser_scan = LaserScan()
        combined_laser_scan.header = self.laser_scan_fl.header
        combined_laser_scan.angle_min = self.laser_scan_fl.angle_min
        combined_laser_scan.angle_max = self.laser_scan_fl.angle_max
        combined_laser_scan.angle_increment = self.laser_scan_fl.angle_increment
        combined_laser_scan.time_increment = self.laser_scan_fl.time_increment
        combined_laser_scan.scan_time = self.laser_scan_fl.scan_time
        combined_laser_scan.range_min = self.laser_scan_fl.range_min
        combined_laser_scan.range_max = self.laser_scan_fl.range_max
        combined_laser_scan.ranges = combined_ranges

        # Publish the combined laser scan data
        self.combined_laser_scan_pub.publish(combined_laser_scan)


def main(args=None):
    rclpy.init(args=args)
    sensor_data_combiner = SensorDataCombiner()
    rclpy.spin(sensor_data_combiner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
