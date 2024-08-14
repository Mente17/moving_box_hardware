#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from gps3 import agps3

class GPS_Sender(Node):

    def __init__(self):
        super().__init__("gps_sender")

        self.gps_socket = agps3.GPSDSocket()
        self.data_stream = agps3.DataStream()
        self.gps_socket.connect()
        self.gps_socket.watch()

        self.create_timer(1, self.timer_callback)
        self.publisher = self.create_publisher(NavSatFix, "/gps/fix", 10)
                
    def timer_callback(self):
        new_data = self.gps_socket.next()
        if not new_data: return

        self.data_stream.unpack(new_data)
        logger = self.get_logger()

        msg = NavSatFix()
        msg.header.frame_id = "gps_frame"

        lat = getattr(self.data_stream, 'lat', 'n/a')
        lon = getattr(self.data_stream, 'lon', 'n/a')
        if lat == "n/a" or lon == "n/a":
            self.has_data = False
            return logger.warn("No GPS data.")
    
        if not self.has_data: logger.info("Successfully publishing data...")
        self.has_data = True

        msg.latitude = float(lat)
        msg.longitude = float(lon)
        self.publisher.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    sender = GPS_Sender()
    rclpy.spin(sender)
    rclpy.shutdown()

if __name__ == '__main__':
    main()