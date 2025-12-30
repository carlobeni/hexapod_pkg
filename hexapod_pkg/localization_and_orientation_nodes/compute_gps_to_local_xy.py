#!/usr/bin/env python3
# compute_gps_to_local_xy.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import config_topics_and_parameters as cfg

class GPSToLocalXY(Node):
    def __init__(self):
        super().__init__("gps_to_local_xy")

        # ==============================
        # PARAMETERS
        # ==============================

        self.declare_parameter("topic_gps_lat_lon_topic", cfg.TOPIC_GZ_GPS)
        self.declare_parameter("topic_gps_to_xy", cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION)

        self.declare_parameter("LAT0", cfg.LAT_REFERENCE)
        self.declare_parameter("LON0", cfg.LON_REFERENCE)

        topic_gps_lat_lon_topic = self.get_parameter("topic_gps_lat_lon_topic").value
        topic_gps_to_xy = self.get_parameter("topic_gps_to_xy").value

        LAT0 = self.get_parameter("LAT0").value
        LON0 = self.get_parameter("LON0").value

        # --- Origin (must match your world) ---
        self.lat0 = math.radians(LAT0)
        self.lon0 = math.radians(LON0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Earth radius (WGS84)
        self.R = 6378137.0

        self.sub = self.create_subscription(
            NavSatFix,
            topic_gps_lat_lon_topic,
            self.cb,
            10
        )
        self.pub = self.create_publisher(PointStamped, topic_gps_to_xy, 10)

        self.get_logger().info("GPS → Local XY node READY (Mercator local)")

    def cb(self, msg: NavSatFix):
        # Ignore empty values
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # Local Mercator conversion
        x = self.R * (lon - self.lon0) * math.cos(self.lat0)
        y = self.R * (lat - self.lat0)

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        out.point.x = x
        out.point.y = y
        out.point.z = msg.altitude

        self.pub.publish(out)



def main(args=None):
    rclpy.init(args=args)
    node = GPSToLocalXY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()