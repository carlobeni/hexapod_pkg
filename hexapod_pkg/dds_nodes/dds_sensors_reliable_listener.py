#!/usr/bin/env python3
# dds_sensors_reliable_listener.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SensorsReliableListener(Node):

    def __init__(self):
        super().__init__("dds_sensors_reliable_listener")
        
        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter("gps_topic", "")
        gps_topic = self.get_parameter("gps_topic").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        if gps_topic:
            self.create_subscription(NavSatFix, gps_topic, self.cb, qos)

        self.get_logger().info("DDS RELIABLE listener READY")

    def cb(self, msg):
        pass


def main():
    rclpy.init()
    rclpy.spin(SensorsReliableListener())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
