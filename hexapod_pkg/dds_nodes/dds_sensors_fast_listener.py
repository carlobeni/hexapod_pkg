#!/usr/bin/env python3
# dds_sensors_fast_listener.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import(
    Imu, 
    MagneticField, 
    Range, 
    Image
    )
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SensorsFastListener(Node):

    def __init__(self):
        super().__init__("dds_sensors_fast_listener")

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter("imu_topic", "")
        self.declare_parameter("mag_topic", "")
        self.declare_parameter("ultrasonic_topic", "")
        self.declare_parameter("ir1_topic", "")
        self.declare_parameter("ir2_topic", "")
        self.declare_parameter("camera_topic", "")

        imu_topic = self.get_parameter("imu_topic").value
        mag_topic = self.get_parameter("mag_topic").value
        ultrasonic_topic = self.get_parameter("ultrasonic_topic").value
        ir1_topic = self.get_parameter("ir1_topic").value
        ir2_topic = self.get_parameter("ir2_topic").value
        camera_topic = self.get_parameter("camera_topic").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        if imu_topic:
            self.create_subscription(Imu, imu_topic, self.cb, qos)
        if mag_topic:
            self.create_subscription(MagneticField, mag_topic, self.cb, qos)
        if ultrasonic_topic:
            self.create_subscription(Range, ultrasonic_topic, self.cb, qos)
        if ir1_topic:
            self.create_subscription(Bool, ir1_topic, self.cb, qos)
        if ir2_topic:
            self.create_subscription(Bool, ir2_topic, self.cb, qos)
        if camera_topic:
            self.create_subscription(Image, camera_topic, self.cb, qos)

        self.get_logger().info("DDS FAST listener READY")

    def cb(self, msg):
        pass  # just listen


def main():
    rclpy.init()
    rclpy.spin(SensorsFastListener())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
