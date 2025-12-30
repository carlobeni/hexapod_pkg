#!/usr/bin/env python3
# read_ultrasonic_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from hexapod_pkg import config_topics_and_parameters as cfg


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Hardware ---
        self.ultrasonido = DistanceSensor(
            echo=24,
            trigger=23,
            max_distance=2.0
        )

        # --- Publisher ---
        self.pub_range = self.create_publisher(
            Range,
            cfg.TOPIC_PI_ULTRASONIC,
            qos
        )

        # 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Ultrasonic node initiated")

    def timer_callback(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.52
        msg.min_range = 0.02
        msg.max_range = 2.0
        msg.range = self.ultrasonido.distance

        self.pub_range.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
