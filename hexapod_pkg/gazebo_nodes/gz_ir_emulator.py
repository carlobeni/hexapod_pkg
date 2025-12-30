#!/usr/bin/env python3
# gz_ir_emulator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import config_topics_and_parameters as cfg

class IREmulator(Node):

    def __init__(self):
        super().__init__("gz_ir_emulator")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==============================
        # IR sensor parameters
        # ==============================
        self.detect_threshold = 0.35
        self.release_threshold = 0.40
        self.min_publish_interval = 0.02  # 50 Hz máx

        # ==============================
        # Independent state
        # ==============================
        self.left_state = False
        self.right_state = False

        self.left_last_time = time.time()
        self.right_last_time = time.time()

        # ==============================
        # Subscriptions
        # ==============================
        self.sub_left = self.create_subscription(
            Range,
            "/gz/sensor_sim/ir_left",
            self.left_callback,
            10
        )

        self.sub_right = self.create_subscription(
            Range,
            "/gz/sensor_sim/ir_right",
            self.right_callback,
            10
        )

        # ==============================
        # Publishers
        # ==============================
        self.pub_left = self.create_publisher(
            Bool,
            cfg.TOPIC_GZ_IR1,
            qos
        )

        self.pub_right = self.create_publisher(
            Bool,
            cfg.TOPIC_GZ_IR2,
            qos
        )

        self.get_logger().info("IR JS40F emulator (LEFT + RIGHT) started")

    # ==============================
    # Comun Logic
    # ==============================
    def process_range(self, d, last_state):
        if not math.isfinite(d):
            return False

        if not last_state and d < self.detect_threshold:
            return True
        if last_state and d > self.release_threshold:
            return False

        return last_state

    # ==============================
    # Callbacks
    # ==============================
    def left_callback(self, msg: Range):
        self.left_state = self.process_and_publish(
            msg.range,
            self.left_state,
            self.pub_left,
            "left"
        )

    def right_callback(self, msg: Range):
        self.right_state = self.process_and_publish(
            msg.range,
            self.right_state,
            self.pub_right,
            "right"
        )

    # ==============================
    # Publicación con rate limit
    # ==============================
    def process_and_publish(self, d, last_state, publisher, side):
        detected = self.process_range(d, last_state)
        now = time.time()

        last_time = self.left_last_time if side == "left" else self.right_last_time

        if detected != last_state or \
           (now - last_time) > self.min_publish_interval:

            publisher.publish(Bool(data=detected))

            if side == "left":
                self.left_last_time = now
            else:
                self.right_last_time = now

            return detected

        return last_state


def main():
    rclpy.init()
    node = IREmulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    