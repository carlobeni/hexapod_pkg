#!/usr/bin/env python3
# read_ir_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import Button
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from hexapod_pkg import config_topics_and_parameters as cfg


class IRNode(Node):
    def __init__(self):
        super().__init__('ir_sensors_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Hardware ---
        # pull_up=True → activo en LOW
        self.ir_izq = Button(17, pull_up=True)
        self.ir_der = Button(27, pull_up=True)

        # --- Publishers ---
        self.pub_ir_izq = self.create_publisher(
            Bool,
            cfg.TOPIC_PI_IR1,
            qos
        )
        self.pub_ir_der = self.create_publisher(
            Bool,
            cfg.TOPIC_PI_IR2,
            qos
        )

        # 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("IR node started")

    def timer_callback(self):
        msg_izq = Bool()
        msg_der = Bool()

        # True = detecta obstáculo
        msg_izq.data = not self.ir_izq.is_pressed
        msg_der.data = not self.ir_der.is_pressed

        self.pub_ir_izq.publish(msg_izq)
        self.pub_ir_der.publish(msg_der)


def main(args=None):
    rclpy.init(args=args)
    node = IRNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
