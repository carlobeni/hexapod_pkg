#!/usr/bin/env python3
# command_listener_node.py  (PI5)

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from hexapod_pkg import config_topics_and_parameters as cfg


class CommandListener(Node):
    """
    command_listener:
    - Subscribes to /cmd_serial (String)
    - Expected format: ID:COMMAND
    - Sends COMMAND to Arduino Mega ONLY when ID changes
    """

    def __init__(self):
        super().__init__("command_listener")
        
        cfg.check_domain_id(self.get_logger())

        # ===== QoS PARAMETERS =====
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("qos_history", "keep_last")
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_durability", "volatile")

        qos = self._build_qos()

        # ===== SERIAL TO MEGA =====
        try:
            self.ser = serial.Serial(
                cfg.MEGA_SERIAL_PORT,
                cfg.MEGA_BAUD,
                timeout=1.0
            )
            self.get_logger().info(
                f"Opened Mega serial {cfg.MEGA_SERIAL_PORT} @ {cfg.MEGA_BAUD}"
            )
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None

        # ===== STATE =====
        self.last_cmd_id = None

        # ===== SUBSCRIBER =====
        self.sub = self.create_subscription(
            String,
            cfg.TOPIC_CMD_SERIAL ,   # "/cmd_serial"
            self.cmd_serial_callback,
            qos
        )

        self.get_logger().info(
            f"Listening commands on {cfg.TOPIC_CMD_SERIAL }"
        )

    # --------------------------------------------------

    def _build_qos(self):
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.get_parameter("qos_depth").value,
            durability=DurabilityPolicy.VOLATILE,
        )

    # --------------------------------------------------

    def cmd_serial_callback(self, msg: String):
        if self.ser is None:
            return

        text = msg.data.strip()

        # Expected format: ID:COMMAND
        if ":" not in text:
            self.get_logger().warn(f"Malformed command: {text}")
            return

        cmd_id, command = text.split(":", 1)
        cmd_id = cmd_id.strip()
        command = command.strip()

        try:
            self.ser.write((command + "\n").encode())
            self.get_logger().info(
                f"{command}"
            )
        except Exception as e:
            self.get_logger().error(f"Write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CommandListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
