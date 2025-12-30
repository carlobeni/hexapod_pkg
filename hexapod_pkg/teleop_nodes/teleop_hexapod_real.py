#!/usr/bin/env python3
# teleop_hexapod_real.py

import sys
import termios
import tty
import select
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hexapod_pkg import config_topics_and_parameters as cfg


class TeleOpHexapod(Node):

    def __init__(self):
        super().__init__("tele_op_hexapod_real")

        self.cmd_pub = self.create_publisher(
            String, cfg.TOPIC_CMD_REAL_ROBOT, 10
        )

        self.last_cmd_sent = None
        self.stop_event = threading.Event()

        self.input_thread = threading.Thread(
            target=self.keyboard_loop, daemon=True
        )
        self.input_thread.start()

        self.get_logger().info(
            "Manual control active:"
            " w = forward"
            " a = turn_left"
            " d = turn_right"
            " s = backward"
            " q = lateral_left"
            " e = lateral_right"
            " x = stop"
        )

    # ---------- Publish command ----------
    def send_cmd(self, cmd: str):
        if cmd == self.last_cmd_sent:
            return

        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

        self.last_cmd_sent = cmd
        self.get_logger().info(f"CMD sent: {cmd}")

    # ---------- keyboard ----------
    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while rclpy.ok() and not self.stop_event.is_set():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == "\x03":  # Ctrl+C
                        self.stop_event.set()
                        rclpy.shutdown()
                        break

                    if key == "x":
                        self.send_cmd("stop")
                    elif key == "1":
                        self.send_cmd("mode_1")
                    elif key == "2":
                        self.send_cmd("mode_2")
                    elif key == "3":
                        self.send_cmd("mode_3")
                    elif key == "w":
                        self.send_cmd("forward")
                    elif key == "s":
                        self.send_cmd("backward")
                    elif key == "a":
                        self.send_cmd("turn_left")
                    elif key == "d":
                        self.send_cmd("turn_right")
                    elif key == "q":
                        self.send_cmd("lateral_left")
                    elif key == "e":
                        self.send_cmd("lateral_right")

        finally:
            # CRITICAL: Always restore terminal
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)

    node = TeleOpHexapod()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
