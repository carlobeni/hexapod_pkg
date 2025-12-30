#!/usr/bin/env python3
# dds_cmd_talker.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from hexapod_pkg import config_topics_and_parameters as cfg


class CommandTalker(Node):

    def __init__(self):
        super().__init__("dds_cmd_talker")

        cfg.check_domain_id(self.get_logger())

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter("cmd_robot_topic", cfg.TOPIC_CMD_GZ_ROBOT)
        self.declare_parameter("cmd_serial_topic", cfg.TOPIC_CMD_SERIAL)

        self.declare_parameter("linear_speed", 127) # linear speed
        self.declare_parameter("angular_speed", 127) # angular velocity
        self.declare_parameter("walk_yaw_trim", -7) # compensation of lateral and linear movement
        #self.declare_parameter("roll_pitch_ang", 10) # angular velocity

        self.declare_parameter("qos_depth", 10)

        # ================= READING =================
        self.cmd_robot_topic = self.get_parameter("cmd_robot_topic").value
        self.cmd_serial_topic = self.get_parameter("cmd_serial_topic").value

        self.v_lin = int(self.get_parameter("linear_speed").value)
        self.v_ang = int(self.get_parameter("angular_speed").value)
        self.walk_yaw_trim = int(self.get_parameter("walk_yaw_trim").value)
        #self.roll_pitch_ang = int(self.get_parameter("roll_pitch_ang").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.get_parameter("qos_depth").value,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_id = 0

        # ================= ROS =================
        self.pub = self.create_publisher(String, self.cmd_serial_topic, qos)
        self.sub = self.create_subscription(
            String,
            self.cmd_robot_topic,
            self.cmd_cb,
            qos
        )

        self.get_logger().info(
            "CommandTalker READY\n"
            f"  SUB ← {self.cmd_robot_topic}\n"
            f"  PUB → {self.cmd_serial_topic}\n"
            f"  linear={self.v_lin} angular={self.v_ang} yaw_trim={self.walk_yaw_trim}"
        )

        # ===== Initialization of the real robot =====
        self.send("SET90", origin="INIT")
        self.send("MODE 1", origin="INIT")

    # =================================================
    def send(self, cmd_txt: str, origin: str = "AUTO"):
        self.cmd_id += 1
        full = f"{self.cmd_id}:{cmd_txt}"

        msg = String()
        msg.data = full
        self.pub.publish(msg)

        # TERMINAL DISPLAY
        self.get_logger().info(
            f"[CMD {origin}]  →  {full}"
        )

    # =================================================
    def cmd_cb(self, msg: String):
        cmd = msg.data.strip()
        serial_cmd = None

        # ---- MOTION ----
        if cmd == "forward":
            serial_cmd = f"WALK {self.v_lin} 0 {self.walk_yaw_trim}"

        elif cmd == "backward":
            serial_cmd = f"WALK {-self.v_lin} 0 {self.walk_yaw_trim}"

        elif cmd == "lateral_left":
            serial_cmd = f"WALK 0 {self.v_lin} {self.walk_yaw_trim}"

        elif cmd == "lateral_right":
            serial_cmd = f"WALK 0 {-self.v_lin} {self.walk_yaw_trim}"

        elif cmd == "turn_left":
            serial_cmd = f"WALK 0 0 {(self.v_ang)}"
            #serial_cmd = f"ROT 0 {self.v_ang} 0 0"

        elif cmd == "turn_right":
            serial_cmd = f"WALK 0 0 {-(self.v_ang)}"
            #serial_cmd = f"ROT 0 {-self.v_ang} 0 0"


        # ---- STOP ----
        elif cmd == "stop":
            serial_cmd = "WALK 0 0 0"

        # ---- MODES ----
        elif cmd == "mode_1":
            serial_cmd = f"MODE 1"
        
        elif cmd == "mode_2":
            serial_cmd = f"MODE 2"

        elif cmd == "mode_3":
            serial_cmd = f"MODE 3"

        # ---- UNKNOW ----
        else:
            self.get_logger().warn(f"[CMD ROBOT] unknown: '{cmd}'")
            return

        # ---- ENVIAR ----
        self.get_logger().info(
            f"[MAP] cmd_robot='{cmd}', cmd_serial='{serial_cmd}'"
        )
        self.send(serial_cmd, origin="ROBOT")


def main(args=None):
    rclpy.init(args=args)
    node = CommandTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
