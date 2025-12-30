#!/usr/bin/env python3
# phone_read_gps.py

import json
import sys
import threading
import time
import websocket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from hexapod_pkg import config_topics_and_parameters as cfg


# ================= CONFIG =================
PHONE_IP = cfg.PHONEIP
PORT = 8080
GPS_URL = f"ws://{PHONE_IP}:{PORT}/gps"
# =========================================


class PhoneGPSNode(Node):

    def __init__(self):
        super().__init__("phone_gps_node")

        cfg.check_domain_id(self.get_logger())

        # ===== QoS PARAMETERS  =====
        # GPS, low rate
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("qos_history", "keep_last")
        self.declare_parameter("qos_depth", 5)
        self.declare_parameter("qos_durability", "volatile")

        qos = self.build_qos_from_params()

        self.pub = self.create_publisher(
            NavSatFix,
            cfg.TOPIC_PI_PHONE_GPS,
            qos
        )

        self.closed = False
        self.ws = None

        self.get_logger().info("Phone GPS NavSatFix node ready")

    # ===== QoS BUILDER (NUEVO) =====
    def build_qos_from_params(self):
        reliability = self.get_parameter("qos_reliability").value
        history = self.get_parameter("qos_history").value
        depth = self.get_parameter("qos_depth").value
        durability = self.get_parameter("qos_durability").value

        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE
            if reliability.lower() == "reliable"
            else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_ALL
            if history.lower() == "keep_all"
            else HistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
            if durability.lower() == "transient_local"
            else DurabilityPolicy.VOLATILE,
        )

    # ---------- WebSocket callbacks ----------
    def on_message(self, ws, message):
        try:
            data = json.loads(message)

            lat = data.get("latitude")
            lon = data.get("longitude")
            accuracy = data.get("accuracy", -1.0)

            if lat is None or lon is None:
                return

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps_link"

            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            msg.latitude = float(lat)
            msg.longitude = float(lon)
            msg.altitude = data.get("altitude", 0.0)

            # accuracy → varianza (m²)
            if accuracy > 0:
                msg.position_covariance[0] = accuracy ** 2
                msg.position_covariance[4] = accuracy ** 2
                msg.position_covariance[8] = accuracy ** 2
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            else:
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"GPS parsing error: {e}")

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")

    def on_close(self, ws, close_code, reason):
        self.closed = True
        self.get_logger().warn(f"GPS connection closed: {reason}")

    def on_open(self, ws):
        self.get_logger().info("GPS conected")
        threading.Thread(
            target=self.send_requests,
            args=(ws,),
            daemon=True
        ).start()

    # ---------- Request loop ----------
    def send_requests(self, ws):
        while not self.closed:
            try:
                ws.send("getLastKnownLocation")
                time.sleep(1.0)  # 1 Hz
            except Exception:
                break

    # ---------- Start websocket ----------
    def start(self):
        self.ws = websocket.WebSocketApp(
            GPS_URL,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )

        self.ws.run_forever()


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = PhoneGPSNode()

    # ROS spin on separate thread
    threading.Thread(
        target=ros_spin,
        args=(node,),
        daemon=True
    ).start()

    node.start()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()