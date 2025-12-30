#!/usr/bin/env python3
# master_monitor.py

import rclpy
from rclpy.node import Node
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Bool, String

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import config_topics_and_parameters as cfg


class RobotMasterMonitor(Node):

    def __init__(self):
        super().__init__("master_monitor")

        # ==============================
        # PARAMETERS
        # ==============================

        self.declare_parameter("topic_goal_gps", cfg.TOPIC_GPS_GOAL)
        self.declare_parameter("topic_ir_left", cfg.TOPIC_GZ_IR1)
        self.declare_parameter("topic_ir_right", cfg.TOPIC_GZ_IR2)
        self.declare_parameter("topic_gps_to_xy", cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION)
        self.declare_parameter("topic_estimate_heading", cfg.TOPIC_HEADING_COMPASS)
        self.declare_parameter("topic_ultrasonic_range", cfg.TOPIC_ULTRASONIC_RANGE)
        self.declare_parameter("topic_estimate_xy", cfg.TOPIC_XY_ODOM_CURRENT_POSITION)
        self.declare_parameter("topic_cmd_robot", cfg.TOPIC_CMD_GZ_ROBOT)

        # -------- CMD ROBOT TOPIC --------
        topic_cmd_robot = self.get_parameter("topic_cmd_robot").value

        # -------- GOAL TOPIC --------
        topic_goal_gps = self.get_parameter("topic_goal_gps").value

        # -------- ROBOT STATES --------
        topic_gps_to_xy = self.get_parameter("topic_gps_to_xy").value
        topic_estimate_xy = self.get_parameter("topic_estimate_xy").value
        topic_estimate_heading = self.get_parameter("topic_estimate_heading").value
        topic_ultrasonic_range = self.get_parameter("topic_ultrasonic_range").value
        topic_ir_left = self.get_parameter("topic_ir_left").value
        topic_ir_right = self.get_parameter("topic_ir_right").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==============================
        # PUBLISHERS
        # ==============================
        self.goal_pub = self.create_publisher(PointStamped, topic_goal_gps, 10)

        # ==============================
        # STATES
        # ==============================
        self.state = {
            "gps_xy": None,
            "heading": None,
            "local_estimate": None,
            "ultrasonic": None,
            "ir_left": None,
            "ir_right": None,
            "cmd_robot": None,
        }


        # ==============================
        # SUBS
        # ==============================
        self.create_subscription(PointStamped, topic_gps_to_xy, self.cb_gps_xy, qos)
        self.create_subscription(Float32, topic_estimate_heading, self.cb_heading, qos)
        self.create_subscription(PointStamped, topic_estimate_xy, self.cb_local_estimate, qos)
        self.create_subscription(Float32, topic_ultrasonic_range, self.cb_ultrasonic, qos)
        self.create_subscription(Bool, topic_ir_left, self.cb_ir_left, qos)
        self.create_subscription(Bool, topic_ir_right, self.cb_ir_right, qos)
        self.create_subscription(String, topic_cmd_robot, self.cb_cmd_robot, 10)

        # ==============================
        # GUI
        # ==============================
        self.root = tk.Tk()
        self.root.title("Robot Master Monitor")
        self.root.geometry("700x650")

        self.build_ui()
        self.timer = self.create_timer(1.0, self.update_ui)

        self.get_logger().info("Robot Master Monitor + Goal Sender ready")

    # ==============================
    # CALLBACKS
    # ==============================
    def cb_gps_xy(self, msg):
        self.state["gps_xy"] = (msg.point.x, msg.point.y)

    def cb_heading(self, msg):
        self.state["heading"] = msg.data

    def cb_local_estimate(self, msg):
        self.state["local_estimate"] = (msg.point.x, msg.point.y)

    def cb_ultrasonic(self, msg):
        self.state["ultrasonic"] = msg.data

    def cb_ir_left(self, msg):
        self.state["ir_left"] = msg.data

    def cb_ir_right(self, msg):
        self.state["ir_right"] = msg.data

    def cb_cmd_robot(self, msg):
        self.state["cmd_robot"] = msg.data

    # ==============================
    # GOAL SENDER
    # ==============================
    def send_goal(self):
        raw = self.goal_entry.get()

        try:
            lat_str, lon_str = raw.split(",")
            lat = float(lat_str.strip())
            lon = float(lon_str.strip())

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            msg.point.x = lat
            msg.point.y = lon
            msg.point.z = 0.0

            self.goal_pub.publish(msg)

            self.get_logger().info(
                f"Target send → GPS({lat:.6f}, {lon:.6f})"
            )

            messagebox.showinfo(
                "Target send",
                f"Lat: {lat}\nLon: {lon}"
            )

        except Exception:
            messagebox.showerror(
                "Invalid format",
                "Use de format:\n-25.3307, -57.5178"
            )

    # ==============================
    # UI
    # ==============================
    def build_ui(self):
        style = ttk.Style()
        style.configure("TLabel", font=("Courier", 10))
        style.configure("TButton", font=("Courier", 10))

        def section(title):
            lbl = ttk.Label(self.root, text=title, font=("Courier", 11, "bold"))
            lbl.pack(pady=(10, 4), anchor="w", padx=10)

        # -------- GOAL --------
        section("SET GOAL (GPS)")
        frame = ttk.Frame(self.root)
        frame.pack(anchor="w", padx=20)

        self.goal_entry = ttk.Entry(frame, width=40)
        self.goal_entry.pack(side="left", padx=(0, 10))
        self.goal_entry.insert(0, "-25.33052805268631, -57.518036477806255")

        btn = ttk.Button(frame, text="Send Target", command=self.send_goal)
        btn.pack(side="left")

        # -------- ROBOT STATE --------
        section("ROBOT STATE")
        self.labels = {}
        for key in self.state.keys():
            l = ttk.Label(self.root, text=f"{key:<18}: ---")
            l.pack(anchor="w", padx=20)
            self.labels[key] = l

        self.time_label = ttk.Label(self.root, text="")
        self.time_label.pack(pady=10, anchor="e", padx=10)

    # ==============================
    # UPDATE UI
    # ==============================
    def update_ui(self):
        def fmt(v):
            return "N/A" if v is None else str(v)

        for k, v in self.state.items():
            self.labels[k].config(text=f"{k:<18}: {fmt(v)}")

        self.time_label.config(
            text=f"Updated: {time.strftime('%H:%M:%S')}"
        )

        self.root.update_idletasks()
        self.root.update()


def main():
    rclpy.init()
    node = RobotMasterMonitor()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    ros_thread.start()

    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
