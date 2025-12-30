#!/usr/bin/env python3
# compute_estimate_xy.py

import math
import csv
import os
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import config_topics_and_parameters as cfg

def gps_to_local_xy(lat, lon, lat_ref, lon_ref):
    meters_per_deg = 111320.0
    dlat = lat - lat_ref
    dlon = lon - lon_ref

    x = dlon * meters_per_deg * math.cos(math.radians(lat_ref))
    y = dlat * meters_per_deg
    return x, y


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter("topic_gps_to_xy", cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION)
        self.declare_parameter("topic_estimate_heading", cfg.TOPIC_HEADING_COMPASS)
        self.declare_parameter("topic_estimate_xy", cfg.TOPIC_XY_ODOM_CURRENT_POSITION)
        self.declare_parameter("topic_cmd_robot", cfg.TOPIC_CMD_GZ_ROBOT)

        self.declare_parameter("linear_v", 0.05)            # forward/backward speed 
        self.declare_parameter("lateral_v", 0.002)          # lateral (crab) speed
        self.declare_parameter("dead_reckoning_weight", 1)  # 1.0 = pure odometry, 0.0 = pure GPS

        self.declare_parameter("LAT0", cfg.LAT_REFERENCE)
        self.declare_parameter("LON0", cfg.LON_REFERENCE)
        self.declare_parameter("LAT_ROBOT_INITIAL", cfg.LAT_REFERENCE)
        self.declare_parameter("LON_ROBOT_INITIAL", cfg.LON_REFERENCE)

        self.declare_parameter("heading_robot_init", 0.0)

        topic_gps_to_xy = self.get_parameter("topic_gps_to_xy").value
        topic_estimate_heading = self.get_parameter("topic_estimate_heading").value
        topic_estimate_xy = self.get_parameter("topic_estimate_xy").value
        topic_cmd_robot = self.get_parameter("topic_cmd_robot").value

        linear_v = self.get_parameter("linear_v").value
        lateral_v = self.get_parameter("lateral_v").value
        dead_reckoning_weight = self.get_parameter("dead_reckoning_weight").value

        LAT0 = self.get_parameter("LAT0").value
        LON0 = self.get_parameter("LON0").value
        LAT_ROBOT_INITIAL = self.get_parameter("LAT_ROBOT_INITIAL").value
        LON_ROBOT_INITIAL = self.get_parameter("LON_ROBOT_INITIAL").value

        x_robot_initial, y_robot_initial = gps_to_local_xy(LAT0, LON0, LAT_ROBOT_INITIAL, LON_ROBOT_INITIAL) 
        heading_robot_init = self.get_parameter("heading_robot_init").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.v = linear_v                       # forward/backward speed 
        self.v_crab_lateral = lateral_v         # lateral (crab) speed
        self.alpha = dead_reckoning_weight      # dead-reckoning weight
        self.update_rate = 20.0                 # Hz 

        # ======================
        # Internal state
        # ======================
        self.x = x_robot_initial
        self.y = y_robot_initial
        self.heading_deg = heading_robot_init

        self.gps_x = None
        self.gps_y = None

        self.hl_cmd = "stop"

        self.last_time = self.get_clock().now()
        self.start_time = self.last_time.nanoseconds * 1e-9

        # ======================
        # Subscribers
        # ======================
        self.create_subscription(
            PointStamped,
            topic_gps_to_xy,
            self.gps_cb,
            10
        )

        self.create_subscription(
            Float32,
            topic_estimate_heading,
            self.heading_cb,
            qos
        )

        self.create_subscription(
            String,
            topic_cmd_robot,
            self.hl_cmd_cb,
            10
        )

        # ======================
        # Publisher
        # ======================
        self.pub_pose = self.create_publisher(
            PointStamped,
            topic_estimate_xy,
            qos
        )

        # ======================
        # Main timer
        # ======================
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.update
        )

        self.get_logger().info("Pose estimator ready")


    # ======================
    # Callbacks
    # ======================

    def gps_cb(self, msg):
        self.gps_x = msg.point.x
        self.gps_y = msg.point.y

    def heading_cb(self, msg):
        self.heading_deg = msg.data

    def hl_cmd_cb(self, msg):
        self.hl_cmd = msg.data.lower()

    # ======================
    # Principal Loop
    # ======================
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        dx = 0.0
        dy = 0.0

        # ----------------------
        # Mode selection
        # ----------------------
        if self.hl_cmd == "forward":
            heading_eff = self.heading_deg
            v_eff = self.v

        elif self.hl_cmd == "backward":
            heading_eff = self.heading_deg
            v_eff = -self.v

        elif self.hl_cmd == "lateral_left":
            heading_eff = self.heading_deg + 90.0
            v_eff = self.v_crab_lateral

        elif self.hl_cmd == "lateral_right":
            heading_eff = self.heading_deg - 90.0
            v_eff = self.v_crab_lateral

        else:
            v_eff = 0.0
            heading_eff = self.heading_deg

        # ----------------------
        # Integration
        # ----------------------
        if v_eff != 0.0:
            heading_rad = math.radians(heading_eff)
            dx = v_eff * math.cos(heading_rad) * dt
            dy = v_eff * math.sin(heading_rad) * dt

            self.x += dx
            self.y += dy

        # ----------------------
        # GPS correction
        # ----------------------
        if self.gps_x is not None:
            self.x = self.alpha * self.x + (1 - self.alpha) * self.gps_x
            self.y = self.alpha * self.y + (1 - self.alpha) * self.gps_y

        # ----------------------
        # Publish
        # ----------------------
        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = 0.0
        self.pub_pose.publish(msg)


def main():
    rclpy.init()
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
