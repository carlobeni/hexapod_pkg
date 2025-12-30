#!/usr/bin/env python3
# target_navigation_with_obstacle_avoidance.py

import math
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, String, Int8MultiArray, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import config_topics_and_parameters as cfg

# ==================================================
# GPS → XY LOCAL (LOCAL ENU)
# ==================================================
def gps_to_local_xy(lat, lon, lat_ref, lon_ref):
    meters_per_deg = 111320.0
    dlat = lat - lat_ref
    dlon = lon - lon_ref

    x = dlon * meters_per_deg * math.cos(math.radians(lat_ref))
    y = dlat * meters_per_deg
    return x, y


def normalize_angle(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


class TargetNav(Node):
    def __init__(self):
        super().__init__("target_nav_with_obstacle_avoidance")

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter("topic_ir_left", cfg.TOPIC_GZ_IR1)
        self.declare_parameter("topic_ir_right", cfg.TOPIC_GZ_IR2)
        self.declare_parameter("topic_estimate_heading", cfg.TOPIC_HEADING_COMPASS)
        self.declare_parameter("topic_ultrasonic_range", cfg.TOPIC_ULTRASONIC_RANGE)
        self.declare_parameter("topic_estimate_xy", cfg.TOPIC_XY_ODOM_CURRENT_POSITION)
        self.declare_parameter("topic_cmd_robot", cfg.TOPIC_CMD_GZ_ROBOT)
        self.declare_parameter("topic_obstacle_occupation_grid", cfg.TOPIC_OBSTACLE_OCCUPATION_GRID)
        self.declare_parameter("topic_goal_gps", cfg.TOPIC_GPS_GOAL)

        self.declare_parameter("LAT0", cfg.LAT_REFERENCE)
        self.declare_parameter("LON0", cfg.LON_REFERENCE)

        topic_ir_left = self.get_parameter("topic_ir_left").value
        topic_ir_right = self.get_parameter("topic_ir_right").value
        topic_estimate_heading = self.get_parameter("topic_estimate_heading").value
        topic_ultrasonic_range = self.get_parameter("topic_ultrasonic_range").value
        topic_estimate_xy = self.get_parameter("topic_estimate_xy").value
        topic_cmd_robot = self.get_parameter("topic_cmd_robot").value
        topic_obstacle_occupation_grid = self.get_parameter("topic_obstacle_occupation_grid").value
        topic_goal_gps = self.get_parameter("topic_goal_gps").value

        self.LAT0 = self.get_parameter("LAT0").value
        self.LON0 = self.get_parameter("LON0").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ================= SUBS =================
        self.create_subscription(
            PointStamped,
            topic_goal_gps,
            self.cb_goal_gps,
            10
        )

        self.create_subscription(
            PointStamped,
            topic_estimate_xy,
            self.cb_gps,
            qos
        )

        self.create_subscription(
            Float32,
            topic_estimate_heading,
            self.cb_heading,
            qos
        )

        self.create_subscription(
            Int8MultiArray,
            topic_obstacle_occupation_grid,
            self.cb_grid,
            10
        )

        self.create_subscription(
            Float32,
            topic_ultrasonic_range,
            self.cb_ultra,
            qos
        )

        # -------- IR LATERALES --------
        self.create_subscription(
            Bool,
            topic_ir_left,
            self.cb_ir_left,
            qos
        )

        self.create_subscription(
            Bool,
            topic_ir_right,
            self.cb_ir_right,
            qos
        )

        # ================= PUB =================
        self.cmd_pub = self.create_publisher(String, topic_cmd_robot, 10)

        # ================= TARGET =================
        self.goal = None

        # ================= STATE =================
        self.x = self.y = self.heading = None
        self.grid = None
        self.N = None
        self.ultra_dist = None

        self.ir_left = False
        self.ir_right = False

        self.state = "NAV"
        self.prev_state = "NAV"
        self.heading_ref = None
        self.last_recover_time = 0.0

        # ==============================
        # PARAMETERS
        # ==============================
        self.heading_tol = 6.0
        self.dist_tol = 0.5

        self.heading_step = 2.0
        self.recover_period = 0.4

        self.ultra_clear_dist = 0.8

        self.clearance_angle = 25.0
        self.inflate_rows = 3
        self.inflate_cols = 4

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("Nav with active lateral IR reflex (crab)")

    # ================= CALLBACKS =================
    def cb_goal_gps(self, msg):
        lat = msg.point.x
        lon = msg.point.y

        gx, gy = gps_to_local_xy(
            lat, lon,
            self.LAT0,
            self.LON0
        )

        self.goal = (gx, gy)

        self.get_logger().info(
            f"New target GPS: ({lat:.6f}, {lon:.6f}) → XY ({gx:.2f}, {gy:.2f})"
        )

    def cb_gps(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y

    def cb_heading(self, msg):
        self.heading = float(msg.data)

    def cb_grid(self, msg):
        self.N = msg.layout.dim[0].size
        self.grid = list(msg.data)

    def cb_ultra(self, msg):
        self.ultra_dist = float(msg.data)

    def cb_ir_left(self, msg):
        self.ir_left = bool(msg.data)

    def cb_ir_right(self, msg):
        self.ir_right = bool(msg.data)

    # ================= PERCEPTION =================
    def obstacle_detected(self):
        if self.grid is None:
            return False

        N = self.N
        mid = N // 2

        for r in range(N - self.inflate_rows, N):
            for c in range(mid - self.inflate_cols, mid + self.inflate_cols + 1):
                if 0 <= r < N and 0 <= c < N:
                    if self.grid[r * N + c] == 1:
                        return True

        if self.ultra_dist is not None and self.ultra_dist > 0:
            if self.ultra_dist < self.ultra_clear_dist:
                return True

        return False

    def grid_density(self):
        if self.grid is None:
            return 0, 0

        N = self.N
        mid = N // 2

        left = right = 0
        for r in range(mid, N):
            for c in range(0, mid):
                left += self.grid[r * N + c]
            for c in range(mid, N):
                right += self.grid[r * N + c]

        return left, right

    # ================= LOOP =================
    def loop(self):
        if self.goal is None:
            return

        if None in (self.x, self.y, self.heading):
            return

        cmd = String()

        # ==================================================
        # === LATERAL IR REFLEX (ABSOLUTE PRIORITY) ===
        # ==================================================
        if self.ir_left or self.ir_right:

            if self.state not in ("CRAB_LEFT", "CRAB_RIGHT"):
                self.prev_state = self.state

            if self.ir_left:
                self.state = "CRAB_RIGHT"
                cmd.data = "lateral_right"
            elif self.ir_right:
                self.state = "CRAB_LEFT"
                cmd.data = "lateral_left"

            self.cmd_pub.publish(cmd)
            return

        # ---- Crab mode exit ----
        if self.state in ("CRAB_LEFT", "CRAB_RIGHT"):
            self.state = self.prev_state

        # ==================================================
        # === NAVIGATION ===
        # ==================================================
        gx, gy = self.goal
        dx = gx - self.x
        dy = gy - self.y

        dist = math.hypot(dx, dy)
        if dist < self.dist_tol:
            cmd.data = "stop"
            self.cmd_pub.publish(cmd)
            return

        heading_goal = normalize_angle(
            math.degrees(math.atan2(dy, dx))
        )

        obstacle = self.obstacle_detected()
        now = time.time()

        # ====== EVASION ======
        if self.state in ("AVOID_LEFT", "AVOID_RIGHT"):
            if obstacle:
                cmd.data = "turn_left" if self.state == "AVOID_LEFT" else "turn_right"
                self.cmd_pub.publish(cmd)
                return
            else:
                offset = self.clearance_angle if self.state == "AVOID_LEFT" else -self.clearance_angle
                self.heading_ref = normalize_angle(self.heading + offset)
                self.state = "RECOVER"
                self.last_recover_time = now

        # ====== DETECTION ======
        if obstacle and self.state not in ("AVOID_LEFT", "AVOID_RIGHT"):
            left, right = self.grid_density()
            self.state = "AVOID_LEFT" if left <= right else "AVOID_RIGHT"
            cmd.data = "turn_left" if self.state == "AVOID_LEFT" else "turn_right"
            self.cmd_pub.publish(cmd)
            return

        # ====== RECOVERY ======
        if self.state == "RECOVER":
            err_goal = normalize_angle(heading_goal - self.heading_ref)

            if abs(err_goal) <= self.heading_tol:
                self.state = "NAV"
            else:
                if now - self.last_recover_time >= self.recover_period:
                    step = self.heading_step if err_goal > 0 else -self.heading_step
                    self.heading_ref = normalize_angle(self.heading_ref + step)
                    self.last_recover_time = now

            err = normalize_angle(self.heading_ref - self.heading)
            cmd.data = "turn_left" if err > 0 else "turn_right" if abs(err) > self.heading_tol else "forward"
            self.cmd_pub.publish(cmd)
            return

        # ====== NAV NORMAL ======
        err = normalize_angle(heading_goal - self.heading)
        cmd.data = "turn_left" if err > self.heading_tol else "turn_right" if err < -self.heading_tol else "forward"
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(TargetNav())
    rclpy.shutdown()


if __name__ == "__main__":
    main()