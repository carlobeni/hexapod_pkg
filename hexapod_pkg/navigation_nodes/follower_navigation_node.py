#!/usr/bin/env python3
# follower_navigation_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import config_topics_and_parameters as cfg


class BallFollowerNav(Node):
    def __init__(self):
        super().__init__("ball_follower_nav")

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter(
            "topic_obstacle_occupation_grid",
            cfg.TOPIC_OBSTACLE_OCCUPATION_GRID
        )
        self.declare_parameter(
            "topic_cmd_robot",
            cfg.TOPIC_CMD_GZ_ROBOT
        )

        self.declare_parameter("center_tol_cols", 1)
        self.declare_parameter("search_cmd", "turn_left")

        topic_grid = self.get_parameter(
            "topic_obstacle_occupation_grid").value
        topic_cmd = self.get_parameter("topic_cmd_robot").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ================= SUB / PUB =================
        self.create_subscription(
            Int8MultiArray,
            topic_grid,
            self.cb_grid,
            qos
        )

        self.cmd_pub = self.create_publisher(String, topic_cmd, 10)

        # ================= STATE =================
        self.grid = None
        self.N = None

        self.center_tol = self.get_parameter("center_tol_cols").value
        self.search_cmd = self.get_parameter("search_cmd").value

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("BallFollowerNav active")

    # ================= CALLBACK =================
    def cb_grid(self, msg):
        self.N = msg.layout.dim[0].size
        self.grid = list(msg.data)

    # ================= PERCEPTION =================
    def find_closest_object_column(self):
        """
        Find the nearest object (lowest occupied row)
        and return the middle column of that object.
        """
        if self.grid is None:
            return None

        N = self.N

        # From the row closest to the robot forward
        for r in range(N - 1, -1, -1):
            cols = []
            for c in range(N):
                if self.grid[r * N + c] == 1:
                    cols.append(c)

            if cols:
                return int(sum(cols) / len(cols))

        return None

    # ================= LOOP =================
    def loop(self):
        cmd = String()

        col = self.find_closest_object_column()

        # ================= SEARCH =================
        if col is None:
            cmd.data = self.search_cmd
            self.cmd_pub.publish(cmd)
            return

        # ================= FOLLOW-UP =================
        center = self.N // 2
        err = col - center

        if abs(err) <= self.center_tol:
            cmd.data = "forward"
        elif err < 0:
            cmd.data = "turn_left"
        else:
            cmd.data = "turn_right"

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(BallFollowerNav())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
