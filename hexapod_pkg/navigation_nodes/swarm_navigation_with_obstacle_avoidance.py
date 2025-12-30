#!/usr/bin/env python3
# swarm_navigation_with_obstacle_avoidance.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import config_topics_and_parameters as cfg

class SwarmNav(Node):
    def __init__(self):
        super().__init__("swarm_nav")

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter("topic_obstacle_occupation_grid", cfg.TOPIC_OBSTACLE_OCCUPATION_GRID)
        self.declare_parameter("topic_cmd_robot", cfg.TOPIC_CMD_GZ_ROBOT)
        self.declare_parameter("center_tol_cols", 1)
        self.declare_parameter("search_cmd", "turn_left")
        
        # Security and Evasion Parameters
        self.declare_parameter("danger_rows_count", 10)
        self.declare_parameter("min_active_cells_proximity", 11)
        # Forced spin time upon encountering a ball (in 0.05s cycles)
        # 20 cycles = 1 second of rotation ignoring detections
        self.declare_parameter("evasion_duration_cycles", 20) 

        topic_grid = self.get_parameter("topic_obstacle_occupation_grid").value
        topic_cmd = self.get_parameter("topic_cmd_robot").value
        
        self.center_tol = self.get_parameter("center_tol_cols").value
        self.search_cmd = self.get_parameter("search_cmd").value
        self.danger_rows = self.get_parameter("danger_rows_count").value
        self.min_prox_cells = self.get_parameter("min_active_cells_proximity").value
        self.evasion_limit = self.get_parameter("evasion_duration_cycles").value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)


        # ================= SUBS / PUB =================
        self.create_subscription(Int8MultiArray, topic_grid, self.cb_grid, qos)
        self.cmd_pub = self.create_publisher(String, topic_cmd, 10)

        # ================= STATE =================
        self.grid = None
        self.N = None
        self.is_object_near = False
        self.evasion_timer = 0  # Counter for forced rotation

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("SwarmNav with Evasion Mode initiated.")


    def cb_grid(self, msg):
        self.N = msg.layout.dim[0].size
        self.grid = list(msg.data)

    def find_centroid_column(self):
        if self.grid is None: return None
        N = self.N
        total_weight = 0
        weighted_sum_cols = 0
        
        # 1. Centroid
        for c in range(N):
            col_density = sum(1 for r in range(N) if self.grid[r * N + c] == 1)
            if col_density > 0:
                weighted_sum_cols += c * col_density
                total_weight += col_density

        # 2. Danger Zone (Proximity)
        active_near_cells = 0
        for r in range(N - self.danger_rows, N):
            for c in range(N):
                if self.grid[r * N + c] == 1:
                    active_near_cells += 1
        
        self.is_object_near = active_near_cells >= self.min_prox_cells
        target_col = int(weighted_sum_cols / total_weight) if total_weight > 0 else None
        
    
        return target_col

    def loop(self):
        if self.grid is None: return
        cmd = String()

        # --- LOGIC OF FORCED EVASION ---
        if self.evasion_timer > 0:
            cmd.data = self.search_cmd
            self.cmd_pub.publish(cmd)
            self.evasion_timer -= 1
            return

        # If we are not evading, we process the matrix normally
        col = self.find_centroid_column()

        # --- EVASION TRIGGER ---
        if self.is_object_near:
            self.get_logger().info("Ball caught! Starting search for a new ball...")
            self.evasion_timer = self.evasion_limit
            cmd.data = self.search_cmd
            self.cmd_pub.publish(cmd)
            return

        # --- NORMAL SEARCH (No detections) ---
        if col is None:
            cmd.data = self.search_cmd
            self.cmd_pub.publish(cmd)
            return

        # --- FOLLOW-UP ---
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
    node = SwarmNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()