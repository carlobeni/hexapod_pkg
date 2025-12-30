#!/usr/bin/env python3
# compute_heading.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from hexapod_pkg import config_topics_and_parameters as cfg
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32

import math


class HeadingEstimatorFast(Node):

    def __init__(self):
        super().__init__('heading_estimator')

        # ==============================
        # PARAMETERS
        # ==============================

        self.declare_parameter("topic_imu", cfg.TOPIC_GZ_IMU_GIR_ACC)
        self.declare_parameter("topic_mag", cfg.TOPIC_GZ_MAG)
        self.declare_parameter("topic_estimate_heading", cfg.TOPIC_HEADING_COMPASS)

        topic_imu = self.get_parameter("topic_imu").value
        topic_mag = self.get_parameter("topic_mag").value
        topic_estimate_heading = self.get_parameter("topic_estimate_heading").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declination_deg = -15.5       # magnetic declination
        self.heading_gain   = 1.125        # scale correction
        self.alpha          = 0.95         # 1=IMU only, 0=MAG only

        self.swap_xy        = False
        self.invert_mag_x   = False
        self.invert_mag_y   = False

        self.declination_rad = math.radians(self.declination_deg)
        # ================= STATE =====================
        self.last_mag = None

        # ================= ROS ========================
        self.create_subscription(
            Imu,
            topic_imu,
            self.imu_cb,
            qos
        )

        self.create_subscription(
            MagneticField,
            topic_mag,
            self.mag_cb,
            qos
        )

        self.pub = self.create_publisher(
            Float32,
            topic_estimate_heading,
            qos
        )

        self.get_logger().info(
            f"Heading compute init | decl={self.declination_deg}° alpha={self.alpha}"
        )

    # =================================================
    #                     CALLBACKS
    # =================================================
    def mag_cb(self, msg: MagneticField):
        self.last_mag = msg.magnetic_field

    def imu_cb(self, msg: Imu):
        if self.last_mag is None:
            return

        q = msg.orientation
        yaw_imu = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        mag_heading = self.compute_mag_heading(
            q.x, q.y, q.z, q.w,
            self.last_mag.x,
            self.last_mag.y,
            self.last_mag.z
        )

        # ---- COMPLEMENTARY  FUCION ----
        fused = self.alpha * yaw_imu + (1.0 - self.alpha) * mag_heading
        fused = self.normalize(fused)

        # ---- GAIN ----
        fused *= self.heading_gain
        fused = self.normalize(fused)

        # ---- TO DEGREES ----
        deg = math.degrees(fused)
        if deg < 0:
            deg += 360.0

        out = Float32()
        out.data = float(deg)
        self.pub.publish(out)

    # =================================================
    #               MAGNETIC HEADING
    # =================================================
    def compute_mag_heading(self, qx, qy, qz, qw, mx, my, mz):

        # --- Axis corrections ---
        if self.swap_xy:
            mx, my = my, mx
        if self.invert_mag_x:
            mx = -mx
        if self.invert_mag_y:
            my = -my

        # --- Body rotation → world (XY only) ---
        r00 = 1 - 2*(qy*qy + qz*qz)
        r01 = 2*(qx*qy - qz*qw)
        r10 = 2*(qx*qy + qz*qw)
        r11 = 1 - 2*(qx*qx + qz*qz)

        mwx = r00 * mx + r01 * my
        mwy = r10 * mx + r11 * my

        heading = math.atan2(mwy, mwx)
        heading = self.normalize(heading)

        # magnetic declination
        heading += self.declination_rad
        return self.normalize(heading)

    # =================================================
    #                 UTILS
    # =================================================
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize(a):
        if a > math.pi:
            a -= 2*math.pi
        elif a < -math.pi:
            a += 2*math.pi
        return a


# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = HeadingEstimatorFast()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


