#!/usr/bin/env python3
# read_BerryImu_node.py

import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from hexapod_pkg import config_topics_and_parameters as cfg
from utils.BerryIMU import IMU

# =============================
# CONSTANTES
# =============================
DEG_TO_RAD = math.pi / 180.0
G_GAIN = 0.070

ACC_LPF_FACTOR = 0.4
MAG_LPF_FACTOR = 0.4
ACC_MEDIANTABLESIZE = 9
MAG_MEDIANTABLESIZE = 9

factor_to_ms2 = (0.224 / 1000) * 9.81

# =============================
# CALIBRACIÓN MAGNETÓMETRO
# =============================
magXmin, magXmax = -510, 1922
magYmin, magYmax = -245, 1614
magZmin, magZmax = -746, 650


class BerryIMUNode(Node):

    def __init__(self):
        super().__init__("read_imu_node")

        cfg.check_domain_id(self.get_logger())

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_imu = self.create_publisher(Imu, cfg.TOPIC_PI_IMU_GIR_ACC, qos)
        self.pub_mag = self.create_publisher(MagneticField, cfg.TOPIC_PI_MAG, qos)

        IMU.detectIMU()
        if IMU.BerryIMUversion == 99:
            self.get_logger().fatal("No BerryIMU found")
            sys.exit(1)

        IMU.initIMU()
        self.get_logger().info("BerryIMU initialized")

        self.oldAcc = [0.0, 0.0, 0.0]
        self.oldMag = [0.0, 0.0, 0.0]

        self.acc_table = [[1.0]*ACC_MEDIANTABLESIZE for _ in range(3)]
        self.mag_table = [[1.0]*MAG_MEDIANTABLESIZE for _ in range(3)]

        self.create_timer(0.03, self.loop)

    def median(self, table, val):
        table.pop()
        table.insert(0, val)
        return sorted(table)[len(table)//2]

    def loop(self):
        now = self.get_clock().now()

        # ---- RAW ----
        ACC_raw = [
            float(IMU.readACCx()),
            float(IMU.readACCy()),
            float(IMU.readACCz())
        ]
        ACC = [a * factor_to_ms2 for a in ACC_raw]

        GYR = [
            float(IMU.readGYRx()) * G_GAIN * DEG_TO_RAD,
            float(IMU.readGYRy()) * G_GAIN * DEG_TO_RAD,
            float(IMU.readGYRz()) * G_GAIN * DEG_TO_RAD,
        ]

        MAG = [
            float(IMU.readMAGx()),
            float(IMU.readMAGy()),
            float(IMU.readMAGz()),
        ]

        # ---- MAG CAL ----
        MAG[0] -= (magXmin + magXmax) / 2.0
        MAG[1] -= (magYmin + magYmax) / 2.0
        MAG[2] -= (magZmin + magZmax) / 2.0

        # ---- LPF ----
        for i in range(3):
            ACC[i] = ACC[i]*ACC_LPF_FACTOR + self.oldAcc[i]*(1-ACC_LPF_FACTOR)
            MAG[i] = MAG[i]*MAG_LPF_FACTOR + self.oldMag[i]*(1-MAG_LPF_FACTOR)

        self.oldAcc = ACC[:]
        self.oldMag = MAG[:]

        # ---- MEDIAN ----
        for i in range(3):
            ACC[i] = self.median(self.acc_table[i], ACC[i])
            MAG[i] = self.median(self.mag_table[i], MAG[i])

        # ---- PUBLISH IMU ----
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = "imu_link"

        imu.angular_velocity.x = GYR[0]
        imu.angular_velocity.y = GYR[1]
        imu.angular_velocity.z = GYR[2]

        imu.linear_acceleration.x = ACC[0]
        imu.linear_acceleration.y = ACC[1]
        imu.linear_acceleration.z = ACC[2]

        self.pub_imu.publish(imu)

        # ---- PUBLISH MAG ----
        mag = MagneticField()
        mag.header = imu.header
        mag.magnetic_field.x = MAG[0]
        mag.magnetic_field.y = MAG[1]
        mag.magnetic_field.z = MAG[2]

        self.pub_mag.publish(mag)


def main():
    rclpy.init()
    node = BerryIMUNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
