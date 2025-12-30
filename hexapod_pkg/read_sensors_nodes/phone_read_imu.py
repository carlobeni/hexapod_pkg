#!/usr/bin/env python3
# phone_read_imu.py

import asyncio
import json
import threading
import websockets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from hexapod_pkg import config_topics_and_parameters as cfg


PHONE_IP = cfg.PHONEIP
PORT = 8080

ACC_SENSOR = "android.sensor.accelerometer"
GYRO_SENSOR = "android.sensor.gyroscope"


class PhoneImuNode(Node):

    def __init__(self):
        super().__init__("phone_imu_raw_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Imu, cfg.TOPIC_PI_PHONE_IMU_GIR_ACC, qos)

        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]

        self.get_logger().info("Phone IMU raw node ready")

    def publish(self):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Accelerometer (m/s^2)
        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]

        # Gyroscope (rad/s)
        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]

        # Orientation unknown
        msg.orientation_covariance[0] = -1.0

        self.pub.publish(msg)


async def accel_task(node: PhoneImuNode):
    url = f"ws://{PHONE_IP}:{PORT}/sensor/connect?type={ACC_SENSOR}"
    node.get_logger().info(f"Accel conected: {url}")

    async with websockets.connect(url) as ws:
        async for msg in ws:
            data = json.loads(msg)
            v = data.get("values", [])
            if len(v) == 3:
                node.acc = v
                node.publish()


async def gyro_task(node: PhoneImuNode):
    url = f"ws://{PHONE_IP}:{PORT}/sensor/connect?type={GYRO_SENSOR}"
    node.get_logger().info(f"Gyro conected: {url}")

    async with websockets.connect(url) as ws:
        async for msg in ws:
            data = json.loads(msg)
            v = data.get("values", [])
            if len(v) == 3:
                node.gyro = v
                node.publish()


def ros_spin(node):
    rclpy.spin(node)

async def run_imu_tasks(node):
    # Pass 'node' into gyro_task and accel_task
    await asyncio.gather(gyro_task(node), accel_task(node))

def main():
    rclpy.init()
    node = PhoneImuNode()

    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()

    asyncio.run(run_imu_tasks(node))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
