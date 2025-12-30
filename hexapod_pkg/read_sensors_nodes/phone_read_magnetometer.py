#!/usr/bin/env python3
# phone_read_magnetometer.py

import asyncio
import json
import threading
import websockets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from hexapod_pkg import config_topics_and_parameters as cfg

PHONE_IP = cfg.PHONEIP
PORT = 8080
MAG_SENSOR = "android.sensor.magnetic_field"


class PhoneMagNode(Node):

    def __init__(self):
        super().__init__("phone_mag_raw_node")

        self.pub = self.create_publisher(
            MagneticField,
            cfg.TOPIC_PI_PHONE_MAG,
            10
        )

        self.get_logger().info("Phone magnetometer raw node listo")

    def publish(self, v):
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Android entrega microTesla → ROS espera Tesla
        msg.magnetic_field.x = v[0] * 1e-6
        msg.magnetic_field.y = v[1] * 1e-6
        msg.magnetic_field.z = v[2] * 1e-6

        self.pub.publish(msg)


async def mag_task(node: PhoneMagNode):
    url = f"ws://{PHONE_IP}:{PORT}/sensor/connect?type={MAG_SENSOR}"
    node.get_logger().info(f"Mag conectado: {url}")

    async with websockets.connect(url) as ws:
        async for msg in ws:
            data = json.loads(msg)
            v = data.get("values", [])
            if len(v) == 3:
                node.publish(v)


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = PhoneMagNode()

    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()
    asyncio.run(mag_task(node))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
