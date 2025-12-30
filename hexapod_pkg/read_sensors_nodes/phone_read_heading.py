#!/usr/bin/env python3
# phone_read_heading.py

import asyncio
import json
import math
import threading
import websockets

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from hexapod_pkg import config_topics_and_parameters as cfg

# ================= CONFIG =================
PHONE_IP = cfg.PHONEIP
PORT = 8080
SENSOR = "android.sensor.game_rotation_vector"

# ----------- CHOSE THE MODE ---------------
MODE = "Z_AXIS"      # (RECOMENDED)
# =========================================


# ================= UTILITIES =================
def rad2deg(x):
    return x * 180.0 / math.pi


def wrap180(a):
    return ((a + 180.0) % 360.0) - 180.0


# ---------- ROTATION VECTOR → QUAT ----------
def rotvec_to_quat(v):
    x, y, z = v[0], v[1], v[2]
    if len(v) >= 4:
        w = v[3]
    else:
        w = math.sqrt(max(0.0, 1.0 - (x*x + y*y + z*z)))
    return (w, x, y, z)


# ---------- QUATERNION ROTATE VECTOR ----------
def quat_rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v

    qw = -x*vx - y*vy - z*vz
    qx =  w*vx + y*vz - z*vy
    qy =  w*vy + z*vx - x*vz
    qz =  w*vz + x*vy - y*vx

    rx = -qw*x + qx*w - qy*z + qz*y
    ry = -qw*y + qy*w - qz*x + qx*z
    rz = -qw*z + qz*w - qx*y + qy*x

    return rx, ry, rz


# ---------- EULER ----------
def euler_from_quat(q):
    w, x, y, z = q

    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    return yaw, pitch, roll


# ---------- HEADING DESDE +Z ----------
def heading_from_z_axis(q):
    z_body = (0.0, 1.0, 0.0)   # +Z on the phone (adjusts depending on how you mount the phone)
    zx, zy, _ = quat_rotate(q, z_body)
    heading = math.atan2(zy, zx)
    return wrap180(rad2deg(heading))


# ================= ROS NODE =================
class PhoneHeadingNode(Node):

    def __init__(self):
        super().__init__("phone_heading_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(
            Float32,
            cfg.TOPIC_PI_PHONE_HEADING,
            qos
        )
        
        # Variable to store the initial angle (Tare)
        self.initial_offset = None

        self.get_logger().info(f"Phone heading node ready | MODO: {MODE} | output: 0-360")

    def publish(self, raw_heading):
        # 1. Set the initial zero if this is the first time
        if self.initial_offset is None:
            self.initial_offset = raw_heading
            self.get_logger().info(f"Referencia 0 establecida en: {raw_heading:.2f}")

        # 2. Calculate the difference from the beginning
        # raw_heading comes in the range -180 to 180
        delta = raw_heading - self.initial_offset

        # 3. Normalize to 0-360
        # The % operator in Python handles negative numbers correctly to go around the circle
        # Ej: -10 % 360 = 350
        final_heading = delta % 360.0

        msg = Float32()
        msg.data = float(final_heading)
        self.pub.publish(msg)


# ================= ASYNC TASK =================
async def heading_task(node: PhoneHeadingNode):
    url = f"ws://{PHONE_IP}:{PORT}/sensor/connect?type={SENSOR}"
    node.get_logger().info(f"Conectando a {url}")

    async with websockets.connect(url) as ws:
        async for msg in ws:
            try:
                data = json.loads(msg)
                v = data.get("values", [])
                if len(v) < 3:
                    continue

                q = rotvec_to_quat(v)

                # ====== MODE SELECTION ======
                if MODE == "Z_AXIS":
                    heading = heading_from_z_axis(q)

                else:
                    yaw, pitch, roll = euler_from_quat(q)
                    yaw_d   = wrap180(rad2deg(yaw))
                    pitch_d = wrap180(rad2deg(pitch))
                    roll_d  = wrap180(rad2deg(roll))

                    if MODE == "YAW":
                        heading = yaw_d
                    elif MODE == "PITCH":
                        heading = pitch_d
                    elif MODE == "ROLL":
                        heading = roll_d
                    else:
                        continue

                # We send the "raw" data (-180 to 180) to the publish function
                # This funtion takes care of converting it to relative 0-360
                node.publish(heading)
            except Exception as e:
                node.get_logger().warn(f"Error parsing message: {e}")


# ================= MAIN =================
def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = PhoneHeadingNode()

    # Separate thread for ROS spin
    threading.Thread(
        target=ros_spin,
        args=(node,),
        daemon=True
    ).start()

    try:
        asyncio.run(heading_task(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()