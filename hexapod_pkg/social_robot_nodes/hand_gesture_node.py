#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# hand_gesture_node.py

import os
import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from hexapod_pkg import config_topics_and_parameters as cfg
from rclpy.qos import qos_profile_sensor_data


# =========================================================
# === LANDMARKS CONSTANTS
# =========================================================
PALM_HAND = [0, 1, 2, 5, 9, 13, 17]


# =========================================================
# === GEOMETRIC FUNCTIONS
# =========================================================
def distance(a, b):
    return np.linalg.norm(
        np.array([a.x - b.x, a.y - b.y, a.z - b.z])
    )


def palm_center(landmarks):
    pts = [landmarks[i] for i in PALM_HAND]
    c = np.mean([[p.x, p.y, p.z] for p in pts], axis=0)

    class P:
        pass

    p = P()
    p.x, p.y, p.z = c
    return p


# =========================================================
# === GESTURE DETECTORS
# =========================================================
def detect_al_pelo(landmarks, max_angle_deg=30):
    palma_pts = np.array([[landmarks[i].x, landmarks[i].y] for i in PALM_HAND])
    pc = palma_pts.mean(axis=0)

    def dist(i):
        p = landmarks[i]
        return np.linalg.norm(np.array([p.x, p.y]) - pc)

    # Extended thumb
    if not (dist(4) > dist(3) > dist(2) and dist(4) > 0.10):
        return False

    # Thumbs up palm
    if landmarks[4].y >= np.min(palma_pts[:, 1]):
        return False

    # Thumb angle
    x2, y2 = landmarks[2].x, landmarks[2].y
    x4, y4 = landmarks[4].x, landmarks[4].y
    dx = x4 - x2
    dy = y2 - y4

    if dy <= 0:
        return False

    angle = np.degrees(np.arctan(abs(dx) / dy))
    return angle <= max_angle_deg


def detect_victory(landmarks):
    pc = palm_center(landmarks)
    return (
        distance(landmarks[8], pc) > 0.12 and
        distance(landmarks[12], pc) > 0.12 and
        distance(landmarks[16], pc) < 0.10 and
        distance(landmarks[20], pc) < 0.10
    )


def detect_rock(landmarks):
    palma_pts = np.array([[landmarks[i].x, landmarks[i].y] for i in PALM_HAND])
    pc = palma_pts.mean(axis=0)

    def dist(i):
        p = landmarks[i]
        return np.linalg.norm(np.array([p.x, p.y]) - pc)

    # Index and little fingers extended
    if not (dist(8) > 0.12 and dist(20) > 0.12):
        return False

    # Middle and ring folded
    if not (dist(12) < 0.10 and dist(16) < 0.10):
        return False

    # Thumb: tip outside the palm
    palm_radius = np.max([dist(i) for i in PALM_HAND])
    return dist(4) > palm_radius


def hand_expression(landmarks, cmd_rock, cmd_al_pelo, cmd_victory):
    if detect_rock(landmarks):
        return cmd_rock

    if detect_al_pelo(landmarks):
        return cmd_al_pelo

    if detect_victory(landmarks):
        return cmd_victory

    return None


# =========================================================
# === NODO ROS 2
# =========================================================
class HandGestureNode(Node):
    def __init__(self):
        super().__init__("hand_gesture_node")

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter('topic_image', cfg.TOPIC_PI_PHONE_CAMERA)
        self.declare_parameter("topic_cmd_robot",cfg.TOPIC_CMD_GZ_ROBOT)

        self.declare_parameter("cmd_rock_to_send","rock")
        self.declare_parameter("cmd_al_pelo_to_send","al_pelo")
        self.declare_parameter("cmd_victory_to_send","victory")

        topic_image = self.get_parameter("topic_image").value
        topic_cmd_robot = self.get_parameter("topic_cmd_robot").value

        self.cmd_rock_to_send = self.get_parameter("cmd_rock_to_send").value
        self.cmd_al_pelo_to_send = self.get_parameter("cmd_al_pelo_to_send").value
        self.cmd_victory_to_send = self.get_parameter("cmd_victory_to_send").value


        self.publisher = self.create_publisher(String, topic_cmd_robot, 10)
        self.subscription = self.create_subscription(
            Image,
            topic_image,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.bridge = CvBridge()
        self.last_gesture = None
        self.timestamp = 0

        # -------- MediaPipe --------
        BaseOptions = mp.tasks.BaseOptions
        HandLandmarker = mp.tasks.vision.HandLandmarker
        HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        pkg_share = get_package_share_directory("hexapod_pkg")
        model_path = os.path.join(
            pkg_share,
            "models",
            "hand_landmarker.task"
        )

        options = HandLandmarkerOptions(
            base_options=BaseOptions(
                model_asset_path=model_path
            ),
            running_mode=VisionRunningMode.VIDEO,
            num_hands=1
        )

        self.detector = HandLandmarker.create_from_options(options)

        self.get_logger().info(f"Subscribed to: {topic_image}")


    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # MediaPipe requires monotonic timestamp in ms
        self.timestamp += 33

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb
        )

        result = self.detector.detect_for_video(
            mp_image,
            self.timestamp
        )

        gesture = None

        if result.hand_landmarks:
            gesture = hand_expression(result.hand_landmarks[0],self.cmd_rock_to_send, self.cmd_al_pelo_to_send, self.cmd_victory_to_send)

        # ===============================
        # === EVENT PUBBLISH
        # ===============================
        if gesture is not None:
            if gesture != self.last_gesture:
                msg_out = String()
                msg_out.data = gesture
                self.publisher.publish(msg_out)
                self.get_logger().info(f"Command sent: {gesture}")
                self.last_gesture = gesture

# =========================================================
# === MAIN
# =========================================================
def main():
    rclpy.init()
    node = HandGestureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

