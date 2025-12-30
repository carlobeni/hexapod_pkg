#!/usr/bin/env python3
# clasic_vision_calibratiom.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class GreenCubeCalibrator(Node):
    def __init__(self):
        super().__init__('green_cube_calibrator')
        self.bridge = CvBridge()

        self.declare_parameter('topic_image', cfg.TOPIC_GZ_CAMERA)
        self.declare_parameter('cube_size_m', 0.30)

        topic_image = self.get_parameter("topic_image").value

        self.sub = self.create_subscription(
            Image,
            topic_image,
            self.cb_img,
            10
        )

        # Valores iniciales
        self.h_min = 35
        self.h_max = 85
        self.s_min = 80
        self.v_min = 60
        self.fx = 525

        cv2.namedWindow("CALIBRATOR")

        cv2.createTrackbar("H min", "CALIBRATOR", self.h_min, 180, lambda x: None)
        cv2.createTrackbar("H max", "CALIBRATOR", self.h_max, 180, lambda x: None)
        cv2.createTrackbar("S min", "CALIBRATOR", self.s_min, 255, lambda x: None)
        cv2.createTrackbar("V min", "CALIBRATOR", self.v_min, 255, lambda x: None)
        cv2.createTrackbar("fx", "CALIBRATOR", self.fx, 1500, lambda x: None)

        self.get_logger().info("Calibrador ready")

    def cb_img(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        self.h_min = cv2.getTrackbarPos("H min", "CALIBRATOR")
        self.h_max = cv2.getTrackbarPos("H max", "CALIBRATOR")
        self.s_min = cv2.getTrackbarPos("S min", "CALIBRATOR")
        self.v_min = cv2.getTrackbarPos("V min", "CALIBRATOR")
        self.fx = max(1, cv2.getTrackbarPos("fx", "CALIBRATOR"))

        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            ratio = w / float(h)
            if ratio < 0.7 or ratio > 1.3:
                continue

            dist = (self.fx * self.get_parameter('cube_size_m').value) / w

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                img,
                f"{dist:.2f} m",
                (x, y - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        cv2.imshow("CALIBRATOR", img)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = GreenCubeCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
