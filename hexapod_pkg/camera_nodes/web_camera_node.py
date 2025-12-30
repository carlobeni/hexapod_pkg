#!/usr/bin/env python3
# web_camera_node.py

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from hexapod_pkg import config_topics_and_parameters as cfg

class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter('topic_image', cfg.TOPIC_PI_PHONE_CAMERA)
        self.declare_parameter('camera_index', 0) # Switched to 1, 2, 3, ..., to choose the desired camera
        
        topic_image = self.get_parameter('topic_image').value
        camera_index = self.get_parameter('camera_index').value

        # ======================
        # Parameters (adjustable)
        # ======================
        self.camera_index = camera_index        
        self.width = 640
        self.height = 640
        self.fps = 15.0

        # ======================
        # Publisher
        # ======================
        self.image_pub = self.create_publisher(
            Image,
            topic_image,
            qos_profile_sensor_data
        )

        # ======================
        # OpenCV Camera (V4L2)
        # ======================
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().fatal(
                f"The camera could not be opened /dev/video{self.camera_index}"
            )
            raise RuntimeError("Camera open failed")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.bridge = CvBridge()

        # ======================
        # Timer
        # ======================
        self.timer = self.create_timer(
            1.0 / self.fps,
            self.timer_callback
        )

        self.get_logger().info(
            f"Webcam node started using /dev/video{self.camera_index}"
        )

    # ======================
    # Callback principal
    # ======================
    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Frame not captured")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'

        self.image_pub.publish(msg)

    # ======================
    # Safe Cleanup
    # ======================
    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
