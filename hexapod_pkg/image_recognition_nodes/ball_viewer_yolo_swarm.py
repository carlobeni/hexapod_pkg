#!/usr/bin/env python3
# ball_viewer_yolo_swarm.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
from rclpy.qos import qos_profile_sensor_data
from hexapod_pkg import config_topics_and_parameters as cfg
import numpy as np

class YoloGridNode(Node):
    def __init__(self):
        super().__init__('yolo_grid_node')

        self.bridge = CvBridge()

        # ==============================
        # PARAMETERS
        # ==============================
        self.declare_parameter('topic_image', cfg.TOPIC_GZ_CAMERA)
        self.declare_parameter('conf_thresh', 0.55)
        self.declare_parameter('frame_skip', 2)
        self.declare_parameter('debug_view', True)
        self.declare_parameter('grid_size', 20)
        self.declare_parameter(
            'topic_obstacle_occupation_grid',
            cfg.TOPIC_OBSTACLE_OCCUPATION_GRID
        )

        topic_image = self.get_parameter('topic_image').value
        topic_grid = self.get_parameter('topic_obstacle_occupation_grid').value

        # -------- SUB / PUB --------
        self.sub_img = self.create_subscription(
            Image, topic_image, self.cb_img, qos_profile_sensor_data
        )

        self.pub_grid = self.create_publisher(
            Int8MultiArray, topic_grid, 10
        )

        self.pub_debug = self.create_publisher(
            String, '/yolo/debug_detections', 10
        )

        # -------- YOLO --------
        pkg_share = get_package_share_directory("hexapod_pkg")
        model_path = os.path.join(pkg_share, "models","pelotitas.pt")

        self.model = YOLO(model_path)
        self.model.to('cuda')

        self.frame_id = 0

        self.get_logger().info("YOLO → Occupation Grid listo")
        self.get_logger().info(f"YOLO CUDA activo: {torch.cuda.is_available()}")

    def cb_img(self, msg):
        self.frame_id += 1
        if self.frame_id % self.get_parameter('frame_skip').value != 0:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        H, W, _ = img.shape

        N = self.get_parameter('grid_size').value
        cell_w = W / N
        cell_h = H / N

        grid = np.zeros((N, N), dtype=np.int8)

        # -------- YOLO INFERENCE --------
        with torch.no_grad():
            result = self.model(
                img,
                conf=self.get_parameter('conf_thresh').value,
                device='cuda',
                verbose=False,
                half=True
            )[0]

        detections = []

        for box in result.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            detections.append(
                f"{cls_name} conf={conf:.2f} bbox=({x1},{y1},{x2},{y2})"
            )

            # -------- BBOX → GRID --------
            c_min = int(x1 / cell_w)
            c_max = int(x2 / cell_w)
            r_min = int(y1 / cell_h)
            r_max = int(y2 / cell_h)

            c_min = np.clip(c_min, 0, N - 1)
            c_max = np.clip(c_max, 0, N - 1)
            r_min = np.clip(r_min, 0, N - 1)
            r_max = np.clip(r_max, 0, N - 1)

            grid[r_min:r_max + 1, c_min:c_max + 1] = 1

            if self.get_parameter('debug_view').value:
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    img, f"{cls_name} {conf:.2f}",
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1
                )

        # -------- PUBLISH GRID --------
        msg_grid = Int8MultiArray()
        msg_grid.layout.dim = [
            MultiArrayDimension(label='rows', size=N, stride=N),
            MultiArrayDimension(label='cols', size=N, stride=1),
        ]
        msg_grid.data = grid.flatten(order='C').tolist()
        self.pub_grid.publish(msg_grid)

        # -------- DEBUG --------
        if detections:
            self.pub_debug.publish(String(data=" | ".join(detections)))
        else:
            self.pub_debug.publish(String(data="no detections"))

        if self.get_parameter('debug_view').value:
            for i in range(1, N):
                cv2.line(img, (0, int(i * cell_h)), (W, int(i * cell_h)), (255, 0, 0), 1)
                cv2.line(img, (int(i * cell_w), 0), (int(i * cell_w), H), (255, 0, 0), 1)

            cv2.imshow("YOLO GRID", img)
            cv2.waitKey(1)


def main():
    rclpy.init()
    rclpy.spin(YoloGridNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
