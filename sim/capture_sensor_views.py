#!/usr/bin/env python3
"""One-off script: saves numbered camera frames (JPEG) and a top-down
scatter-plot rendering of each LiDAR scan (PNG) while the vehicle moves,
so both can be turned into short videos with ffmpeg afterward. Not part of
the ROS2 package -- purely for visually checking what the sensors see."""
import os
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2


class SensorViewCapture(Node):
    def __init__(self, out_dir, duration_sec):
        super().__init__("sensor_view_capture")
        self.camera_dir = os.path.join(out_dir, "camera")
        self.lidar_dir = os.path.join(out_dir, "lidar")
        os.makedirs(self.camera_dir, exist_ok=True)
        os.makedirs(self.lidar_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.camera_count = 0
        self.lidar_count = 0

        self.create_subscription(Image, "/camera", self.on_camera, 10)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_lidar, 10)
        self.create_timer(duration_sec, self.on_timeout)
        self.done = False

    def on_camera(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        path = os.path.join(self.camera_dir, f"{self.camera_count:05d}.jpg")
        cv2.imwrite(path, cv_image)
        self.camera_count += 1

    def on_lidar(self, msg):
        points = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        )
        fig, ax = plt.subplots(figsize=(6, 6), dpi=100)
        if len(points) > 0:
            sc = ax.scatter(points["x"], points["y"], c=points["z"], s=2, cmap="viridis", vmin=-0.1, vmax=1.2)
        ax.set_xlim(-15, 15)
        ax.set_ylim(-15, 15)
        ax.set_aspect("equal")
        ax.set_title(f"LiDAR top-down view (frame {self.lidar_count})")
        ax.scatter([0], [0], c="red", marker="^", s=80, label="vehicle")
        ax.legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(os.path.join(self.lidar_dir, f"{self.lidar_count:05d}.png"))
        plt.close(fig)
        self.lidar_count += 1

    def on_timeout(self):
        self.done = True


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sensor_capture"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    rclpy.init()
    node = SensorViewCapture(out_dir, duration)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info(f"captured {node.camera_count} camera frames, {node.lidar_count} lidar frames")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
