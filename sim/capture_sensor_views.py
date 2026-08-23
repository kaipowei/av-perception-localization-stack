#!/usr/bin/env python3
"""One-off script: saves numbered camera frames (JPEG) and a top-down
scatter-plot rendering of each LiDAR scan (PNG) while the vehicle moves,
so both can be turned into short videos with ffmpeg afterward. Not part of
the ROS2 package -- purely for visually checking what the sensors see."""
import os
import sys

import cv2
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

IMG_SIZE = 600
WORLD_EXTENT = 15.0  # meters, matches the room's half-width


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
        # Matplotlib originally drew this (rebuilding a whole Figure per
        # frame) and couldn't keep up with the sensor rate -- render time
        # scaled with point count, so even *fewer* points only partly
        # helped and slower driving didn't help at all, since the gap is
        # per-frame processing time, not how fast the vehicle moves.
        # Pure-numpy/OpenCV pixel assignment has no such per-point Python
        # overhead: point-to-pixel mapping and coloring are vectorized, and
        # only the marker/text calls have any fixed (small) per-frame cost.
        points = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        )
        img = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)

        if len(points) > 0:
            x, y, z = points["x"], points["y"], points["z"]
            # skip_nans above only drops NaN -- "no return" rays can also
            # show up as +/-inf, which casts to garbage instead of raising.
            finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            x, y, z = x[finite], y[finite], z[finite]

        if len(points) > 0 and len(x) > 0:
            px = ((x + WORLD_EXTENT) / (2 * WORLD_EXTENT) * IMG_SIZE).astype(np.int32)
            py = ((WORLD_EXTENT - y) / (2 * WORLD_EXTENT) * IMG_SIZE).astype(np.int32)
            in_bounds = (px >= 0) & (px < IMG_SIZE) & (py >= 0) & (py < IMG_SIZE)
            px, py, z = px[in_bounds], py[in_bounds], z[in_bounds]

            z_norm = np.clip((z - (-0.1)) / (1.2 - (-0.1)), 0.0, 1.0)
            z_u8 = (z_norm * 255).astype(np.uint8).reshape(-1, 1)
            colors = cv2.applyColorMap(z_u8, cv2.COLORMAP_VIRIDIS).reshape(-1, 3)
            img[py, px] = colors
            img = cv2.dilate(img, np.ones((2, 2), np.uint8))  # points are 1px, hard to see otherwise

        center = IMG_SIZE // 2
        cv2.drawMarker(img, (center, center), (0, 0, 255), cv2.MARKER_TRIANGLE_UP, 14, 2)
        cv2.putText(
            img, f"LiDAR top-down (frame {self.lidar_count})", (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite(os.path.join(self.lidar_dir, f"{self.lidar_count:05d}.png"), img)
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
