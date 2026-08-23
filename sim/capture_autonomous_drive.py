#!/usr/bin/env python3
"""One-off script: renders a top-down view of the vehicle actually driving
itself -- fused pose, driven trail, current planned path, obstacles, goal --
one PNG per fused_odometry update, turned into a video with ffmpeg
afterward. Same pattern as capture_sensor_views.py (Phase 2): plain
numpy/OpenCV pixel math, no matplotlib, because that's what kept up with
the sensor rate last time."""
import math
import os
import sys

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

IMG_SIZE = 700
X_MIN, X_MAX = -2.0, 9.0
Y_MIN, Y_MAX = -2.0, 9.0
SCALE = IMG_SIZE / (X_MAX - X_MIN)


def to_px(x: float, y: float) -> tuple[int, int]:
    px = int((x - X_MIN) * SCALE)
    py = int(IMG_SIZE - (y - Y_MIN) * SCALE)
    return px, py


class DriveCapture(Node):
    def __init__(self, out_dir: str, goal_x: float, goal_y: float, duration_sec: float):
        super().__init__("drive_capture")
        self.frame_dir = os.path.join(out_dir, "frames")
        os.makedirs(self.frame_dir, exist_ok=True)
        self.goal = (goal_x, goal_y)

        self.trail: list[tuple[float, float]] = []
        self.obstacles: list[tuple[float, float, float, float]] = []
        self.path_pts: list[tuple[float, float]] = []
        self.pose = (0.0, 0.0, 0.0)
        self.frame_count = 0

        self.create_subscription(Odometry, "fused_odometry", self.on_odometry, 10)
        self.create_subscription(MarkerArray, "obstacle_markers", self.on_obstacles, 10)
        self.create_subscription(Path, "planned_path", self.on_path, 10)
        # fused_odometry actually publishes near IMU rate (100Hz, see
        # ekf_fusion_node) -- rendering and writing a PNG on every callback
        # would mean thousands of frames and probably choke WSL's disk I/O.
        # A fixed 10Hz render timer decouples frame rate from that, which
        # also makes the output a normal, predictable-length video.
        self.create_timer(0.1, self.render_frame)
        self.create_timer(duration_sec, self.on_timeout)
        self.done = False

    def on_obstacles(self, msg: MarkerArray):
        self.obstacles = [
            (m.pose.position.x, m.pose.position.y, m.scale.x, m.scale.y) for m in msg.markers
        ]

    def on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_odometry(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        self.pose = (x, y, yaw)
        self.trail.append((x, y))

    def render_frame(self):
        img = np.full((IMG_SIZE, IMG_SIZE, 3), 24, dtype=np.uint8)

        for gx in range(int(X_MIN), int(X_MAX) + 1, 2):
            x0, y0 = to_px(gx, Y_MIN)
            x1, y1 = to_px(gx, Y_MAX)
            cv2.line(img, (x0, y0), (x1, y1), (45, 45, 45), 1)
        for gy in range(int(Y_MIN), int(Y_MAX) + 1, 2):
            x0, y0 = to_px(X_MIN, gy)
            x1, y1 = to_px(X_MAX, gy)
            cv2.line(img, (x0, y0), (x1, y1), (45, 45, 45), 1)

        for cx, cy, sx, sy in self.obstacles:
            x0, y0 = to_px(cx - sx / 2, cy - sy / 2)
            x1, y1 = to_px(cx + sx / 2, cy + sy / 2)
            cv2.rectangle(img, (x0, y1), (x1, y0), (60, 70, 220), -1)

        gx, gy = to_px(*self.goal)
        cv2.circle(img, (gx, gy), 14, (100, 220, 100), 2)
        cv2.putText(img, "GOAL", (gx + 18, gy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 220, 100), 1)

        if len(self.path_pts) > 1:
            pts = np.array([to_px(x, y) for x, y in self.path_pts], dtype=np.int32)
            cv2.polylines(img, [pts], False, (200, 180, 80), 1, cv2.LINE_AA)

        if len(self.trail) > 1:
            pts = np.array([to_px(x, y) for x, y in self.trail], dtype=np.int32)
            cv2.polylines(img, [pts], False, (240, 160, 60), 2, cv2.LINE_AA)

        x, y, yaw = self.pose
        cx, cy = to_px(x, y)
        tip = to_px(x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw))
        left = to_px(x + 0.35 * math.cos(yaw + 2.5), y + 0.35 * math.sin(yaw + 2.5))
        right = to_px(x + 0.35 * math.cos(yaw - 2.5), y + 0.35 * math.sin(yaw - 2.5))
        cv2.fillPoly(img, [np.array([tip, left, right], dtype=np.int32)], (60, 160, 250))

        dist = math.hypot(self.goal[0] - x, self.goal[1] - y)
        cv2.putText(
            img, f"autonomous drive -- dist to goal: {dist:.2f} m",
            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1,
        )

        cv2.imwrite(os.path.join(self.frame_dir, f"{self.frame_count:05d}.png"), img)
        self.frame_count += 1

    def on_timeout(self):
        self.done = True


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/drive_capture"
    goal_x = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    goal_y = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
    duration = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0

    rclpy.init()
    node = DriveCapture(out_dir, goal_x, goal_y, duration)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info(f"captured {node.frame_count} frames to {node.frame_dir}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
