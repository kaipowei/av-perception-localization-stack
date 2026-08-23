#!/usr/bin/env python3
"""One-off script: sweeps the 'vehicle' model through a fixed loop of
waypoints using Gazebo's set_pose service, so a rosbag recorded at the same
time captures LiDAR/camera data with actual variation instead of a static
scene. Not part of the ROS2 package -- there's no drivetrain yet (that's
Phase 4 territory); this is a stand-in just for dataset collection."""
import math
import subprocess
import sys
import time

WAYPOINTS = [(0, 0), (6, 6), (6, -6), (-6, -6), (-6, 6), (0, 0)]
# Each waypoint step spawns a fresh `gz service` process (no persistent
# Python gz-transport binding is installed), and that per-call connection
# overhead dominates over the intended pause -- so few, larger steps, not
# many small ones. Measured necessary after an earlier run with STEP=0.5
# (106 calls) took several minutes instead of the intended ~16s.
STEP = 2.0  # meters between interpolated poses
PAUSE = 0.1  # seconds between poses


def set_pose(world, x, y, yaw):
    req = f"name: 'vehicle', position: {{x: {x:.3f}, y: {y:.3f}, z: 0.2}}, orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.4f}, w: {math.cos(yaw / 2):.4f}}}"
    subprocess.run(
        [
            "gz", "service", "-s", f"/world/{world}/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000", "--req", req,
        ],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else "test_track"
    for (x0, y0), (x1, y1) in zip(WAYPOINTS, WAYPOINTS[1:]):
        dist = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        steps = max(1, int(dist / STEP))
        for i in range(steps + 1):
            t = i / steps
            set_pose(world, x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw)
            time.sleep(PAUSE)


if __name__ == "__main__":
    main()
