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
YAW_STEP = math.radians(10)  # max heading change per pose at a corner


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
    # Video-demo use wants few, large steps (fast, whole loop in ~seconds).
    # ICP scan-matching wants the opposite: consecutive LiDAR frames need
    # enough overlap to register reliably, which means small steps between
    # poses even though that means more `gz service` calls and a slower
    # sweep overall -- pass a smaller step explicitly for that use.
    step = float(sys.argv[2]) if len(sys.argv) > 2 else STEP

    current_yaw = math.atan2(WAYPOINTS[1][1] - WAYPOINTS[0][1], WAYPOINTS[1][0] - WAYPOINTS[0][0])
    for (x0, y0), (x1, y1) in zip(WAYPOINTS, WAYPOINTS[1:]):
        target_yaw = math.atan2(y1 - y0, x1 - x0)

        # Corners used to snap the heading instantly to the new segment's
        # direction -- a ~90 degree jump in one ICP frame is the same
        # failure as the too-large translation step: the true displacement
        # exceeds max_correspondence_distance and ICP can't find correct
        # correspondences at all. Rotate in place in small steps first.
        yaw_diff = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
        rotate_steps = max(1, int(abs(yaw_diff) / YAW_STEP))
        for i in range(1, rotate_steps + 1):
            yaw = current_yaw + yaw_diff * (i / rotate_steps)
            set_pose(world, x0, y0, yaw)
            time.sleep(PAUSE)
        current_yaw = target_yaw

        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(dist / step))
        for i in range(1, steps + 1):
            t = i / steps
            set_pose(world, x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, current_yaw)
            time.sleep(PAUSE)


if __name__ == "__main__":
    main()
