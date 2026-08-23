# av-perception-slam-stack

C++17 / CUDA / ROS2 perception + localization stack for a ground vehicle:
LiDAR point clouds in, obstacle detections and a live pose estimate out.

This is a companion project to
[friction-aware-planner](https://github.com/kaipowei/friction-aware-planner)
(motion planning + friction-aware speed control). That project explicitly
scoped **out** perception and SLAM to stay focused on its own thesis
(friction-limited planning). This project picks that up: it exists to add the
sensing and localization layer — LiDAR/point-cloud processing, obstacle
detection, and scan-matching localization — that a full autonomy stack needs
alongside planning and control. No code dependency between the two repos;
the connection is narrative (two projects, one stack), not a git submodule.

Built specifically to close a resume gap: production C++ and CUDA experience,
and SLAM/localization/mapping — all explicitly called out as requirements in
robotics-software and AV-engineer internship postings (NVIDIA's AV & Robotics
internship among them) but not backed by any prior project.

See [docs/learning-log.md](docs/learning-log.md) for a plain-language,
step-by-step record of what was built, why, and what it produced — written to
double as interview prep material.

## Status

Phase 1 complete: a C++17 ROS2 package with a native-CMake CUDA voxel-grid
downsampler, correctness-checked against a CPU reference implementation and
benchmarked (GPU wins above ~100k points; loses below that to launch/
transfer overhead), wired into a live source → processor node pipeline.

Phase 2 complete: a Gazebo (Harmonic) test world with a simulated LiDAR +
camera, ground segmentation + Euclidean clustering (PCL) on the
GPU-downsampled cloud, a YOLO-based camera detector (`fa_perception_py`),
and a recorded rosbag dataset (116.5s / 3677 messages across LiDAR,
downsampled points, obstacle markers, and 2D detections). See
[docs/learning-log.md](docs/learning-log.md) for the full story, including
why the dev environment ended up as Ubuntu 24.04 + ROS2 Jazzy + CUDA 12.6,
and two real-sensor-data problems (NaN "no-return" LiDAR points, the
vehicle detecting its own chassis) that never showed up with Phase 1's
synthetic data. Phase 3 (ICP-based localization/SLAM, fused with the
friction-aware planner's EKF) is next.

## Requirements

- ROS2 Jazzy
- PCL (Point Cloud Library) 1.12+
- colcon
- CUDA toolkit 12.6 (for the GPU-accelerated processing nodes)
- Developed and run inside WSL2 Ubuntu 24.04 on Windows — see the learning
  log for why this specific combination

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```
