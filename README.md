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
downsampled points, obstacle markers, and 2D detections).

Phase 3 (in progress): `fa_localization_cpp`'s `scan_matcher_node` does
frame-to-frame LiDAR odometry (ICP between consecutive scans, accumulated
into a pose estimate), constrained to planar (x, y, yaw) motion since this
is a ground vehicle, verified against Gazebo's ground truth at ~0.54m
final position error over a ~42m loop (~1.3%). `ekf_fusion_node` fuses
that with a new IMU sensor (gyro-integrated yaw as the predict step, ICP
as the correction) — the fused estimate's position (0.44m error, ~1.04%)
is measurably better than ICP alone (0.53m, ~1.26%), the actual point of
fusion rather than a symmetry exercise. See
[docs/learning-log.md](docs/learning-log.md) sections 8-9 for the full
"test a known-answer case first" debugging story — five real bugs total
(a too-tight ICP correspondence distance with no initial guess, an
inverted transform sign, corners snapping the heading instead of turning
gradually, unconstrained z/roll/pitch drifting despite the vehicle having
no physical way to move in those directions, and — found only once the
EKF made a persistent yaw bias visible — the very first frame silently
having the same instant-heading-snap bug as the corners). Closing the loop
into the friction-aware planner is still ahead.

See [docs/learning-log.md](docs/learning-log.md) for the full Phase 1/2
story too, including why the dev environment ended up as Ubuntu 24.04 +
ROS2 Jazzy + CUDA 12.6, and two real-sensor-data problems (NaN "no-return"
LiDAR points, the vehicle detecting its own chassis) that never showed up
with Phase 1's synthetic data.

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
