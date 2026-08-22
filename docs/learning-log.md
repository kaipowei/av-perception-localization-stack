# Learning Log

This file exists for one reason: so that six months from now, in an
interview, I can explain *every* piece of this project — not just "I did
LiDAR perception and SLAM," but why each specific technique was chosen, what
problem it actually solves, and what the numbers say about whether it
worked. Each entry follows the same shape:

- **Context** — what problem this step exists to solve, and why now
- **Action** — exactly what was built or decided
- **Result** — what came out of it (numbers, behavior, a decision made)

Jargon is explained the first time it shows up. Entries are numbered and
never edited after the fact — if something later turns out to be wrong, a
new entry corrects it and links back, the same way the code's own git
history isn't rewritten.

---

## 0. Why a separate repo, and what "SLAM" actually means here

**Context.** The plan going in was to extend the existing
`friction-aware-planner` project with a perception + localization layer,
because that project already covers motion planning (Hybrid-A\*, MPC) and
state estimation (an EKF for road friction), and most robotics-software /
AV-engineer internship postings want the same person to show perception and
SLAM too. But that project's own `ROADMAP.md` explicitly says: *"Explicitly
out of scope for this project: Full SLAM (localization is assumed known)."*
That was a deliberate decision made to keep that project's story focused —
adding SLAM into it quietly would contradict its own documented scope and
look inconsistent to anyone (e.g. an interviewer) who reads both the code
and the devlog. So this became its own repo instead: same vehicle,
narratively a continuation, but no code or git dependency between them.

**Action.** Created `av-perception-slam-stack` as a fresh git repo. Checked
the local dev machine for what's actually available to build this with,
since ROS2 + CUDA development normally happens on Linux, not native
Windows:

| Tool | Found | Why it matters |
|---|---|---|
| WSL2 Ubuntu | yes | Windows can't run ROS2 natively in any real way; WSL2 gives a real Linux kernel + GPU passthrough |
| ROS2 (distro: Lyrical) | yes, at `/opt/ros/lyrical` | the middleware everything else plugs into — nodes talk to each other over ROS2 topics |
| colcon | yes | the build tool that compiles a ROS2 workspace (like `make`, but ROS2-aware — knows how to build many small packages together and wire up their dependencies) |
| PCL (Point Cloud Library) 1.15.1 | yes | the standard C++ library for point-cloud math — ICP, plane fitting, clustering, all built in, so those don't need to be hand-written from scratch |
| NVIDIA GPU (RTX 4060) visible in WSL | yes (`nvidia-smi` works) | confirms WSL2 can actually see and use the GPU |
| CUDA compiler (`nvcc`) | **no** | the GPU driver being visible doesn't mean the *toolkit to compile CUDA code* is installed — that's a separate install, needed before Phase 1's GPU kernel work |

**Result.** Everything needed for the ROS2 + C++ + PCL side of Phase 1 is
already in place. CUDA toolkit install is the one remaining gap, deferred
until the point in Phase 1 where a GPU kernel is actually being written —
no reason to install a multi-GB toolkit before there's code that needs it.

---

## 1. First working node pair: generate a point cloud, read it back

**Context.** Before touching CUDA at all, the goal was to get the most
basic version of the pipeline working end to end: one C++ node that
produces a point cloud, one that reads it, using the exact same message
format (`sensor_msgs/PointCloud2`) that a real LiDAR driver — or, in
Phase 2, Gazebo's simulated LiDAR — would publish. Getting this boring part
right first means Phase 2 only has to swap the *source* of the data; the
reading/parsing code doesn't change.

**Action.**

- `point_cloud_source_node.cpp`: publishes a synthetic scan at 10 Hz —
  8000 points scattered across a flat 20m×20m "ground" plane, plus two
  raised clusters of points (400 and 300 points) standing in for
  obstacles. Real LiDAR data isn't available yet (that's Phase 2, once
  Gazebo is wired up), so this is a stand-in — but it's built with actual
  structure (a ground + distinct obstacle blobs) rather than pure random
  noise, so ground-segmentation and clustering work later has something
  real to prove itself against.
- `point_cloud_processor_node.cpp`: subscribes to the same topic, converts
  the ROS message back into a `pcl::PointCloud<pcl::PointXYZ>` with
  `pcl::fromROSMsg`, and logs the point count and x/y/z bounding box.
- Package uses PCL (Point Cloud Library) for the point-cloud type and the
  ROS-message conversion helpers, rather than hand-rolling the byte layout
  of `PointCloud2` — that byte layout (stride, offsets, field types) is
  exactly the kind of thing that's easy to get subtly wrong, and PCL's
  conversion functions are the standard tool for it in ROS2 C++ code.

**Result.** Two bugs hit along the way, both worth remembering:

1. `ament_target_dependencies()` — the CMake macro every ROS2 C++ tutorial
   uses to link a node against its dependencies — doesn't exist in this
   ROS2 release (distro: Lyrical). It's been replaced by linking directly
   against namespaced CMake targets (`rclcpp::rclcpp`,
   `sensor_msgs::sensor_msgs`, `pcl_conversions::pcl_conversions`), which
   is the more modern CMake pattern anyway. Confirmed by grepping the
   installed `*Export.cmake` files under `/opt/ros/lyrical/share/*/cmake/`
   for the actual exported target names rather than guessing.
2. Building with `build/` and `install/` under the Windows-mounted path
   (`/mnt/c/...`) took 8m47s for four small source files — real time vastly
   exceeded actual CPU time (`user`+`sys` was under 1m40s), which is the
   signature of WSL2's cross-filesystem overhead (DrvFs), not real compute.
   Redirecting colcon's build/install output to a native WSL path
   (`~/ros2_build/...`, via `--build-base`/`--install-base`) while leaving
   source on the Windows side cut it to 5m31s — better, not fully solved,
   and not worth chasing further right now. Worth revisiting if build times
   become a real drag once CUDA compilation is added.

Ran both nodes together (`ros2 run fa_perception_cpp point_cloud_source_node`
/ `point_cloud_processor_node`, sourcing the native-path install). The
processor logged:

```
8700 points, bounds x[-10.00, 10.00] y[-10.00, 10.00] z[-0.02, 1.15]
```

8700 = 8000 ground + 400 + 300 obstacle points, matching the source node's
generation counts exactly. The z bound of 1.15 matches the second obstacle
blob's center height (0.75) plus its radius (0.4) — not a coincidental
number, confirmation the geometry is generated and read back correctly, not
just "a node ran without crashing."
