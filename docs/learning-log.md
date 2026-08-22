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

---

## 2. Real simulated LiDAR through the unmodified Phase 1 pipeline

**Context.** Phase 1's whole point in using `sensor_msgs/PointCloud2` for the
synthetic scan was so a real sensor source could later be swapped in without
touching the processing code. Phase 2 starts by proving that promise true:
build a small Gazebo world, mount a simulated LiDAR + camera on a vehicle,
and point Phase 1's `point_cloud_processor_node` at the simulated LiDAR's
topic instead of the synthetic one.

**Action.**

- `sim/worlds/test_track.sdf`: a 30x30m enclosed world (ground plane + four
  wall boxes) with two obstacle boxes placed at the same coordinates as
  Phase 1's synthetic blobs, so the two data sources are visually and
  numerically comparable. A `vehicle` model (a plain box chassis — no
  drivetrain yet, that comes with actual driving in a later step) carries
  two sensors: a `gpu_lidar` (640 horizontal x 16 vertical samples, 360
  degrees, matching a small automotive-style LiDAR) and a `camera`.
- Confirmed the simulator layer works on its own first, before touching
  ROS2 at all: ran `gz sim -s -r --headless-rendering` (server-only, no
  window — WSL2 has no display by default) and used `gz topic -e` to
  read a raw LiDAR message directly from Gazebo's own transport layer.
- Bridged `/lidar/points` into ROS2 with `ros_gz_bridge`'s
  `parameter_bridge`:
  `/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked`
  — this one line is the entire translation from Gazebo's own message
  type to the ROS2 message type Phase 1's node already expects.
- Ran `point_cloud_processor_node` completely unmodified, just remapped at
  launch time (`--ros-args -r points_raw:=/lidar/points`) so it subscribes
  to the bridged LiDAR topic instead of the synthetic source node.

**Result.**

```
10240 -> 1464 points (voxel_size=0.30) in 2.676 ms on GPU
```

10240 = 640 x 16, exactly the horizontal x vertical sample counts set in the
LiDAR's SDF definition — confirms the scan pattern is configured as
intended. The GPU downsample ran in 2.676ms with zero source changes to the
Phase 1 code, only a topic remap: proof that treating `PointCloud2` as the
hard boundary between "wherever the points come from" and "what happens to
them" actually pays off once a second data source (Gazebo, after only
synthetic data) shows up.

---

## 3. Ground segmentation + clustering — and what real sensor data breaks that synthetic data never did

**Context.** Raw/downsampled points aren't obstacles yet — the next step is
telling "flat ground I can drive over" apart from "solid thing I can't,"
then grouping the solid points into individual objects with a position and
size. This is also the first time the pipeline touched *real* simulated
sensor output end to end instead of hand-generated data, and that exposed
two problems Phase 1's synthetic scan was too clean to ever surface.

**Action.**

- `obstacle_detector_node.cpp`: `pcl::SACSegmentation` (RANSAC) fits the
  single largest plane in the downsampled cloud and removes it — good
  enough here because the test track has exactly one dominant flat
  surface (the ground), not a general solution for slopes/curbs/multiple
  ground planes.
- The remaining "non-ground" points go through
  `pcl::EuclideanClusterExtraction` (distance-based grouping via a KD-tree)
  to split them into per-object clusters; each cluster gets a bounding box
  computed with `pcl::getMinMax3D` and published as a `visualization_msgs`
  `MarkerArray` for RViz, plus logged.

**Result — first crash, and why.** First run crashed inside PCL's KD-tree
radius search on a `NaN`-coordinate point. Root cause: this world has no
ceiling, and the LiDAR's vertical FOV (+/-15 degrees) means upward-angled
rays fired at anything other than very close range fly out over the 1m
walls into open sky and never hit anything within the 30m max range —
Gazebo reports those "no return" rays as `NaN` points. Phase 1's synthetic
scan was hand-generated and could never produce this; it only showed up the
moment a real (simulated) sensor entered the pipeline. Fixed by calling
`pcl::removeNaNFromPointCloud` on the raw cloud before it ever reaches the
GPU downsampler.

**Result — second problem, not a crash but wrong output.** With the crash
fixed, detection found 7 clusters, not 2. Two of them (with LiDAR-visible
size and position close to obstacle_1 at (3, 2) and obstacle_2 at (-4, -3))
were real. The other five weren't:

- Four ~20-30m, wafer-thin clusters — the perimeter walls. They're flat
  like the ground, but *vertical*, so the single-largest-plane removal
  (which only looks for one plane, and picks the biggest one — the floor)
  never touches them; they survive into "non-ground" and get clustered as
  if they were obstacles.
- One cluster roughly the size and position of the vehicle's own chassis
  (1.36 x 0.77m, centered near the sensor origin) — the LiDAR seeing its
  own vehicle body, a standard self-occlusion problem every real LiDAR
  integration has to handle.

**Fix.** Two independent filters, each solving a different half of the
problem:

1. In `point_cloud_processor_node`, drop any point within `min_range`
   (default 1.0m) of the sensor origin — cheap and correct here because the
   cloud is already expressed in the LiDAR's own frame, so distance from
   the origin *is* distance from the sensor. Removes the self-detection
   cluster.
2. In `obstacle_detector_node`, discard any cluster whose x or y extent
   exceeds `max_obstacle_extent` (default 3.0m) — real obstacles on this
   track are known to be under 1.5m; anything bigger is almost certainly a
   wall segment. This is a track-specific heuristic, not a general
   wall-detector — a real system would fit *all* dominant planes, not just
   the biggest one, and classify each by its normal vector (near-vertical
   normal = wall, near-horizontal = ground).

**Final result:**

```
cluster 0: 130 points, center (2.49, 1.96, 0.02), size (0.98 x 0.93 x 0.95)
cluster 1: 91 points, center (-4.48, -2.95, 0.27), size (0.76 x 0.72 x 1.44)
cluster 2: 6 points, center (-1.11, 0.05, -0.10), size (0.05 x 0.70 x 0.05)
```

Clusters 0 and 1 match obstacle_1 (3, 2, 1x1x1) and obstacle_2 (-4, -3,
0.8x0.8x1.5) — the detected center is offset from the true box center
because the LiDAR only ever sees the near-facing surface, not the far side
of a solid object, which is the expected physical limitation, not an error.
Cluster 2 is a small (6-point) leftover, most likely a sliver the min-range
filter didn't fully catch — left in and reported honestly rather than
tuning `min_cluster_size` up until it disappears from the log.

---

## 2. Installing CUDA hit a wall that had nothing to do with this project

**Context.** Before writing the GPU kernel, the CUDA compiler (`nvcc`)
needed installing on top of the WSL2 Ubuntu (distro: Lyrical) environment
from Entry 0. The GPU driver being visible (`nvidia-smi` works) is not the
same thing as having the *toolkit* to compile CUDA code — that's a separate
install from NVIDIA's own apt repository.

**Action.** Installed CUDA 12.6 via NVIDIA's `wsl-ubuntu` apt repo (the
WSL-specific one — it doesn't touch the GPU driver, which WSL2 gets from
Windows, not from Linux). Wrote a tiny "add two float arrays" `.cu` file to
smoke-test it before touching real project code.

**Result.** It didn't compile. Three attempts, in order:

1. CUDA 12.6 refused to compile at all — its bundled math headers declare
   functions (`cospi`, `sinpi`, `rsqrt`) with an exception specification
   that conflicts with how this system's very new glibc (Lyrical runs on
   Ubuntu 26.04, released days before this) declares the same functions.
   Tried an older host compiler (`g++-13`, since CUDA 12.6 caps at GCC 13
   and the system default is GCC 15) — same error, so it wasn't a compiler
   version problem.
2. Tried CUDA 12.9 instead, on the theory that a newer release might have
   updated headers — identical error. Not a version problem either.
3. Tried disabling the glibc language extensions that expose those
   particular function declarations (`-D_POSIX_C_SOURCE=200809L`,
   `-D_XOPEN_SOURCE=700`) and strict `-std=c++17` — still identical.

Three independent fixes failing identically is strong evidence this isn't
a flag I'm missing — it's a genuine, currently-unresolved incompatibility
between every CUDA release through 12.9 and a glibc newer than any of them
were tested against. Ubuntu 26.04 is bleeding-edge; CUDA's officially
validated distros are 22.04/24.04 LTS. Lesson: when a brand-new OS release
and a vendor toolkit disagree, the toolkit's compatibility matrix wins —
don't fight it, switch to what's actually validated.

---

## 3. Standing up a second, CUDA-validated environment

**Context.** Rather than keep fighting Ubuntu 26.04, installed a second
WSL distro — Ubuntu 24.04, one of the versions NVIDIA actually tests CUDA
against — dedicated to this project.

**Action.** `wsl --install -d Ubuntu-24.04`, then CUDA 12.6 the same way as
before. This time `nvcc` compiled and ran the smoke test cleanly on the
first try — GCC 13.3 is the *default* compiler on 24.04, already inside
CUDA 12.6's supported range, no workaround needed. Confirmed with a real
GPU compute test, not just "the compiler exists": summed two arrays of
2^20 floats on the GPU, checked every element came back correct.

**Result.** CUDA works. But now the environment is split: ROS2 lives only
in the 26.04 distro, CUDA only works in the 24.04 one. First approach:
compile the CUDA kernel as a standalone shared library (`.so`) in 24.04
with the CUDA runtime statically linked in (`--cudart static`), so it only
depends on `libcuda.so` — the GPU driver's userspace library, which both
WSL distros share via `/usr/lib/wsl/lib` since it's provided by the single
Windows host driver, not by either Linux install. Verified this actually
works: compiled a plain C++ test program with `g++` in the 26.04 distro
(no CUDA toolkit needed there at all) that linked against and called into
the `.so` built in 24.04. It worked — cross-distro linking through a shared
GPU driver library is a real, valid pattern.

It was also more architecture than the problem needed. Asked whether ROS2
(Jazzy, the release that officially targets Ubuntu 24.04) could just be
installed *inside* the already-working CUDA distro instead of bridging two
environments — yes, and that's obviously better: one build, one
environment, and Phase 2/3 won't have to repeat this dance for every future
CUDA-touching node. Installed ROS2 Jazzy + PCL in Ubuntu 24.04 and made it
the single dev environment for this project going forward. 26.04/Lyrical
now belongs entirely to the unrelated `friction-aware-planner` project and
this repo doesn't touch it again. The cross-distro validation wasn't wasted
effort — it proved the CUDA library itself was correct before the
architecture even mattered — but the lesson worth keeping is to check
"what does everyone else actually do" before building a bridge for a
problem that a simpler environment choice avoids entirely.

---

## 4. The voxel-grid downsampler, and getting it to build alongside ROS2

**Context.** The first real GPU workload: voxel-grid downsampling, the
standard first step in almost every LiDAR pipeline. A raw scan can be
hundreds of thousands of points; before doing anything else with it
(ground segmentation, clustering — Phase 2), the point count needs cutting
down without losing the scene's shape. The standard way: divide space into
a grid of cubes ("voxels") of a fixed size, and keep at most one point per
occupied cube.

**Action.** Implemented the same voxel-bucketing rule twice — once for CPU,
once for GPU — sharing the actual bucketing math (`voxel_key.hpp`) between
them so a CPU/GPU disagreement can only mean a real bug, not two different
definitions of "which voxel is this point in." Packs each point's grid
cell into a single 64-bit integer key (20 bits per axis).

- **CPU version**: builds a hash map from voxel key to "first point index
  seen for that key" — an `unordered_map` walk, one pass over the points.
- **GPU version**: no hash map (GPU-friendly hash tables are a real research
  topic on their own — not worth building from scratch for this). Instead:
  compute every point's key in parallel, then use Thrust (CUDA's built-in
  STL-like library) to sort points by key and keep the first of each run of
  equal keys. Sorting turns "group by voxel" into a problem GPUs are
  already extremely good at, which is the actual reason this design was
  chosen over hand-rolling a parallel hash table.

Getting this to build *inside* the ROS2 package (rather than as a separate
manually-invoked script, now that ROS2 + CUDA are in the same environment)
took three more CMake fixes, found by reading the actual failure each time
rather than guessing:

1. PCL's own CMake config pulls in VTK, whose config expects an
   `MPI::MPI_C` target to already exist — even though `libopenmpi-dev` was
   already installed, CMake still needs an explicit `find_package(MPI)`
   call to create that target, and it has to run *before*
   `find_package(pcl_conversions)`, because `pcl_conversions` triggers
   PCL's own `find_package(PCL)` as a side effect.
2. That `find_package(MPI)` call silently found nothing useful at first —
   CMake's FindMPI only searches for components matching the project's own
   *enabled languages*, and the project only declared `CXX` and `CUDA`, not
   `C`. Fixed by adding `C` to `project(... LANGUAGES C CXX CUDA)`.
3. `pcl_conversions` turned out to be header-only on this ROS2 release
   (Jazzy) and exports no CMake target at all — just a variable,
   `pcl_conversions_INCLUDE_DIRS`. On the other ROS2 release used in Entry
   1 (Lyrical) it *does* export a target. Made the `CMakeLists.txt` handle
   both: link the target only `if(TARGET pcl_conversions::pcl_conversions)`,
   and always add `${pcl_conversions_INCLUDE_DIRS}` explicitly so the
   headers are found either way.

**Result.** Correctness: CPU and GPU produce the *exact same output point
count* at every tested size (9883 / 88622 / 285625 / 367047 points at 10k /
100k / 500k / 1M input points) — since both use the identical bucketing
rule, the count of occupied voxels is a strict invariant, so an exact match
is a real correctness check, not a coincidence.

Performance (RTX 4060 Laptop GPU, Release build, `voxel_size=0.2`):

| points | CPU | GPU | speedup |
|---|---|---|---|
| 10,000 | 0.85 ms | 3.46 ms | 0.25x (GPU *slower*) |
| 100,000 | 8.98 ms | 5.25 ms | 1.71x |
| 500,000 | 60.0 ms | 16.8 ms | 3.58x |
| 1,000,000 | 127.5 ms | 28.2 ms | 4.52x |

Below roughly 50-100k points, GPU loses — kernel-launch and host↔device
memory-transfer overhead costs more than the compute saves. Above that, it
wins by a growing margin. This crossover is the actual finding, not "GPU is
faster" — a real LiDAR scan (tens of thousands to a few hundred thousand
points) sits right around where this decision matters, which is the honest
answer to "why GPU-accelerate this."

Wired the downsampler into `point_cloud_processor_node`: it now calls
`voxel_downsample` on every incoming scan and republishes the result on
`points_downsampled`. Live, on real ROS2 traffic: 8700 points in, ~6800-6837
out, 2.4-3.1 ms per frame — comfortably inside the 100 ms budget a 10 Hz
source allows. The very first call cost ~400 ms (CUDA's one-time lazy
context initialization), which would have made the first frame look like a
bug; added a throwaway warm-up call to the node's constructor to absorb
that cost before real data ever arrives, which is a detail worth
remembering to reproduce for the SLAM/localization node in Phase 3, since
it will call into CUDA too.

**Phase 1 is now complete**: a C++17 ROS2 package with a native-CMake-built
CUDA voxel downsampler, correctness-verified against an independent CPU
implementation, benchmarked with an honest (not cherry-picked) result, and
wired into a live two-node pipeline.
