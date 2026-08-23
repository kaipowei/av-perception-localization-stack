# Learning Log

This file exists for one reason: so that six months from now, in an
interview, I can explain every piece of this project — not just "I did
LiDAR perception and SLAM" but why I picked each technique, what problem it
actually solves, and what the numbers say about whether it worked. Each
entry is roughly Context / Action / Result. Entries are numbered and I
don't go back and edit them after the fact — if something later turns out
to be wrong, the next entry says so and links back, same as git history
isn't rewritten.

(Note: entries 2-4 originally got tacked onto the end of this file out of
order — I wrote them, then kept adding Phase 2/3 stuff in the middle
without going back to fix the numbering. Reordered everything into actual
chronological order below. Content's identical, just in the right place
now.)

---

## 0. Why a separate repo, and what "SLAM" actually means here

**Context.** Going in, the plan was to bolt a perception + localization
layer onto the existing `friction-aware-planner` project, since that one
already has motion planning (Hybrid-A*, MPC) and an EKF (for road
friction), and most robotics/AV internship postings want the same person
to show perception and SLAM too. Except that project's own `ROADMAP.md`
says, in plain words: *"Explicitly out of scope for this project: Full
SLAM (localization is assumed known)."* That was on purpose — kept the
project's story tight. Quietly adding SLAM into it later would contradict
its own docs, which is exactly the kind of thing an interviewer notices
when they read both the code and the devlog. So: new repo. Same vehicle,
same narrative, but zero code or git dependency between the two.

**Action.** Made `av-perception-slam-stack`, fresh git repo. Before
writing anything, checked what was actually available on this machine —
ROS2 + CUDA is a Linux thing, not native Windows:

| Tool | Found | Why it matters |
|---|---|---|
| WSL2 Ubuntu | yes | Windows can't run ROS2 for real; WSL2 gives a real Linux kernel + GPU passthrough |
| ROS2 (Lyrical) | yes, `/opt/ros/lyrical` | the middleware everything talks over |
| colcon | yes | builds a ROS2 workspace, like `make` but knows how packages depend on each other |
| PCL 1.15.1 | yes | point-cloud math (ICP, plane fitting, clustering) already built, don't hand-roll it |
| RTX 4060 visible in WSL | yes, `nvidia-smi` works | confirms WSL2 can actually see the GPU |
| CUDA compiler (`nvcc`) | no | driver visible ≠ toolkit installed, that's separate |

**Result.** Everything for ROS2 + C++ + PCL is there. CUDA's the one gap,
and I left it — no reason to install a multi-GB toolkit before there's a
kernel that needs it.

---

## 1. First working node pair: generate a point cloud, read it back

**Context.** Before touching CUDA, wanted the dumbest possible version of
the pipeline working end to end: one node that makes a point cloud, one
that reads it, both talking the exact message format
(`sensor_msgs/PointCloud2`) a real LiDAR driver — or Gazebo's simulated one,
Phase 2 — would actually use. Get that boring part right first and Phase 2
only has to swap where the data comes from, not how it's read.

**Action.**

- `point_cloud_source_node.cpp`: publishes a fake scan at 10 Hz — 8000
  points on a flat 20x20m "ground," plus two raised blobs (400 and 300
  points) standing in for obstacles. No real LiDAR yet, so this is a
  placeholder, but built with real structure instead of pure noise so
  ground segmentation later has something to actually prove itself on.
- `point_cloud_processor_node.cpp`: subscribes, converts back to
  `pcl::PointCloud<pcl::PointXYZ>` with `pcl::fromROSMsg`, logs point count
  and bounding box.
- Used PCL for the point type and conversion helpers instead of hand-rolling
  the `PointCloud2` byte layout myself — stride/offset/field-type bugs are
  exactly the kind of thing that's easy to get subtly wrong, and PCL's
  conversion functions are the standard tool everyone actually uses here.

**Result.** Two things worth remembering:

1. `ament_target_dependencies()` — the CMake macro every ROS2 tutorial
   uses — doesn't exist on this ROS2 release (Lyrical). Replaced by linking
   straight against namespaced targets (`rclcpp::rclcpp`,
   `sensor_msgs::sensor_msgs`, `pcl_conversions::pcl_conversions`), which
   turns out to be the modern pattern anyway. Found the real target names by
   grepping the installed `*Export.cmake` files instead of guessing.
2. Building with `build/`/`install/` on the Windows-mounted path
   (`/mnt/c/...`) took 8m47s for four tiny files, while actual CPU time was
   under 1m40s — that gap is WSL2's cross-filesystem overhead (DrvFs), not
   real compute. Pointing colcon's build/install output at native WSL
   instead (source stays on the Windows side) cut it to 5m31s. Not fully
   fixed, not worth chasing harder right now.

Ran both nodes. Processor logged:

```
8700 points, bounds x[-10.00, 10.00] y[-10.00, 10.00] z[-0.02, 1.15]
```

8700 = 8000 ground + 400 + 300, matches the source exactly. The z bound of
1.15 matches the second blob's center height (0.75) plus radius (0.4) —
not a coincidence, that's confirmation the geometry round-trips correctly,
not just "a node ran and didn't crash."

---

## 2. Installing CUDA hit a wall that had nothing to do with this project

**Context.** Before writing the actual GPU kernel, needed `nvcc` on top of
the WSL2 Ubuntu (Lyrical) setup from entry 0. GPU driver being visible
(`nvidia-smi` works) isn't the same as having the toolkit to compile CUDA —
that's a separate install off NVIDIA's own apt repo.

**Action.** Installed CUDA 12.6 via NVIDIA's `wsl-ubuntu` apt repo (WSL-
specific, doesn't touch the driver — WSL2 gets that from Windows, not
Linux). Wrote a tiny "add two float arrays" `.cu` file as a smoke test
before touching real project code.

**Result.** Didn't compile. Three tries:

1. CUDA 12.6 flat-out refused — its bundled math headers declare functions
   (`cospi`, `sinpi`, `rsqrt`) with an exception spec that conflicts with
   how this system's very new glibc (Lyrical runs on Ubuntu 26.04, released
   days before this) declares the same functions. Tried an older host
   compiler (g++-13, since CUDA 12.6 caps at GCC 13 and the system default
   is GCC 15) — identical error, so not a compiler-version thing.
2. Tried CUDA 12.9 instead, betting a newer release had fixed headers —
   same error. Not a version thing either.
3. Tried disabling the glibc extensions that expose those declarations
   (`-D_POSIX_C_SOURCE=200809L`, `-D_XOPEN_SOURCE=700`) plus strict
   `-std=c++17` — still identical.

Three unrelated fixes failing the exact same way is strong evidence it's
not a flag I'm missing — it's a real, currently-unresolved clash between
every CUDA release up through 12.9 and a glibc newer than any of them were
tested against. Ubuntu 26.04 is bleeding-edge; CUDA's officially validated
distros are 22.04/24.04 LTS. Lesson: when a brand-new OS and a vendor
toolkit disagree, the toolkit's compatibility matrix wins. Don't fight it.

---

## 3. Standing up a second, CUDA-validated environment

**Context.** Rather than keep fighting 26.04, installed a second WSL
distro — Ubuntu 24.04, one of the versions NVIDIA actually tests against —
just for this project.

**Action.** `wsl --install -d Ubuntu-24.04`, then CUDA 12.6 the same way.
This time `nvcc` compiled and ran the smoke test clean on the first try —
GCC 13.3 is 24.04's default, already inside CUDA 12.6's supported range,
no workaround needed. Confirmed with a real GPU compute check, not just
"the compiler exists": summed two arrays of 2^20 floats on the GPU,
checked every element came back right.

**Result.** CUDA works, but now the environment's split — ROS2 only lives
in 26.04, CUDA only works in 24.04. First fix: compile the kernel as a
standalone `.so` in 24.04 with the CUDA runtime statically linked
(`--cudart static`), so it only needs `libcuda.so` — the driver's userspace
library, which both distros share through `/usr/lib/wsl/lib` since it comes
from the one Windows host driver, not either Linux install. Actually tested
this, not just assumed it: a plain g++ program in 26.04 (no CUDA toolkit
at all) linked against and called into the `.so` built in 24.04, and it
worked. Cross-distro linking through a shared driver library is a real,
valid pattern.

It was also more architecture than the problem needed. Asked whether ROS2
Jazzy (the release that officially targets 24.04) could just go *inside*
the already-working CUDA distro instead — yes, obviously better, one
environment instead of two, and Phase 2/3 won't have to repeat this dance
every time a node touches the GPU. Installed ROS2 Jazzy + PCL in 24.04 and
made it the one dev environment going forward. 26.04/Lyrical now belongs
entirely to `friction-aware-planner` and this repo doesn't touch it again.
The cross-distro work wasn't wasted — it proved the CUDA library itself was
correct before the architecture even mattered — but the actual lesson is
check what everyone else does before building a bridge for a problem a
simpler environment choice just avoids.

---

## 4. The voxel-grid downsampler, and Phase 1 wraps up

**Context.** First real GPU workload: voxel-grid downsampling, the
standard first step of basically every LiDAR pipeline. A raw scan can be
hundreds of thousands of points; before doing anything with it (ground
segmentation, clustering — Phase 2), the count needs cutting down without
losing the shape of the scene. Standard approach: chop space into a grid
of fixed-size cubes ("voxels"), keep at most one point per occupied cube.

**Action.** Wrote the same bucketing rule twice, once CPU once GPU,
sharing the actual math (`voxel_key.hpp`) between them — so a CPU/GPU
disagreement can only mean a real bug, not two slightly different
definitions of "which voxel is this point in." Packs each point's grid
cell into one 64-bit key (20 bits per axis).

- **CPU:** hash map from voxel key to "first point seen for that key," one
  pass over the points.
- **GPU:** no hash map — GPU hash tables are their own research topic, not
  worth building from scratch here. Instead: compute every point's key in
  parallel, then use Thrust (CUDA's built-in STL-ish library) to sort by
  key and keep the first of each run. Turns "group by voxel" into a sort,
  which GPUs are already very good at — that's the actual reason for this
  design, not just "sorting is fun."

Getting this to build inside the ROS2 package (now that ROS2 and CUDA
share an environment) took three more CMake fixes, each found by reading
the actual error instead of guessing:

1. PCL's own CMake config pulls in VTK, whose config wants an `MPI::MPI_C`
   target to already exist — even with `libopenmpi-dev` installed, CMake
   still needs an explicit `find_package(MPI)` to create that target, and
   it has to run before `find_package(pcl_conversions)`, since
   `pcl_conversions` triggers PCL's own `find_package(PCL)` as a side
   effect.
2. That `find_package(MPI)` call found nothing useful at first — CMake's
   FindMPI only looks for components matching the project's enabled
   languages, and the project only declared `CXX` and `CUDA`, not `C`.
   Fixed by adding `C` to `project(... LANGUAGES C CXX CUDA)`.
3. `pcl_conversions` turns out to be header-only on this ROS2 release
   (Jazzy) and exports no CMake target — just a variable,
   `pcl_conversions_INCLUDE_DIRS`. On Lyrical (entry 1) it does export a
   target. Made `CMakeLists.txt` handle both: link the target only
   `if(TARGET pcl_conversions::pcl_conversions)`, and always add
   `${pcl_conversions_INCLUDE_DIRS}` explicitly so headers get found
   either way.

**Result.** Correctness: CPU and GPU produce the exact same output point
count at every size tested (9883 / 88622 / 285625 / 367047 at 10k / 100k /
500k / 1M input points) — since both use the identical bucketing rule, the
count of occupied voxels is a strict invariant, so an exact match is a
real check, not luck.

Performance (RTX 4060 Laptop GPU, Release build, `voxel_size=0.2`):

| points | CPU | GPU | speedup |
|---|---|---|---|
| 10,000 | 0.85 ms | 3.46 ms | 0.25x (GPU *slower*) |
| 100,000 | 8.98 ms | 5.25 ms | 1.71x |
| 500,000 | 60.0 ms | 16.8 ms | 3.58x |
| 1,000,000 | 127.5 ms | 28.2 ms | 4.52x |

Below roughly 50-100k points GPU loses — kernel-launch and memory-transfer
overhead costs more than the compute saves. Above that it wins, by more the
bigger the cloud gets. That crossover is the actual finding here, not "GPU
is faster" — a real LiDAR scan (tens of thousands to a few hundred
thousand points) sits right around where this decision matters, which is
the honest answer to "why bother GPU-accelerating this at all."

Wired it into `point_cloud_processor_node`: calls `voxel_downsample` on
every incoming scan, republishes on `points_downsampled`. Live: 8700 in,
~6800-6837 out, 2.4-3.1 ms per frame — well inside the 100ms budget a
10Hz source gives you. First call cost ~400ms (CUDA's one-time lazy
context init), which would've looked like a bug on the first real frame —
added a throwaway warm-up call in the constructor to eat that cost before
any real data shows up. Worth remembering to do the same thing again for
Phase 3's localization node, since that calls into CUDA too.

**Phase 1 done:** a C++17 ROS2 package with a native-CMake CUDA voxel
downsampler, checked against an independent CPU implementation, benchmarked
honestly (not cherry-picked), wired into a live two-node pipeline.

---

## 5. Real simulated LiDAR through the unmodified Phase 1 pipeline

**Context.** The whole reason Phase 1 used `sensor_msgs/PointCloud2` for
the fake scan was so a real sensor could get swapped in later without
touching the processing code. Phase 2 starts by actually proving that:
small Gazebo world, simulated LiDAR + camera on a vehicle, point Phase 1's
`point_cloud_processor_node` at the simulated LiDAR instead of the fake
source.

**Action.**

- `sim/worlds/test_track.sdf`: 30x30m enclosed world (ground + four wall
  boxes), two obstacle boxes at the same coordinates as Phase 1's fake
  blobs so the two datasets line up. `vehicle` model (plain box chassis,
  no drivetrain yet — that's later) carries a `gpu_lidar` (640 horizontal x
  16 vertical samples, 360°, roughly small-automotive-LiDAR spec) and a
  `camera`.
- Checked the simulator worked on its own first, before touching ROS2:
  `gz sim -s -r --headless-rendering` (server only, no window — WSL2 has
  no display by default) and `gz topic -e` to read a raw LiDAR message
  straight off Gazebo's own transport.
- Bridged `/lidar/points` into ROS2 with `ros_gz_bridge`:
  `/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked` —
  that one line is the entire translation from Gazebo's message type to
  what Phase 1's node already expects.
- Ran `point_cloud_processor_node` completely unmodified, just remapped at
  launch (`--ros-args -r points_raw:=/lidar/points`).

**Result.**

```
10240 -> 1464 points (voxel_size=0.30) in 2.676 ms on GPU
```

10240 = 640 x 16, exactly the sample counts set in the LiDAR's SDF —
confirms the scan pattern is configured how I meant. GPU downsample ran in
2.676ms with zero source changes, just a topic remap: proof that treating
`PointCloud2` as the hard boundary between "where points come from" and
"what happens to them" actually pays off the first time a second data
source shows up.

---

## 6. Ground segmentation + clustering — and what real sensor data breaks that fake data never did

**Context.** Raw or downsampled points aren't obstacles yet — next step is
telling "flat ground I can drive on" apart from "solid thing I can't,"
then grouping the solid points into individual objects with a position and
size. Also the first time the pipeline touched real simulated sensor
output end to end instead of hand-made data, and that turned up two
problems Phase 1's clean fake scan never could.

**Action.**

- `obstacle_detector_node.cpp`: `pcl::SACSegmentation` (RANSAC) fits the
  single largest plane in the downsampled cloud and removes it — good
  enough here since this track has exactly one dominant flat surface (the
  ground), not a general answer for slopes/curbs/multiple ground planes.
- Remaining "non-ground" points go through
  `pcl::EuclideanClusterExtraction` (distance-based grouping via a KD-tree)
  and get split into per-object clusters, each with a bounding box from
  `pcl::getMinMax3D`, published as a `visualization_msgs` `MarkerArray`.

**Result — first crash, and why.** First run crashed inside PCL's KD-tree
on a NaN-coordinate point. Cause: this world has no ceiling, and the
LiDAR's vertical FOV (±15°) means upward-angled rays past very close range
fly out over the 1m walls into open sky and never hit anything within the
30m max range — Gazebo reports those "no return" rays as NaN. Phase 1's
fake scan was hand-generated and could never produce this; it only showed
up once a real (simulated) sensor entered the loop. Fixed with
`pcl::removeNaNFromPointCloud` on the raw cloud before it hits the GPU
downsampler.

**Result — second problem, not a crash, just wrong.** With the crash
fixed, detection found 7 clusters, not 2. Two matched real obstacles
(close in size/position to obstacle_1 at (3,2) and obstacle_2 at (-4,-3)).
The other five didn't:

- Four ~20-30m wafer-thin clusters — the perimeter walls. Flat like the
  ground, but vertical, so the single-largest-plane removal (which only
  ever looks for one plane, the biggest — the floor) never touches them;
  they survive into "non-ground" and get clustered like obstacles.
- One cluster about the size and position of the vehicle's own chassis
  (1.36 x 0.77m, centered near the sensor) — the LiDAR seeing itself, a
  standard self-occlusion problem every real LiDAR setup has to deal with.

**Fix.** Two filters, each solving half the problem:

1. `point_cloud_processor_node`: drop any point within `min_range`
   (default 1.0m) of the origin — correct here because the cloud's already
   in the LiDAR's own frame, so distance from origin *is* distance from
   sensor. Kills the self-detection.
2. `obstacle_detector_node`: throw out any cluster whose x or y extent
   exceeds `max_obstacle_extent` (default 3.0m) — real obstacles here are
   known to be under 1.5m, anything bigger is basically guaranteed to be a
   wall. Track-specific heuristic, not a real wall detector — a proper
   system would fit all the dominant planes, not just the biggest, and
   classify each by its normal (near-vertical = wall, near-horizontal =
   ground).

**Final result:**

```
cluster 0: 130 points, center (2.49, 1.96, 0.02), size (0.98 x 0.93 x 0.95)
cluster 1: 91 points, center (-4.48, -2.95, 0.27), size (0.76 x 0.72 x 1.44)
cluster 2: 6 points, center (-1.11, 0.05, -0.10), size (0.05 x 0.70 x 0.05)
```

Clusters 0 and 1 match obstacle_1 (3, 2, 1x1x1) and obstacle_2 (-4, -3,
0.8x0.8x1.5) — the center's offset from the true box center because the
LiDAR only ever sees the near face, not the far side of a solid object,
which is a physical limit, not a bug. Cluster 2 is a small 6-point
leftover, probably a sliver the min-range filter missed — left it in and
said so, instead of nudging `min_cluster_size` up until it quietly
disappears from the log.

---

## 7. Camera + YOLO — why a colored box was never going to work, and what fixed it once it did

**Context.** Plan: 2D detection on the camera feed as a loose companion to
the LiDAR obstacle detector. The two Gazebo obstacle boxes from entry 6 are
plain colored boxes with no real-world shape, and a COCO-trained YOLO
recognizes objects mostly by silhouette, not surface color — texture-
mapping a photo onto a cube's faces wouldn't fix that either, since the
*shape* is still a cube, and a stretched texture is arguably a worse input
than a flat color. Rather than build the node and stare at an empty
detection log, said this up front, and the fix was adding a model with
real geometry: "Standing person" from Gazebo Fuel. Took a couple tries to
even find it — `gz fuel list` has no keyword search, and guessing the name
("Person standing") 404'd; the real name was "Standing person," found
through Fuel's search API. Downloaded with `gz fuel download`, dropped
into the world via `<include><uri>...</uri></include>`, which resolves
against the local Fuel cache once downloaded. Before writing the detector
node, saved and looked at one actual camera frame
(`sim/save_one_frame.py`, throwaway script, not part of the package) to
make sure the model actually rendered as a recognizable person instead of
debugging a detector against a scene that might not even look right.

**Action.** New package `fa_perception_py` (ament_python — Python and C++
ROS2 nodes get kept in separate packages, not mixed into one build
system). `yolo_detector_node.py` subscribes to the camera topic, runs a
pretrained `yolov8n.pt`, publishes `vision_msgs/Detection2DArray`. On
purpose not fused with the LiDAR detector yet — proving the camera path
works on its own first.

**Result — a second real environment fight.** First run segfaulted inside
`cv_bridge`'s C++ image conversion. Cause: `cv_bridge` is a compiled ROS2
(apt) package built against NumPy's 1.x C ABI; installing `ultralytics` via
pip pulled in NumPy 2.5.2, and NumPy 2.x broke binary compatibility with
anything compiled against 1.x — a known kind of problem when apt-installed
ROS2 packages and pip-installed ML packages share a Python environment.
Fixed by pinning `numpy<2` (1.26.4) after the fact; pip warned that
pip-installed `opencv-python` "requires numpy>=2," but that turned out too
conservative — both `cv_bridge` and `cv2` worked fine against 1.26.4 in
practice.

**Final result**, live against the simulated camera:

```
person (0.78) at [309, 71, 352, 302]
```

— repeated every frame, confidence 0.78, box coordinates matching where
the person actually sits on screen. Confirms camera → YOLO → ROS2 works
end to end against something with real geometry, and by contrast with
entry 6 (the same detector code gets 0 hits on the colored obstacle boxes)
confirms the "a box won't be recognized" call was right, not a guess I got
lucky on.

---

## 8. Recording a dataset — and why it ended up done by hand

**Context.** Last Phase 2 item: drive the vehicle around while recording
every perception topic to a rosbag, so there's an actual dataset (not
just live demos) for later phases. No drivetrain yet, so "driving" meant
sweeping the pose through waypoints via Gazebo's `set_pose` service.

**What went wrong first, twice.**

1. First attempt scripted the waypoint sweep by shelling out to `gz
   service` once per waypoint (`sim/drive_loop.py`, originally `STEP=0.5`m
   → ~106 calls). Each call pays the full cost of spawning a process and
   opening a fresh transport connection, which ended up dominating the
   intended 0.15s pause — a 16-second drive stretched into minutes.
   Recording ran the whole time, and recording the raw, uncompressed
   camera topic (640x480 at ~15Hz) added ~14MB/s on its own, so the bag
   hit 641MB before anything useful got learned from it. Fixed the
   waypoint density (`STEP=2.0`, ~27 calls) and dropped raw `/camera` from
   the recorded topics — `/camera_info` alone is enough to prove the
   camera path is live without the size cost. Real fix would be
   `compressed_image_transport`, noted but not built, since it wasn't
   needed for what this recording's actually for (LiDAR + detections, not
   video).
2. Tried Gazebo's windowed GUI (`gz sim -r`, no headless flag) so I could
   watch it live instead of reasoning from logs. WSLg (confirmed working —
   `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`) is set up on this machine,
   but a GUI launched through my own tool-invoked `wsl -d ... -e bash -lc`
   ran clean, zero errors, and just never produced a window — apparently a
   different session context than an interactively-opened terminal, which
   WSLg's forwarding doesn't reach. Confirmed by having Eric run the exact
   same command in his own terminal — window showed up immediately.

**What actually worked: doing it by hand.** Rather than keep debugging
scripted teleop and cross-session GUI weirdness for a one-time recording,
Eric ran the whole thing himself across four terminals — Gazebo GUI in
one, `ros_gz_bridge` in another, the three perception nodes in a third,
`ros2 bag record` in a fourth — and dragged the vehicle around by hand
with Gazebo's translate gizmo while it recorded. Simpler than getting the
automation right, and the dataset doesn't care how the vehicle moved.

**Result:**

```
Duration: 116.5s | Messages: 3677
/lidar/points        607  (raw LiDAR)
/points_downsampled  616  (after GPU voxel downsample)
/obstacle_markers    605  (ground segmentation + clustering)
/detections_2d       929  (YOLO)
/camera_info         920
```

A clean shutdown (Ctrl+C, not killing the terminal) turned out necessary
for `ros2 bag` to actually write its metadata file — an earlier check
where the recorder was still alive produced "Could not find metadata in
bag directory" until it got stopped properly.

---

## 9. Chasing a choppy demo video — three wrong guesses before the real one

**Context.** Wanted a short video of what the camera and LiDAR actually
see (not Gazebo's third-person GUI view), for a portfolio keepsake. Built
`sim/capture_sensor_views.py`: subscribes to `/camera` and
`/lidar/points`, saves a JPEG per camera frame and a top-down render per
LiDAR scan, `ffmpeg` turns each sequence into an mp4. First attempt
(scripted `drive_loop.py`, headless, 25s) came out smooth. Every attempt
after that — all manual dragging in Gazebo's GUI — came out choppy,
several seconds between updates. Figuring out why took three wrong turns
before the real one, worth keeping because each guess was reasonable and
each got disproven by an actual measurement, not just abandoned.

**Wrong guess 1: LiDAR too dense.** The top-down render used matplotlib,
rebuilding a whole Figure/Axes and calling `scatter()` fresh every frame —
expensive, and render time scales with point count. The LiDAR had briefly
gone from 640x16 (10,240 pts/scan) to 1024x64 (65,536 pts/scan) for a
different reason (visual density, entry 8), and dropping it back to
640x16 did measurably help (≈130 frames/60s at 1024x64 vs ≈364 frames/60s
at 640x16) but the video was still choppy. Part of it, not the whole
thing.

**Wrong guess 2: not enough frames, drive slower.** Figured if capture's
running at some fixed low rate, moving slower means less distance per
frame and it'd look smoother. Changed nothing — the real problem isn't
distance-per-frame, it's that frames were arriving unevenly in wall-clock
time no matter how fast or slow the drag was.

**Wrong guess 3 (partly right): matplotlib is just slow.** Rewrote the
LiDAR renderer from matplotlib to plain NumPy + OpenCV — map each point's
(x,y) straight to a pixel, color by height with `cv2.applyColorMap`
(vectorized over the whole array, no per-point loop), skip Figure/Axes
entirely. Real, worth keeping. But re-running the manual-drag capture with
it was still choppy. Wrong as the explanation for what Eric was actually
seeing, right as a general improvement anyway.

**The actual cause.** Eric noticed the very first video (headless,
scripted) had been smooth, and every choppy one since involved manually
dragging the model in the GUI. That's the variable that mattered:
dragging interactively forces Gazebo to keep re-rendering the 3D GUI view
*at the same time* as the LiDAR/camera sensors, and on this WSL2 setup's
shared/virtualized GPU, that contention drags the simulation's real-time
factor down — sim time crawls relative to wall clock while you're
interacting, so sensor topics update sparsely in real terms even though
nothing about the capture script changed. Re-ran the same OpenCV capture
headless + scripted and got 790 camera / 546 LiDAR frames in 25 seconds
(~32/22 Hz) — about 10x the manual-drag rate, visibly smooth.

**Kept for later:** anywhere data rate matters (this recording, Phase 3's
SLAM data too), drive the sim headless with a script on this machine, not
by hand in the GUI. Keeping the OpenCV rewrite regardless — it's faster
and it fixed a real bug too (`skip_nans` only drops NaN, not the +/-inf a
real "no return" ray can produce, which were silently casting to garbage
pixel indices before the bounds check caught them).

---

## 10. Phase 3, first attempt: point-to-point ICP fails on a ground-dominated scene

**Context.** First localization test: `scan_matcher_node` (new package
`fa_localization_cpp`) runs `pcl::IterativeClosestPoint` between
consecutive downsampled scans and accumulates the relative transforms into
a running pose, tested by driving the fixed loop
`(0,0)->(6,6)->(6,-6)->(-6,-6)->(-6,6)->(0,0)` with a fine step (0.3m, via
`drive_loop.py test_track 0.3` — the coarse 2m steps used for demo videos
move too far between consecutive frames for ICP to find correct matches)
and checking the final estimate against Gazebo's ground truth
(`gz model -m vehicle -p`).

**A false alarm first.** First run showed ICP's estimate wildly off *and*
Gazebo's own reported end pose not matching the loop's known endpoint
(0,0). Cause: a leftover Gazebo GUI instance from earlier manual-drag
testing had never actually died — an earlier `pkill -f "gz sim"` had
matched its own command line and killed itself instead of the target (same
gotcha as entry 9) — and it was still running, colliding with the fresh
headless instance on the same world name and scrambling which instance the
bridge/queries were actually talking to. Killed the stray one by exact
PID, confirmed exactly one `gz sim` process, re-ran: ground truth then
matched the loop exactly (start (0,0,0.2,yaw=0), end (0,0,0.2,yaw=-0.785)
— a clean loop closure, as expected).

**The real problem.** With ground truth trustworthy now, the gap was
obvious: real end (0,0), ICP's estimate (-28.18, 5.57) — off by ~29m over
one loop, drifting in z too, which shouldn't happen at all for a vehicle
that only moves in the ground plane. The z-drift is the tell: this room's
point clouds are mostly ground (entry 6 — most points are the floor).
Plain point-to-point ICP matching two scans that are both mostly the same
flat plane has almost no gradient constraining translation *within* that
plane — sliding a cloud sideways across a matching flat floor barely
changes point-to-point distances, so the optimizer has basically no signal
for how far it actually moved and can wander off into directions (z) that
make no physical sense. Known, documented failure mode of point-to-point
ICP on plane-heavy scenes, not a bug in the accumulation math.

**Fix, part 1:** reuse entry 6's already-verified ground removal
(`obstacle_detector_node`'s RANSAC largest-plane fit) inside
`scan_matcher_node` before handing scans to ICP, so matching only ever
sees wall/obstacle points. Applied it, re-ran the full loop — got *worse*
(52m error, up from 29m). Ground removal wasn't wrong exactly, just not
the dominant bug, and it made the point sets sparser, which made the real
problem (next entry) worse.

---

## 11. Three more bugs before ICP odometry actually worked, found by testing single known motions instead of guessing on the full loop

Debugging the full 122-second loop directly wasn't going anywhere — every
fix either did nothing or made it worse, and there was no way to tell why
from a single "off by 29m" number. Switched approach: command one single,
exactly-known motion (a 2m teleport in +x, nothing else) and check ICP's
single-step estimate against it directly. That's what actually cracked it
— three separate real bugs, each found by making the test smaller, not by
thinking harder about the big one.

**Bug 1 — `max_correspondence_distance` too tight, no initial guess.** The
2m-teleport test showed ICP reporting `converged=true, fitness≈2.35` (a
bad fit) then contributing basically zero motion to the estimate —
confirmed with temporary debug logging of point counts and fitness per
frame. Cause: `setMaxCorrespondenceDistance` was 0.5m, meaning ICP can
only match a point to another within 0.5m of it. When the true
displacement between scans exceeds that, ICP has no way to find the
correct match at all — it "converges" on whatever the few
within-threshold pairs suggest, silently wrong instead of erroring. Fixed
by raising the threshold (0.5 → 2.0m) and, more important, seeding
`icp.align()` with the previous step's estimated transform as an initial
guess (constant-velocity assumption) instead of identity — gets
correspondence search starting from roughly the right place instead of
"assume nothing moved."

**Bug 2 — inverted transform, a sign flip.** With bug 1 fixed, the
teleport test gave a magnitude-correct estimate (~1.96m, ~2% error) but
the sign was backwards (-1.96 instead of +1.96). Original code inverted
`icp.getFinalTransformation()` on the reasoning "it maps the new scan onto
the old scan's frame, which is the opposite of how the vehicle moved."
That reasoning was wrong. Worked through the math properly: a world-fixed
point observed from the new pose sits at `P_new = P_world - motion`, and
ICP's T solves `T * P_new ≈ P_old = P_world`, so `T = +motion` directly, no
inversion needed. Dropped the `.inverse()` call; single-step test then
matched ground truth almost exactly.

**Bug 3 — corners snap the heading instantly, same failure as bug 1 but
rotational.** Single-step translation worked now, but the full loop still
drifted badly (+64m after fixing bugs 1+2 — worse than before, actually).
`drive_loop.py`'s corners were setting the new segment's yaw on the very
first pose of that segment, meaning the heading jumped up to ~90° between
two consecutive LiDAR frames at every one of the 4 corners. Same problem
as bug 1, just rotational — the true angular jump exceeds what ICP can
find correspondences for in one step. Fixed by rotating in place in small
(10°) steps at each corner before translating along the next segment, so
no frame-to-frame gap ever needs a big heading change. Result: full-loop
error dropped to ~5.5m (from 64m) — real fix, but the leftover 5.5m
pointed at one more thing.

**Bug 4 (not really a bug) — z/roll/pitch have no physical meaning for
this vehicle.** With bugs 1-3 fixed, x/y tracked ground truth to within
~5m over the loop, but z had drifted to +20m — impossible for a vehicle
that only moves in the ground plane. Point-to-point ICP doesn't know
that; it hands back a full 6-DOF transform even when three of those DOF
have no physical basis here, and small per-frame noise in those directions
compounds like anything else. Fixed not by debugging why ICP finds
spurious z/roll/pitch motion, but by making it not matter:
`constrainToPlanarMotion()` projects every per-frame transform onto x, y,
and yaw only, before it's used for anything (composing the global pose,
seeding the next guess). Standard simplification for ground vehicles —
SE(2) instead of full SE(3) — not a patch over an unexplained bug. The
vehicle genuinely only has 3 degrees of freedom.

**Final result**, full loop, ground truth (0,0) → (0,0):

```
final ICP estimate: x=0.32 y=-0.44 z=0.00
```

~0.54m position error over a ~42m loop (~1.3%), z exactly zero the whole
way. For frame-to-frame LiDAR odometry with no map and no loop closure,
0.5-2% of distance traveled is a normal, credible number — not SLAM yet
(nothing corrects against a map), but a working localization estimate
with an honest, measured error bound, which is what this phase was
actually for.

---

## 12. IMU + EKF fusion — and a fifth bug that only showed up once a second sensor got added

**Context.** Added an IMU to the vehicle (`gz-sim-imu-system` plugin + an
`imu`-type sensor in the SDF) and a new `ekf_fusion_node`: a 3-state
[x, y, yaw] EKF, predicting with gyro-integrated yaw (fast, smooth, no
absolute reference) and correcting with entry 11's ICP odometry (slower,
occasionally a bad single-frame estimate, but doesn't drift on its own) —
same predict-with-one-sensor / correct-with-another pattern as
friction-aware-planner's own EKF, just applied to pose instead of
friction/sideslip.

**A fifth bug, only visible with two sensors running.** First full-loop
run: ICP alone was accurate (yaw implicitly right too, since position
tracked well), but the *fused* estimate's yaw was off by 33° (-1.36 rad
vs the true -0.785) and just stuck there for the rest of the run instead
of correcting. Added temporary debug logging of the ICP-measured yaw
feeding into the EKF each step and found it: `drive_loop.py` initialized
its internal `current_yaw` to segment 1's own heading (0.785 rad) instead
of the vehicle's real starting heading in the SDF (0 rad) — meaning the
very first pose command jumped the vehicle 45° in a single frame, the
exact same "angular jump exceeds what ICP can match" failure as bug 3 in
entry 11, just on frame one instead of a later corner, where that entry's
per-corner rotation fix never got applied. Fixed by initializing
`current_yaw = 0.0` so the first transition gets the same gradual
in-place rotation as every other corner.

Worth flagging as a pattern, not just a one-off: bug 3 was "fixed" in the
sense that the full-loop test passed afterward, but the fix only covered
the symptom the test was actually checking for. Same root cause (instant
heading snap), second instance the test never happened to exercise, until
a different consumer — the EKF, sensitive to a persistent yaw bias in a
way ICP-alone's position metric wasn't — made it visible.

**Result after the fix**, full loop, ground truth (0,0), yaw -0.785:

| | x | y | position error | yaw |
|---|---|---|---|---|
| ICP alone | 0.29 | -0.44 | 0.53m (~1.26%) | not logged directly |
| EKF fused | 0.20 | -0.39 | 0.44m (~1.04%) | -0.57 (0.21 rad / ~12° off) |

Fused position now measurably beats ICP alone — that's the actual point
of fusion, not a symmetry exercise: IMU-predicted yaw between ICP updates
smooths out exactly the kind of occasional bad single-frame ICP estimate
entry 11 spent four bugs getting right. Yaw still carries a ~12° residual
offset, down from 33° but not zero. Diminishing returns past this point
(tuning process/measurement noise further, or chasing whatever's left of
the initial-heading transient) — decided it wasn't worth chasing since the
position result already shows the fusion is doing real work, reported
honestly instead of tuned until the number looked clean.
