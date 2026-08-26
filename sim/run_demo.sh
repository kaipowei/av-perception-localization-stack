#!/usr/bin/env bash
# One-command launch: everything the README's Quick Start otherwise walks
# through as five separate terminals (simulator, sensor bridge, perception,
# localization, planner+driver), started here from a single invocation.
# Ctrl+C tears the whole thing down cleanly.
#
# Assumes `colcon build` already ran once (see README). If fa_bridge_py's
# venv lives somewhere other than ~/ros2_build/venvs/fa_bridge, point
# FA_BRIDGE_VENV at its python3 before running this.
set -eo pipefail
# ROS2's own setup.bash scripts reference a few variables before they're
# ever set (e.g. AMENT_TRACE_SETUP_FILES) -- `set -u` and sourcing them
# don't mix, so this deliberately doesn't turn nounset on.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_PYTHON="${FA_BRIDGE_VENV:-$HOME/ros2_build/venvs/fa_bridge/bin/python3}"
# Default assumes `colcon build` ran inside ros2_ws, per the README. If your
# build lives elsewhere -- e.g. WSL2's DrvFs mount can't handle colcon's
# build artifacts, so this project's own dev setup builds to a WSL-native
# directory instead -- point INSTALL_SETUP at that setup.bash.
INSTALL_SETUP="${INSTALL_SETUP:-$REPO_ROOT/ros2_ws/install/setup.bash}"

if [ ! -x "$VENV_PYTHON" ]; then
  echo "error: fa_bridge_py's venv python not found at $VENV_PYTHON" >&2
  echo "set FA_BRIDGE_VENV=/path/to/venv/bin/python3 if yours lives elsewhere" >&2
  echo "(see README Requirements -- this venv needs friction-aware-planner installed in it)" >&2
  exit 1
fi

if [ ! -f "$INSTALL_SETUP" ]; then
  echo "error: no built workspace at $INSTALL_SETUP" >&2
  echo "run colcon build first (see README Quick Start), or set INSTALL_SETUP=/path/to/install/setup.bash" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$INSTALL_SETUP"

cleanup() {
  echo ""
  echo "shutting down..."
  kill -- -$$ 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[1/3] starting Gazebo..."
gz sim -s -r --headless-rendering "$REPO_ROOT/sim/worlds/test_track.sdf" &
sleep 4

echo "[2/3] starting the sensor bridge, perception, and localization nodes..."
ros2 run ros_gz_bridge parameter_bridge \
  /lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /camera@sensor_msgs/msg/Image[gz.msgs.Image \
  /imu@sensor_msgs/msg/Imu[gz.msgs.IMU &
ros2 run fa_perception_cpp point_cloud_processor_node --ros-args -r points_raw:=/lidar/points &
ros2 run fa_perception_cpp obstacle_detector_node &
ros2 run fa_localization_cpp scan_matcher_node &
ros2 run fa_localization_cpp ekf_fusion_node &
sleep 4

echo "[3/3] starting the planner and driver -- the vehicle starts moving now"
"$VENV_PYTHON" -m fa_bridge_py.planner_bridge_node &
ros2 run fa_bridge_py vehicle_driver_node &

echo ""
echo "running -- the vehicle is driving itself toward (6, 6)."
echo "watch progress with: ros2 topic echo /fused_odometry"
echo "press Ctrl+C to stop everything."
wait
