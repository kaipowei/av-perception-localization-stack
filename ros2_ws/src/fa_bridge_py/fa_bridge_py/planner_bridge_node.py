"""This is the node the whole second repo was building toward: take the fused
pose from ekf_fusion_node and the obstacle boxes from obstacle_detector_node,
and actually hand them to friction-aware-planner's Hybrid-A* and MPC instead
of just leaving a "these two projects should talk to each other someday" note
in the README.

Two controllers run every step, for two different jobs. MPC assumes the
dynamic bicycle model -- real tire slip, a friction-circle bound on how hard
it'll steer -- and its output (published on mpc_advisory_steer_cmd) is kept
purely as a demonstration that the friction-aware part of this stack works:
at this speed and this road mu, it caps itself to a real, computed steering
limit instead of demanding whatever the path needs. That's the correct,
intended behavior, not a bug -- but it also means MPC's output is far too
conservative to actually drive with here. vehicle_driver_node (Phase 4)
actuates through a pure kinematic model with no slip, so applying a
slip-aware friction bound to it wastes turning capability the vehicle
actually has. Pure Pursuit has no such bound and matches the kinematic model
exactly, so it's what actually drives (planned_steer_cmd). See learning-log
entry 14 for how this mismatch showed up and why this split is the fix.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from faplanner.controllers.mpc import MPCController
from faplanner.controllers.pure_pursuit import PurePursuit
from faplanner.models.dynamic import DynamicParams, DynamicState
from faplanner.models.kinematic import KinematicState
from faplanner.planners.hybrid_astar import hybrid_astar, reconstruct_path
from faplanner.sim.closed_loop import nearest_index
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Empty, Float64
from visualization_msgs.msg import MarkerArray


class PlannerBridgeNode(Node):
    def __init__(self):
        super().__init__("planner_bridge_node")
        self.declare_parameter("goal_x", 6.0)
        self.declare_parameter("goal_y", 6.0)
        self.declare_parameter("goal_heading", math.pi / 4.0)
        # is_free() rejects anything past this, and hybrid_astar never
        # special-cases the start pose -- if the vehicle ever ends up past
        # this boundary, nearly every motion primitive's own path points
        # near the start are still out of bounds too, so replanning can't
        # recover. 8.0 was tight enough that a real avoidance maneuver in
        # the two-obstacle scenario (needing to swing out past (8.8, 4.5))
        # pushed the vehicle past it and deadlocked there -- see
        # learning-log entry 16. Widened with real room to spare, short of
        # the actual walls at +/-15.
        self.declare_parameter("world_half_extent", 12.0)
        self.declare_parameter("obstacle_margin", 0.6)
        self.declare_parameter("nominal_speed", 3.5)
        # replanning too often (tried 0.5s) never lets nearest_idx advance
        # past the first couple points of a path before it gets thrown away
        # and reset to 0 -- those first points are barely turned yet, so the
        # vehicle only ever sees "keep going straight" and never actually
        # tracks the curve. Replanning too rarely (tried 3.0s) lets the
        # vehicle run outside the planning bounds before a fresh path can
        # catch up. 2.0s is enough time for nearest_idx to reach the part of
        # each path that's actually turned, while still refreshing before
        # the vehicle gets far off course. See learning-log entry 14.
        self.declare_parameter("replan_period_sec", 2.0)
        self.declare_parameter("min_obstacle_extent", 0.3)

        self.goal = (
            self.get_parameter("goal_x").value,
            self.get_parameter("goal_y").value,
            self.get_parameter("goal_heading").value,
        )
        self.world_half_extent = self.get_parameter("world_half_extent").value
        self.obstacle_margin = self.get_parameter("obstacle_margin").value
        self.nominal_speed = self.get_parameter("nominal_speed").value
        self.min_obstacle_extent = self.get_parameter("min_obstacle_extent").value

        self.pose = None  # (x, y, yaw), from fused_odometry
        self.obstacles = []  # (cx, cy, half_x, half_y), from obstacle_markers
        self.path_x = None
        self.path_y = None
        self.last_idx = 0

        # built once at startup -- MPCController's __init__ builds the cvxpy
        # QP, not cheap enough to redo every control step
        self.mpc = MPCController(DynamicParams())
        self.pure_pursuit = PurePursuit(wheelbase=DynamicParams().wheelbase)

        self.path_pub = self.create_publisher(Path, "planned_path", 10)
        self.steer_pub = self.create_publisher(Float64, "planned_steer_cmd", 10)
        self.mpc_advisory_pub = self.create_publisher(Float64, "mpc_advisory_steer_cmd", 10)

        self.create_subscription(Odometry, "fused_odometry", self.on_odometry, 10)
        self.create_subscription(MarkerArray, "obstacle_markers", self.on_obstacles, 10)
        # normally replanning just waits for the next periodic timer tick,
        # which is fine for steady driving but too slow for reacting to a
        # newly-appeared obstacle at this track's scale (a couple seconds'
        # wait at ~3.3 m/s eats most of the runway between "obstacle shows
        # up" and "vehicle gets there") -- this lets an external trigger
        # (sim/spawn_second_obstacle.py) force an immediate replan instead
        # of tuning distance thresholds to paper over a slow reaction
        self.create_subscription(Empty, "force_replan", lambda _msg: self.replan(), 10)

        replan_period = self.get_parameter("replan_period_sec").value
        self.create_timer(replan_period, self.replan)
        self.create_timer(0.1, self.control_step)

    def on_odometry(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_obstacles(self, msg: MarkerArray):
        # obstacle_detector_node's own extent filter only rejects clusters
        # bigger than max_obstacle_extent_ (walls) -- it doesn't catch the
        # other end: thin sliver clusters near-degenerate in one axis, which
        # turned up in testing scattered around the track at a fairly
        # consistent radius from the vehicle. Best guess is ground-plane-fit
        # edge noise (16-beam LiDAR ground returns land in rings at fixed
        # ranges set by the beam elevation angles, and RANSAC's single global
        # plane doesn't cleanly swallow all of them). Feeding those to
        # hybrid_astar as real obstacles is technically correct but produces
        # absurdly long detours around clutter that isn't actually there --
        # real obstacles here are chunky in both x and y, so a minimum on the
        # smaller axis filters the slivers out without touching the two real
        # boxes.
        # obstacle_detector_node publishes these in the LiDAR's own frame
        # (it just copies the input cloud's header -- see
        # point_cloud_processor_node/obstacle_detector_node, neither ever
        # sets frame_id to "odom"), i.e. positions relative to the vehicle,
        # not world coordinates. Treating marker.pose.position directly as
        # a world (x, y) only looked right because obstacle_1 sits close to
        # the origin and the vehicle hadn't turned much by the time it got
        # there -- with the vehicle's pose near identity, "vehicle frame"
        # and "world frame" were nearly the same thing by coincidence. It
        # fell apart for real once a second obstacle showed up further
        # along the route, after the vehicle had already turned ~30-40deg:
        # the reported position was off by meters, not centimeters. Fix is
        # the actual frame transform, using the current fused pose.
        if self.pose is None:
            return
        vx, vy, vyaw = self.pose
        cos_yaw, sin_yaw = math.cos(vyaw), math.sin(vyaw)

        obstacles = []
        for marker in msg.markers:
            if min(marker.scale.x, marker.scale.y) < self.min_obstacle_extent:
                continue
            dx, dy = marker.pose.position.x, marker.pose.position.y
            world_x = vx + dx * cos_yaw - dy * sin_yaw
            world_y = vy + dx * sin_yaw + dy * cos_yaw
            half_x = marker.scale.x / 2.0 + self.obstacle_margin
            half_y = marker.scale.y / 2.0 + self.obstacle_margin
            obstacles.append((world_x, world_y, half_x, half_y))
        self.obstacles = obstacles

    def is_free(self, x: float, y: float) -> bool:
        # world_half_extent stands in for the perimeter walls, which
        # obstacle_detector_node deliberately filters out (max_obstacle_extent)
        # -- so the wall geometry never reaches this node any other way
        if abs(x) > self.world_half_extent or abs(y) > self.world_half_extent:
            return False
        for cx, cy, half_x, half_y in self.obstacles:
            if abs(x - cx) < half_x and abs(y - cy) < half_y:
                return False
        return True

    def replan(self):
        if self.pose is None:
            return

        # goal_heading_tol left wide open (default is 25 deg): this node
        # replans from scratch every couple hundred ms while the vehicle is
        # moving, and forcing every one of those intermediate replans to
        # also arrive at a specific final heading turned out to make
        # hybrid_astar's discretized search find bizarre, long detours
        # instead of the direct route -- confirmed by isolating the call
        # with a start pose partway along the route (see learning-log entry
        # 14). Arrival heading only matters for the very last stop, which
        # nothing here currently cares about anyway.
        goal_node = hybrid_astar(
            start=self.pose, goal=self.goal, is_free_fn=self.is_free, goal_heading_tol=math.pi
        )
        if goal_node is None:
            self.get_logger().warn("hybrid-A* found no path from current pose to the goal")
            return

        xs, ys, _ = reconstruct_path(goal_node)
        self.path_x, self.path_y = xs, ys
        self.last_idx = 0  # new path, old index means nothing on it
        self.publish_path(xs, ys)
        self.get_logger().info(
            f"replanned: {len(xs)} waypoints, dodging {len(self.obstacles)} obstacle(s)"
        )

    def publish_path(self, xs: np.ndarray, ys: np.ndarray):
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in zip(xs, ys):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path.poses.append(pose)
        self.path_pub.publish(path)

    def control_step(self):
        if self.pose is None or self.path_x is None or len(self.path_x) < 2:
            return

        x, y, yaw = self.pose
        # no velocity estimate comes out of the EKF (it only tracks x/y/yaw),
        # so v is a fixed nominal speed rather than something measured --
        # good enough for both controllers' lookahead/friction-bound math,
        # not a real speed controller
        state_dyn = DynamicState(x=x, y=y, heading=yaw, vx=self.nominal_speed)
        state_kin = KinematicState(x=x, y=y, heading=yaw, v=self.nominal_speed)
        # start_idx carries forward from the last control_step, not 0 --
        # searching from 0 every time meant this always locked onto the
        # first couple points of whatever path replan() just produced,
        # which are barely turned yet (hybrid_astar's motion primitives need
        # some arc length to accumulate heading change). Both controllers'
        # path_heading/lookahead logic is computed from a window right
        # around nearest_idx, so with idx stuck near 0 they only ever saw
        # "basically straight ahead" and never actually corrected -- see
        # learning-log entry 14 for how this looked in practice (steer
        # staying ~0deg for seconds while the vehicle drove straight past
        # the goal).
        idx = nearest_index(self.path_x, self.path_y, x, y, start_idx=self.last_idx)
        self.last_idx = idx
        if idx >= len(self.path_x) - 2:
            # run off the end of the current plan (replan() will produce a
            # fresh one shortly) -- publish neutral steer instead of just
            # going silent, so a stale hard-lock command never sits on the
            # topic waiting to be picked up. See vehicle_driver_node's own
            # timeout guard for the other half of this fix.
            self.steer_pub.publish(Float64(data=0.0))
            return

        mpc_steer = self.mpc.steer(state_dyn, self.path_x, self.path_y, idx)
        self.mpc_advisory_pub.publish(Float64(data=mpc_steer))

        pursuit_steer = self.pure_pursuit.steer(state_kin, self.path_x, self.path_y, idx)
        self.steer_pub.publish(Float64(data=pursuit_steer))

        self.get_logger().info(
            f"steer: pursuit {math.degrees(pursuit_steer):.1f} deg (driving), "
            f"mpc {math.degrees(mpc_steer):.1f} deg (friction-limited advisory) "
            f"(waypoint {idx}/{len(self.path_x)})",
            throttle_duration_sec=2.0,
        )


def main():
    rclpy.init()
    node = PlannerBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
