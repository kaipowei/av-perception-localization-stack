"""The piece that was still missing: something that actually moves the
vehicle using planner_bridge_node's steer output, instead of that output
just sitting on a topic nobody listens to.

There's no physics-based drivetrain on the Gazebo vehicle -- no wheel
joints, no suspension, no tire model -- and building one is a separate,
fairly large chunk of Gazebo/SDF work that has little to do with what this
repo is actually about (perception, localization, planning). Cutting that
corner honestly instead of pretending it isn't cut: this node integrates
the exact kinematic bicycle model friction-aware-planner's own planner and
MPC already reason in (same wheelbase, same steering convention), and syncs
the result into Gazebo with the same set_pose service sim/drive_loop.py
already uses for teleporting. The vehicle's motion is real, closed-loop,
and driven entirely by its own perception -> localization -> planning ->
control chain -- the actuation layer underneath it is a kinematic
integrator standing in for a real drivetrain, not a physics simulation of
one.
"""
from __future__ import annotations

import math
import subprocess

import rclpy
from faplanner.models.kinematic import KinematicBicycleModel, KinematicParams, KinematicState
from rclpy.node import Node
from std_msgs.msg import Float64


class VehicleDriverNode(Node):
    def __init__(self):
        super().__init__("vehicle_driver_node")
        self.declare_parameter("world", "test_track")
        self.declare_parameter("vehicle_name", "vehicle")
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("nominal_speed", 3.5)
        self.declare_parameter("max_accel", 2.0)
        self.declare_parameter("max_decel", 3.0)
        self.declare_parameter("goal_x", 6.0)
        self.declare_parameter("goal_y", 6.0)
        self.declare_parameter("goal_tol", 1.0)
        self.declare_parameter("steer_timeout_sec", 0.5)
        # Pure Pursuit has no built-in cap, and a steer request well past
        # what the vehicle can mechanically do isn't a request to honor --
        # it's a sign the tracker has already diverged (found once, the
        # hard way: a commanded angle climbing 40deg -> 62deg step by step
        # while the vehicle kept falling further behind a path it
        # physically couldn't turn tight enough for, which fed back into
        # replan() finding no path at all once the vehicle drifted outside
        # the planning bounds -- see learning-log entry 16). Treating an
        # oversized request the same as a stale one and braking instead of
        # committing to it is the fix.
        self.declare_parameter("max_reasonable_steer_deg", 45.0)

        self.world = self.get_parameter("world").value
        self.vehicle_name = self.get_parameter("vehicle_name").value
        self.nominal_speed = self.get_parameter("nominal_speed").value
        self.max_accel = self.get_parameter("max_accel").value
        self.max_decel = self.get_parameter("max_decel").value
        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        self.goal_tol = self.get_parameter("goal_tol").value
        self.steer_timeout_sec = self.get_parameter("steer_timeout_sec").value
        self.max_reasonable_steer = math.radians(self.get_parameter("max_reasonable_steer_deg").value)

        # matches the vehicle's actual starting pose in test_track.sdf --
        # the EKF's own state_ also starts at Zero(), so this and the
        # localization stack agree on where "zero" is without needing to
        # sync against fused_odometry first
        self.state = KinematicState(x=0.0, y=0.0, heading=0.0, v=0.0)
        self.model = KinematicBicycleModel()
        self.latest_steer = 0.0
        self.last_steer_stamp = None
        self.reached_goal = False

        self.create_subscription(Float64, "planned_steer_cmd", self.on_steer, 10)

        rate = self.get_parameter("control_rate_hz").value
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.control_step)

        self.get_logger().info(
            f"driving toward ({self.goal_x}, {self.goal_y}), tol={self.goal_tol}m"
        )

    def on_steer(self, msg: Float64):
        self.latest_steer = msg.data
        self.last_steer_stamp = self.get_clock().now()

    def control_step(self):
        if self.reached_goal:
            return

        # planner_bridge_node stops publishing once the vehicle runs past
        # the end of its current planned path (see learning-log entry 14) --
        # without this, the last steer command received just keeps getting
        # applied forever, which is how the vehicle first ended up driving
        # straight past the goal at full lock instead of stopping or
        # correcting. A stale command is worse than no command.
        stamp_age = (
            (self.get_clock().now() - self.last_steer_stamp).nanoseconds / 1e9
            if self.last_steer_stamp is not None else math.inf
        )
        stale = stamp_age > self.steer_timeout_sec
        # a request past what the vehicle can mechanically do means the
        # tracker has already diverged -- see the parameter comment above
        # and learning-log entry 16. Two earlier attempts both failed in
        # opposite directions: braking to a full stop deadlocked (at v=0,
        # nearest_idx never advances, so Pure Pursuit keeps looking at the
        # same unturned stretch of path and keeps demanding the same
        # oversized angle forever); clipping the oversized angle down to
        # the mechanical limit and creeping forward made it worse --
        # holding close to max_steer for several consecutive steps just
        # drives the vehicle in a tight circle (radius = wheelbase /
        # tan(max_steer)) that never intersects the path it's supposedly
        # tracking, so the divergence never resolves and the vehicle wanders
        # far from where it started. Zeroing the steer (straight line, not a
        # circle) while still creeping keeps the vehicle actually covering
        # new ground -- new ground is what gives the next replan a
        # different, hopefully saner, situation to work with.
        diverged = abs(self.latest_steer) > self.max_reasonable_steer
        distrust_steer = stale or diverged
        if distrust_steer:
            self.latest_steer = 0.0

        dist_to_goal = math.hypot(self.goal_x - self.state.x, self.goal_y - self.state.y)
        if dist_to_goal < self.goal_tol:
            self.reached_goal = True
            self.set_pose(self.state.x, self.state.y, self.state.heading)
            self.get_logger().info(
                f"reached goal: ({self.state.x:.2f}, {self.state.y:.2f}), "
                f"{dist_to_goal:.2f}m from target -- stopping"
            )
            return

        # simple speed ramp toward nominal_speed, braking early as the goal
        # gets close so the vehicle doesn't blow past it and turn around --
        # and slowing to a crawl, not a dead stop, when the steer command
        # can't be trusted. A full stop was the first fix here and it
        # created the worse deadlock described above; a slow creep keeps
        # the vehicle actually making progress while distrusting a bad
        # command, instead of freezing in place waiting for a signal that
        # can't arrive without the vehicle moving first.
        if distrust_steer:
            target_speed = 1.0
        elif dist_to_goal > 3.0:
            target_speed = self.nominal_speed
        else:
            target_speed = self.nominal_speed * 0.4
        accel = max(-self.max_decel, min(self.max_accel, target_speed - self.state.v))

        self.state = self.model.step(self.state, accel=accel, steer_angle=self.latest_steer, dt=self.dt)
        self.set_pose(self.state.x, self.state.y, self.state.heading)

        self.get_logger().info(
            f"driving: x={self.state.x:.2f} y={self.state.y:.2f} "
            f"v={self.state.v:.2f} steer={math.degrees(self.latest_steer):.1f}deg "
            f"dist_to_goal={dist_to_goal:.2f}m",
            throttle_duration_sec=1.0,
        )

    def set_pose(self, x: float, y: float, yaw: float):
        req = (
            f"name: '{self.vehicle_name}', "
            f"position: {{x: {x:.4f}, y: {y:.4f}, z: 0.2}}, "
            f"orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.5f}, w: {math.cos(yaw / 2):.5f}}}"
        )
        subprocess.run(
            [
                "gz", "service", "-s", f"/world/{self.world}/set_pose",
                "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                "--timeout", "300", "--req", req,
            ],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )


def main():
    rclpy.init()
    node = VehicleDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
