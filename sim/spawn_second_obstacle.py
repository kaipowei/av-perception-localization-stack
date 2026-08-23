#!/usr/bin/env python3
"""One-off script: spawns a second obstacle directly ahead of the vehicle
partway through an autonomous drive, then forces an immediate replan
against it -- so the demo shows the vehicle noticing and reacting to a new
obstacle mid-drive instead of avoiding a scene it already knew about from
frame one. Not part of the ROS2 package, same category as drive_loop.py.

Triggers on a fixed delay, not on watching the vehicle's position: this
scenario is deterministic (same start, same obstacles, same speed), and a
position-based trigger turned out to be at the mercy of this node's own
ROS2 discovery/message latency on startup, which fired it 2+ seconds later
than the vehicle's actual position justified -- close enough to the new
obstacle by the time it fired that there was barely any reaction room left.
A fixed delay, calibrated once against a real run's timestamps, sidesteps
that entirely. See learning-log entry 15."""
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

# single-quoted XML attributes on purpose -- keeps this string free of
# double quotes so it can be dropped straight into the protobuf text-format
# request below without escaping
OBSTACLE_SDF = (
    "<sdf version='1.9'><model name='obstacle_dynamic'><static>true</static>"
    "<pose>{x} {y} 0.5 0 0 0</pose><link name='link'>"
    "<collision name='collision'><geometry><box><size>1 1 1</size></box></geometry></collision>"
    "<visual name='visual'><geometry><box><size>1 1 1</size></box></geometry>"
    "<material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse></material>"
    "</visual></link></model></sdf>"
)


class SpawnTrigger(Node):
    def __init__(self, world: str, obs_x: float, obs_y: float, delay_sec: float):
        super().__init__("spawn_trigger")
        self.world = world
        self.obs_x = obs_x
        self.obs_y = obs_y
        self.done = False
        self.force_replan_pub = self.create_publisher(Empty, "force_replan", 10)
        self.create_timer(delay_sec, self.spawn)

    def spawn(self):
        sdf = OBSTACLE_SDF.format(x=self.obs_x, y=self.obs_y)
        req = f'sdf: "{sdf}", name: "obstacle_dynamic"'
        result = subprocess.run(
            [
                "gz", "service", "-s", f"/world/{self.world}/create",
                "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
                "--timeout", "1000", "--req", req,
            ],
            capture_output=True, text=True,
        )
        self.get_logger().info(
            f"spawned second obstacle at ({self.obs_x}, {self.obs_y}): "
            f"{result.stdout.strip()} {result.stderr.strip()}"
        )
        # give the LiDAR (10Hz) and obstacle_detector_node one real cycle to
        # actually see the new box before forcing a replan against it --
        # otherwise the force-replan message can race ahead of the
        # perception update and replan against a stale, empty-ish obstacle
        # list, wasting the whole point of forcing it early
        self.create_timer(0.4, self.trigger_replan)

    def trigger_replan(self):
        self.force_replan_pub.publish(Empty())
        self.get_logger().info("forced an immediate replan against the new obstacle")
        self.done = True


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else "test_track"
    # (5.5, 3.3) -- only ~2.8m from the (6, 6) goal -- left the vehicle too
    # little room to both dodge it and line up on the goal at the same
    # time, and was the real cause of the wide, sometimes-unstable loops
    # documented in learning-log entry 16. (4.3, 2.3) gives real separation
    # from the goal while still sitting squarely in the route out of
    # obstacle_1's dodge.
    obs_x = float(sys.argv[2]) if len(sys.argv) > 2 else 4.3
    obs_y = float(sys.argv[3]) if len(sys.argv) > 3 else 2.3
    delay_sec = float(sys.argv[4]) if len(sys.argv) > 4 else 6.0

    rclpy.init()
    node = SpawnTrigger(world, obs_x, obs_y, delay_sec)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
