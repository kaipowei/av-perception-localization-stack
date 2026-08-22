#!/usr/bin/env python3
"""One-off script: subscribe to /camera, save the first frame as a PNG, exit.
Not part of the ROS2 package -- just a manual sanity-check tool."""
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrameSaver(Node):
    def __init__(self, out_path):
        super().__init__("frame_saver")
        self.out_path = out_path
        self.bridge = CvBridge()
        self.saved = False
        self.sub = self.create_subscription(Image, "/camera", self.on_image, 10)

    def on_image(self, msg):
        if self.saved:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(self.out_path, cv_image)
        self.get_logger().info(f"saved {self.out_path}")
        self.saved = True


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/frame.png"
    rclpy.init()
    node = FrameSaver(out_path)
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
