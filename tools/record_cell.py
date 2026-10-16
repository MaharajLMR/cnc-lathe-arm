#!/usr/bin/env python3
"""Record the Gazebo cell camera to numbered PNG frames.

Subscribes to /cell/image (bridged by sim.launch.py) and
/replay/current_waypoint, and captions each frame with sim time and the
waypoint the arm is moving to. Encode with tools/make_demo_video.sh.

Usage:
    python3 tools/record_cell.py --out /tmp/frames --every 3
"""

import argparse
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self, out: str, every: int):
        super().__init__("cell_recorder", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        os.makedirs(out, exist_ok=True)
        self.out, self.every = out, every
        self.n_seen = self.n_saved = 0
        self.waypoint = "waiting for replay"
        self.create_subscription(Image, "/cell/image", self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, "/replay/current_waypoint", self.on_wp, 10)

    def on_wp(self, msg: String) -> None:
        self.waypoint = msg.data

    def on_image(self, msg: Image) -> None:
        self.n_seen += 1
        if self.n_seen % self.every:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lines = [f"t = {t:5.1f} s (sim)", f"teach-mode replay: {self.waypoint}"]
        cv2.rectangle(img, (0, 0), (470, 70), (25, 25, 25), -1)
        for i, line in enumerate(lines):
            cv2.putText(img, line, (10, 28 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (240, 240, 240), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(self.out, f"frame_{self.n_saved:05d}.png"), img)
        self.n_saved += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--every", type=int, default=1)
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(args.out, args.every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"saved {node.n_saved} frames to {args.out}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
