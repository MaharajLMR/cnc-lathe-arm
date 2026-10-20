#!/usr/bin/env python3
"""Record the Gazebo cell cameras to numbered PNG frames.

Subscribes to /cell/image and /cell/closeup (both bridged by sim.launch.py)
and to /replay/current_waypoint, and captions each frame with sim time and the
point the arm is moving to. Frames go to <out>/<camera>/. Encode them with
tools/make_demo_video.sh.

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

CAMERAS = {
    "wide": "/cell/image",          # the whole cell: arm, tray, machine
    "machine": "/cell/machine",     # the lathe on its own
    "pick": "/cell/pick",           # the tray, close
    "closeup": "/cell/closeup",     # the chuck, from inside the guards
}


class Recorder(Node):
    def __init__(self, out: str, every: int):
        super().__init__("cell_recorder", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.out, self.every = out, every
        self.n_seen = {c: 0 for c in CAMERAS}
        self.n_saved = {c: 0 for c in CAMERAS}
        self.waypoint = "-"
        for cam in CAMERAS:
            os.makedirs(os.path.join(out, cam), exist_ok=True)
        for cam, topic in CAMERAS.items():
            self.create_subscription(
                Image, topic, lambda m, c=cam: self.on_image(m, c), qos_profile_sensor_data)
        self.create_subscription(String, "/replay/current_waypoint", self.on_wp, 10)

    def on_wp(self, msg: String) -> None:
        self.waypoint = msg.data

    def on_image(self, msg: Image, cam: str) -> None:
        self.n_seen[cam] += 1
        # the machine shot is a still of the lathe, it does not need every frame
        every = self.every * 3 if cam == "machine" else self.every
        if self.n_seen[cam] % every:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lines = [f"t = {t:5.1f} s (sim)", f"point: {self.waypoint}"]
        cv2.rectangle(img, (0, 0), (470, 70), (25, 25, 25), -1)
        for i, line in enumerate(lines):
            cv2.putText(img, line, (10, 28 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (240, 240, 240), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(self.out, cam, f"frame_{self.n_saved[cam]:05d}.png"), img)
        self.n_saved[cam] += 1


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
        node.get_logger().info(f"saved {dict(node.n_saved)} to {args.out}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
