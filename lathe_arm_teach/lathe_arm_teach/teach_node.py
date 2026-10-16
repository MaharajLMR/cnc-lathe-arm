"""
Teach mode node, jog the arm joint-by-joint and save waypoints to YAML.

Controls:
  1-5        : select joint (1=base ... 5=wrist_roll)
  +/-        : jog selected joint by JOG_STEP radians
  s          : save current pose as a named waypoint (prompts for name)
  p          : preview current joint values
  q          : quit and save all waypoints to --output file

Publishes joint positions to /joint_states so robot_state_publisher
updates RViz in real time.

Usage:
  ros2 run lathe_arm_teach teach_node --ros-args -p output:=my_waypoints.yaml
"""

import sys
import tty
import termios
import yaml
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


JOINT_NAMES = [
    "joint_1_base_shoulder",
    "joint_2_shoulder_upper",
    "joint_3_upper_elbow",
    "joint_4_elbow_wrist",
    "joint_5_wrist_roll",
]

JOG_STEP = 0.05   # radians per key press
PUBLISH_RATE = 20  # Hz


def _getch() -> str:
    """Read a single character without echo."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TeachNode(Node):
    def __init__(self):
        super().__init__("teach_node")
        self.declare_parameter("output", "waypoints.yaml")

        self._positions = [0.0] * 5
        self._active_joint = 0   # 0-indexed
        self._waypoints: list[dict] = []
        self._output = self.get_parameter("output").get_parameter_value().string_value

        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE, self._publish)

        self.get_logger().info(
            f"Teach mode ready. Output: {self._output}\n"
            "Keys: 1-5=select joint  +/-=jog  s=save  p=preview  q=quit"
        )

    def _publish(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self._positions)
        self._pub.publish(msg)

    def run_interactive(self):
        """Main keyboard loop, runs in the current thread."""
        print(f"\n  Active joint: {JOINT_NAMES[self._active_joint]}")
        while rclpy.ok():
            ch = _getch()

            if ch == 'q':
                self._save_to_file()
                self.get_logger().info(f"Saved {len(self._waypoints)} waypoints to {self._output}")
                break

            elif ch in '12345':
                self._active_joint = int(ch) - 1
                print(f"\n  Active joint: {JOINT_NAMES[self._active_joint]}")

            elif ch in ('+', '='):
                self._positions[self._active_joint] += JOG_STEP
                self._print_joints()

            elif ch == '-':
                self._positions[self._active_joint] -= JOG_STEP
                self._print_joints()

            elif ch == 's':
                # Exit raw mode temporarily to get input
                fd = sys.stdin.fileno()
                old = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                name = input("\n  Waypoint name: ").strip()
                if name:
                    self._waypoints.append({
                        "name": name,
                        "joints": list(self._positions),
                    })
                    print(f"  Saved '{name}': {[round(x, 4) for x in self._positions]}")

            elif ch == 'p':
                self._print_joints()

            rclpy.spin_once(self, timeout_sec=0.0)

    def _print_joints(self):
        vals = "  ".join(
            f"J{i+1}={v:+.3f}" for i, v in enumerate(self._positions)
        )
        active_name = JOINT_NAMES[self._active_joint].split("_")[-1]
        print(f"\r  [{active_name}]  {vals}    ", end="", flush=True)

    def _save_to_file(self):
        data = {"waypoints": self._waypoints}
        path = Path(self._output)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            yaml.dump(data, f, default_flow_style=False)


def main():
    rclpy.init()
    node = TeachNode()
    node.run_interactive()
    node.destroy_node()
    rclpy.shutdown()
