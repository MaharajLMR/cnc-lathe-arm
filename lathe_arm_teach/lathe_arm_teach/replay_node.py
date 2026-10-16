"""
Replay node — loads a YAML waypoint file and executes the sequence via MoveIt 2.

Usage:
  ros2 run lathe_arm_teach replay_node --ros-args -p waypoints:=my_waypoints.yaml
  ros2 run lathe_arm_teach replay_node --ros-args -p waypoints:=my_waypoints.yaml -p loop:=true
"""

from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    "joint_1_base_shoulder",
    "joint_2_shoulder_upper",
    "joint_3_upper_elbow",
    "joint_4_elbow_wrist",
    "joint_5_wrist_roll",
]

MOVE_DURATION_S = 2.0   # seconds per waypoint


class ReplayNode(Node):
    def __init__(self):
        super().__init__("replay_node")
        self.declare_parameter("waypoints", "waypoints.yaml")
        self.declare_parameter("loop", False)

        wp_file = self.get_parameter("waypoints").get_parameter_value().string_value
        self._loop = self.get_parameter("loop").get_parameter_value().bool_value

        self._waypoints = self._load(wp_file)
        self.get_logger().info(
            f"Loaded {len(self._waypoints)} waypoints from {wp_file}"
        )

        self._client = ActionClient(
            self, FollowJointTrajectory,
            "/lathe_arm_controller/follow_joint_trajectory"
        )
        # Current waypoint name, for HMIs and demo-video captions.
        self._wp_pub = self.create_publisher(String, "/replay/current_waypoint", 10)

    def _load(self, path: str) -> list[dict]:
        p = Path(path)
        if not p.exists():
            self.get_logger().error(f"Waypoint file not found: {path}")
            raise FileNotFoundError(path)
        with open(p) as f:
            data = yaml.safe_load(f)
        wps = data.get("waypoints", [])
        if not wps:
            raise ValueError(f"No waypoints in {path}")
        return wps

    def execute(self):
        """Execute all waypoints in sequence. Repeats if loop=true."""
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("JTC action server not available")
            return

        while True:
            for wp in self._waypoints:
                self.get_logger().info(f"Moving to '{wp['name']}'")
                self._wp_pub.publish(String(data=wp["name"]))
                self._send_goal(wp["joints"])
            if not self._loop:
                break
            self.get_logger().info("Loop: repeating sequence")

    def _send_goal(self, joints: list[float]):
        pt = JointTrajectoryPoint()
        pt.positions = list(joints)
        pt.velocities = [0.0] * len(joints)
        dur = Duration()
        dur.sec = int(MOVE_DURATION_S)
        dur.nanosec = int((MOVE_DURATION_S % 1) * 1e9)
        pt.time_from_start = dur

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = [pt]
        goal.goal_time_tolerance.sec = 1

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


def main():
    rclpy.init()
    node = ReplayNode()
    node.execute()
    node.destroy_node()
    rclpy.shutdown()
