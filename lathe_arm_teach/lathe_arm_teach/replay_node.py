"""
Plays a saved points file on the arm, same format the HMI writes.

Each point is [j1, j2, j3, j4, gripper]. The arm moves first, then the
gripper, so it doesn't open or close while swinging. Each arm move gets the
time its slowest joint needs at pot_teleop's speed limits (stretched by PEAK),
or a fixed time if move_time is set.

    ros2 run lathe_arm_teach replay_node --ros-args -p waypoints:=points.yaml
    ros2 run lathe_arm_teach replay_node --ros-args -p waypoints:=points.yaml -p loop:=true
"""

from pathlib import Path

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from .pot_teleop import move_time

ARM = ["j1_base_yaw", "j2_boom_lift", "j3_wrist_pitch", "j4_wrist_roll"]
GRIPPER = ["j5_gripper", "j5_gripper_mirror"]
PEAK = 1.9    # the trajectory spline peaks at about 1.9x its average speed, so this keeps the peak inside SPEED


def load(path):
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(path)
    wps = (yaml.safe_load(p.read_text()) or {}).get("waypoints", [])
    if not wps:
        raise ValueError(f"no waypoints in {path}")
    for wp in wps:
        if len(wp["joints"]) != 5:
            raise ValueError(f"point {wp['name']} needs 5 values, has {len(wp['joints'])}")
    return wps


class ReplayNode(Node):
    def __init__(self):
        super().__init__("replay_node")
        self.declare_parameter("waypoints", "waypoints.yaml")
        self.declare_parameter("loop", False)
        self.declare_parameter("move_time", 0.0)      # 0 = from the speed limits
        self.points = load(self.get_parameter("waypoints").value)
        self.loop = self.get_parameter("loop").value
        self.move_time = self.get_parameter("move_time").value
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.pos = {}
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.name_pub = self.create_publisher(String, "/replay/current_waypoint", 10)
        self.get_logger().info(f"loaded {len(self.points)} points")

    def on_joints(self, msg):
        self.pos.update(zip(msg.name, msg.position))

    def settle(self, names, target, tol=0.02, timeout=8.0):
        # the action returns when the trajectory time is up, not when the joints got there
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            if all(abs(self.pos.get(n, 1e9) - t) < tol for n, t in zip(names, target)):
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().warn("joints did not settle")

    def run(self):
        for c in (self.arm, self.grip):
            if not c.wait_for_server(timeout_sec=15.0):
                self.get_logger().error("controller action server not available")
                return
        last, last_grip = None, None
        while rclpy.ok():
            for wp in self.points:
                self.get_logger().info(f"-> {wp['name']}")
                self.name_pub.publish(String(data=wp["name"]))
                q = wp["joints"]
                if last is None:
                    self.move(self.arm, ARM, q[:4], 3.0)          # from wherever the arm is
                elif max(abs(a - b) for a, b in zip(last, q[:4])) > 1e-4:
                    t = self.move_time or PEAK * move_time(last, q[:4], 0.5)
                    self.move(self.arm, ARM, q[:4], t)
                last = q[:4]
                if last_grip is None or abs(q[4] - last_grip) > 1e-4:
                    self.move(self.grip, GRIPPER, [q[4], q[4]], 1.0)
                    last_grip = q[4]
            if not self.loop:
                break

    def move(self, client, names, positions, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=list(positions), velocities=[0.0] * len(positions),
            time_from_start=Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)))]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected")
            return
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        self.settle(names, positions, tol=0.02 if len(names) > 2 else 0.002)


def main():
    rclpy.init()
    node = ReplayNode()
    node.run()
    node.destroy_node()
    rclpy.try_shutdown()
