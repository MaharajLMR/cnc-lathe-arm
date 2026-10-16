"""
Sim only: holds the part when the gripper closes on it.

Gazebo isn't good at holding things by finger friction, so the URDF has a
detachable joint between the gripper and the part. This node attaches it when
the fingers close near the part and lets go when they open.

It keeps track of where the part is itself: on the stand at the start, then
wherever the gripper let go of it.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from . import arm_model

CLOSED = 0.024     # finger position below this counts as closed
OPEN = 0.027
REACH = 0.06       # fingertips must be this close to the grip point on the part
START = (0.855, -0.599, 0.793)   # grip point of the part on its stand (lathe_cell.sdf)


class PartGrasp(Node):
    def __init__(self):
        super().__init__("part_grasp")
        self.attach = self.create_publisher(Empty, "/part/attach", 10)
        self.detach = self.create_publisher(Empty, "/part/detach", 10)
        self.holding = True            # the plugin attaches the part when the robot spawns
        self.seen = 0
        self.was_open = False          # only a close after an open grabs the part
        self.part = START
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)

    def on_joints(self, msg):
        if "j5_gripper" not in msg.name:
            return
        self.seen += 1
        if self.seen <= 100:           # robot just came up: let go of the part (a few times, to be sure)
            if self.seen % 25 == 1:
                self.detach.publish(Empty())
            self.holding = False
            return
        f = msg.position[msg.name.index("j5_gripper")]
        q = [msg.position[msg.name.index(n)] if n in msg.name else 0.0 for n in arm_model.JOINTS]
        tip = arm_model.tool_position(q)
        if f > OPEN:
            self.was_open = True
        if not self.holding and self.was_open and f < CLOSED and math.dist(tip, self.part) < REACH:
            self.attach.publish(Empty())
            self.holding = True
            self.was_open = False
            self.get_logger().info("part attached")
        elif not self.holding and self.was_open and f < CLOSED:
            self.get_logger().info(f"fingers closed, part is {math.dist(tip, self.part):.3f} m away", throttle_duration_sec=1.0)
        elif self.holding and f > OPEN:
            self.detach.publish(Empty())
            self.holding = False
            self.part = tip
            self.get_logger().info("part released")


def main():
    rclpy.init()
    node = PartGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
