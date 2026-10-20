"""
Sim only: holds the billet when the gripper closes on it, and hands it to the
chuck when the gripper lets go inside the machine.

Gazebo isn't good at holding things by finger friction, so the URDF has a
detachable joint between the gripper and the billet, and the chuck model has a
second one. This node drives both. Without the chuck joint the billet would
drop on the floor the moment the fingers opened: the spindle is horizontal, so
there is nothing underneath it.

It keeps track of where the billet is itself: in the tray pocket at the start,
then wherever the gripper let go of it.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from . import arm_model, cell

CLOSED = 0.024     # finger position below this counts as closed
OPEN = 0.027
REACH = 0.06       # fingertips must be this close to the grip point on the billet
SEATED = 0.05      # letting go this close to the chuck point counts as loaded
HANDOVER = 0.15    # seconds the chuck and the gripper both hold it for


class PartGrasp(Node):
    def __init__(self):
        super().__init__("part_grasp")
        self.attach = self.create_publisher(Empty, "/part/attach", 10)
        self.detach = self.create_publisher(Empty, "/part/detach", 10)
        self.chuck_attach = self.create_publisher(Empty, "/chuck/attach", 10)
        self.chuck_detach = self.create_publisher(Empty, "/chuck/detach", 10)
        self.holding = True            # both plugins attach the billet when the world loads
        self.seen = 0
        self.was_open = False          # only a close after an open grabs the billet
        self.part = cell.PICK
        self.handover = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)

    def let_go(self):
        """Second half of the handover: open the gripper's joint."""
        self.handover.cancel()
        self.handover = None
        self.detach.publish(Empty())

    def on_joints(self, msg):
        if "j5_gripper" not in msg.name:
            return
        self.seen += 1
        if self.seen <= 100:           # world just came up: let go of the billet with both
            if self.seen % 25 == 1:
                self.detach.publish(Empty())
                self.chuck_detach.publish(Empty())
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
            self.get_logger().info("billet attached")
        elif not self.holding and self.was_open and f < CLOSED:
            self.get_logger().info(
                f"fingers closed, billet is {math.dist(tip, self.part):.3f} m away",
                throttle_duration_sec=1.0)
        elif self.holding and f > OPEN:
            self.holding = False
            self.part = tip
            if math.dist(tip, cell.CHUCK) < SEATED:
                # Close the chuck's joint first and open the gripper's a moment
                # later, so the billet is never loose. Doing it the other way
                # round drops it: the spindle is horizontal, and it only takes
                # 0.2 s to fall far enough to land on the machine.
                self.chuck_attach.publish(Empty())
                self.handover = self.create_timer(HANDOVER, self.let_go)
                self.get_logger().info("billet seated in the chuck")
            else:
                self.detach.publish(Empty())
                self.get_logger().info("billet released")


def main():
    rclpy.init()
    node = PartGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
