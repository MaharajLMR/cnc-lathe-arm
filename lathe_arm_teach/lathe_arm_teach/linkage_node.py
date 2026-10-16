"""
Keeps the drawn linear actuator attached to the boom.

URDF can't close the loop column - actuator - boom, so the actuator body and
rod sit on two extra joints (act_pitch, act_extend). This node watches the
boom angle and commands those joints from the linkage geometry.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from . import arm_model

ROD_LENGTH = 0.45   # act_rod length in the URDF


class LinkageNode(Node):
    def __init__(self):
        super().__init__("linkage_node")
        self.pub = self.create_publisher(Float64MultiArray, "/linkage_controller/commands", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)

    def on_joints(self, msg: JointState):
        if "j2_boom_lift" not in msg.name:
            return
        boom = msg.position[msg.name.index("j2_boom_lift")]
        pitch, length = arm_model.actuator_pose(boom)
        self.pub.publish(Float64MultiArray(data=[pitch, length - ROD_LENGTH]))


def main():
    rclpy.init()
    node = LinkageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
