"""
Potentiometer teleop: knobs (or the web HMI's sliders) drive the arm.

On the Raspberry Pi 5 the five pots are read through ADS1115 ADCs on I2C
(the Pi has no analog inputs): channels 0-3 on the board at 0x48, channel 0
on the board at 0x49. In simulation, or when the HMI is set to "web", the
HMI publishes slider positions on /hmi/pots instead.

Readings are smoothed and sent as short trajectories to the arm and gripper
controllers ten times a second, only when something moved. Each move is
given enough time for the slowest joint at its speed limit, so a knob
spun fast doesn't ask the base motor for an impossible jump.

    ros2 run lathe_arm_teach pot_teleop                         # web sliders
    ros2 run lathe_arm_teach pot_teleop --ros-args -p source:=ads1115
"""

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import pots

ARM = ["j1_base_yaw", "j2_boom_lift", "j3_wrist_pitch", "j4_wrist_roll"]
GRIPPER = ["j5_gripper", "j5_gripper_mirror"]
ADS_CHANNELS = [(0x48, 0), (0x48, 1), (0x48, 2), (0x48, 3), (0x49, 0)]
SPEED = [0.5, 0.35, 1.5, 1.5]     # rad/s we let the four arm joints move (a bit under the URDF limits)


def move_time(current, target, minimum):
    """Seconds needed to go from current to target with every joint inside SPEED."""
    return max([minimum] + [abs(t - c) / v for c, t, v in zip(current, target, SPEED)])


class Ads1115:
    """Minimal single-shot reader for ADS1115 channels (needs smbus2 on the Pi)."""

    def __init__(self, bus=1):
        import smbus2
        self.bus = smbus2.SMBus(bus)

    def read(self, addr, channel):
        import time
        mux = 0x4 + channel                      # AINx vs GND
        config = 0x8000 | (mux << 12) | (0x1 << 9) | 0x0100 | 0x00E0 | 0x0003   # start, +-4.096 V, single, 860 SPS
        self.bus.write_i2c_block_data(addr, 0x01, [config >> 8, config & 0xFF])
        time.sleep(0.0015)
        hi, lo = self.bus.read_i2c_block_data(addr, 0x00, 2)
        value = (hi << 8) | lo
        return value - 65536 if value > 32767 else value


class PotTeleop(Node):
    def __init__(self):
        super().__init__("pot_teleop")
        self.declare_parameter("source", "web")          # web | ads1115
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("move_time", 0.3)         # shortest time for a streamed point
        self.source = self.get_parameter("source").value
        self.move_time = self.get_parameter("move_time").value
        self.mode = "web" if self.source == "web" else "pots"
        self.keep_level = True
        self.raw = None
        self.q = None
        self.filter = pots.PotFilter()
        self.ads = Ads1115() if self.source == "ads1115" else None

        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.grip_pub = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.frac_pub = self.create_publisher(Float32MultiArray, "/pots/fractions", 10)
        self.create_subscription(Float32MultiArray, "/hmi/pots", self.on_web_pots, 10)
        self.create_subscription(String, "/hmi/mode", self.on_mode, 10)
        self.create_subscription(Bool, "/hmi/keep_level", self.on_keep_level, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)
        self.get_logger().info(f"pot teleop running, source={self.source}")

    def on_web_pots(self, msg):
        if self.mode == "web":
            self.raw = list(msg.data)

    def on_mode(self, msg):
        if msg.data != self.mode:
            self.mode = msg.data
            self.filter.last_sent = None                # resend on the next change

    def on_keep_level(self, msg):
        self.keep_level = msg.data

    def on_joints(self, msg):
        if all(n in msg.name for n in ARM):
            self.q = [msg.position[msg.name.index(n)] for n in ARM]

    def tick(self):
        if self.mode == "pots" and self.ads is not None:
            self.raw = [pots.ads1115_to_fraction(self.ads.read(a, c)) for a, c in ADS_CHANNELS]
        if self.raw is None or self.mode not in ("pots", "web"):
            return                                       # holding, or a recorded sequence is playing
        f = self.filter.update(self.raw)
        self.frac_pub.publish(Float32MultiArray(data=f))
        if not self.filter.changed(f):
            return
        q = pots.fractions_to_joints(f, keep_level=self.keep_level)
        self.send(q)

    def send(self, q):
        t = move_time(self.q, q[:4], self.move_time) if self.q else 3.0
        dur = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        arm = JointTrajectory(joint_names=ARM, points=[JointTrajectoryPoint(positions=q[:4], time_from_start=dur)])
        grip = JointTrajectory(joint_names=GRIPPER, points=[JointTrajectoryPoint(positions=[q[4], q[4]], time_from_start=dur)])
        self.arm_pub.publish(arm)
        self.grip_pub.publish(grip)


def main():
    rclpy.init()
    node = PotTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
