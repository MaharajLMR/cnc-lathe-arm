"""
Web HMI for the arm, served from the Raspberry Pi (or your laptop in sim).

Shows the arm live, lets you drive it with on-screen sliders when the
hardware knobs aren't in use, and records/replays teach points. Open
http://<pi>:8000 from any phone or laptop on the same network.

Talks to pot_teleop through /hmi/mode, /hmi/pots and /hmi/keep_level, and
plays recorded points straight to the arm and gripper controllers.
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import arm_model, pots

ARM = ["j1_base_yaw", "j2_boom_lift", "j3_wrist_pitch", "j4_wrist_roll"]
GRIPPER = ["j5_gripper", "j5_gripper_mirror"]


def duration(t):
    return Duration(sec=int(t), nanosec=int((t % 1) * 1e9))


class Hmi(Node):
    def __init__(self):
        super().__init__("hmi_server")
        self.declare_parameter("port", 8000)
        self.declare_parameter("points_file", os.path.expanduser("~/lathe_arm_points.yaml"))
        self.declare_parameter("seconds_per_point", 3.0)
        self.points_file = self.get_parameter("points_file").value
        self.step = self.get_parameter("seconds_per_point").value
        self.lock = threading.Lock()
        self.q = [0.0] * 5
        self.fractions = None
        self.web_pots = None
        self.mode = "web"
        self.keep_level = True
        self.points = []
        self.play_until = 0.0

        self.mode_pub = self.create_publisher(String, "/hmi/mode", 10)
        self.pots_pub = self.create_publisher(Float32MultiArray, "/hmi/pots", 10)
        self.level_pub = self.create_publisher(Bool, "/hmi/keep_level", 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.grip_pub = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Float32MultiArray, "/pots/fractions", self.on_fractions, 10)
        self.create_timer(0.5, self.tick)
        if os.path.exists(self.points_file):
            self.load()

    # ROS side
    def on_joints(self, msg):
        names = ARM + ["j5_gripper"]
        with self.lock:
            for i, n in enumerate(names):
                if n in msg.name:
                    self.q[i] = msg.position[msg.name.index(n)]

    def on_fractions(self, msg):
        self.fractions = list(msg.data)

    def tick(self):
        self.mode_pub.publish(String(data="playback" if self.playing else self.mode))
        self.level_pub.publish(Bool(data=self.keep_level))
        if self.play_until and time.time() > self.play_until:      # playback finished
            self.play_until = 0.0
            self.sliders_to_arm()

    @property
    def playing(self):
        return time.time() < self.play_until

    def sliders_to_arm(self):
        """Move the web sliders to where the arm is, so switching back doesn't make it jump."""
        with self.lock:
            self.web_pots = pots.joints_to_fractions(self.q)
        self.pots_pub.publish(Float32MultiArray(data=self.web_pots))

    # actions from the page
    def set_pots(self, f):
        self.web_pots = [min(1.0, max(0.0, float(x))) for x in f][:5]
        if self.mode == "web" and not self.playing:
            self.pots_pub.publish(Float32MultiArray(data=self.web_pots))

    def set_mode(self, mode):
        if mode in ("web", "pots", "hold"):
            if mode == "web":
                self.sliders_to_arm()
            self.mode = mode

    def stop(self):
        self.play_until = 0.0
        self.mode = "hold"
        with self.lock:
            q = list(self.q)
        self.send([{"joints": q}], hold=True)

    def save_point(self, name):
        with self.lock:
            q = [round(v, 4) for v in self.q]
        self.points.append({"name": name or f"p{len(self.points) + 1}", "joints": q})

    def play(self):
        if self.points:
            self.send(self.points)

    def send(self, points, hold=False):
        dt = 0.5 if hold else self.step
        arm = JointTrajectory(joint_names=ARM)
        grip = JointTrajectory(joint_names=GRIPPER)
        for i, p in enumerate(points, start=1):
            q = p["joints"]
            arm.points.append(JointTrajectoryPoint(positions=q[:4], time_from_start=duration(i * dt)))
            grip.points.append(JointTrajectoryPoint(positions=[q[4], q[4]], time_from_start=duration(i * dt)))
        self.arm_pub.publish(arm)
        self.grip_pub.publish(grip)
        if not hold:
            self.play_until = time.time() + len(points) * dt + 0.5

    def save(self):
        with open(self.points_file, "w") as f:
            yaml.safe_dump({"waypoints": self.points}, f, sort_keys=False)

    def load(self):
        with open(self.points_file) as f:
            self.points = (yaml.safe_load(f) or {}).get("waypoints", [])

    def state(self):
        with self.lock:
            q = list(self.q)
        return {
            "q": q,
            "deg": pots.degrees(q),
            "actuator_mm": round(1000 * arm_model.actuator_length(q[1]), 1),
            "stroke_mm": round(1000 * (arm_model.actuator_length(q[1]) - arm_model.ACT_MIN), 1),
            "tool": [round(v, 3) for v in arm_model.tool_position(q)],
            "mode": "playback" if self.playing else self.mode,
            "keep_level": self.keep_level,
            "web_pots": self.web_pots,
            "knobs": self.fractions,
            "points": self.points,
            "geometry": {"pivot_x": arm_model.PIVOT_X, "pivot_z": arm_model.PIVOT_Z, "boom": arm_model.BOOM,
                         "tool": arm_model.TOOL, "act_low": arm_model.ACT_LOW, "act_on_boom": arm_model.ACT_ON_BOOM},
        }


def serve(node: Hmi, port: int):
    page = resources.files("lathe_arm_teach").joinpath("hmi.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/":
                self.reply(page, "text/html; charset=utf-8")
            elif self.path == "/state":
                self.reply(node.state())
            else:
                self.send_error(404)

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}")
            actions = {
                "/pots": lambda: node.set_pots(body["f"]),
                "/mode": lambda: node.set_mode(body["mode"]),
                "/keep_level": lambda: setattr(node, "keep_level", bool(body["on"])),
                "/stop": node.stop,
                "/point/save": lambda: node.save_point(body.get("name", "")),
                "/point/delete": lambda: node.points.pop(int(body["i"])),
                "/point/clear": node.points.clear,
                "/play": node.play,
                "/save": node.save,
                "/load": node.load,
            }
            if self.path not in actions:
                self.send_error(404)
                return
            actions[self.path]()
            self.reply({"ok": True})

    ThreadingHTTPServer.allow_reuse_address = True
    ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()


def main():
    rclpy.init()
    node = Hmi()
    port = node.get_parameter("port").value
    threading.Thread(target=serve, args=(node, port), daemon=True).start()
    node.get_logger().info(f"HMI on http://localhost:{port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
