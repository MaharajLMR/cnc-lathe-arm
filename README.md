# Low-Cost CNC Lathe Loading Arm (5-DOF)

[![CI](https://github.com/MaharajLMR/cnc-lathe-arm/actions/workflows/ci.yml/badge.svg)](https://github.com/MaharajLMR/cnc-lathe-arm/actions/workflows/ci.yml)

**Digital twin of a 5-DOF serial arm that automates CNC lathe loading/unloading. Teach mode lets an operator record a load cycle in minutes; the ROS 2 node replays it with MoveIt 2. Competed at DriveNext RANE SLD Hackathon 2026 — 1st runner-up, ₹75,000.**

> **Competition photos:** DriveNext RANE SLD Hackathon 2026 — 1st runner-up, ₹75,000.
> *(Add photos to `media/rane/` from phone.)*

> **Demo GIF:** Record with `ros2 launch lathe_arm_bringup sim.launch.py` + Kazam.

---

## Results

*All numbers produced by scripts in `analysis/` — no fabrication.*

| Metric | Value | Script |
|---|---|---|
| 28/28 unit tests | **PASS** | `pytest tests/` |
| Workspace sampled (40,000 configs) | X ∈ [−0.375, 0.804] m, Z ∈ [−0.595, 0.958] m | `analysis/workspace_map.py` |
| Max theoretical reach | **0.985 m** | L1+L2+L3+L4+L5 |
| Home EE position (all joints zero) | (0.805, 0, 0.210) m | `analysis/fk_kinematics.py` |
| Near-singular waypoints in load cycle (μ < 0.01) | **2 / 8** (home pose only) | `analysis/workspace_map.py` |
| Estimated load/unload cycle time | **5.55 s** | `analysis/cycle_time_estimator.py` |
| Cycle kinematics: v_max | 0.80 m/s, a_max 3.0 m/s² | `analysis/cycle_time_estimator.py` |
| Build cost (BOM) | **₹ 52,370** | `firmware/bom.csv` |
| Load/unload cycle time (sim, recorded) | — s | record rosbag after M3 |

---

## Architecture

```
lathe_arm_description/   URDF (5-DOF revolute, L1=0.18 L2=0.35 L3=0.30 L4=0.09 L5=0.065 m)
lathe_arm_bringup/       sim.launch.py, rviz.launch.py, controllers.yaml
lathe_arm_moveit/        MoveIt 2 config (SRDF, kinematics, OMPL, controllers)
lathe_arm_teach/
  teach_node.py          keyboard jog → save waypoints.yaml
  replay_node.py         load waypoints.yaml → FollowJointTrajectory execute
  waypoints/
    example_load_cycle.yaml   8-waypoint load cycle (within joint limits)
analysis/
  fk_kinematics.py       pure-Python FK + numerical Jacobian + manipulability
  workspace_map.py       workspace sweep → workspace_side/top.png + manipulability.png
  cycle_time_estimator.py  trapezoidal profile → cycle_estimate.png
firmware/
  README.md              MCU (STM32F446RE), Dynamixel XM430, protocol details
  bom.csv                Bill of Materials — ₹52,370 total
  pinout.md              STM32 → U2D2 → Dynamixel wiring table
docs/DESIGN.md           kinematic design + dimensions
```

---

## Kinematics

5-DOF revolute chain with DH-equivalent transforms:

```
T_ee = Tz(0.030) Rz(q1) Tz(L1) Ry(q2) Tx(L2) Ry(q3) Tx(L3) Ry(q4) Tx(L4) Rx(q5) Tx(L5)
```

| Link | Length | Joint axis | Limit |
|---|---|---|---|
| Shoulder height L1 | 0.180 m | Z (yaw) | ±90° |
| Upper arm L2 | 0.350 m | Y (pitch) | −57° to +109° |
| Forearm L3 | 0.300 m | Y (pitch) | −120° to +11.5° |
| Wrist pitch L4 | 0.090 m | Y (pitch) | ±90° |
| Wrist roll L5 | 0.065 m | X (roll) | ±180° |

Total reach: 0.985 m. Payload: ~5 kg.

---

## Build & Run

```bash
# Prerequisites: ROS 2 Jazzy, ros-jazzy-ros2-control, ros-jazzy-moveit

cd ~/ros2_ws/src
git clone https://github.com/MaharajLMR/cnc-lathe-arm.git
cd ~/ros2_ws
rosdep install --from-paths src/cnc-lathe-arm --ignore-src -r -y
colcon build --packages-select lathe_arm_description lathe_arm_bringup lathe_arm_moveit lathe_arm_teach
source install/setup.bash

# Visualise in RViz (no Gazebo — drag joint sliders)
ros2 launch lathe_arm_bringup rviz.launch.py

# Full simulation (Gazebo + ros2_control)
ros2 launch lathe_arm_bringup sim.launch.py

# Teach mode (second terminal after sim is running)
ros2 run lathe_arm_teach teach_node --ros-args -p output:=my_cycle.yaml
# Keys: 1-5=select joint  +/-=jog  s=save  p=preview  q=quit + save

# Replay a recorded cycle
ros2 run lathe_arm_teach replay_node --ros-args -p waypoints:=my_cycle.yaml
```

---

## Tests

```bash
# All unit tests (no ROS required) — 28 tests
pytest tests/ -v

# Analysis (no ROS required)
python3 analysis/fk_kinematics.py
python3 analysis/workspace_map.py      # → results/workspace_side.png, workspace_top.png, manipulability.png
python3 analysis/cycle_time_estimator.py  # → results/cycle_estimate.png

# Integration tests
colcon test && colcon test-result --verbose
```

---

## Real hardware

> *Add RANE SLD Hackathon 2026 competition photos here.*
> Place images in `media/rane/` and reference as `![Competition photo](media/rane/photo1.jpg)`.

---

## My role

Solo design and build. Competed at **DriveNext RANE SLD Hackathon 2026** (1st runner-up, ₹75,000). Original MCU firmware (STM32F446RE + Dynamixel XM430) in `firmware/`. This repo is a digital twin of that hardware, built to MoveIt 2 + Gazebo Harmonic standard.

---

## Roadmap

- [x] M0: Repo structure + DESIGN.md + teach mode (4/4 tests pass)
- [x] M1: Pure-Python FK + Jacobian + manipulability (14 tests pass, workspace plots)
- [x] M2: MoveIt 2 config — SRDF, kinematics.yaml, OMPL, controllers
- [x] M3: Cycle time estimator — 5.55 s, workspace analysis complete
- [x] M4: Industrial URDF (L=0.18/0.35/0.30/0.09/0.065 m), firmware BOM, 28/28 tests pass
- [ ] M5: Load/unload cycle in Gazebo sim + rosbag cycle time measurement
- [ ] M6: Teach mode end-to-end tested in sim
- [ ] M7: RANE competition photos + demo GIF

---

## License

MIT — see [LICENSE](LICENSE).
