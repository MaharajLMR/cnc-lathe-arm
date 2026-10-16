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
| 62/62 unit tests | **PASS** | `pytest tests/` |
| Workspace sampled (40,000 configs) | X ∈ [−0.375, 0.804] m, Z ∈ [−0.595, 0.958] m | `analysis/workspace_map.py` |
| Max theoretical reach | **0.985 m** | L1+L2+L3+L4+L5 |
| Home EE position (all joints zero) | (0.805, 0, 0.210) m | `analysis/fk_kinematics.py` |
| Near-singular waypoints in load cycle (μ < 0.01) | **2 / 8** (home pose only) | `analysis/workspace_map.py` |
| Estimated load/unload cycle time | **5.55 s** | `analysis/cycle_time_estimator.py` |
| Cartesian path planning: 7 segments, 170 points | **0.000 mm** max linear deviation on straight segments | `analysis/cartesian_path_planner.py` |
| Joint limit violations on full load cycle | **0** | `analysis/cartesian_path_planner.py` |
| Batch scheduling (10 parts, seed=42) | diameter_asc: **55.50 s**, 0 changeovers, 100% utilisation | `analysis/batch_scheduler.py` |
| Fault detection: load-cycle trajectory | **90.0%** fault-free; 13× E04 near-singular (home pose) | `analysis/fault_detector.py` |
| OPC-UA interface | 8 nodes (state/cycles/joints/EE/e-stop/faults), verified with a live asyncua client | `lathe_arm_opcua/` |
| Build cost (BOM) | **₹ 52,370** (STM32F446RE + 5× Dynamixel XM430) | `firmware/bom.csv` |
| Load/unload cycle time (sim, recorded) | — s | record rosbag after sim run |

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
  fk_kinematics.py           pure-Python FK + numerical Jacobian + manipulability
  workspace_map.py           workspace sweep → workspace_side/top.png, manipulability.png
  cycle_time_estimator.py    trapezoidal profile → cycle_estimate.png
  cartesian_path_planner.py  straight-line Cartesian approach/retract via DLS IK
  batch_scheduler.py         multi-part scheduling (3 strategies + Gantt chart)
  fault_detector.py          5 fault types: limits, velocity, tracking, singularity, safety zone
lathe_arm_opcua/
  opcua_server.py            asyncua OPC-UA server (8 nodes) for SCADA/MES integration
  opcua_nodes.py             address space definition
firmware/
  README.md                  MCU (STM32F446RE), Dynamixel XM430, protocol details
  bom.csv                    Bill of Materials — ₹52,370 total
  pinout.md                  STM32 → U2D2 → Dynamixel wiring table
docs/DESIGN.md               kinematic design + dimensions
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

## Advanced Features

### Cartesian path planning (`analysis/cartesian_path_planner.py`)
CNC lathe insert/retract segments require straight Cartesian lines — not joint-interpolated arcs, which can clip the chuck jaw or workpart. The planner implements damped-least-squares (DLS) numerical IK on the position Jacobian and verifies linearity via the perpendicular-deviation metric. Result: **0.000 mm** deviation on all four linear segments over the full load cycle.

### Multi-part batch scheduling (`analysis/batch_scheduler.py`)
Schedules batches of N workparts under three strategies:
- **priority_fifo** — high-priority parts first (57.0 s, 3 changeovers, 97% utilisation)
- **diameter_asc** — sort by diameter to minimise gripper changeovers (**55.5 s, 0 changeovers, 100%**)
- **naive_fifo** — insertion order (58.0 s, 5 changeovers, 96%)

A gripper changeover penalty (0.5 s) is applied whenever consecutive diameters differ by more than 20 mm. Outputs a Gantt chart (`results/batch_schedule.png`).

### OPC-UA industrial interface (`lathe_arm_opcua/`)
Exposes 8 OPC-UA nodes (IEC 62541) for SCADA/MES/ERP integration:
`RobotState`, `CycleCount`, `LastCycleTime_s`, `BatchProgress`, `EStop` (read/write), `JointPositions`, `EEPosition`, `Faults`. Degrades gracefully if `asyncua` is not installed. Connect with UA Expert, Ignition, or `python-opcua`.

### Fault detection (`analysis/fault_detector.py`)
Five fault types monitored in real-time:

| Code | Condition | Load-cycle events |
|---|---|---|
| E01 | Joint position outside limits | 0 |
| E02 | Velocity > 150% rated | 1 |
| E03 | Tracking error > 1° (±0.5% sensor noise) | 3 |
| E04 | Near-singular (μ < 0.01) | 13 (home pose) |
| E05 | EE entered operator safety zone | 0 |

The 13 E04 events confirm the all-zero home pose is kinematically singular — the recommended fix is to move the home position off-axis (documented as a design note in `docs/DESIGN.md`).

---

## Tests

```bash
# All unit tests (no ROS required) — 62 tests (live OPC-UA test needs `pip install asyncua`)
pytest tests/ -v

# Analysis (no ROS required)
python3 analysis/fk_kinematics.py
python3 analysis/workspace_map.py            # → results/workspace_{side,top}.png, manipulability.png
python3 analysis/cycle_time_estimator.py     # → results/cycle_estimate.png
python3 analysis/cartesian_path_planner.py   # → results/cartesian_paths.png, cartesian_plan.json
python3 analysis/batch_scheduler.py          # → results/batch_schedule.png, batch_comparison.json
python3 analysis/fault_detector.py           # → results/fault_report.json

# OPC-UA server (requires: pip install asyncua)
python3 -m lathe_arm_opcua.opcua_server --port 4840

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
- [x] M5: Cartesian path planner, batch scheduler, OPC-UA interface, fault detector — 60/60 tests
- [ ] M6: Load/unload cycle in Gazebo sim + rosbag cycle time measurement
- [ ] M7: Teach mode end-to-end tested in sim
- [ ] M8: RANE competition photos + demo GIF

---

## License

MIT — see [LICENSE](LICENSE).
