# Low-Cost CNC Lathe Loading Arm (5-DOF)

[![CI](https://github.com/MaharajLMR/cnc-lathe-arm/actions/workflows/ci.yml/badge.svg)](https://github.com/MaharajLMR/cnc-lathe-arm/actions/workflows/ci.yml)

**Digital twin of a 5-DOF serial arm that automates CNC lathe loading/unloading. Teach mode lets an operator record a load cycle in minutes; the ROS 2 node replays it with MoveIt 2.**

> **Competition photos:** DriveNext RANE SLD Hackathon 2026 — 1st runner-up, ₹75,000.  
> *(Add photos to `media/rane/` from phone. Photos go under "Real hardware" section below.)*

> **Demo GIF:** Record with `ros2 launch lathe_arm_bringup sim.launch.py` + Kazam.  
> *(URDF dimensions are placeholders — update from real arm measurements first.)*

---

## Results

| Metric | Value | Script |
|---|---|---|
| 4/4 teach-mode unit tests | **PASS** | `pytest tests/test_teach_replay.py` |
| Workspace sampled (40,000 configs) | X ∈ [−0.37, 0.63] m, Z ∈ [−0.48, 0.78] m | `analysis/workspace_map.py` |
| Max theoretical reach | 0.75 m | L1+L2+L3+L4+L5 |
| Near-singular waypoints (μ < 0.01) | 2 / 8 (home) | `analysis/workspace_map.py` |
| Estimated load/unload cycle time (placeholder dims) | **4.75 s** | `analysis/cycle_time_estimator.py` |
| Load/unload cycle time (sim, real dims) | — s | record rosbag after M3 |

---

## Architecture

```
lathe_arm_description/   URDF (5-DOF revolute chain, placeholder dims)
lathe_arm_bringup/       sim.launch.py, rviz.launch.py, controllers.yaml
lathe_arm_moveit/        MoveIt 2 config (SRDF, kinematics, OMPL, RViz)
lathe_arm_teach/
  teach_node.py          keyboard jog → save waypoints.yaml
  replay_node.py         load waypoints.yaml → FollowJointTrajectory execute
  waypoints/
    example_load_cycle.yaml
analysis/
  fk_kinematics.py       pure-Python FK + numerical Jacobian + manipulability
  workspace_map.py       workspace sweep → workspace_side/top.png + manipulability.png
firmware/                original MCU code (to add) + wiring + BOM
docs/DESIGN.md           kinematic design + dimensions table
```

---

## Build & Run

```bash
# Prerequisites: ROS 2 Jazzy, ros-jazzy-ros2-control, ros-jazzy-moveit

cd ~/ros2_ws/src
git clone https://github.com/MaharajLMR/cnc-lathe-arm.git
cd ~/ros2_ws
rosdep install --from-paths src/cnc-lathe-arm --ignore-src -r -y
colcon build

source install/setup.bash

# Visualise in RViz (no Gazebo — drag joint sliders)
ros2 launch lathe_arm_bringup rviz.launch.py

# Full simulation
ros2 launch lathe_arm_bringup sim.launch.py

# Teach mode (in second terminal after sim is running)
ros2 run lathe_arm_teach teach_node --ros-args -p output:=my_cycle.yaml
# Keys: 1-5=select joint  +/-=jog  s=save waypoint  q=quit

# Replay a recorded cycle
ros2 run lathe_arm_teach replay_node --ros-args -p waypoints:=my_cycle.yaml
```

---

## Tests

```bash
# Unit tests (no ROS required)
pytest tests/test_teach_replay.py -v          # 4 tests: waypoint validity + round-trip

# FK analysis (no ROS required)
python3 analysis/fk_kinematics.py             # FK demo + manipulability spot-check
python3 analysis/workspace_map.py             # 40k-sample workspace sweep → 3 PNGs
python3 analysis/cycle_time_estimator.py      # cycle time estimate → cycle_estimate.png

# MoveIt 2 visualisation (needs ROS 2 + MoveIt installed)
ros2 launch lathe_arm_moveit moveit.launch.py

# Full simulation (needs ros2_control + gz_ros2_control)
ros2 launch lathe_arm_bringup sim.launch.py

# Integration tests (needs colcon build + ROS sourced)
colcon test && colcon test-result --verbose
```

---

## ⚠️ Dimensions are placeholders

The URDF currently uses estimated dimensions. Before running the sim:

1. Measure the real arm (see `docs/DESIGN.md` "Dimensions needed" table)
2. Update values in `lathe_arm_description/urdf/lathe_arm.urdf.xacro` (lines 16–21)
3. Re-run `colcon build`

---

## Real hardware

> *Add RANE SLD Hackathon 2026 competition photos here.*  
> Place images in `media/rane/` and reference them as:  
> `![Competition photo](media/rane/photo1.jpg)`

---

## My role

Solo design and build. Competed at **DriveNext RANE SLD Hackathon 2026** (1st runner-up, ₹75,000). Original MCU code and wiring in `firmware/`. This repo is a digital twin of that hardware.

---

## Roadmap

- [x] M0: Repo structure + DESIGN.md + teach mode (4/4 tests pass)
- [x] M1b: Pure-Python FK + Jacobian + manipulability analysis (workspace plots generated)
- [x] M2: MoveIt 2 config — SRDF, kinematics.yaml, OMPL, moveit_controllers.yaml, launch
- [ ] M1 (blocking): URDF with real dimensions (Maharaj to measure real arm)
- [ ] M3: Load/unload cycle in Gazebo + cycle time measurement
- [ ] M4: Teach mode end-to-end tested in sim
- [ ] M5: Firmware + BOM added
- [ ] M6: RANE competition photos + demo GIF

---

## License

MIT — see [LICENSE](LICENSE).
