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
| Load/unload cycle time (sim) | — s | record rosbag + `ros2 topic echo /cycle_time` |
| Waypoint replay RMS joint error | — deg | measure from `/joint_states` during replay |
| 4/4 teach-mode unit tests | **PASS** | `pytest tests/` |

---

## Architecture

```
lathe_arm_description/   URDF (5-DOF revolute chain, placeholder dims)
lathe_arm_bringup/       sim.launch.py, rviz.launch.py, controllers.yaml
lathe_arm_teach/
  teach_node.py          keyboard jog → save waypoints.yaml
  replay_node.py         load waypoints.yaml → MoveIt 2 execute
  waypoints/
    example_load_cycle.yaml
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
pytest tests/ -v   # 4 tests, all pass

# Integration (needs colcon build + ros2 sourced)
colcon test
colcon test-result --verbose
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
- [ ] M1: URDF with real dimensions (Maharaj to provide measurements)
- [ ] M2: MoveIt 2 config + Gazebo sim running
- [ ] M3: Load/unload cycle measured (cycle time)
- [ ] M4: Teach mode end-to-end tested in sim
- [ ] M5: Firmware + BOM added
- [ ] M6: RANE competition photos + demo GIF

---

## License

MIT — see [LICENSE](LICENSE).
