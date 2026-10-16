# cnc-lathe-arm

[![CI](https://github.com/MaharajLMR/cnc-lathe-arm/actions/workflows/ci.yml/badge.svg)](https://github.com/MaharajLMR/cnc-lathe-arm/actions/workflows/ci.yml)

A 5-axis arm that loads raw parts into a CNC lathe and takes finished ones out, in ROS 2 Jazzy with ros2_control and Gazebo Harmonic.

At the DriveNext RANE hackathon I built a small lathe-loading arm (it came 1st runner-up). This repo is me scaling that idea up to something closer to an industrial cell. The link lengths, joint limits and the parts list in `firmware/` are my design choices for that bigger version; they aren't measurements of the hackathon robot. [docs/DESIGN.md](docs/DESIGN.md) explains them.

![load cycle](docs/load_cycle_gazebo.gif)

That's the example cycle running in Gazebo: pick a part off the stand, put it in the chuck, back off, go home. The lathe and the stand are just visuals, there's no contact physics.

## What's in here

- **Teach mode.** Jog the joints with the keyboard (`1`-`5` picks a joint, `+`/`-` moves it, `s` saves a waypoint) and it writes a YAML file. `replay_node` plays the file back through the joint trajectory controller.
- **Kinematics** in plain Python (`analysis/fk_kinematics.py`): forward kinematics, Jacobian, manipulability. The arm reaches 0.985 m. The all-zero home pose is singular, which the fault checker flags; I've left it as a known issue.
- **Straight-line moves** for going in and out of the chuck (`analysis/cartesian_path_planner.py`). Joint-space moves curve, and near the chuck jaws you want a straight line. It uses damped least-squares IK and the straight segments stay on the line to well under a millimetre.
- **Batch scheduling** (`analysis/batch_scheduler.py`). If parts of different diameters are queued, doing them sorted by diameter avoids gripper changes. On the 10-part example that's 55.5 s vs 58 s in arrival order.
- **Fault checks** (`analysis/fault_detector.py`): joint limits, speed, tracking error, singular poses, and a no-go box where an operator would stand.
- **OPC-UA server** (`lathe_arm_opcua/`) exposing the arm's state, cycle count, joints and e-stop, since that's how a lathe cell would usually talk to a PLC or SCADA system. Tested against a real asyncua client.
- A **MoveIt 2 config** is included, but the replay currently sends joint goals directly rather than planning through MoveIt.

## Running it

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/MaharajLMR/cnc-lathe-arm.git
cd ~/ros2_ws && colcon build && source install/setup.bash

ros2 launch lathe_arm_bringup sim.launch.py              # Gazebo with the lathe cell
ros2 run lathe_arm_teach teach_node --ros-args -p output:=my_cycle.yaml
ros2 run lathe_arm_teach replay_node --ros-args -p waypoints:=my_cycle.yaml
```

`ros2 launch lathe_arm_bringup rviz.launch.py` shows the arm in RViz with joint sliders if you don't want Gazebo.

`tools/run_cycle_demo.sh` runs the example cycle headless and records it.

The analysis scripts don't need ROS:

```bash
python3 analysis/fk_kinematics.py
python3 analysis/workspace_map.py
python3 analysis/cartesian_path_planner.py
python3 analysis/batch_scheduler.py
```

## Tests

`pytest tests` covers kinematics, teach/replay, the planner, the scheduler, the fault checks and the OPC-UA nodes. The live OPC-UA test is skipped unless `asyncua` is installed.

## Things I'd still like to do

- Plan the load cycle through MoveIt instead of replaying joint goals.
- Give the cell real collision geometry so the gripper actually has to clear the chuck.
- Move the home pose off the singularity.
- Measure cycle time in simulation; 5.55 s is currently a trapezoidal-profile estimate.

MIT license.
