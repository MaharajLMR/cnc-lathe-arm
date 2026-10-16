# P3 Design — 5-DOF CNC Lathe Loading Arm

**Author:** Maharaj L M R  
**Date:** 2026-10-16  
**Status:** Design dimensions fixed; kinematics, MoveIt 2 config and analysis complete.

---

## Background

The lathe-loading concept placed 1st runner-up (₹75,000) at the **DriveNext RANE SLD Hackathon 2026**.  
Task: automatically load and unload a part into a CNC lathe chuck.  
This repo scales that concept into an industrial design study in ROS 2, MoveIt 2 and Gazebo Harmonic. The dimensions below are chosen design values, sized for a typical small CNC lathe cell; they are not measurements of the competition prototype.

---

## Design dimensions

| Parameter | Value | Role |
|---|---|---|
| L1 base → shoulder | 0.18 m | shoulder height above the base flange |
| L2 upper arm | 0.35 m | main reach link |
| L3 forearm | 0.30 m | second reach link |
| L4 wrist pitch link | 0.09 m | wrist pitch offset |
| L5 wrist roll → tool | 0.065 m | tool offset to gripper centre |
| Total reach | 0.985 m | sum of L1–L5 |
| J1 base yaw | ±90° | swing between part rack and chuck |
| J2 shoulder | −57° … +109° | |
| J3 elbow | −120° … +11° | |
| J4 wrist pitch | ±90° | |
| J5 wrist roll | ±180° | |

Values are mirrored in `lathe_arm_description/urdf/lathe_arm.urdf.xacro` and `analysis/fk_kinematics.py`; `tests/test_urdf_kinematics.py` checks the two agree.

---

## Architecture

```
┌─────────────────────────────────────────────┐
│  lathe_arm_description/                      │
│    URDF (5-DOF revolute chain)               │
│    Collision geometry for arm + part         │
│    Gazebo scene: arm base + lathe chuck      │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│  lathe_arm_moveit/                           │
│    MoveIt 2 config (via moveit_setup_assistant│
│    or handwritten): SRDF, kinematics.yaml,   │
│    ompl_planning.yaml, joint_limits.yaml     │
│    Planners: OMPL RRTConnect + STOMP          │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│  lathe_arm_bringup/                          │
│    sim.launch.py: Gazebo + arm + MoveIt 2    │
│    load_cycle.launch.py: automated demo      │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│  lathe_arm_teach/                            │
│    teach_node.py:                            │
│      jog interface (keyboard/interactive     │
│      marker) → record waypoints as YAML      │
│    replay_node.py:                           │
│      load YAML → MoveIt 2 move_group         │
│      → execute sequence                      │
└──────────────────────────────────────────────┘
```

---

## Load/Unload Cycle

1. **HOME** → approach above chuck (pre-grasp pose)
2. Descend → align gripper with part
3. Gripper CLOSE (pick part)
4. Lift → retract to safe height
5. Navigate to chuck approach
6. Insert part into chuck → chuck CLAMP
7. Gripper OPEN → retract
8. **HOME**
9. (Reverse for unload)

Cycle time is currently estimated from trapezoidal profiles (`analysis/cycle_time_estimator.py`); a rosbag measurement follows once the Gazebo sim runs.

---

## Teach Mode

The "teach mode" lets an operator jog the arm joint-by-joint, then save each learned position as a named waypoint in YAML:

```yaml
waypoints:
  - name: home
    joints: [0.0, 0.0, 0.0, 0.0, 0.0]
  - name: pre_grasp
    joints: [0.52, -0.30, 0.80, -0.50, 0.0]
  - name: grasp
    joints: [0.52, -0.45, 1.10, -0.65, 0.0]
  - name: chuck_insert
    joints: [1.20, -0.20, 0.60, -0.40, 0.0]
```

`replay_node.py` loads this YAML and sends each pose to MoveIt 2's `move_group` action in sequence.

---

## Bill of Materials

Proposed hardware for the industrial version is costed in [`firmware/bom.csv`](../firmware/bom.csv): **₹52,370** total (STM32F446RE + 5× Dynamixel XM430 + U2D2 + 24 V supply + frame).

---

## Milestones

| # | Name | Done? |
|---|---|---|
| M0 | Repo structure + DESIGN.md + teach mode (4/4 tests pass) | ✓ |
| M1b | Pure-Python FK + Jacobian + workspace analysis (14/14 tests pass) | ✓ |
| M2 | MoveIt 2 config: SRDF, kinematics.yaml, OMPL, controllers, launch | ✓ |
| M1 | URDF with design dimensions | ✓ |
| M3 | Load/unload cycle in Gazebo + cycle time | needs M1 + ros2-control |
| M4 | Teach mode end-to-end in sim | needs M3 |
| M5 | Proposed hardware: BOM + pinout + firmware architecture | ✓ |
| M6 | Gazebo demo GIF | needs ros2-control install |
| M7 | CI green + README complete | needs M1–M4 |
