# P3 Design — 5-DOF CNC Lathe Loading Arm

**Author:** Maharaj L M R  
**Date:** 2026-10-14  
**Status:** Structure complete; URDF pending arm dimensions from Maharaj.

---

## Background

Physical arm competed at **DriveNext RANE SLD Hackathon 2026** (1st runner-up, ₹75,000).  
Task: automatically load and unload a part into a CNC lathe chuck.  
This repo is a digital twin — same kinematics and workflow, but in Gazebo Harmonic with MoveIt 2.

---

## Dimensions needed (Questions for Maharaj)

| # | Parameter | Why needed | Maharaj's answer |
|---|---|---|---|
| 1 | Base-to-shoulder height (link 1 Z length) | URDF joint 1 | |
| 2 | Upper arm length (link 2, shoulder→elbow) | URDF joint 2 | |
| 3 | Forearm length (link 3, elbow→wrist) | URDF joint 3 | |
| 4 | Wrist pitch link length (link 4) | URDF joint 4 | |
| 5 | Wrist roll / end-effector offset (link 5) | URDF joint 5 | |
| 6 | Joint angle ranges for all 5 DOF | URDF limits | |
| 7 | Approximate payload (part mass) | MoveIt 2 planning | |
| 8 | Mounting: floor / wall / ceiling | URDF world frame | |
| 9 | MCU used (Arduino / STM32 / ESP32) | Firmware section | |
| 10 | Distance from arm base to lathe chuck centre | Sim scene layout | |
| 11 | Chuck inner diameter / part diameter | Collision geometry | |

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

Cycle time measured from a rosbag replay — reported in README.

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

## Bill of Materials (from real hardware — fill in costs)

| Component | Maharaj's cost (₹) |
|---|---|
| MCU (to fill) | |
| 5× servo motors | |
| Arm frame / chassis material | |
| Power supply | |
| End-effector / gripper | |
| Miscellaneous (wiring, brackets) | |
| **Total** | |

*These numbers back the "low-cost" claim in the repo title.*

---

## Milestones

| # | Name | Done? |
|---|---|---|
| M0 | Repo structure + DESIGN.md + teach mode (4/4 tests pass) | ✓ |
| M1b | Pure-Python FK + Jacobian + workspace analysis (14/14 tests pass) | ✓ |
| M2 | MoveIt 2 config: SRDF, kinematics.yaml, OMPL, controllers, launch | ✓ |
| M1 | URDF with real dimensions | **needs Maharaj's measurements** |
| M3 | Load/unload cycle in Gazebo + cycle time | needs M1 + ros2-control |
| M4 | Teach mode end-to-end in sim | needs M3 |
| M5 | Firmware + BOM section complete | needs Maharaj's files |
| M6 | RANE competition photos added | needs Maharaj's photos |
| M7 | CI green + README complete | needs M1–M4 |
