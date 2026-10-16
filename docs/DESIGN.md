# Design notes

## Where this came from

At the DriveNext RANE hackathon I built a small arm that loaded a lathe. This repo takes the same job and sizes the arm for a real small CNC lathe cell. None of the numbers below are measured from the hackathon robot; they're what I picked for the bigger version.

## Dimensions

| | Length / range | What it's for |
|---|---|---|
| L1 base to shoulder | 0.18 m | shoulder height above the base flange |
| L2 upper arm | 0.35 m | main reach |
| L3 forearm | 0.30 m | second reach link |
| L4 wrist pitch link | 0.09 m | wrist offset |
| L5 wrist roll to tool | 0.065 m | gripper centre |
| J1 base yaw | ±90° | swing between the part stand and the chuck |
| J2 shoulder | -57° to +109° | |
| J3 elbow | -120° to +11° | |
| J4 wrist pitch | ±90° | |
| J5 wrist roll | ±180° | |

Total reach is 0.985 m. The same numbers are in the URDF and in `analysis/fk_kinematics.py`, and `tests/test_urdf_kinematics.py` checks they match.

## How the pieces fit

- `lathe_arm_description`: the URDF, with the gz_ros2_control plugin.
- `lathe_arm_bringup`: `sim.launch.py` starts Gazebo with the lathe cell and spawns the joint state broadcaster and the trajectory controller; `rviz.launch.py` is RViz only.
- `lathe_arm_teach`:
  - `teach_node` reads keys and jogs one joint at a time; `s` writes the current pose to the YAML file.
  - `replay_node` sends each waypoint to `/lathe_arm_controller/follow_joint_trajectory`, 2 s per move, and publishes which waypoint it's on.
- `lathe_arm_moveit`: SRDF, kinematics and OMPL config. It isn't used by the replay yet (see the README's to-do list).
- `lathe_arm_opcua`: an asyncua OPC-UA server with the arm's state, cycle count, joints, end-effector position, e-stop and faults.

The Gazebo cell (`lathe_arm_bringup/worlds/lathe_cell.sdf`) puts the part stand and the chuck exactly at the end-effector positions of the example cycle's grasp and insert waypoints, so the recording lines up. They're visual only.

## The load cycle

Home, approach the part, grasp, lift, swing to the chuck, insert, retract, home. The example in `lathe_arm_teach/waypoints/example_load_cycle.yaml` is 8 waypoints. With trapezoidal profiles at 0.8 m/s and 3 m/s² it comes out at about 5.5 s (`analysis/cycle_time_estimator.py`). I haven't measured it in simulation yet.

Home is all joints at zero, with the arm stretched out straight, which is a singular pose. The fault checker flags it every time. It should be moved, but I left it so the example matches what's in the tests.

## Hardware for the bigger version

`firmware/` has a proposed parts list (about ₹52,000: STM32F446RE, five Dynamixel XM430 servos, U2D2, 24 V supply, frame) and a pinout. It's a plan, not something I've built.
