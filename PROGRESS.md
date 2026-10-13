# PROGRESS — CNC Lathe Loading Arm (5-DOF)

**Last updated:** 2026-10-14 (overnight session #3)

## Current milestone: M4 complete (28/28 tests pass)

## Done
- [x] Repo structure, git init
- [x] docs/DESIGN.md with kinematic chain, transform derivation
- [x] URDF with industrial dimensions (L1=0.18, L2=0.35, L3=0.30, L4=0.09, L5=0.065 m)
- [x] Updated joint limits: j2 −57° to +109°, j3 −120° to +11.5°
- [x] ros2_control tags in URDF (gz_ros2_control/GazeboSimSystem)
- [x] lathe_arm_bringup: sim.launch.py, rviz.launch.py, controllers.yaml
- [x] lathe_arm_moveit: SRDF (planning group "lathe_arm", named states), kinematics.yaml, OMPL, controllers
- [x] lathe_arm_teach: teach_node.py (keyboard jog + YAML save)
- [x] lathe_arm_teach: replay_node.py (YAML → FollowJointTrajectory)
- [x] lathe_arm_teach: example_load_cycle.yaml (8 waypoints, all within new limits)
- [x] analysis/fk_kinematics.py: FK + Jacobian + manipulability (updated dimensions)
- [x] analysis/workspace_map.py: 40k-sample workspace sweep → 3 PNGs
- [x] analysis/cycle_time_estimator.py: trapezoidal profile → 5.55 s total cycle
- [x] firmware/README.md: MCU (STM32F446RE), Dynamixel XM430, e-stop, protocol
- [x] firmware/bom.csv: 12-line BOM, ₹52,370 total
- [x] firmware/pinout.md: STM32 → U2D2 → Dynamixel wiring
- [x] tests/test_fk_kinematics.py: 14/14 pass (updated for new base offset)
- [x] tests/test_teach_replay.py: 4/4 pass (updated joint limits)
- [x] tests/test_urdf_kinematics.py: 10/10 pass (new — URDF structure + FK cross-check)
- [x] **Total: 28/28 tests pass**
- [x] .github/workflows/ci.yml
- [x] README.md updated with real analysis numbers
- [x] LICENSE

## Key results (from analysis scripts)
| Metric | Value |
|---|---|
| Max reach | 0.985 m |
| Home EE position | (0.805, 0, 0.210) m |
| Workspace X range | −0.375 to +0.804 m |
| Workspace Z range | −0.595 to +0.958 m |
| Near-singular waypoints | 2/8 (home only, μ=0) |
| Cycle time estimate | 5.55 s |
| Build cost | ₹52,370 |

## Next
- [ ] M5: Load/unload cycle in Gazebo sim + rosbag cycle time measurement
- [ ] M6: Teach mode end-to-end tested in sim
- [ ] M7: RANE competition photos added to media/rane/ (from Maharaj's phone)
- [ ] M8: Demo GIF + README final

## Blockers (hardware-only)
- Firmware source: copy original STM32 code into firmware/src/ when available
- Photos: RANE hackathon competition photos from Maharaj's phone → media/rane/
