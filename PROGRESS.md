# PROGRESS — CNC Lathe Loading Arm (5-DOF)

**Last updated:** 2026-10-14 (overnight session)

## Current milestone: M0 complete

## Done
- [x] Repo structure, git init
- [x] docs/DESIGN.md with full dimensions table (questions for Maharaj)
- [x] URDF placeholder (5 revolute joints, correct axis/limit structure)
- [x] ros2_control tags in URDF
- [x] lathe_arm_bringup: sim.launch.py, rviz.launch.py, controllers.yaml
- [x] lathe_arm_teach: teach_node.py (keyboard jog + YAML save)
- [x] lathe_arm_teach: replay_node.py (YAML → JointTrajectoryController)
- [x] lathe_arm_teach: example_load_cycle.yaml (8 waypoints)
- [x] tests/test_teach_replay.py: 4/4 pass
- [x] .github/workflows/ci.yml
- [x] README.md, LICENSE, PROGRESS.md
- [x] firmware/README.md (BOM placeholder)

## Next (need Maharaj's input)
- [ ] M1: Replace placeholder URDF dimensions with real arm measurements
- [ ] M2: MoveIt 2 config (run moveit_setup_assistant or write by hand)
- [ ] M3: Test load/unload cycle in Gazebo, measure cycle time
- [ ] M4: Firmware code + wiring diagram added
- [ ] M5: RANE competition photos added to media/rane/

## Blockers
- URDF dimensions: need real measurements (see docs/DESIGN.md table)
- Firmware: need original MCU code files from Maharaj
- Photos: need RANE hackathon competition photos from Maharaj's phone
