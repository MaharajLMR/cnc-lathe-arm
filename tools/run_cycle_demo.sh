#!/usr/bin/env bash
# Headless Gazebo run of the teach-mode load cycle, recorded from the cell camera.
# Usage: tools/run_cycle_demo.sh [out_dir]
# Needs the workspace built and sourced (colcon build; source install/setup.bash).
set -u
OUT=${1:-$(pwd)/demo_run}
mkdir -p "$OUT/frames"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}
export GZ_PARTITION=${GZ_PARTITION:-lathedemo}
HERE=$(cd "$(dirname "$0")" && pwd)
WP=$(ros2 pkg prefix lathe_arm_teach)/share/lathe_arm_teach/waypoints/example_load_cycle.yaml
[ -f "$WP" ] || WP="$HERE/../lathe_arm_teach/waypoints/example_load_cycle.yaml"

ros2 launch lathe_arm_bringup sim.launch.py headless:=true > "$OUT/launch.log" 2>&1 &
LAUNCH=$!
for i in $(seq 1 60); do
  ros2 control list_controllers 2>/dev/null | grep -q "lathe_arm_controller.*active" && break
  sleep 2
done
ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
python3 "$HERE/record_cell.py" --out "$OUT/frames" --every 3 > "$OUT/recorder.log" 2>&1 &
REC=$!
sleep 3
START=$(date +%s.%N)
ros2 run lathe_arm_teach replay_node --ros-args -p waypoints:="$WP" -p use_sim_time:=true > "$OUT/replay.log" 2>&1
END=$(date +%s.%N)
echo "replay wall time: $(echo "$END - $START" | bc) s" | tee "$OUT/cycle_time.txt"
timeout 5 ros2 topic echo --once /joint_states > "$OUT/final_joint_states.txt" 2>&1
sleep 2
kill -INT $REC 2>/dev/null; sleep 2
kill -INT $LAUNCH 2>/dev/null; sleep 5
kill -9 $LAUNCH 2>/dev/null
pkill -9 -f "[g]z sim -s -r --headless-rendering .*lathe_cell" 2>/dev/null
echo "frames: $(ls "$OUT/frames" | wc -l)"
