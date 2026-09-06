#!/bin/bash
# Runs INSIDE the mowgli-ros2 image. env: ANCHOR (true/false), RATE, TAG (output subdir), EXTRA (extra -p args), OVERLAY (setup.bash of a local build)
set -o pipefail
source /opt/ros/kilted/setup.bash; source /ros2_ws/install/setup.bash
[ -n "${OVERLAY:-}" ] && source "$OVERLAY"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
OUT=/data/replay/${TAG:-run}; mkdir -p "$OUT"
BAG=/data/bag_lidar_rtk_20260906_2128
PF=$(ros2 pkg prefix fusion_graph)/share/fusion_graph/config/fusion_graph.yaml
# 1) clock first, so the node's bootstrap happens at bag time, not at t=0
ros2 bag play "$BAG" --clock 100 -r ${RATE:-1} --topics /scan /wheel_odom /imu/data /gps/fix /gps/status /tf_static /behavior_tree_node/high_level_status ${PLAY_EXTRA_TOPICS:-} > "$OUT/play.log" 2>&1 &
PLAY=$!
sleep 1.5
# 2) latched seed pose = the robot's own fused pose at bag start
python3 /data/seed_pose.py "$BAG" 25 > "$OUT/seed.log" 2>&1 &
SEED=$!
sleep 1
# 3) the node under test
ros2 run fusion_graph fusion_graph_node --ros-args -r __node:=fusion_graph_node --params-file "$PF" \
  -p use_sim_time:=true -p datum_lat:=48.879649550 -p datum_lon:=2.172814460 -p lever_arm_x:=0.3 -p lever_arm_y:=0.0 \
  -p dock_pose_x:=6.272769 -p dock_pose_y:=2.798334 -p dock_pose_yaw:=-0.934583 \
  -p use_scan_matching:=true -p use_loop_closure:=true -p use_lidar_map_anchor:=${ANCHOR:-true} -p scan_topic:=/scan ${EXTRA:-} \
  > "$OUT/node.log" 2>&1 &
NODE=$!
sleep 1
python3 /data/replay_log.py "$OUT/log.csv" > "$OUT/log.txt" 2>&1 &
LOG=$!
wait $PLAY
sleep 2
kill -INT $LOG $NODE $SEED 2>/dev/null; sleep 2; kill -9 $LOG $NODE $SEED 2>/dev/null
echo "rows: $(wc -l < "$OUT/log.csv")"; cat "$OUT/seed.log"; grep -E "bootstrap|seeding from dock|wrong-fix" "$OUT/node.log" | cut -c1-160 | head -5
python3 /data/replay_eval.py "$OUT/log.csv"
