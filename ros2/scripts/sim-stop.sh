#!/bin/bash
# sim-stop.sh — Gracefully stop all ROS2/Webots processes and clean DDS state.
#
# 1. Send SIGINT to ros2 launch processes (triggers graceful shutdown of all nodes)
# 2. Wait up to 5s for children to exit
# 3. SIGKILL any stragglers
# 4. Clean Cyclone DDS shared memory and Webots IPC sockets
set -e
trap "" INT  # Ignore SIGINT so we don't die from the signals we send

# Find ros2 launch python processes (not this script)
PIDS=$(ps -eo pid,args | grep '[p]ython3.*ros2.launch' | awk '{print $1}')

if [ -n "$PIDS" ]; then
  echo "Stopping ros2 launch (SIGINT)..."
  kill -INT $PIDS 2>/dev/null || true

  for i in 1 2 3 4 5; do
    sleep 1
    REMAINING=$(ps -eo pid,args | grep '[p]ython3.*ros2.launch' | awk '{print $1}')
    [ -z "$REMAINING" ] && break
  done

  REMAINING=$(ps -eo pid,args | grep '[p]ython3.*ros2.launch' | awk '{print $1}')
  if [ -n "$REMAINING" ]; then
    echo "Force killing stragglers (SIGKILL)..."
    kill -9 $REMAINING 2>/dev/null || true
    sleep 1
  fi
fi

# Kill any remaining Webots processes (they survive ros2 launch SIGINT — the
# webots binary is launched with --batch, the ros2_supervisor connects via
# extern controller, and webots_controller_MowgliMower runs the
# controller_manager). All three must be killed; otherwise the next launch
# fails with "Cannot connect to Webots instance" because the IPC socket
# stays bound.
WB_PIDS=$(ps -eo pid,args | grep -E '[w]ebots-bin|[w]ebots/webots|[w]ebots_controller|[r]os2_supervisor.py|[w]ebots_ros2_driver' | awk '{print $1}')
if [ -n "$WB_PIDS" ]; then
  echo "Killing Webots processes..."
  kill -9 $WB_PIDS 2>/dev/null || true
  sleep 1
fi

# Kill ALL remaining ROS2 nodes (not just launch) to prevent stale DDS
# participants. Includes the sim-side nodes (sim_navsat_rtk_fix,
# sim_imu_noise, sim_wheel_slip), the coverage_server, and
# the scan_deskew node.
ROS_PIDS=$(ps -eo pid,args | grep -E '[r]os2|[p]arameter_bridge|[f]oxglove|[e]kf_node|[b]ehavior_tree|[c]overage_planner|[c]overage_server|[o]pennav_coverage|[m]ap_server|[n]avsat_to_pose|[n]avsat_to_absolute_pose|[d]iagnostics|[c]ontroller_server|[p]lanner_server|[s]moother_server|[b]t_navigator|[c]ollision_monitor|[v]elocity_smoother|[l]ifecycle_manager|[o]pennav_docking|[r]obot_state_publisher|[f]ake_hardware|[w]aypoint_follower|[b]ehavior_server|[s]im_navsat|[s]im_imu|[s]im_wheel|[c]og_to_imu|[d]ock_yaw|[c]ostmap_scan_filter|[s]can_deskew|[t]wist_mux|[o]bstacle_tracker|[m]ag_yaw_publisher|[s]tatic_transform_publisher|[w]heel_odometry_node|[c]alibrate_imu_yaw' | awk '{print $1}')
if [ -n "$ROS_PIDS" ]; then
  kill -9 $ROS_PIDS 2>/dev/null || true
  sleep 1
fi

# Clean DDS shared memory and stale Webots IPC sockets.
# /tmp/webots/<user>/<port>/ipc/<robot_name>/extern is the Unix domain
# socket the webots_controller_* connects to; if Webots crashed without
# cleaning up, the next launch sees the socket but no listener and
# stalls in "retrying for another N seconds".
rm -rf /dev/shm/cyclone* /dev/shm/dds* /dev/shm/iox*
rm -rf /tmp/webots/* 2>/dev/null || true

# Remove stale SLAM posegraph to prevent TF time-jump errors on next sim launch.
rm -f /ros2_ws/maps/garden_map.posegraph /ros2_ws/maps/garden_map.data

echo "All clean."
