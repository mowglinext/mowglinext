#!/bin/bash
# sim-stop.sh — Gracefully stop all ROS2/Gazebo processes and clean DDS state.
#
# 1. Send SIGINT to ros2 launch processes (triggers graceful shutdown of all nodes)
# 2. Wait up to 5s for children to exit
# 3. SIGKILL any stragglers
# 4. Clean Cyclone DDS shared memory and Gazebo transport state
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

# Kill any remaining Gazebo processes (they survive ros2 launch SIGINT)
GZ_PIDS=$(ps -eo pid,args | grep '[g]z sim' | awk '{print $1}')
if [ -n "$GZ_PIDS" ]; then
  echo "Killing Gazebo processes..."
  kill -9 $GZ_PIDS 2>/dev/null || true
  sleep 1
fi

# Kill ALL remaining ROS2 nodes (not just launch) to prevent stale DDS participants.
# topic_tools/relay added 2026-05-05: sim_full_system.launch.py spawns two relays
# (/wheel_odom→/mowgli/hardware/wheel_odom and /imu/data→/mowgli/hardware/imu)
# which the launch SIGINT handler doesn't always reach, especially after a
# kill -9 on the parent. Without explicit cleanup they accumulate at ~5-6 % CPU
# each across restarts; 14 sim launches in one session left 59 orphaned relays
# and pushed load average to 106, freezing the next sim.
ROS_PIDS=$(ps -eo pid,args | grep -E '[r]os2|[g]z_ros2|[p]arameter_bridge|[f]oxglove|[e]kf_node|[s]lam_toolbox|[b]ehavior_tree|[c]overage_planner|[m]ap_server|[n]avsat_to_pose|[d]iagnostics|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[c]ollision_monitor|[v]elocity_smoother|[l]ifecycle_manager|[o]pennav_docking|[r]obot_state_publisher|[f]ake_hardware|[t]opic_tools/relay|[c]og_to_imu|[m]ag_yaw|[c]ostmap_scan|[s]im_imu_noise|[s]im_navsat_rtk|[s]im_wheel_slip|[s]im_cmd_vel|[c]alibrate_imu|[l]ocalization_monitor|[w]ait_for_map|[d]ock_yaw_to_set|[o]bstacle_tracker|[w]aypoint_follower|[s]moother_server|[b]ehavior_server' | awk '{print $1}')
if [ -n "$ROS_PIDS" ]; then
  kill -9 $ROS_PIDS 2>/dev/null || true
  sleep 1
fi

# Clean DDS shared memory and Gazebo transport state
rm -rf /dev/shm/cyclone* /dev/shm/dds* /dev/shm/iox* /tmp/gz-* /tmp/ign-*

# Remove stale SLAM posegraph to prevent TF time-jump errors on next sim launch.
rm -f /ros2_ws/maps/garden_map.posegraph /ros2_ws/maps/garden_map.data

# Ensure Xvfb is running on :99 for headless Gazebo rendering.
# Start it here (not in the launch file) so it's ready before Gazebo starts,
# allowing /clock to publish before other nodes — preventing TF time jumps.
if command -v Xvfb >/dev/null 2>&1; then
  if ! pgrep -f "Xvfb :99" >/dev/null 2>&1; then
    rm -f /tmp/.X99-lock
    Xvfb :99 -screen 0 1024x768x24 -ac &>/dev/null &
    sleep 0.5
  fi
  export DISPLAY=:99
fi

echo "All clean."
