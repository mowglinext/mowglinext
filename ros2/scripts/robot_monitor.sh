#!/usr/bin/env bash
# =============================================================================
# MowgliNext Robot Monitor
# Comprehensive real-time robot health check and diagnostics.
# Usage: ./robot_monitor.sh [--loop N]  (repeat every N seconds, default: once)
# =============================================================================

set -euo pipefail

LOOP_SEC="${1:-0}"
[[ "$LOOP_SEC" == "--loop" ]] && LOOP_SEC="${2:-30}"

ROS_SETUP="source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash"
CONTAINER="mowgli-ros2"

ros2cmd() {
  docker exec "$CONTAINER" bash -c "$ROS_SETUP && $1" 2>/dev/null
}

topic_once() {
  ros2cmd "timeout 5 ros2 topic echo $1 --once 2>/dev/null"
}

tf_echo() {
  ros2cmd "timeout 3 ros2 run tf2_ros tf2_echo $1 $2 2>&1" | grep -E "Translation|RPY.*degree" | tail -2
}

topic_rate() {
  ros2cmd "timeout 4 ros2 topic hz $1 --window 5 2>&1" | grep "average rate" | tail -1 | awk '{printf "%.1f", $3}'
}

BOLD='\033[1m'
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
CYAN='\033[36m'
RESET='\033[0m'

run_check() {
  local ts
  ts=$(date '+%Y-%m-%d %H:%M:%S')
  echo -e "\n${BOLD}${CYAN}═══════════════════════════════════════════════════════${RESET}"
  echo -e "${BOLD}  MowgliNext Robot Monitor — $ts${RESET}"
  echo -e "${BOLD}${CYAN}═══════════════════════════════════════════════════════${RESET}\n"

  # --- Container Status ---
  echo -e "${BOLD}[CONTAINERS]${RESET}"
  for c in mowgli-ros2 mowgli-gui mowgli-gps mowgli-lidar mowgli-mqtt; do
    status=$(docker inspect --format='{{.State.Status}}' "$c" 2>/dev/null || echo "missing")
    uptime=$(docker inspect --format='{{.State.StartedAt}}' "$c" 2>/dev/null | head -1)
    if [ "$status" = "running" ]; then
      echo -e "  ${GREEN}●${RESET} $c: running (since ${uptime:-?})"
    else
      echo -e "  ${RED}●${RESET} $c: $status"
    fi
  done

  # --- High Level Status ---
  echo -e "\n${BOLD}[BT STATE]${RESET}"
  local hl
  hl=$(topic_once "/behavior_tree_node/high_level_status")
  local bt_state bt_name emergency is_charging battery gps_quality
  bt_state=$(echo "$hl" | grep "^state:" | awk '{print $2}')
  bt_name=$(echo "$hl" | grep "state_name:" | awk '{print $2}')
  emergency=$(echo "$hl" | grep "emergency:" | awk '{print $2}')
  is_charging=$(echo "$hl" | grep "is_charging:" | awk '{print $2}')
  battery=$(echo "$hl" | grep "battery_percent:" | awk '{printf "%.1f", $2}')
  gps_quality=$(echo "$hl" | grep "gps_quality_percent:" | awk '{printf "%.0f", $2 * 100}')
  current_area=$(echo "$hl" | grep "current_area:" | awk '{print $2}')

  bt_name=$(echo "$bt_name" | tr -d '\n\r')
  echo "  State: $bt_name (id=$bt_state)"
  [ "$emergency" = "true" ] && echo -e "  ${RED}⚠ EMERGENCY ACTIVE${RESET}" || echo -e "  Emergency: ${GREEN}none${RESET}"
  echo "  Battery: ${battery}%  Charging: $is_charging"
  echo "  GPS Quality: ${gps_quality}%  Current Area: $current_area"

  # --- GPS ---
  echo -e "\n${BOLD}[GPS]${RESET}"
  local gps
  gps=$(topic_once "/gps/fix")
  local lat lon alt gps_status cov
  lat=$(echo "$gps" | grep "latitude:" | awk '{printf "%.9f", $2}')
  lon=$(echo "$gps" | grep "longitude:" | awk '{printf "%.9f", $2}')
  alt=$(echo "$gps" | grep "altitude:" | awk '{printf "%.1f", $2}')
  gps_status=$(echo "$gps" | grep "^  status:" | head -1 | awk '{print $2}')
  cov=$(echo "$gps" | grep "^- " | head -1 | awk '{printf "%.4f", $2}')

  case "$gps_status" in
    -1) gps_label="${RED}NO_FIX${RESET}" ;;
    0)  gps_label="${GREEN}FIX${RESET}" ;;
    1)  gps_label="${GREEN}RTK_FLOAT${RESET}" ;;
    2)  gps_label="${GREEN}RTK_FIXED${RESET}" ;;
    *)  gps_label="${YELLOW}UNKNOWN($gps_status)${RESET}" ;;
  esac
  echo -e "  Status: $gps_label  Accuracy: $(python3 -c "import math; print(f'{math.sqrt($cov)*100:.1f}cm')" 2>/dev/null || echo "${cov}m²")"
  echo "  Position: $lat, $lon  Alt: ${alt}m"

  # --- TF Chain (parallel lookups for speed) ---
  echo -e "\n${BOLD}[TF CHAIN]${RESET}"
  local tmpdir
  tmpdir=$(mktemp -d)
  tf_echo map base_footprint > "$tmpdir/mb" &
  tf_echo map odom > "$tmpdir/mo" &
  tf_echo odom base_footprint > "$tmpdir/ob" &
  wait
  local tf_mb tf_mo tf_ob
  tf_mb=$(cat "$tmpdir/mb")
  tf_mo=$(cat "$tmpdir/mo")
  tf_ob=$(cat "$tmpdir/ob")
  rm -rf "$tmpdir"

  local robot_pos robot_rpy
  robot_pos=$(echo "$tf_mb" | grep "Translation" | sed 's/.*\[/[/')
  robot_rpy=$(echo "$tf_mb" | grep "RPY.*degree" | sed 's/.*\[/[/')
  local slam_rpy fusion_rpy
  slam_rpy=$(echo "$tf_mo" | grep "RPY.*degree" | sed 's/.*\[/[/')
  fusion_rpy=$(echo "$tf_ob" | grep "RPY.*degree" | sed 's/.*\[/[/')

  echo "  map→base_footprint:  $robot_pos  RPY: $robot_rpy"
  echo "  map→odom (SLAM):     $(echo "$tf_mo" | grep "Translation" | sed 's/.*\[/[/')  RPY: $slam_rpy"
  echo "  odom→base (Fusion):  $(echo "$tf_ob" | grep "Translation" | sed 's/.*\[/[/')  RPY: $fusion_rpy"

  # Extract yaw for drift check
  local robot_yaw
  robot_yaw=$(echo "$robot_rpy" | awk -F',' '{gsub(/[\[\] ]/,"",$3); print $3}')
  echo "  Robot yaw: ${robot_yaw}°"

  # Roll/pitch check
  local roll pitch
  roll=$(echo "$robot_rpy" | awk -F',' '{gsub(/[\[\] ]/,"",$1); printf "%.2f", $1}')
  pitch=$(echo "$robot_rpy" | awk -F',' '{gsub(/[\[\] ]/,"",$2); printf "%.2f", $2}')
  if python3 -c "import sys; sys.exit(0 if abs($roll) < 5 and abs($pitch) < 5 else 1)" 2>/dev/null; then
    echo -e "  Flat constraint: ${GREEN}OK${RESET} (roll=${roll}° pitch=${pitch}°)"
  else
    echo -e "  Flat constraint: ${RED}DRIFT${RESET} (roll=${roll}° pitch=${pitch}°)"
  fi

  # --- EKF z-drift (should stay ~0 under two_d_mode) ---
  local odom
  odom=$(topic_once "/odometry/filtered_map")
  local odom_z
  odom_z=$(echo "$odom" | grep "z:" | head -1 | awk '{printf "%.2f", $2}')
  local map_z
  map_z=$(echo "$tf_mb" | grep "Translation" | sed 's/.*\[//' | sed 's/\]//' | awk -F',' '{gsub(/ /,"",$3); printf "%.2f", $3}')
  echo "  z-drift: odom=${odom_z}m  map=${map_z}m"

  # --- Coverage ---
  echo -e "\n${BOLD}[COVERAGE]${RESET}"
  for i in 0 1 2 3; do
    local cov_result
    cov_result=$(ros2cmd "ros2 service call /map_server_node/get_coverage_status mowgli_interfaces/srv/GetCoverageStatus '{area_index: $i}' 2>/dev/null")
    local success
    success=$(echo "$cov_result" | grep "success=" | sed 's/.*success=//' | sed 's/,.*//')
    if [ "$success" = "True" ]; then
      local pct total mowed obs strips
      pct=$(echo "$cov_result" | grep "coverage_percent=" | sed 's/.*coverage_percent=//' | sed 's/,.*//' | awk '{printf "%.1f", $1}')
      total=$(echo "$cov_result" | grep "total_cells=" | sed 's/.*total_cells=//' | sed 's/,.*//')
      mowed=$(echo "$cov_result" | grep "mowed_cells=" | sed 's/.*mowed_cells=//' | sed 's/,.*//')
      obs=$(echo "$cov_result" | grep "obstacle_cells=" | sed 's/.*obstacle_cells=//' | sed 's/,.*//')
      strips=$(echo "$cov_result" | grep "strips_remaining=" | sed 's/.*strips_remaining=//' | sed 's/)//')
      echo "  Area $i: ${pct}% done | ${mowed}/${total} cells | ${obs} obstacles | ${strips} strips left"
    else
      break
    fi
  done

  # --- Robot in Grid ---
  echo -e "\n${BOLD}[GRID POSITION]${RESET}"
  local grid_info robot_x robot_y grid_ox grid_oy grid_w grid_h grid_res
  grid_info=$(topic_once "/map_server_node/coverage_cells --no-arr")
  robot_x=$(echo "$tf_mb" | grep "Translation" | sed 's/.*\[//' | sed 's/\]//' | awk -F',' '{gsub(/ /,"",$1); print $1}')
  robot_y=$(echo "$tf_mb" | grep "Translation" | sed 's/.*\[//' | sed 's/\]//' | awk -F',' '{gsub(/ /,"",$2); print $2}')
  grid_ox=$(echo "$grid_info" | grep -A5 "origin:" | grep "x:" | head -1 | awk '{printf "%.3f", $2}')
  grid_oy=$(echo "$grid_info" | grep -A5 "origin:" | grep "y:" | head -1 | awk '{printf "%.3f", $2}')
  grid_w=$(echo "$grid_info" | grep "width:" | awk '{print $2}')
  grid_h=$(echo "$grid_info" | grep "height:" | awk '{print $2}')
  grid_res=$(echo "$grid_info" | grep "resolution:" | awk '{printf "%.3f", $2}')

  if [ -n "$robot_x" ] && [ -n "$grid_ox" ] && [ -n "$grid_w" ] && [ -n "$grid_res" ]; then
    python3 -c "
rx, ry = $robot_x, $robot_y
ox, oy = $grid_ox, $grid_oy
w, h, res = $grid_w, $grid_h, $grid_res
gx_max = ox + w * res
gy_max = oy + h * res
cell_x = int((rx - ox) / res)
cell_y = int((ry - oy) / res)
inside = ox <= rx <= gx_max and oy <= ry <= gy_max
status = '${GREEN}INSIDE${RESET}' if inside else '${RED}OUTSIDE${RESET}'
print(f'  Robot: ({rx:.2f}, {ry:.2f})  Cell: ({cell_x}, {cell_y})  {status}')
print(f'  Grid: [{ox:.2f}, {gx_max:.2f}] x [{oy:.2f}, {gy_max:.2f}]')
" 2>/dev/null
  else
    echo "  Could not determine grid position"
  fi

  # --- Sensor Rates (check publishers exist, skip hz measurement for speed) ---
  echo -e "\n${BOLD}[SENSORS]${RESET}"
  local topic_list
  topic_list=$(ros2cmd "ros2 topic list 2>/dev/null")
  for topic in /odometry/filtered_map /odometry/filtered /gps/fix /imu/data /scan /wheel_odom /gnss/heading; do
    if echo "$topic_list" | grep -q "^${topic}$"; then
      echo -e "  ${GREEN}●${RESET} $topic"
    else
      echo -e "  ${RED}●${RESET} $topic — NOT FOUND"
    fi
  done

  # --- SLAM ---
  echo -e "\n${BOLD}[SLAM]${RESET}"
  local slam_info
  slam_info=$(topic_once "/map --no-arr")
  local map_w map_h map_res
  map_w=$(echo "$slam_info" | grep "width:" | awk '{print $2}')
  map_h=$(echo "$slam_info" | grep "height:" | awk '{print $2}')
  map_res=$(echo "$slam_info" | grep "resolution:" | awk '{printf "%.2f", $2}')
  echo "  Map: ${map_w:-?}x${map_h:-?} cells @ ${map_res:-?}m"

  # --- Errors ---
  echo -e "\n${BOLD}[RECENT ERRORS]${RESET}"
  local err_count
  err_count=$(docker logs --since 60s "$CONTAINER" 2>&1 | grep -ciE "\[error\]|\[fatal\]" || true)
  err_count="${err_count:-0}"
  if [ "$err_count" -gt 0 ]; then
    echo -e "  ${RED}$err_count errors in last 60s:${RESET}"
    docker logs --since 60s "$CONTAINER" 2>&1 | grep -iE "\[error\]|\[fatal\]" | tail -5 | sed 's/^/    /'
  else
    echo -e "  ${GREEN}No errors in last 60s${RESET}"
  fi

  # --- TF Conflicts ---
  local tf_warn
  tf_warn=$(docker logs --since 60s "$CONTAINER" 2>&1 | grep -ciE "multiple authority|already authority|transform.*error" || true)
  tf_warn="${tf_warn:-0}"
  if [ "$tf_warn" -gt 0 ]; then
    echo -e "  ${RED}TF conflicts detected: $tf_warn${RESET}"
  else
    echo -e "  ${GREEN}No TF conflicts${RESET}"
  fi

  echo -e "\n${CYAN}───────────────────────────────────────────────────────${RESET}"
}

# Run once or loop
run_check
if [ "$LOOP_SEC" -gt 0 ] 2>/dev/null; then
  while true; do
    sleep "$LOOP_SEC"
    run_check
  done
fi
