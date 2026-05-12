#!/usr/bin/env bash

check_devices() {
  step "Check: Hardware devices"

  : "${GPS_PORT:=/dev/gps}"
  : "${LIDAR_PORT:=/dev/lidar}"
  : "${LIDAR_TYPE:=unknown}"
  : "${LIDAR_ENABLED:=true}"

  local devices=()

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    devices+=("${MAVROS_PORT:-/dev/mavros}:Pixhawk MAVROS serial")
  else
    devices+=("/dev/mowgli:Mowgli STM32 board")
    devices+=("${GPS_PORT}:GPS receiver")
  fi

  if [[ "${LIDAR_ENABLED}" == "true" && "${LIDAR_TYPE}" != "none" ]]; then
    devices+=("${LIDAR_PORT}:LiDAR device")
  fi

  for dev_info in "${devices[@]}"; do
    local dev="${dev_info%%:*}"
    local name="${dev_info#*:}"

    if [ -e "$dev" ]; then
      if [ -L "$dev" ]; then
        local target
        target="$(readlink -f "$dev")"
        info "$name ($dev -> $target)"
      else
        info "$name ($dev)"
      fi
    else
      fail "$name ($dev) — not found"

      case "$dev" in
        /dev/mowgli)
          add_issue "Mowgli board not detected. Flash the Mowgli firmware to the STM32 board and connect it via USB."
          echo -e "       ${DIM}Firmware: https://github.com/cedbossneo/Mowgli${NC}"
          echo -e "       ${DIM}Flash with: STM32CubeProgrammer or st-flash${NC}"
          ;;
        *)
          local uart_dev=""
          local sensor_name=""
          if [[ "$dev" == "$GPS_PORT" ]]; then
            uart_dev="${GPS_UART_DEVICE:-}"
            sensor_name="GPS"
          elif [[ "$dev" == "$LIDAR_PORT" ]]; then
            uart_dev="${LIDAR_UART_DEVICE:-}"
            sensor_name="LiDAR (${LIDAR_TYPE})"
          fi

          if [[ -n "$uart_dev" && ! -e "$uart_dev" ]]; then
            # UART device itself doesn't exist — likely needs reboot for dtoverlay
            warn "$sensor_name: UART device $uart_dev not available yet — reboot required"
            add_issue "$sensor_name UART $uart_dev not found. Reboot to enable UART overlays, then re-check: sudo reboot"
          elif [[ -n "$uart_dev" ]]; then
            # UART device exists but symlink missing — udev rule issue
            add_issue "$sensor_name symlink $dev not found but $uart_dev exists. Check udev rules: cat /etc/udev/rules.d/50-mowgli.rules"
            echo -e "       ${DIM}Try: sudo udevadm control --reload-rules && sudo udevadm trigger${NC}"
          else
            add_issue "$sensor_name device $dev not detected. Check connection and docker/.env configuration."
          fi
          ;;
      esac
    fi
  done

  if [[ "${HARDWARE_BACKEND:-mowgli}" != "mavros" && "${GPS_CONNECTION:-}" == "uart" && -L "${GPS_PORT}" && -n "${GPS_UART_DEVICE:-}" ]]; then
    local gps_target
    gps_target="$(readlink -f "$GPS_PORT")"
    if [[ "$gps_target" != "$GPS_UART_DEVICE" ]]; then
      warn "GPS symlink mismatch: $GPS_PORT -> $gps_target (expected $GPS_UART_DEVICE)"
      add_issue "GPS symlink $GPS_PORT points to $gps_target instead of $GPS_UART_DEVICE. Re-run installer or fix udev rules."
    fi
  fi

  if [[ "${HARDWARE_BACKEND:-mowgli}" != "mavros" && "${GPS_CONNECTION:-}" == "usb" && -L "${GPS_PORT}" && -n "${GPS_BY_ID:-}" && -e "${GPS_BY_ID}" ]]; then
    local gps_target
    local gps_expected
    gps_target="$(readlink -f "$GPS_PORT")"
    gps_expected="$(readlink -f "$GPS_BY_ID")"
    if [[ "$gps_target" != "$gps_expected" ]]; then
      warn "GPS symlink mismatch: $GPS_PORT -> $gps_target (expected $gps_expected from $GPS_BY_ID)"
      add_issue "GPS symlink $GPS_PORT points to $gps_target instead of the selected USB device $GPS_BY_ID. Re-run installer or fix udev rules."
    fi
  fi

  if [[ "${LIDAR_CONNECTION:-}" == "uart" && -L "${LIDAR_PORT}" && -n "${LIDAR_UART_DEVICE:-}" ]]; then
    local lidar_target
    lidar_target="$(readlink -f "$LIDAR_PORT")"
    if [[ "$lidar_target" != "$LIDAR_UART_DEVICE" ]]; then
      warn "LiDAR symlink mismatch: $LIDAR_PORT -> $lidar_target (expected $LIDAR_UART_DEVICE)"
      add_issue "LiDAR symlink $LIDAR_PORT points to $lidar_target instead of $LIDAR_UART_DEVICE. Re-run installer or fix udev rules."
    fi
  fi
}

check_containers() {
  step "Check: Containers"

  : "${LIDAR_ENABLED:=true}"
  : "${LIDAR_TYPE:=unknown}"
  : "${GNSS_BACKEND:=gps}"

  local services=(mowgli gui mosquitto)

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    services+=(mavros ntrip)
  else
    case "${GNSS_BACKEND}" in
      gps)     services+=(gps) ;;
      ublox)   services+=(gnss_ublox) ;;
      unicore) services+=(gnss_unicore) ;;
      *)
        fail "Unknown GNSS_BACKEND=${GNSS_BACKEND}"
        add_issue "Unknown GNSS_BACKEND=${GNSS_BACKEND}. Re-run installer."
        ;;
    esac
  fi

  if [[ "${LIDAR_ENABLED}" == "true" && "${LIDAR_TYPE}" != "none" ]]; then
    services+=(lidar)
  fi

  if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]]; then
    services+=(tfluna_front)
  fi

  if [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]]; then
    services+=(tfluna_edge)
  fi

  for svc in "${services[@]}"; do
    local container=""
    case "$svc" in
      mowgli)       container="mowgli-ros2" ;;
      gps)          container="mowgli-gps" ;;
      gnss_ublox)   container="mowgli-gps" ;;
      gnss_unicore) container="mowgli-gps" ;;
      lidar)        container="mowgli-lidar" ;;
      gui)          container="mowgli-gui" ;;
      mosquitto)    container="mowgli-mqtt" ;;
      mavros)       container="mowgli-mavros" ;;
      ntrip)        container="mowgli-ntrip" ;;
      tfluna_front) container="mowgli-tfluna-front" ;;
      tfluna_edge)  container="mowgli-tfluna-edge" ;;
    esac

    local status
    status="$(docker inspect -f '{{.State.Status}}' "$container" 2>/dev/null || echo "missing")"

    if [[ "$status" == "running" ]]; then
      local uptime
      uptime="$(docker inspect -f '{{.State.StartedAt}}' "$container" 2>/dev/null | cut -dT -f2 | cut -d. -f1)"
      info "$svc ($container) — running since $uptime"
    else
      fail "$svc ($container) — $status"
      if [[ "$status" == "missing" ]]; then
        add_issue "Container $container not found."
      else
        add_issue "Container $container is $status. Check logs: docker logs $container --tail 30"
      fi
    fi
  done

  if docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    local dead_nodes
    dead_nodes=$(
      docker logs mowgli-ros2 --tail 200 2>&1 \
        | grep -oP "process has died.*cmd '([^']+)'" \
        | grep -oP "(?<=cmd ')[^']+" \
        | xargs -r -I{} basename {} 2>/dev/null \
        | sort -u || true
    )

    if [[ -n "$dead_nodes" ]]; then
      warn "Crashed nodes inside mowgli-ros2:"
      while read -r node; do
        [[ -n "$node" ]] && echo -e "       ${RED}$node${NC}"
      done <<< "$dead_nodes"
      add_issue "Some ROS nodes crashed inside mowgli-ros2. Check: docker logs mowgli-ros2 | grep 'process has died'"
    fi
  fi
}

check_firmware() {
  step "Check: Mowgli firmware"

  if ! docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    warn "mowgli-ros2 not running — skipping firmware check"
    return
  fi

  local status_data
  status_data="$(
    docker exec mowgli-ros2 bash -lc \
      "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /hardware_bridge/status --once 2>/dev/null" \
      2>/dev/null || echo ""
  )"

  if [[ -z "$status_data" ]]; then
    fail "No data on /hardware_bridge/status — hardware bridge cannot communicate with Mowgli board"
    add_issue "Mowgli firmware not responding. Ensure the STM32 is flashed with Mowgli firmware and /dev/mowgli is accessible."
    echo -e "       ${DIM}Flash firmware: https://github.com/cedbossneo/Mowgli${NC}"
    echo -e "       ${DIM}Check serial: docker logs mowgli-ros2 | grep hardware_bridge${NC}"
  else
    local mower_status
    mower_status="$(echo "$status_data" | grep "mower_status:" | awk '{print $2}' | head -1)"

    if [[ "$mower_status" == "255" ]]; then
      warn "Firmware reporting mower_status=255 (not initialised)"
      add_issue "Mowgli board is connected but reporting uninitialised state. Press the power button on the mower or check the firmware."
    else
      info "Firmware responding — mower_status=${mower_status:-unknown}"
    fi

    local charging esc_power
    charging="$(echo "$status_data" | grep "is_charging:" | awk '{print $2}' | head -1)"
    esc_power="$(echo "$status_data" | grep "esc_power:" | awk '{print $2}' | head -1)"
    echo -e "       ${DIM}Charging: ${charging:-?} | ESC power: ${esc_power:-?}${NC}"
  fi
}

check_gps() {
  step "Check: GPS"

  : "${GNSS_BACKEND:=gps}"

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    info "MAVROS backend: GPS is handled through Pixhawk/MAVROS"

    local gps_status
    gps_status="$(docker inspect -f '{{.State.Status}}' mowgli-gps 2>/dev/null || echo "missing")"
    if [[ "$gps_status" == "running" ]]; then
      fail "mowgli-gps is running in MAVROS mode"
      add_issue "mowgli-gps must not run when HARDWARE_BACKEND=mavros. Regenerate docker/docker-compose.yaml and restart the stack."
    else
      info "Direct GPS container disabled (${gps_status})"
    fi

    local mavros_status
    mavros_status="$(docker inspect -f '{{.State.Status}}' mowgli-mavros 2>/dev/null || echo "missing")"
    if [[ "$mavros_status" != "running" ]]; then
      fail "mowgli-mavros is ${mavros_status}"
      add_issue "MAVROS backend selected but mowgli-mavros is not running. Check docker logs mowgli-mavros --tail 50."
      return
    fi
    info "MAVROS container running"

    local mavros_state
    mavros_state="$(
      docker exec mowgli-ros2 bash -lc \
        "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /mavros/state --once 2>/dev/null" \
        2>/dev/null || echo ""
    )"
    if [[ -z "$mavros_state" ]]; then
      fail "No MAVROS state on /mavros/state"
      add_issue "MAVROS is running but /mavros/state has no data. Check Pixhawk serial connection, MAVROS_PORT, MAVROS_BAUD, and docker logs mowgli-mavros --tail 50."
    else
      info "MAVROS state available on /mavros/state"
    fi

    local mavros_global
    mavros_global="$(
      docker exec mowgli-ros2 bash -lc \
        "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /mavros/global_position/global --once 2>/dev/null" \
        2>/dev/null || echo ""
    )"
    if [[ -z "$mavros_global" ]]; then
      fail "No MAVROS global position on /mavros/global_position/global"
      add_issue "MAVROS is not publishing global GPS position. Check Pixhawk GPS lock and MAVROS global_position plugin."
    else
      info "MAVROS global position available"
    fi

    local rtcm_info
    rtcm_info="$(
      docker exec mowgli-ros2 bash -lc \
        "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic info /rtcm 2>/dev/null" \
        2>/dev/null || echo ""
    )"
    if echo "$rtcm_info" | grep -q "Publisher count: [1-9]"; then
      info "RTCM topic has publisher(s)"
    else
      warn "No RTCM publisher detected on /rtcm"
      add_issue "No RTCM publisher on /rtcm in MAVROS mode. Check mowgli-ntrip logs and NTRIP configuration."
    fi
    return
  fi

  local gps_container="mowgli-gps"
  case "${GNSS_BACKEND}" in
    gps|ublox|unicore) ;;
    *)
      fail "Unknown GNSS_BACKEND=${GNSS_BACKEND}"
      add_issue "Unknown GNSS_BACKEND=${GNSS_BACKEND}. Re-run installer."
      return
      ;;
  esac

  if ! docker inspect -f '{{.State.Status}}' "$gps_container" 2>/dev/null | grep -q running; then
    warn "$gps_container not running — skipping GPS check"
    return
  fi

  local fix_data
  fix_data="$(
    docker exec mowgli-ros2 bash -lc \
      "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /gps/fix --once 2>/dev/null" \
      2>/dev/null || echo ""
  )"

  if [[ -z "$fix_data" ]]; then
    fail "No GPS fix data on /gps/fix"
    add_issue "GPS not publishing. Check: docker logs $gps_container --tail 30"
    return
  fi

  local lat lon status_val cov
  lat="$(echo "$fix_data" | grep "latitude:" | awk '{print $2}' | head -1)"
  lon="$(echo "$fix_data" | grep "longitude:" | awk '{print $2}' | head -1)"
  status_val="$(echo "$fix_data" | grep -A1 "status:" | grep "status:" | tail -1 | awk '{print $2}')"
  cov="$(echo "$fix_data" | grep -m1 "^- " | awk '{print $2}')"

  local accuracy="0"
  if [[ -n "$cov" ]]; then
    accuracy="$(echo "$cov" | awk '{printf "%.2f", sqrt($1)}')"
  fi

  local acc_num="0"
  acc_num="$(echo "$accuracy" | awk '{printf "%d", $1 * 100}')"

  if [[ "$status_val" == "2" ]] || [[ "$acc_num" -le 5 && "$acc_num" -gt 0 ]]; then
    info "GPS: RTK FIXED — ${accuracy}m accuracy (lat=$lat lon=$lon)"
  elif [[ "$status_val" == "1" ]] || [[ "$acc_num" -le 20 && "$acc_num" -gt 0 ]]; then
    info "GPS: RTK FLOAT — ${accuracy}m accuracy (lat=$lat lon=$lon)"
  elif [[ "$status_val" == "0" ]]; then
    warn "GPS: Standard fix — ${accuracy}m accuracy (no RTK corrections)"
  else
    fail "GPS: No fix (status=$status_val)"
  fi

  if [[ "${GNSS_BACKEND}" != "gps" ]]; then
    info "GNSS backend ${GNSS_BACKEND}: direct /gps/fix check passed"
  fi

  local ntrip_logs
  ntrip_logs="$(docker logs "$gps_container" --tail 80 2>&1 || true)"

  if echo "$ntrip_logs" | grep -q "Connected to http"; then
    local ntrip_url
    ntrip_url="$(echo "$ntrip_logs" | grep -oP "Connected to \K[^ ]+" | tail -1)"
    info "NTRIP connected: $ntrip_url"
  elif echo "$ntrip_logs" | grep -q "Unable to connect"; then
    fail "NTRIP connection failed"
    add_issue "NTRIP cannot connect. Check ntrip_host and ntrip_mountpoint in docker/config/mowgli/mowgli_robot.yaml"
  elif echo "$ntrip_logs" | grep -q "Network is unreachable"; then
    fail "NTRIP: network unreachable"
    add_issue "No internet connection for NTRIP. Check your network configuration."
  fi

  if echo "$ntrip_logs" | grep -q "Forwarded.*RTCM messages"; then
    local rtcm_count
    rtcm_count="$(echo "$ntrip_logs" | grep -oP "Forwarded \K\d+" | tail -1)"
    info "RTCM bridge: $rtcm_count corrections forwarded to GPS"
  elif echo "$ntrip_logs" | grep -q "NTRIP enabled: true"; then
    warn "RTCM bridge not forwarding yet — RTK may take a few minutes to converge"
  fi

  local datum_lat datum_lon
  datum_lat="$(grep "datum_lat:" "$DOCKER_DIR/config/mowgli/mowgli_robot.yaml" 2>/dev/null | head -1 | awk '{print $2}')"
  datum_lon="$(grep "datum_lon:" "$DOCKER_DIR/config/mowgli/mowgli_robot.yaml" 2>/dev/null | head -1 | awk '{print $2}')"

  if [[ "${datum_lat:-0}" == "0" || "${datum_lat:-0.0}" == "0.0" || "${datum_lon:-0}" == "0" || "${datum_lon:-0.0}" == "0.0" || -z "$datum_lat" ]]; then
    fail "GPS datum is not set — robot position will be wrong"

    if [[ -n "$lat" && "$lat" != "0.0" ]]; then
      echo -e "       ${DIM}GPS has a fix at: $lat, $lon${NC}"
      if confirm "Set datum to current GPS position ($lat, $lon)?"; then
        local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
        sed -i "s/datum_lat:.*/datum_lat: $lat/" "$yaml_file"
        sed -i "s/datum_lon:.*/datum_lon: $lon/" "$yaml_file"
        info "Datum set to $lat, $lon in $yaml_file"
        info "Restart mowgli to apply: docker compose -f $FINAL_COMPOSE_FILE --env-file $FINAL_ENV_FILE restart gps mowgli"
      else
        add_issue "Set datum_lat and datum_lon in docker/config/mowgli/mowgli_robot.yaml to your docking station coordinates"
      fi
    else
      add_issue "Set datum_lat and datum_lon in docker/config/mowgli/mowgli_robot.yaml to your docking station coordinates"
    fi
  else
    info "Datum: $datum_lat, $datum_lon"
  fi
}

check_lidar() {
  step "Check: LiDAR"

  : "${LIDAR_ENABLED:=true}"
  : "${LIDAR_PORT:=/dev/lidar}"
  : "${LIDAR_TYPE:=unknown}"
  : "${LIDAR_BAUD:=?}"

  if [[ "${LIDAR_ENABLED}" != "true" || "${LIDAR_TYPE}" == "none" ]]; then
    info "LiDAR disabled — skipping LiDAR checks"
    return
  fi

  info "LiDAR config: type=${LIDAR_TYPE} port=${LIDAR_PORT} baud=${LIDAR_BAUD}"

  if ! docker inspect -f '{{.State.Status}}' mowgli-lidar 2>/dev/null | grep -q running; then
    warn "mowgli-lidar not running — skipping LiDAR check"
    return
  fi

  local scan_check
  scan_check="$(
    docker exec mowgli-ros2 bash -lc \
      "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic info /scan 2>/dev/null" \
      2>/dev/null || echo ""
  )"

  local pub_count
  pub_count="$(echo "$scan_check" | grep "Publisher count:" | awk '{print $3}')"

  if [[ "${pub_count:-0}" -ge 1 ]] 2>/dev/null; then
    info "LiDAR publishing on /scan ($pub_count publisher)"
  else
    fail "No publisher on /scan — LiDAR data not reaching ROS"
    add_issue "LiDAR not publishing. Check: docker logs mowgli-lidar --tail 20"
    echo -e "       ${DIM}Expected serial port: ${LIDAR_PORT}${NC}"
  fi
}

# check_slam() — REMOVED. SLAM (slam_toolbox, Cartographer, etc.) has been
# removed from the MowgliNext stack. The map frame is provided by
# navsat_transform + RTK GPS via robot_localization's dual EKF.
# See CLAUDE.md "Architecture Invariants" for details.

check_rangefinders() {
  step "Check: Rangefinders"

  if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]]; then
    if [ -e "${TFLUNA_FRONT_PORT:-/dev/tfluna_front}" ]; then
      info "TF-Luna front detected (${TFLUNA_FRONT_PORT})"
    else
      fail "TF-Luna front not detected (${TFLUNA_FRONT_PORT})"
      add_issue "TF-Luna front not detected. Check selected UART and TFLUNA_FRONT_PORT in docker/.env."
    fi
  fi

  if [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]]; then
    if [ -e "${TFLUNA_EDGE_PORT:-/dev/tfluna_edge}" ]; then
      info "TF-Luna edge detected (${TFLUNA_EDGE_PORT})"
    else
      fail "TF-Luna edge not detected (${TFLUNA_EDGE_PORT})"
      add_issue "TF-Luna edge not detected. Check selected UART and TFLUNA_EDGE_PORT in docker/.env."
    fi
  fi
}

check_gui() {
  step "Check: GUI & connectivity"

  if ! docker inspect -f '{{.State.Status}}' mowgli-gui 2>/dev/null | grep -q running; then
    fail "mowgli-gui not running"
    add_issue "GUI container not running."
    return
  fi

  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")"

  if curl -sf -o /dev/null --connect-timeout 3 "http://$ip:4006" 2>/dev/null || \
     curl -sf -o /dev/null --connect-timeout 3 "http://localhost:4006" 2>/dev/null; then
    info "GUI accessible at http://$ip"
  else
    warn "GUI might be starting up — try http://$ip in your browser"
  fi

  local fg_info
  fg_info="$(
    docker exec mowgli-ros2 bash -lc \
      "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list 2>/dev/null" \
      2>/dev/null | grep foxglove_bridge || echo ""
  )"

  if [[ -n "$fg_info" ]]; then
    info "Foxglove Bridge WebSocket active (ws://$ip:8765)"
  else
    warn "Foxglove Bridge node not found — GUI may not receive live data"
  fi

  echo ""
  echo -e "  ${BOLD}Access points:${NC}"
  echo -e "    GUI:       ${CYAN}http://$ip${NC}"
  echo -e "    Foxglove:  ${CYAN}ws://$ip:8765${NC}"
  echo -e "    Rosbridge: ${CYAN}ws://$ip:9090${NC}"
  echo -e "    MQTT:      ${CYAN}$ip:1883${NC}"
}
