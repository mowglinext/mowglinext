#!/usr/bin/env bash

container_name_for_service() {
  local svc="${1:-}"

  case "$svc" in
    mowgli)       printf 'mowgli-ros2\n' ;;
    gps)          printf 'mowgli-gps\n' ;;
    lidar)        printf 'mowgli-lidar\n' ;;
    gui)          printf 'mowgli-gui\n' ;;
    mosquitto)    printf 'mowgli-mqtt\n' ;;
    mavros)       printf 'mowgli-mavros\n' ;;
    ntrip)        printf 'mowgli-ntrip\n' ;;
    vesc)         printf 'mowgli-vesc\n' ;;
    tfluna_front) printf 'mowgli-tfluna-front\n' ;;
    tfluna_edge)  printf 'mowgli-tfluna-edge\n' ;;
    *)            return 1 ;;
  esac
}

print_logs_command_for_container() {
  local container="${1:?print_logs_command_for_container: missing container}"
  local tail_lines="${2:-30}"

  printf 'docker logs %q --tail %q' "$container" "$tail_lines"
}

expected_runtime_services() {
  : "${LIDAR_ENABLED:=true}"
  : "${LIDAR_TYPE:=unknown}"
  : "${GNSS_BACKEND:=gps}"

  local services=(mowgli gui mosquitto)
  local gnss_backend
  local gnss_stack
  local gnss_service

  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
  gnss_stack="$(effective_gnss_stack 2>/dev/null || true)"

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    services+=(mavros ntrip)
  else
    if ! is_supported_gnss_backend "$gnss_backend"; then
      return 1
    fi

    if [[ "$gnss_stack" != "disabled" ]]; then
      gnss_service="$(compose_gnss_service_name "$gnss_backend" 2>/dev/null || true)"
      [ -n "$gnss_service" ] && services+=("$gnss_service")
    fi
  fi

  if [[ "${LIDAR_ENABLED}" == "true" && "${LIDAR_TYPE}" != "none" ]]; then
    services+=(lidar)
  fi

  if effective_tfluna_front_enabled; then
    services+=(tfluna_front)
  fi

  if effective_tfluna_edge_enabled; then
    services+=(tfluna_edge)
  fi

  if effective_vesc_enabled; then
    services+=(vesc)
  fi

  printf '%s\n' "${services[@]}"
}

check_devices() {
  step "Check: Hardware devices"

  : "${LIDAR_PORT:=/dev/lidar}"
  : "${LIDAR_TYPE:=unknown}"
  : "${LIDAR_ENABLED:=true}"

  local devices=()
  local gnss_backend
  local gnss_device
  local gnss_connection

  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
  gnss_device="$(gnss_serial_device_from_state)"
  gnss_connection="$(gnss_connection_from_serial_device "$gnss_device")"

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    devices+=("${MAVROS_PORT:-/dev/mavros}:Pixhawk MAVROS serial")
  else
    devices+=("/dev/mowgli:Mowgli STM32 board")
    devices+=("${gnss_device}:GPS receiver")
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
          if [[ "$dev" == "$gnss_device" ]]; then
            if [[ "$gnss_connection" == "uart" ]]; then
              uart_dev="$gnss_device"
            fi
            sensor_name="GPS"
          elif [[ "$dev" == "$LIDAR_PORT" ]]; then
            uart_dev="${LIDAR_UART_DEVICE:-}"
            sensor_name="LiDAR (${LIDAR_TYPE})"
          fi

          if [[ -n "$uart_dev" && ! -e "$uart_dev" ]]; then
            # UART device itself doesn't exist — likely needs reboot for dtoverlay
            warn "$sensor_name: UART device $uart_dev not available yet — reboot required"
            add_issue "$sensor_name UART $uart_dev not found. Reboot to enable UART overlays, then re-check with: $(rerun_check_command)"
          elif [[ -n "$uart_dev" ]]; then
            # UART device exists but symlink missing — udev rule issue
            add_issue "$sensor_name symlink $dev not found but $uart_dev exists. Check udev rules: cat /etc/udev/rules.d/50-mowgli.rules. Then re-run: $(rerun_check_command)"
            echo -e "       ${DIM}Try: sudo udevadm control --reload-rules && sudo udevadm trigger${NC}"
          else
            add_issue "$sensor_name device $dev not detected. Check connection and docker/.env configuration."
          fi
          ;;
      esac
    fi
  done

  if [[ "${LIDAR_CONNECTION:-}" == "uart" && -L "${LIDAR_PORT}" && -n "${LIDAR_UART_DEVICE:-}" ]]; then
    local lidar_target
    lidar_target="$(readlink -f "$LIDAR_PORT")"
    if [[ "$lidar_target" != "$LIDAR_UART_DEVICE" ]]; then
      warn "LiDAR symlink mismatch: $LIDAR_PORT -> $lidar_target (expected $LIDAR_UART_DEVICE)"
      add_issue "LiDAR symlink $LIDAR_PORT points to $lidar_target instead of $LIDAR_UART_DEVICE. Re-run $(installer_main_command) or fix udev rules."
    fi
  fi
}

yaml_gps_value() {
  local file="$1"
  local key="$2"
  local line value

  line="$(grep -m1 -E "^[[:space:]]+${key}:" "$file" 2>/dev/null || true)"
  value="${line#*:}"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%%#*}"
  value="${value%"${value##*[![:space:]]}"}"
  value="${value#\"}"
  value="${value%\"}"
  printf '%s\n' "$value"
}

check_generated_gps_yaml_alignment() {
  step "Check: Generated GPS YAML"

  local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  local yaml_receiver_family yaml_serial_device yaml_serial_baud
  local expected_receiver_family expected_serial_device expected_serial_baud

  if [[ ! -f "$yaml_file" ]]; then
    fail "Generated mowgli_robot.yaml missing"
    add_issue "Missing $yaml_file. Re-run $(installer_main_command) to regenerate the ROS config from docker/.env."
    return
  fi

  yaml_receiver_family="$(yaml_gps_value "$yaml_file" gnss_receiver_family)"
  yaml_serial_device="$(yaml_gps_value "$yaml_file" gnss_serial_device)"
  yaml_serial_baud="$(yaml_gps_value "$yaml_file" gnss_serial_baud)"
  expected_receiver_family="$(gnss_receiver_family_from_state)"
  expected_serial_device="$(gnss_serial_device_from_state)"
  expected_serial_baud="$(gnss_serial_baud_from_state)"

  if [[ "$yaml_receiver_family" != "${expected_receiver_family:-auto}" ]]; then
    fail "GNSS receiver family diverges between docker/.env and mowgli_robot.yaml"
    add_issue "docker/.env has GNSS_RECEIVER_FAMILY=${expected_receiver_family:-auto} but $yaml_file has gnss_receiver_family=${yaml_receiver_family:-missing}. Re-run $(installer_main_command) to resync."
  fi

  if [[ "$yaml_serial_device" != "${expected_serial_device:-/dev/ttyAMA4}" ]]; then
    fail "GNSS serial device diverges between docker/.env and mowgli_robot.yaml"
    add_issue "docker/.env has GNSS_SERIAL_DEVICE=${expected_serial_device:-/dev/ttyAMA4} but $yaml_file has gnss_serial_device=${yaml_serial_device:-missing}. Re-run $(installer_main_command) to resync."
  fi

  if [[ "$yaml_serial_baud" != "${expected_serial_baud:-921600}" ]]; then
    fail "GNSS serial baud diverges between docker/.env and mowgli_robot.yaml"
    add_issue "docker/.env has GNSS_SERIAL_BAUD=${expected_serial_baud:-921600} but $yaml_file has gnss_serial_baud=${yaml_serial_baud:-missing}. Re-run $(installer_main_command) to resync."
  fi

  if [[ "$yaml_receiver_family" == "${expected_receiver_family:-auto}" \
    && "$yaml_serial_device" == "${expected_serial_device:-/dev/ttyAMA4}" \
    && "$yaml_serial_baud" == "${expected_serial_baud:-921600}" ]]; then
    info "docker/.env and mowgli_robot.yaml agree on gnss_receiver_family/gnss_serial_device/gnss_serial_baud"
  fi
}

check_containers() {
  step "Check: Containers"

  local services=()
  if ! mapfile -t services < <(expected_runtime_services); then
    fail "Unknown GNSS_BACKEND=${GNSS_BACKEND}"
    add_issue "Unknown GNSS_BACKEND=${GNSS_BACKEND}. Re-run $(installer_main_command)."
    return
  fi

  for svc in "${services[@]}"; do
    local container=""
    container="$(container_name_for_service "$svc" 2>/dev/null || true)"

    local status
    status="$(docker_cmd inspect -f '{{.State.Status}}' "$container" 2>/dev/null || echo "missing")"

    if [[ "$status" == "running" ]]; then
      local uptime
      uptime="$(docker_cmd inspect -f '{{.State.StartedAt}}' "$container" 2>/dev/null | cut -dT -f2 | cut -d. -f1)"
      info "$svc ($container) — running since $uptime"
    else
      fail "$svc ($container) — $status"
      if [[ "$status" == "missing" ]]; then
        add_issue "Container $container not found."
      else
        add_issue "Container $container is $status. Check logs: $(print_logs_command_for_container "$container" 30)"
      fi
    fi
  done

  if docker_cmd inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    local dead_nodes
    dead_nodes=$(
      docker_cmd logs mowgli-ros2 --tail 200 2>&1 \
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
      add_issue "Some ROS nodes crashed inside mowgli-ros2. Check: $(print_logs_command_for_container mowgli-ros2 200) | grep 'process has died'"
    fi
  fi
}

check_firmware() {
  step "Check: Mowgli firmware"

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    info "MAVROS backend: skipping direct Mowgli firmware check"
    return
  fi

  if ! docker_cmd inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    warn "mowgli-ros2 not running — skipping firmware check"
    return
  fi

  local status_data
  status_data="$(
    docker_cmd exec mowgli-ros2 bash -lc \
      "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /hardware_bridge/status --once 2>/dev/null" \
      2>/dev/null || echo ""
  )"

  if [[ -z "$status_data" ]]; then
    fail "No data on /hardware_bridge/status — hardware bridge cannot communicate with Mowgli board"
    add_issue "Mowgli firmware not responding. Ensure the STM32 is flashed with Mowgli firmware and /dev/mowgli is accessible."
    echo -e "       ${DIM}Flash firmware: https://github.com/cedbossneo/Mowgli${NC}"
    echo -e "       ${DIM}Check serial: $(print_logs_command_for_container mowgli-ros2 200) | grep hardware_bridge${NC}"
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

  : "${GNSS_BACKEND:=universal}"
  local gnss_backend
  local gnss_stack
  local gps_container

  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
  gnss_stack="$(effective_gnss_stack 2>/dev/null || true)"

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    info "MAVROS backend: GPS is handled through Pixhawk/MAVROS"

    local gps_status
    gps_status="$(docker_cmd inspect -f '{{.State.Status}}' mowgli-gps 2>/dev/null || echo "missing")"
    if [[ "$gps_status" == "running" ]]; then
      fail "mowgli-gps is running in MAVROS mode"
      add_issue "mowgli-gps must not run when HARDWARE_BACKEND=mavros. Re-run $(installer_main_command) and restart the expected services: $(print_restart_command_for_backend mavros)"
    else
      info "Direct GPS container disabled (${gps_status})"
    fi

    local mavros_status
    mavros_status="$(docker_cmd inspect -f '{{.State.Status}}' mowgli-mavros 2>/dev/null || echo "missing")"
    if [[ "$mavros_status" != "running" ]]; then
      fail "mowgli-mavros is ${mavros_status}"
      add_issue "MAVROS backend selected but mowgli-mavros is not running. Check logs: $(print_logs_command_for_container mowgli-mavros 50)"
      return
    fi
    info "MAVROS container running"

    local mavros_state
    mavros_state="$(
      docker_cmd exec mowgli-ros2 bash -lc \
        "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /mavros/state --once 2>/dev/null" \
        2>/dev/null || echo ""
    )"
    if [[ -z "$mavros_state" ]]; then
      fail "No MAVROS state on /mavros/state"
      add_issue "MAVROS is running but /mavros/state has no data. Check Pixhawk serial connection, MAVROS_PORT, MAVROS_BAUD, and logs: $(print_logs_command_for_container mowgli-mavros 50)"
    else
      info "MAVROS state available on /mavros/state"
    fi

    local mavros_global
    mavros_global="$(
      docker_cmd exec mowgli-ros2 bash -lc \
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
      docker_cmd exec mowgli-ros2 bash -lc \
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

  if ! is_supported_gnss_backend "$gnss_backend"; then
    fail "Unknown GNSS_BACKEND=${GNSS_BACKEND}"
    add_issue "Unknown GNSS_BACKEND=${GNSS_BACKEND}. Re-run $(installer_main_command)."
    return
  fi

  gps_container="$(compose_gnss_container_name "$gnss_backend" 2>/dev/null || true)"
  if [ -z "$gps_container" ]; then
    info "Direct GNSS container disabled"
    return
  fi

  if ! docker_cmd inspect -f '{{.State.Status}}' "$gps_container" 2>/dev/null | grep -q running; then
    warn "$gps_container not running — skipping GPS check"
    return
  fi

  if [[ "$gnss_stack" == "universal" ]]; then
    info "Universal GNSS mode: expected direct sidecar ${gps_container}"
  fi

  local fix_data
  fix_data="$(
    docker_cmd exec mowgli-ros2 bash -lc \
      "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /gps/fix --once 2>/dev/null" \
      2>/dev/null || echo ""
  )"

  if [[ -z "$fix_data" ]]; then
    fail "No GPS fix data on /gps/fix"
    if [[ "$gnss_stack" == "universal" ]]; then
      add_issue "Universal GNSS sidecar is not publishing /gps/fix. Check logs: $(print_logs_command_for_container "$gps_container" 80)"
    else
      add_issue "GPS not publishing. Check logs: $(print_logs_command_for_container "$gps_container" 30)"
    fi
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

  info "Universal GNSS sidecar: /gps/fix is reaching mowgli-ros2"

  if [[ "${GNSS_NTRIP_ENABLED:-false}" == "true" ]]; then
    local rtcm_info
    rtcm_info="$(
      docker_cmd exec mowgli-ros2 bash -lc \
        "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic info /rtcm 2>/dev/null" \
        2>/dev/null || echo ""
    )"
    if echo "$rtcm_info" | grep -q "Publisher count: [1-9]"; then
      info "Universal GNSS NTRIP: RTCM topic has publisher(s)"
    else
      warn "Universal GNSS NTRIP enabled but no RTCM publisher detected on /rtcm"
      add_issue "GNSS_NTRIP_ENABLED=true but /rtcm has no publishers. Check $(print_logs_command_for_container "$gps_container" 80) and the NTRIP settings in docker/config/mowgli/mowgli_robot.yaml."
    fi
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
        info "Restart to apply: $(print_restart_command_for_backend)"
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

  if ! docker_cmd inspect -f '{{.State.Status}}' mowgli-lidar 2>/dev/null | grep -q running; then
    warn "mowgli-lidar not running — skipping LiDAR check"
    return
  fi

  local scan_check
  scan_check="$(
    docker_cmd exec mowgli-ros2 bash -lc \
      "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic info /scan 2>/dev/null" \
      2>/dev/null || echo ""
  )"

  local pub_count
  pub_count="$(echo "$scan_check" | grep "Publisher count:" | awk '{print $3}')"

  if [[ "${pub_count:-0}" -ge 1 ]] 2>/dev/null; then
    info "LiDAR publishing on /scan ($pub_count publisher)"
  else
    fail "No publisher on /scan — LiDAR data not reaching ROS"
    add_issue "LiDAR not publishing. Check logs: $(print_logs_command_for_container mowgli-lidar 20)"
    echo -e "       ${DIM}Expected serial port: ${LIDAR_PORT}${NC}"
  fi
}

# check_slam() — REMOVED. SLAM (slam_toolbox, Cartographer, etc.) has been
# removed from the MowgliNext stack. The map frame is provided by
# navsat_transform + RTK GPS via robot_localization's dual EKF.
# See CLAUDE.md "Architecture Invariants" for details.

check_rangefinders() {
  step "Check: Rangefinders"

  if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" || "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]]; then
    if ! feature_is_available tfluna; then
      warn_unavailable_feature_once \
        tfluna \
        "TF-Luna rangefinder services are not available on this branch yet; skipping TF-Luna device checks."
      return
    fi
  fi

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

  if ! docker_cmd inspect -f '{{.State.Status}}' mowgli-gui 2>/dev/null | grep -q running; then
    fail "mowgli-gui not running"
    add_issue "GUI container not running."
    return
  fi

  local ip
  ip="$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")"

  if curl -sf -o /dev/null --connect-timeout 3 "http://$ip:4006" 2>/dev/null || \
     curl -sf -o /dev/null --connect-timeout 3 "http://localhost:4006" 2>/dev/null; then
    info "GUI accessible at http://$ip:4006"
  else
    warn "GUI might be starting up — try http://$ip:4006 in your browser"
  fi

  local fg_info
  fg_info="$(
    docker_cmd exec mowgli-ros2 bash -lc \
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
  echo -e "    GUI:       ${CYAN}http://$ip:4006${NC}"
  echo -e "    Foxglove:  ${CYAN}ws://$ip:8765${NC}"
  echo -e "    Rosbridge: ${CYAN}ws://$ip:9090${NC}"
  echo -e "    MQTT:      ${CYAN}$ip:1883${NC}"
}
