#!/usr/bin/env bash
#interactive_config

# ── Global configuration ────────────────────────────────────────────────────

REPO_URL="https://github.com/cedbossneo/mowglinext.git"
REPO_BRANCH="main"
IMAGE_TAG="main"
REPO_DIR="${MOWGLI_HOME:-$HOME/mowglinext}"
DOCKER_SUBDIR="install"
INSTALL_DIR="${REPO_DIR}/${DOCKER_SUBDIR}"
DOCKER_DIR="$REPO_DIR/docker"
COMPOSE_SRC_DIR="$INSTALL_DIR/compose"
FINAL_COMPOSE_FILE="$DOCKER_DIR/docker-compose.yaml"
FINAL_ENV_FILE="$DOCKER_DIR/.env"
UDEV_RULES_FILE="/etc/udev/rules.d/50-mowgli.rules"

MOWGLI_ROS2_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/mowgli-ros2:${IMAGE_TAG}"
GPS_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/gps:${IMAGE_TAG}"
UNICORE_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/unicore:${IMAGE_TAG}"
LIDAR_LDLIDAR_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/lidar-ldlidar:${IMAGE_TAG}"
LIDAR_RPLIDAR_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/lidar-rplidar:${IMAGE_TAG}"
LIDAR_STL27L_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/lidar-stl27l:${IMAGE_TAG}"
MAVROS_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/mavros:${IMAGE_TAG}"
NMEA_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/nmea:${IMAGE_TAG}"
GUI_IMAGE_DEFAULT="ghcr.io/cedbossneo/mowglinext/mowglinext-gui:${IMAGE_TAG}"

CHECK_ONLY=false
CLI_PRESET=false

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --check)
        CHECK_ONLY=true
        ;;
      --lang=*)
        MOWGLI_LANG="${1#*=}"
        ;;
      --gnss=*)
        CLI_PRESET=true
        local gnss_spec="${1#*=}"
        case "$gnss_spec" in
          gps|ublox|unicore)
            GNSS_BACKEND="$gnss_spec"
            ;;
          *)
            error "Unknown GNSS backend: $gnss_spec (expected gps|ublox|unicore)"
            exit 1
            ;;
        esac
        ;;
      --gps=*)
        CLI_PRESET=true
        local gps_spec="${1#*=}"
        # Format: protocol-connection  e.g. ubx-usb, ubx-uart, nmea-usb, nmea-uart
        local gps_proto="${gps_spec%%-*}"
        local gps_conn="${gps_spec##*-}"
        case "$gps_proto" in
          ubx)  GPS_PROTOCOL="UBX";  GPS_BAUD="460800" ;;
          nmea) GPS_PROTOCOL="NMEA"; GPS_BAUD="115200" ;;
          *)    error "Unknown GPS protocol: $gps_proto (expected ubx or nmea)"; exit 1 ;;
        esac
        case "$gps_conn" in
          usb)  GPS_CONNECTION="usb";  GPS_UART_DEVICE="" ;;
          uart) GPS_CONNECTION="uart" ;;
          *)    error "Unknown GPS connection: $gps_conn (expected usb or uart)"; exit 1 ;;
        esac
        ;;
      --gps-uart=*)
        GPS_UART_DEVICE="${1#*=}"
        ;;
      --lidar=*)
        CLI_PRESET=true
        local lidar_spec="${1#*=}"
        case "$lidar_spec" in
          none)
            LIDAR_ENABLED="false"; LIDAR_TYPE="none"; LIDAR_MODEL=""
            LIDAR_CONNECTION=""; LIDAR_UART_DEVICE=""
            ;;
          rplidar-usb)
            LIDAR_ENABLED="true"; LIDAR_TYPE="rplidar"; LIDAR_MODEL="RPLIDAR_A1"
            LIDAR_CONNECTION="usb"; LIDAR_BAUD="115200"; LIDAR_UART_DEVICE=""
            ;;
          rplidar-uart)
            LIDAR_ENABLED="true"; LIDAR_TYPE="rplidar"; LIDAR_MODEL="RPLIDAR_A1"
            LIDAR_CONNECTION="uart"; LIDAR_BAUD="115200"
            ;;
          ldlidar-usb)
            LIDAR_ENABLED="true"; LIDAR_TYPE="ldlidar"; LIDAR_MODEL="LDLiDAR_LD19"
            LIDAR_CONNECTION="usb"; LIDAR_BAUD="230400"; LIDAR_UART_DEVICE=""
            ;;
          ldlidar-uart)
            LIDAR_ENABLED="true"; LIDAR_TYPE="ldlidar"; LIDAR_MODEL="LDLiDAR_LD19"
            LIDAR_CONNECTION="uart"; LIDAR_BAUD="230400"
            ;;
          stl27l-usb)
            LIDAR_ENABLED="true"; LIDAR_TYPE="stl27l"; LIDAR_MODEL="STL27L"
            LIDAR_CONNECTION="usb"; LIDAR_BAUD="230400"; LIDAR_UART_DEVICE=""
            ;;
          stl27l-uart)
            LIDAR_ENABLED="true"; LIDAR_TYPE="stl27l"; LIDAR_MODEL="STL27L"
            LIDAR_CONNECTION="uart"; LIDAR_BAUD="230400"
            ;;
          *)
            error "Unknown lidar spec: $lidar_spec"
            echo "  Expected: none, rplidar-usb, rplidar-uart, ldlidar-usb, ldlidar-uart, stl27l-usb, stl27l-uart"
            exit 1
            ;;
        esac
        ;;
      --lidar-uart=*)
        LIDAR_UART_DEVICE="${1#*=}"
        ;;
      --tfluna=*)
        CLI_PRESET=true
        local tf_spec="${1#*=}"
        case "$tf_spec" in
          none)
            TFLUNA_FRONT_ENABLED="false"; TFLUNA_EDGE_ENABLED="false"
            ;;
          front)
            TFLUNA_FRONT_ENABLED="true"; TFLUNA_EDGE_ENABLED="false"
            ;;
          edge)
            TFLUNA_FRONT_ENABLED="false"; TFLUNA_EDGE_ENABLED="true"
            ;;
          both)
            TFLUNA_FRONT_ENABLED="true"; TFLUNA_EDGE_ENABLED="true"
            ;;
          *)
            error "Unknown tfluna spec: $tf_spec (expected none, front, edge, both)"
            exit 1
            ;;
        esac
        ;;
      --tfluna-front-uart=*)
        TFLUNA_FRONT_UART_DEVICE="${1#*=}"
        ;;
      --tfluna-edge-uart=*)
        TFLUNA_EDGE_UART_DEVICE="${1#*=}"
        ;;
      *)
        warn "Unknown argument: $1"
        ;;
    esac
    shift
  done

  # CLI flags act as presets — skip interactive prompts for configured sensors
  if [[ "$CLI_PRESET" == "true" ]]; then
    PRESET_LOADED=true
  fi
}

# Track issues for the final summary
ISSUES=()

add_issue() {
  ISSUES+=("$1")
}

# Load existing config values from mowgli_robot.yaml for use as defaults
load_existing_config() {
  local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  if [ ! -f "$yaml_file" ]; then
    return
  fi

  _yaml_val() {
    grep -E "^\s+${1}:" "$yaml_file" 2>/dev/null | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
  }

  PREV_DATUM_LAT=$(_yaml_val datum_lat)
  PREV_DATUM_LON=$(_yaml_val datum_lon)
  PREV_NTRIP_ENABLED=$(_yaml_val ntrip_enabled)
  PREV_NTRIP_HOST=$(_yaml_val ntrip_host)
  PREV_NTRIP_PORT=$(_yaml_val ntrip_port)
  PREV_NTRIP_USER=$(_yaml_val ntrip_user)
  PREV_NTRIP_PASSWORD=$(_yaml_val ntrip_password)
  PREV_NTRIP_MOUNTPOINT=$(_yaml_val ntrip_mountpoint)
}

interactive_config() {
  step "5/6  Mower configuration"

  local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  # Defaults live in install/config/ (versioned templates). The runtime
  # copies under docker/config/ are git-ignored so user edits survive
  # `git pull` and the installer's `git reset --hard`.
  local defaults="$INSTALL_DIR/config"
  mkdir -p "$DOCKER_DIR/config/mowgli"
  mkdir -p "$DOCKER_DIR/config/om"
  mkdir -p "$DOCKER_DIR/config/mqtt"
  mkdir -p "$DOCKER_DIR/config/db"

  # CycloneDDS
  if [ ! -f "$DOCKER_DIR/config/cyclonedds.xml" ]; then
    cp "$defaults/cyclonedds.xml" "$DOCKER_DIR/config/cyclonedds.xml"
    info "Created cyclonedds.xml"
  fi

  # Mosquitto
  if [ ! -f "$DOCKER_DIR/config/mqtt/mosquitto.conf" ]; then
    cp "$defaults/mqtt/mosquitto.conf" "$DOCKER_DIR/config/mqtt/mosquitto.conf"
    info "Created mosquitto.conf"
  fi

  # Load previous values for defaults
  load_existing_config

  # If config already exists, ask whether to reconfigure
  SKIP_WRITE_CONFIG=false
  if [ -f "$yaml_file" ]; then
    info "mowgli_robot.yaml already exists"
    if ! confirm "Do you want to reconfigure it?"; then
      SKIP_WRITE_CONFIG=true
      return
    fi
  fi

  echo ""
  echo -e "${BOLD}Let's configure your mower. You can change these later in:${NC}"
  echo -e "  ${DIM}$yaml_file${NC}"
  echo ""

  # GPS datum
  local datum_lat="${PREV_DATUM_LAT:-0.0}" datum_lon="${PREV_DATUM_LON:-0.0}"

  if [[ "$datum_lat" != "0.0" && "$datum_lat" != "0" && -n "$datum_lat" ]]; then
    echo -e "${CYAN}GPS Datum${NC} — currently set to $datum_lat, $datum_lon"
    echo ""
    echo -e "  ${BOLD}1)${NC} Keep current datum ($datum_lat, $datum_lon)"
    echo -e "  ${BOLD}2)${NC} Enter new coordinates manually"
    echo -e "  ${BOLD}3)${NC} Auto-detect from GPS after startup"
    echo ""
    prompt "  Choose" "1"
    local datum_choice="$REPLY"

    case "$datum_choice" in
      2)
        echo -e "  ${DIM}Find coordinates on Google Maps: right-click dock > copy coordinates${NC}"
        prompt "  Latitude?" "$datum_lat"
        datum_lat="$REPLY"
        prompt "  Longitude?" "$datum_lon"
        datum_lon="$REPLY"
        ;;
      3)
        datum_lat="0.0"
        datum_lon="0.0"
        info "Datum will be auto-detected from GPS after startup"
        ;;
      *)
        info "Keeping current datum: $datum_lat, $datum_lon"
        ;;
    esac
  else
    echo -e "${CYAN}GPS Datum${NC} — map origin coordinates (should be near your dock)"
    echo ""
    echo -e "  ${BOLD}1)${NC} Auto-detect from GPS after startup (mower must be on the dock)"
    echo -e "  ${BOLD}2)${NC} Enter coordinates manually"
    echo -e "  ${BOLD}3)${NC} Skip (configure later)"
    echo ""
    prompt "  Choose" "1"
    local datum_choice="$REPLY"

    case "$datum_choice" in
      2)
        echo -e "  ${DIM}Find coordinates on Google Maps: right-click dock > copy coordinates${NC}"
        prompt "  Latitude?" "0.0"
        datum_lat="$REPLY"
        prompt "  Longitude?" "0.0"
        datum_lon="$REPLY"
        if [[ "$datum_lat" == "0.0" || "$datum_lon" == "0.0" ]]; then
          warn "Datum is 0.0 — GPS localisation won't work"
          add_issue "Set datum_lat and datum_lon in config/mowgli/mowgli_robot.yaml"
        fi
        ;;
      1)
        datum_lat="0.0"
        datum_lon="0.0"
        info "Datum will be auto-detected from GPS after startup"
        ;;
      *)
        warn "Datum skipped — you must set it before mowing"
        add_issue "Set datum_lat and datum_lon in config/mowgli/mowgli_robot.yaml"
        ;;
    esac
  fi

  # NTRIP — use previous values as defaults
  echo ""
  echo -e "${CYAN}NTRIP RTK${NC} — correction stream for centimetre-level GPS accuracy"
  echo -e "${DIM}Free in France: crtk.net (user: centipede / pass: centipede)${NC}"
  echo -e "${DIM}Default mountpoint NEAR picks the closest base via NMEA GGA (use NEAR4 on legacy receivers).${NC}"
  echo -e "${DIM}Find your nearest base station at https://centipede.fr${NC}"

  local prev_ntrip="${PREV_NTRIP_ENABLED:-false}"
  local ntrip_enabled="false"
  local ntrip_host="${PREV_NTRIP_HOST:-crtk.net}"
  local ntrip_port="${PREV_NTRIP_PORT:-2101}"
  local ntrip_user="${PREV_NTRIP_USER:-centipede}"
  local ntrip_password="${PREV_NTRIP_PASSWORD:-centipede}"
  local ntrip_mountpoint="${PREV_NTRIP_MOUNTPOINT:-NEAR}"

  if [[ "$prev_ntrip" == "true" && -n "$ntrip_mountpoint" ]]; then
    echo -e "  ${DIM}Currently: ${ntrip_host}:${ntrip_port}/${ntrip_mountpoint}${NC}"
  fi

  if confirm "  Enable NTRIP corrections?"; then
    ntrip_enabled="true"
    echo ""
    echo -e "  ${DIM}Enter NTRIP parameters (press Enter to keep current value):${NC}"
    prompt "    Host?" "$ntrip_host"
    ntrip_host="$REPLY"
    prompt "    Port?" "$ntrip_port"
    ntrip_port="$REPLY"
    prompt "    User?" "$ntrip_user"
    ntrip_user="$REPLY"
    prompt "    Password?" "$ntrip_password"
    ntrip_password="$REPLY"
    prompt "    Mountpoint (nearest base station)?" "$ntrip_mountpoint"
    ntrip_mountpoint="$REPLY"

    if [[ -z "$ntrip_mountpoint" ]]; then
      warn "No mountpoint set — NTRIP won't connect without one"
      add_issue "Set ntrip_mountpoint in $yaml_file to your nearest base station"
    fi
  fi

  # Store config vars for write_config and auto_detect
  CONFIG_DATUM_LAT="$datum_lat"
  CONFIG_DATUM_LON="$datum_lon"
  CONFIG_NTRIP_ENABLED="$ntrip_enabled"
  CONFIG_NTRIP_HOST="$ntrip_host"
  CONFIG_NTRIP_PORT="$ntrip_port"
  CONFIG_NTRIP_USER="$ntrip_user"
  CONFIG_NTRIP_PASSWORD="$ntrip_password"
  CONFIG_NTRIP_MOUNTPOINT="$ntrip_mountpoint"
  CONFIG_LIDAR_X="0.20"
  CONFIG_LIDAR_Y="0.0"
  CONFIG_LIDAR_Z="0.22"
  CONFIG_LIDAR_YAW="0.0"
  CONFIG_DOCK_X="0.0"
  CONFIG_DOCK_Y="0.0"
  CONFIG_DOCK_YAW="0.0"
}

# Patch a single mowgli/ros__parameters key in-place. Preserves
# indentation, comments, and every other key. If the key is missing
# (only happens when the seeded template is older than the installer)
# we append it under the ros__parameters block.
_yaml_patch_key() {
  local file="$1" key="$2" value="$3"
  if grep -qE "^[[:space:]]+${key}:" "$file"; then
    # Replace value, preserving leading whitespace and any trailing
    # comment on the same line.
    python3 - "$file" "$key" "$value" <<'PY'
import re, sys
path, key, value = sys.argv[1], sys.argv[2], sys.argv[3]
pat = re.compile(r'^(\s+' + re.escape(key) + r':\s*)([^#\n]*)(\s*#.*)?$')
with open(path) as f:
    lines = f.readlines()
for i, line in enumerate(lines):
    m = pat.match(line)
    if m:
        comment = m.group(3) or ''
        lines[i] = f"{m.group(1)}{value}{comment}\n"
        break
with open(path, 'w') as f:
    f.writelines(lines)
PY
  else
    # Append under the first ros__parameters: line in the mowgli block.
    python3 - "$file" "$key" "$value" <<'PY'
import sys
path, key, value = sys.argv[1], sys.argv[2], sys.argv[3]
with open(path) as f:
    lines = f.readlines()
out = []
inserted = False
for line in lines:
    out.append(line)
    if not inserted and line.strip() == 'ros__parameters:':
        indent = ' ' * (len(line) - len(line.lstrip()) + 4)
        out.append(f"{indent}{key}: {value}\n")
        inserted = True
with open(path, 'w') as f:
    f.writelines(out)
PY
  fi
}

write_config() {
  local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  local template="$INSTALL_DIR/config/mowgli/mowgli_robot.yaml"

  : "${GPS_PROTOCOL:=UBX}"
  : "${GPS_PORT:=/dev/gps}"
  : "${GPS_BAUD:=460800}"

  # Seed from the comprehensive template if the runtime yaml doesn't
  # exist yet. We never overwrite an existing file — that would wipe
  # GUI-managed values like chassis dims, IMU calibration, fusion
  # graph flags, etc.
  if [ ! -f "$yaml_file" ]; then
    if [ -f "$template" ]; then
      cp "$template" "$yaml_file"
      info "Seeded $yaml_file from install template"
    else
      warn "Install template missing at $template — writing minimal yaml"
      cat > "$yaml_file" <<EOF
mowgli:
  ros__parameters:
    ntrip_enabled: false
EOF
    fi
  else
    info "Patching existing $yaml_file in place"
  fi

  # Patch in only the keys the installer is responsible for.
  _yaml_patch_key "$yaml_file" datum_lat       "$CONFIG_DATUM_LAT"
  _yaml_patch_key "$yaml_file" datum_lon       "$CONFIG_DATUM_LON"
  _yaml_patch_key "$yaml_file" gps_port        "\"$GPS_PORT\""
  _yaml_patch_key "$yaml_file" gps_baudrate    "$GPS_BAUD"
  _yaml_patch_key "$yaml_file" gps_protocol    "$GPS_PROTOCOL"
  _yaml_patch_key "$yaml_file" ntrip_enabled   "$CONFIG_NTRIP_ENABLED"
  _yaml_patch_key "$yaml_file" ntrip_host      "\"$CONFIG_NTRIP_HOST\""
  _yaml_patch_key "$yaml_file" ntrip_port      "$CONFIG_NTRIP_PORT"
  _yaml_patch_key "$yaml_file" ntrip_user      "\"$CONFIG_NTRIP_USER\""
  _yaml_patch_key "$yaml_file" ntrip_password  "\"$CONFIG_NTRIP_PASSWORD\""
  _yaml_patch_key "$yaml_file" ntrip_mountpoint "\"$CONFIG_NTRIP_MOUNTPOINT\""

  # LiDAR enable + the LiDAR-aware localizer flags. When the operator
  # picked a LiDAR in the previous step we also want the GTSAM
  # factor-graph backend on with scan-matching + loop closure, so
  # the GUI doesn't show LiDAR plugged in but ignored.
  local lidar_on="false"
  if [[ "${LIDAR_ENABLED:-false}" == "true" ]]; then
    lidar_on="true"
  fi
  _yaml_patch_key "$yaml_file" lidar_enabled     "$lidar_on"
  _yaml_patch_key "$yaml_file" use_fusion_graph  "$lidar_on"
  _yaml_patch_key "$yaml_file" use_scan_matching "$lidar_on"
  _yaml_patch_key "$yaml_file" use_loop_closure  "$lidar_on"

  # Lidar mounting only patched when explicitly set (auto-detect step
  # leaves them alone so the GUI / template defaults survive).
  if [[ -n "${CONFIG_LIDAR_X:-}" ]]; then
    _yaml_patch_key "$yaml_file" lidar_x   "$CONFIG_LIDAR_X"
    _yaml_patch_key "$yaml_file" lidar_y   "$CONFIG_LIDAR_Y"
    _yaml_patch_key "$yaml_file" lidar_z   "$CONFIG_LIDAR_Z"
    _yaml_patch_key "$yaml_file" lidar_yaw "$CONFIG_LIDAR_YAW"
  fi

  _yaml_patch_key "$yaml_file" dock_pose_x   "$CONFIG_DOCK_X"
  _yaml_patch_key "$yaml_file" dock_pose_y   "$CONFIG_DOCK_Y"
  _yaml_patch_key "$yaml_file" dock_pose_yaw "$CONFIG_DOCK_YAW"

  info "Wrote $yaml_file"

  cat > "$DOCKER_DIR/config/om/mower_config.sh" <<EOF
export OM_DATUM_LAT=$CONFIG_DATUM_LAT
export OM_DATUM_LONG=$CONFIG_DATUM_LON
export OM_GPS_PROTOCOL=$GPS_PROTOCOL
export OM_GPS_PORT=$GPS_PORT
export OM_GPS_BAUDRATE=$GPS_BAUD
export OM_USE_NTRIP=$( [[ "$CONFIG_NTRIP_ENABLED" == "true" ]] && echo "True" || echo "False" )
export OM_NTRIP_HOSTNAME=$CONFIG_NTRIP_HOST
export OM_NTRIP_PORT=$CONFIG_NTRIP_PORT
export OM_NTRIP_USER=$CONFIG_NTRIP_USER
export OM_NTRIP_PASSWORD=$CONFIG_NTRIP_PASSWORD
export OM_NTRIP_ENDPOINT=$CONFIG_NTRIP_MOUNTPOINT
export OM_TOOL_WIDTH=0.13
export OM_ENABLE_MOWER=true
export OM_AUTOMATIC_MODE=0
export OM_BATTERY_FULL_VOLTAGE=28.5
export OM_BATTERY_EMPTY_VOLTAGE=24.0
export OM_BATTERY_CRITICAL_VOLTAGE=23.0
EOF

  info "Wrote mower_config.sh"
}

auto_detect_position() {
  step "Auto-detect: GPS datum & dock position"

  if [[ "$CONFIG_DATUM_LAT" != "0.0" && "$CONFIG_DATUM_LAT" != "0" ]]; then
    info "Datum already set ($CONFIG_DATUM_LAT, $CONFIG_DATUM_LON) — skipping auto-detect"
    return
  fi

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    warn "GPS datum auto-detect is not available for MAVROS on this branch"
    add_issue "Set datum_lat and datum_lon manually in docker/config/mowgli/mowgli_robot.yaml"
    return
  fi

  if ! docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    warn "mowgli-ros2 container not running — cannot auto-detect"
    add_issue "Set datum_lat and datum_lon manually in config/mowgli/mowgli_robot.yaml"
    return
  fi

  if ! docker inspect -f '{{.State.Status}}' mowgli-gps 2>/dev/null | grep -q running; then
    warn "GPS container not running — cannot auto-detect"
    add_issue "Set datum_lat and datum_lon manually in config/mowgli/mowgli_robot.yaml"
    return
  fi

  echo -e "${DIM}Waiting for GPS fix (up to 60s)...${NC}"

  local fix_data="" lat="" lon=""
  local attempt=0
  while [[ $attempt -lt 12 ]]; do
    fix_data=$(docker exec mowgli-ros2 bash -c "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /gps/fix --once 2>/dev/null" 2>/dev/null || true)
    lat=$(echo "$fix_data" | grep "latitude:" | awk '{print $2}')
    lon=$(echo "$fix_data" | grep "longitude:" | awk '{print $2}')

    if [[ -n "$lat" && "$lat" != "0.0" ]]; then
      break
    fi

    attempt=$((attempt + 1))
    sleep 5
  done

  if [[ -z "$lat" || "$lat" == "0.0" ]]; then
    warn "Could not get a GPS fix — set datum manually"
    add_issue "Set datum_lat and datum_lon in config/mowgli/mowgli_robot.yaml"
    return
  fi

  info "GPS position: $lat, $lon"

  local is_charging="false"
  if docker inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    local status_data
    status_data=$(docker exec mowgli-ros2 bash -c "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /hardware_bridge/status --once 2>/dev/null" 2>/dev/null || true)
    is_charging=$(echo "$status_data" | grep "is_charging:" | awk '{print $2}')
  fi

  CONFIG_DATUM_LAT="$lat"
  CONFIG_DATUM_LON="$lon"
  info "Datum auto-set to GPS position: $lat, $lon"

  if [[ "$is_charging" == "true" ]]; then
    CONFIG_DOCK_X="0.0"
    CONFIG_DOCK_Y="0.0"
    CONFIG_DOCK_YAW="0.0"
    info "Mower is charging — dock position set to map origin (0, 0)"
    echo -e "       ${DIM}The datum IS your dock, so dock_pose = (0, 0, 0)${NC}"
  else
    warn "Mower is not charging — dock position left at (0, 0)"
    echo -e "       ${DIM}To set dock position later: drive to dock, then read /gps/pose${NC}"
    add_issue "Set dock_pose_x/y/yaw in config/mowgli/mowgli_robot.yaml (drive mower to dock, read the pose)"
  fi

  write_config
  info "Config updated with auto-detected position"

  echo -e "${DIM}Restarting containers with new config...${NC}"
  docker compose \
    -f "$FINAL_COMPOSE_FILE" \
    --env-file "$FINAL_ENV_FILE" \
    restart gps mowgli 2>&1 | tail -3
  sleep 10
}

run_mower_configuration_step() {
  SKIP_WRITE_CONFIG=false
  interactive_config
  if ! $SKIP_WRITE_CONFIG; then
    write_config
  fi
}
