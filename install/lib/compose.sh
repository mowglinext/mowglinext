#!/usr/bin/env bash
# Ensure config files mounted as bind-mount *files* exist before compose runs.
# Docker creates a directory when the host path is missing, which breaks the
# container with "not a directory" errors.

: "${REPO_DIR:?REPO_DIR is not set}"
: "${INSTALL_DIR:?INSTALL_DIR is not set}"

DOCKER_DIR="${DOCKER_DIR:-$REPO_DIR/docker}"
COMPOSE_SRC_DIR="${COMPOSE_SRC_DIR:-$INSTALL_DIR/compose}"
FINAL_COMPOSE_FILE="${FINAL_COMPOSE_FILE:-$DOCKER_DIR/docker-compose.yaml}"
FINAL_ENV_FILE="${FINAL_ENV_FILE:-$DOCKER_DIR/.env}"

ensure_default_configs() {
  # Defaults live in install/config/ (versioned templates). The runtime
  # copies under docker/config/ are git-ignored so user edits survive
  # `git pull` and the installer's `git reset --hard`.
  local defaults="$INSTALL_DIR/config"

  if [ ! -d "$defaults" ]; then
    warn "Defaults config directory missing: $defaults"
    return 1
  fi

  mkdir -p "$DOCKER_DIR/config/mqtt"
  mkdir -p "$DOCKER_DIR/config/mowgli"
  mkdir -p "$DOCKER_DIR/config/om"
  mkdir -p "$DOCKER_DIR/config/db"

  if [ ! -f "$DOCKER_DIR/config/mqtt/mosquitto.conf" ]; then
    cp "$defaults/mqtt/mosquitto.conf" "$DOCKER_DIR/config/mqtt/mosquitto.conf"
    info "Created default mosquitto.conf"
  fi

  if [ ! -f "$DOCKER_DIR/config/cyclonedds.xml" ]; then
    cp "$defaults/cyclonedds.xml" "$DOCKER_DIR/config/cyclonedds.xml"
    info "Created default cyclonedds.xml"
  fi
}


build_compose_stack() {
  COMPOSE_FILES=()

  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.base.yml")
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gui.yml")
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mqtt.yml")

  # In Mowgli mode, select one direct GNSS stack.
  # In MAVROS mode, GPS is handled via Pixhawk/MAVROS + NTRIP sidecar,
  # so direct GNSS compose fragments must not be included.
  if [[ "${HARDWARE_BACKEND:-mowgli}" != "mavros" ]]; then
    case "${GNSS_BACKEND:-gps}" in
      gps)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gps.yml")
        ;;
      ublox)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.ublox.yaml")
        ;;
      unicore)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.unicore.yaml")
        ;;
      nmea)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.nmea.yaml")
        ;;
      *)
        error "Unknown GNSS_BACKEND: ${GNSS_BACKEND:-unset} (expected: gps, ublox, unicore, nmea)"
        return 1
        ;;
    esac
  fi

  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.watchtower.yml")

  # Foxglove bridge is controlled via the ENABLE_FOXGLOVE env var passed
  # to the ROS2 container (see docker-compose.base.yml).  No separate
  # compose file is needed — the launch file starts/skips the node.
  if [[ "${LIDAR_ENABLED:-true}" == "true" && "${LIDAR_TYPE:-none}" != "none" ]]; then
    case "${LIDAR_TYPE:-}" in
      rplidar)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.lidar-rplidar.yml")
        ;;
      ldlidar)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.lidar-ldlidar.yml")
        ;;
      stl27l)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.lidar-stl27l.yml")
        ;;
      *)
        warn "Unknown LIDAR_TYPE: ${LIDAR_TYPE:-unset}"
        ;;
    esac
  fi

  [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.tfluna-front.yml")

  [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.tfluna-edge.yml")

  [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mavros.yml")

  [[ "${ENABLE_VESC:-false}" == "true" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.vesc.yml")

  info "Selected compose fragments:"
  for f in "${COMPOSE_FILES[@]}"; do
    echo "  - $f"
  done
}

# Extract service definitions from a compose template file.
# Outputs everything between the "services:" line and the next top-level key
# (or EOF), preserving indentation. Skips x-ros2-env anchors since the merged
# file defines its own single anchor.

write_compose_merged() {
# Generate a single self-contained docker-compose.yaml by merging all
# selected compose templates. Users get one readable file instead of
# needing to understand Docker Compose include/project mechanics.
  mkdir -p "$DOCKER_DIR"

  local compose_args=()
  local f

  for f in "${COMPOSE_FILES[@]}"; do
    if [[ ! -f "$f" ]]; then
      echo "Missing fragment: $f" >&2
      return 1
    fi
    compose_args+=("-f" "$f")
  done

  # `config --no-interpolate` keeps `${MOWGLI_ROS2_IMAGE}` and friends as
  # literal references in the generated compose file instead of baking
  # the values from .env at install time. Without it, editing .env later
  # (image-tag bumps, switching `:main` ↔ `:dev`, watchtower picking up
  # a new pin) was silently ignored — the compose file shipped with the
  # values resolved at first install.
  (
    cd "$REPO_DIR" || exit 1
    docker compose \
      --project-directory "$REPO_DIR" \
      --env-file "$FINAL_ENV_FILE" \
      "${compose_args[@]}" \
      config --no-interpolate > "$FINAL_COMPOSE_FILE"
  )

  info "Generated: $FINAL_COMPOSE_FILE"
}

run_compose_stack() {
  ensure_default_configs
  build_compose_stack
  write_compose_merged

  info "Final compose: $FINAL_COMPOSE_FILE"
  info "Env file: $FINAL_ENV_FILE"

  info "Pulling selected images..."
  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" pull
  echo ""
  info "Starting stack..."
  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" up -d
  echo ""
  info "Current containers:"
  docker compose -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" ps
}
