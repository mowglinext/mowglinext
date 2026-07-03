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

  # A prior `compose up` with the host path missing makes Docker create a
  # *directory* at a bind-mounted file path. The `[ ! -f ]` guards below stay
  # true for a directory and `cp src dir/` would copy *into* it, leaving the
  # broken empty-dir mount in place (container fails with "not a directory",
  # or — for cyclonedds.xml — DDS silently ignores the unreadable URI and
  # falls back to defaults). Recover here so any entrypoint that brings up the
  # stack self-heals, not just the full installer's migrate_runtime_paths.
  fix_path_type_conflict "$DOCKER_DIR/config/mqtt/mosquitto.conf" "file"
  fix_path_type_conflict "$DOCKER_DIR/config/cyclonedds.xml" "file"

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
  local gnss_backend
  local gnss_stack
  local gnss_service

  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.base.yml")
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gui.yml")
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mqtt.yml")

  # In Mowgli mode, select one direct GNSS stack.
  # In MAVROS mode, GPS is handled via Pixhawk/MAVROS + NTRIP sidecar,
  # so direct GNSS compose fragments must not be included.
  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
  gnss_stack="$(effective_gnss_stack 2>/dev/null || true)"
  if ! is_supported_gnss_backend "$gnss_backend"; then
    error "Unknown GNSS_BACKEND: ${GNSS_BACKEND:-unset} (expected: $(list_supported_gnss_backends))"
    return 1
  fi

  case "$gnss_stack" in
    universal|disabled)
      ;;
    *)
      error "Unknown GNSS_STACK: ${GNSS_STACK:-unset} (expected: universal|disabled)"
      return 1
      ;;
  esac

  if [[ "$gnss_stack" != "disabled" && "$gnss_backend" != "disabled" ]]; then
    gnss_service="$(compose_gnss_service_name "$gnss_backend" 2>/dev/null || true)"
    case "$gnss_service" in
      gps)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gps.yml")
        ;;
      *)
        error "No compose fragment mapped for GNSS backend: ${gnss_backend}"
        return 1
        ;;
    esac
    if [[ "$gnss_stack" == "universal" ]]; then
      info "Universal GNSS selected: GNSS runs in the mowgli-gps sidecar."
    fi
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

  if effective_tfluna_front_enabled; then
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.tfluna-front.yml")
  fi

  if effective_tfluna_edge_enabled; then
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.tfluna-edge.yml")
  fi

  [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mavros.yml")

  if effective_vesc_enabled; then
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.vesc.yml")
  fi

  info "Selected compose fragments:"
  for f in "${COMPOSE_FILES[@]}"; do
    echo "  - $f"
  done
}

# Extract service definitions from a compose template file.
# Outputs everything between the "services:" line and the next top-level key
# (or EOF), preserving indentation. Skips x-ros2-env anchors since the merged
# file defines its own single anchor.

compose_extract_anchor_block() {
  local file="$1"
  awk '
    /^x-ros2-env:/ { in_anchor=1 }
    in_anchor {
      if ($0 ~ /^services:/) {
        exit
      }
      print
    }
  ' "$file"
}

compose_extract_section() {
  local file="$1"
  local section="$2"

  awk -v section="$section" '
    $0 ~ ("^" section ":") { in_section=1; next }
    in_section && $0 ~ /^[A-Za-z0-9_-]+:/ { exit }
    in_section { print }
  ' "$file"
}

write_compose_merged_fallback() {
  local anchor_written=false
  local volumes_written=false
  local f
  local section

  {
    for f in "${COMPOSE_FILES[@]}"; do
      if [[ "$anchor_written" == "false" ]]; then
        section="$(compose_extract_anchor_block "$f")"
        if [[ -n "$section" ]]; then
          printf '%s\n' "$section"
          printf '\n'
          anchor_written=true
        fi
      fi
    done

    printf 'services:\n'
    for f in "${COMPOSE_FILES[@]}"; do
      section="$(compose_extract_section "$f" services)"
      if [[ -n "$section" ]]; then
        printf '%s\n' "$section"
      fi
    done

    for f in "${COMPOSE_FILES[@]}"; do
      section="$(compose_extract_section "$f" volumes)"
      if [[ -n "$section" ]]; then
        if [[ "$volumes_written" == "false" ]]; then
          printf '\nvolumes:\n'
          volumes_written=true
        fi
        printf '%s\n' "$section"
      fi
    done
  } > "$FINAL_COMPOSE_FILE"
}

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
  if ! (
    cd "$REPO_DIR" || exit 1
    docker_compose_cmd \
      --project-directory "$REPO_DIR" \
      --env-file "$FINAL_ENV_FILE" \
      "${compose_args[@]}" \
      config --no-interpolate > "$FINAL_COMPOSE_FILE"
  ) || [[ ! -s "$FINAL_COMPOSE_FILE" ]]; then
    warn "docker compose config unavailable — generating a fallback merged compose for local validation"
    write_compose_merged_fallback
  fi

  info "Generated: $FINAL_COMPOSE_FILE"
}

run_compose_stack() {
  ensure_default_configs
  build_compose_stack
  write_compose_merged

  info "Final compose: $FINAL_COMPOSE_FILE"
  info "Env file: $FINAL_ENV_FILE"

  info "Pulling selected images..."
  docker_compose_cmd -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" pull
  echo ""
  info "Starting stack..."
  docker_compose_cmd -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" up -d
  echo ""
  info "Current containers:"
  docker_compose_cmd -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" ps
}
