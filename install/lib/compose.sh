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
  mkdir -p "$DOCKER_DIR/config/mavros"
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
  # The GNSS sidecar reads mowgli_robot.yaml directly. A derived parameters
  # file left by an earlier install is dead weight holding the NTRIP password.
  rm -rf "$DOCKER_DIR/config/universal_gnss"
  fix_path_type_conflict "$DOCKER_DIR/config/mavros/mowgli_robot.yaml" "file"
  fix_path_type_conflict "$DOCKER_DIR/config/mavros/esc_wheel_odometry.yaml" "file"
  fix_path_type_conflict "$DOCKER_DIR/config/mavros/hardware_bridge.yaml" "file"

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
  local gnss_source

  if ! is_supported_hardware_backend "${HARDWARE_BACKEND:-mowgli}"; then
    error "Unknown HARDWARE_BACKEND: ${HARDWARE_BACKEND:-unset} (expected mowgli, mavros or openmower)"
    return 1
  fi

  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.base.yml")
  COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gui.yml")
  # The mosquitto broker only matters to operators integrating Home Assistant
  # or another MQTT client (docs/MQTT_CONTROL.md); it is opt-in (--mqtt=on).
  if [[ "${ENABLE_MQTT:-false}" == "true" ]]; then
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mqtt.yml")
  fi

  # GNSS ownership is independent from the robot hardware backend. A MAVROS
  # robot may use either a receiver connected directly to the SoC or a receiver
  # connected to the Pixhawk. The two paths use different UG runtime fragments.
  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
  gnss_stack="$(effective_gnss_stack 2>/dev/null || true)"
  gnss_source="$(effective_gnss_source 2>/dev/null || true)"
  if ! is_supported_gnss_backend "$gnss_backend"; then
    error "Unknown GNSS_BACKEND: ${GNSS_BACKEND:-unset} (expected: $(list_supported_gnss_backends))"
    return 1
  fi
  if ! is_supported_gnss_source "$gnss_source"; then
    error "Unknown GNSS_SOURCE: ${GNSS_SOURCE:-unset} (expected: $(list_supported_gnss_sources))"
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
    # A robot updated only through the host updater (never re-ran the
    # installer) can receive this compose fragment with an .env predating
    # UNIVERSAL_GNSS_IMAGE — the fragment itself already tolerates that via
    # its own `${UNIVERSAL_GNSS_IMAGE:-default}`, so this pre-check must too
    # (install/CLAUDE.md: "a compose fragment must work on a robot that never
    # ran this installer version"). Falling back here also means the value
    # gets persisted to .env on the next write_env, instead of the gap
    # reappearing on every regen.
    if [[ -z "${UNIVERSAL_GNSS_IMAGE:-}" ]]; then
      UNIVERSAL_GNSS_IMAGE="$UNIVERSAL_GNSS_IMAGE_DEFAULT"
      warn "UNIVERSAL_GNSS_IMAGE was unset in .env; defaulting to $UNIVERSAL_GNSS_IMAGE_DEFAULT"
    fi
    gnss_service="$(compose_gnss_service_name "$gnss_backend" 2>/dev/null || true)"
    case "$gnss_service:$gnss_source" in
      gps:direct)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gps.yml")
        info "Universal GNSS selected: receiver is connected directly to the companion computer."
        ;;
      gps:mavros)
        COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.gps-mavros.yml")
        info "Universal GNSS selected: receiver is provided by MAVROS; UG runs NTRIP-only."
        ;;
      *)
        error "No compose fragment mapped for GNSS backend/source: ${gnss_backend}/${gnss_source}"
        return 1
        ;;
    esac
  fi

  # Watchtower is gone: the host updater does managed releases and
  # `mowglinext.sh update` does everything else. install_host_updater still
  # removes a Watchtower container left by an older install.

  if [[ -f "$DOCKER_DIR/.updater-managed" ]]; then
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.updater.yml")
  fi

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

  [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.mavros.yml")

  # OpenMower v1 electronics: the bridge sidecar replaces mowgli-ros2's
  # hardware_bridge_node (gated in mowgli.launch.py); GNSS stays universal.
  [[ "${HARDWARE_BACKEND:-mowgli}" == "openmower" ]] && \
    COMPOSE_FILES+=("$COMPOSE_SRC_DIR/docker-compose.openmower.yml")

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

# Record the checksum of the file this installer just generated, in the same
# place and format the host updater uses (stack-definition.sha256). When this
# install later becomes updater-managed, the updater compares the installed
# file against THAT baseline: untouched means "generated by an older release"
# and is adopted whatever the fragments became since; a mismatch means a hand
# edit and is refused. Without it the only available comparison is against the
# new target, which every fragment change trips (field report 2026-09-20).
record_compose_baseline() {
  local digest=""
  if command -v sha256sum >/dev/null 2>&1; then
    digest="$(sha256sum "$FINAL_COMPOSE_FILE" | awk '{print $1}')"
  elif command -v shasum >/dev/null 2>&1; then
    digest="$(shasum -a 256 "$FINAL_COMPOSE_FILE" | awk '{print $1}')"
  fi
  if [[ ! "$digest" =~ ^[a-f0-9]{64}$ ]]; then
    warn "$MSG_COMPOSE_BASELINE_UNAVAILABLE"
    rm -f "$DOCKER_DIR/stack-definition.sha256"
    return 0
  fi
  if ! printf 'sha256:%s' "$digest" 2>/dev/null > "$DOCKER_DIR/stack-definition.sha256"; then
    # A stale checksum would make the updater call this file hand-edited.
    rm -f "$DOCKER_DIR/stack-definition.sha256" 2>/dev/null || true
    warn "$MSG_COMPOSE_BASELINE_UNAVAILABLE"
  fi
}

# Exit status 3 from `installer-stack` means the updater cannot vouch for the
# installed Compose file: either it has no recorded baseline and differs from
# the current definition (a hand edit and evolved fragments look the same), or
# its checksum no longer matches the baseline (a hand edit). Ask the operator;
# on consent the updater keeps the exact old file next to the new one
# (docker-compose.yaml.legacy-<UTC> / .edited-<UTC>) and regenerates it.
# A non-interactive run consents with MOWGLI_ADOPT_LEGACY_COMPOSE=true (or
# implicitly, since confirm() answers yes without a terminal — the old file
# is kept either way).
run_updater_installer_stack() {
  local selected_gnss="$1" selected_lidar="$2" status=0
  local binary="${MOWGLI_UPDATER_STACK_BINARY:-/usr/local/bin/mowgli-updater}"
  local stack_args=(installer-stack "$DOCKER_DIR" "${COMPOSE_PROJECT_NAME:-install}" "$COMPOSE_SRC_DIR" "$selected_gnss" "$selected_lidar")

  # A deleted compose file is recreated from the installed definition; there
  # is nothing to keep, but the operator is told before it happens.
  if [[ ! -f "$FINAL_COMPOSE_FILE" ]]; then
    warn "$MSG_COMPOSE_MISSING"
    if ! confirm "$MSG_COMPOSE_MISSING_CONFIRM"; then
      error "$MSG_COMPOSE_MISSING_DECLINED"
      return 1
    fi
  fi

  # The updater daemon renders with the directory recorded at its install
  # (/etc/mowgli-updater.json). If this shell resolves the checkout to another
  # spelling (symlinked home, moved checkout, MOWGLI_HOME), every bind mount
  # source differs and the next reviewed update is refused as "adds writable
  # storage". Say so now, with both paths, instead of later.
  local updater_config="${MOWGLI_UPDATER_CONFIG:-/etc/mowgli-updater.json}" daemon_dir=""
  if [[ -r "$updater_config" ]]; then
    daemon_dir="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("directory",""))' "$updater_config" 2>/dev/null || true)"
    if [[ -n "$daemon_dir" && "$daemon_dir" != "$DOCKER_DIR" ]]; then
      error "$MSG_UPDATER_DIRECTORY_MISMATCH"
      error "  updater: $daemon_dir"
      error "  this run: $DOCKER_DIR"
      return 1
    fi
  fi

  # The broker is a local (non-release) service: installer-stack must know
  # whether to add or drop it when it regenerates the file.
  export MOWGLI_ENABLE_MQTT="${ENABLE_MQTT:-false}"
  export MOWGLI_REGENERATE_STACK="${MOWGLI_REGENERATE_STACK:-false}"

  if [[ "${MOWGLI_ADOPT_LEGACY_COMPOSE:-}" == "true" ]]; then
    MOWGLI_ADOPT_LEGACY_COMPOSE=true "$binary" "${stack_args[@]}"
    return $?
  fi
  MOWGLI_ADOPT_LEGACY_COMPOSE=false "$binary" "${stack_args[@]}" || status=$?
  [[ "$status" -eq 3 ]] || return "$status"

  warn "$MSG_COMPOSE_LEGACY_EXPLAIN"
  warn "$MSG_COMPOSE_LEGACY_BACKUP"
  if ! confirm "$MSG_COMPOSE_LEGACY_CONFIRM"; then
    error "$MSG_COMPOSE_LEGACY_DECLINED"
    return 1
  fi
  MOWGLI_ADOPT_LEGACY_COMPOSE=true "$binary" "${stack_args[@]}"
}

write_compose_merged() {
# Generate a single self-contained docker-compose.yaml by merging all
# selected compose templates. Users get one readable file instead of
# needing to understand Docker Compose include/project mechanics.
  mkdir -p "$DOCKER_DIR"

  if [[ -f "$DOCKER_DIR/.updater-managed" ]]; then
    # The worker reads HARDWARE_BACKEND from docker/.env itself (the bundle's
    # "backend" group), so only the backends a release covers get this far.
    # Spelled out rather than updater_hardware_supported: docker/stack.sh
    # sources this file without lib/updater.sh.
    case "${HARDWARE_BACKEND:-mowgli}" in
      mowgli|openmower) ;;
      *)
        error "$MSG_UPDATER_STACK_BACKEND"
        return 1
        ;;
    esac
    local selected_gnss="none" selected_lidar="none"
    if [[ "$(effective_gnss_stack)" != "disabled" && "$(effective_gnss_backend)" != "disabled" ]]; then
      selected_gnss="universal"
    fi
    if [[ "${LIDAR_ENABLED:-true}" == "true" ]]; then selected_lidar="${LIDAR_TYPE:-none}"; fi
    run_updater_installer_stack "$selected_gnss" "$selected_lidar" || return 1
    if [[ "${MOWGLI_REGENERATE_STACK:-false}" == "true" ]]; then
      # The updater's digest pins would put the old images back under the
      # new definition; images now follow the .env tags. The pins are kept
      # as a dated copy, and the next reviewed update writes fresh ones.
      if [[ -f "$DOCKER_DIR/update-images.json" ]]; then
        backup_path_if_exists "$DOCKER_DIR/update-images.json"
        info "$MSG_UPDATE_MANUAL_PINS"
      fi
    elif [[ -s "$DOCKER_DIR/stack-release.json" ]] && [[ "$(cat "$DOCKER_DIR/stack-release.json")" != "null" ]]; then
      info "$MSG_UPDATER_STACK_REVIEW"
    fi
    prune_backup_if_unchanged "$FINAL_COMPOSE_FILE" "${MIGRATED_COMPOSE_BACKUP:-}"
    return 0
  fi

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
  # (image-tag bumps, switching `:main` ↔ `:dev`, the updater picking up
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
    if [[ -f "$DOCKER_DIR/.updater-managed" ]]; then
      error "Managed updates require Docker Compose; refusing an ambiguous fallback merge."
      return 1
    fi
    warn "docker compose config unavailable — generating a fallback merged compose for local validation"
    write_compose_merged_fallback
  fi

  record_compose_baseline
  info "Generated: $FINAL_COMPOSE_FILE"
  prune_backup_if_unchanged "$FINAL_COMPOSE_FILE" "${MIGRATED_COMPOSE_BACKUP:-}"
}

run_compose_stack() {
  if [[ -e /var/lib/mowgli-updater/maintenance ]]; then
    error "Update recovery is pending; resolve it before changing the stack."
    return 1
  fi
  ensure_default_configs
  build_compose_stack
  write_compose_merged

  info "Final compose: $FINAL_COMPOSE_FILE"
  info "Env file: $FINAL_ENV_FILE"

  info "Pulling selected images..."
  local update_args=()
  [[ ! -f "$DOCKER_DIR/update-images.json" ]] || update_args=(-f "$DOCKER_DIR/update-images.json")
  docker_compose_cmd -f "$FINAL_COMPOSE_FILE" "${update_args[@]}" --env-file "$FINAL_ENV_FILE" pull
  echo ""
  info "Starting stack..."
  docker_compose_cmd -f "$FINAL_COMPOSE_FILE" "${update_args[@]}" --env-file "$FINAL_ENV_FILE" up -d
  echo ""
  info "Current containers:"
  docker_compose_cmd -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" ps
}
