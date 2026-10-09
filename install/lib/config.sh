#!/usr/bin/env bash

# ── Global configuration ────────────────────────────────────────────────────

REPO_URL="https://github.com/mowglinext/mowglinext.git"

# Fork-local override (gitignored). Create install/lib/config.local.sh to
# point at your own fork without touching tracked files:
#   REPO_URL="https://github.com/<you>/mowglinext.git"
_config_local="${BASH_SOURCE[0]%/*}/config.local.sh"
# shellcheck source=/dev/null
[[ -f "$_config_local" ]] && source "$_config_local"
unset _config_local

REPO_BRANCH="${REPO_BRANCH:-}"
# IMAGE_TAG selects which GHCR image tag to pull. "main" = stable,
# "dev" = integration, and feature branches can use custom tags such as
# "feat-universal-gnss-integration". Can be overridden from docker/.env,
# a preset, or `--image-tag=` on the CLI. recompute_image_defaults()
# rebuilds the *_IMAGE_DEFAULT vars from the live IMAGE_TAG.
IMAGE_TAG="${IMAGE_TAG:-main}"
REPO_DIR="${MOWGLI_HOME:-$HOME/mowglinext}"
DOCKER_SUBDIR="install"
INSTALL_DIR="${REPO_DIR}/${DOCKER_SUBDIR}"
DOCKER_DIR="$REPO_DIR/docker"
COMPOSE_SRC_DIR="$INSTALL_DIR/compose"
FINAL_COMPOSE_FILE="$DOCKER_DIR/docker-compose.yaml"
FINAL_ENV_FILE="$DOCKER_DIR/.env"
UDEV_RULES_FILE="/etc/udev/rules.d/50-mowgli.rules"

# Derive the GHCR image prefix from REPO_URL so forks automatically point
# at their own registry namespace. Strips the trailing .git and extracts
# the owner/repo path from the GitHub URL.
_ghcr_prefix() {
  local path="${REPO_URL%.git}"
  path="${path##*github.com/}"
  printf 'ghcr.io/%s' "$path"
}

recompute_image_defaults() {
  local prefix
  prefix="$(_ghcr_prefix)"

  MOWGLI_ROS2_IMAGE_DEFAULT="${prefix}/mowgli-ros2:${IMAGE_TAG}"
  LIDAR_LDLIDAR_IMAGE_DEFAULT="${prefix}/lidar-ldlidar:${IMAGE_TAG}"
  LIDAR_RPLIDAR_IMAGE_DEFAULT="${prefix}/lidar-rplidar:${IMAGE_TAG}"
  LIDAR_STL27L_IMAGE_DEFAULT="${prefix}/lidar-stl27l:${IMAGE_TAG}"
  MAVROS_IMAGE_DEFAULT="${prefix}/mavros:${IMAGE_TAG}"
  OPENMOWER_IMAGE_DEFAULT="${prefix}/openmower:${IMAGE_TAG}"
  GUI_IMAGE_DEFAULT="${prefix}/mowglinext-gui:${IMAGE_TAG}"
  # Universal GNSS is a separately released runtime. Never derive it from
  # MowgliNext IMAGE_TAG; the integration targets ROS 2 Lyrical.
  # Pinned by DIGEST, not only by tag: this container owns the GNSS serial
  # port and runs privileged-adjacent on every robot, and a tag on a third-party
  # registry can be re-pushed. Must match install/deployment.json (test-gated).
  UNIVERSAL_GNSS_IMAGE_DEFAULT="ghcr.io/pepeuch/universal-gnss-ros2-lyrical:v0.7.1-rc3@sha256:4e7960132882f2f83fb2b1e7d1430b4dfd00081d4be15d7d8ab20dacf7f22bc5"
}

is_release_image_channel() {
  case "${1:-}" in
    main|dev) return 0 ;;
    *) return 1 ;;
  esac
}

sanitize_image_tag() {
  local raw="${1:-}"

  raw="${raw#refs/heads/}"
  raw="${raw#origin/}"
  raw="${raw,,}"
  raw="$(printf '%s' "$raw" | sed -E 's/[^a-z0-9_.-]+/-/g; s/^[.-]+//; s/[.-]+$//; s/-+/-/g')"
  raw="${raw:0:128}"
  raw="$(printf '%s' "$raw" | sed -E 's/^[.-]+//; s/[.-]+$//')"
  [ -n "$raw" ] || raw="current"

  printf '%s\n' "$raw"
}

is_valid_image_tag() {
  [[ "${1:-}" =~ ^[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}$ ]]
}

current_repo_branch_name() {
  local repo_dir="${1:-$REPO_DIR}"

  [ -d "$repo_dir/.git" ] || return 1
  git -C "$repo_dir" symbolic-ref --quiet --short HEAD 2>/dev/null
}

default_repo_branch_name() {
  current_repo_branch_name "$REPO_DIR" 2>/dev/null || printf 'main\n'
}

normalize_repo_branch_name() {
  local raw="${1:-}"

  raw="${raw#refs/heads/}"
  raw="${raw#origin/}"
  printf '%s\n' "$raw"
}

resolve_repo_branch_spec() {
  local spec="${1:-}"
  local current_branch

  case "$spec" in
    ""|current|current-branch|current_branch|keep)
      current_branch="$(current_repo_branch_name "$REPO_DIR" 2>/dev/null || true)"
      [ -n "$current_branch" ] || return 1
      printf '%s\n' "$current_branch"
      return 0
      ;;
  esac

  spec="$(normalize_repo_branch_name "$spec")"
  [ -n "$spec" ] || return 1
  printf '%s\n' "$spec"
}

resolve_image_tag_spec() {
  local spec="${1:-}"
  local current_branch

  case "$spec" in
    ""|current|current-branch|current_branch)
      current_branch="$(current_repo_branch_name "$REPO_DIR" 2>/dev/null || true)"
      [ -n "$current_branch" ] || return 1
      printf '%s\n' "$(sanitize_image_tag "$current_branch")"
      return 0
      ;;
  esac

  if is_valid_image_tag "$spec"; then
    printf '%s\n' "$spec"
    return 0
  fi

  spec="$(sanitize_image_tag "$spec")"
  if is_valid_image_tag "$spec"; then
    printf '%s\n' "$spec"
    return 0
  fi

  return 1
}

if [[ -z "${REPO_BRANCH:-}" ]]; then
  REPO_BRANCH="$(default_repo_branch_name)"
fi

select_repo_branch() {
  if [[ "${REPO_BRANCH_PRESET:-false}" == "true" ]]; then
    info "Repository branch pre-selected: ${REPO_BRANCH}"
    return 0
  fi

  if [ ! -d "$REPO_DIR/.git" ]; then
    return 0
  fi

  local current_branch current_ref previous requested_branch
  current_branch="$(current_repo_branch_name "$REPO_DIR" 2>/dev/null || true)"
  current_ref="$(repo_current_ref "$REPO_DIR" 2>/dev/null || true)"
  previous="${REPO_BRANCH:-${current_branch:-main}}"

  if [[ "${NON_INTERACTIVE:-false}" == "true" ]]; then
    REPO_BRANCH="${current_branch:-$previous}"
    info "Repository branch: ${REPO_BRANCH} (current checkout)"
    return 0
  fi

  echo ""
  echo -e "${CYAN:-}${BOLD:-}Repository branch${NC:-}"
  if [[ -n "$current_branch" ]]; then
    echo "  1) keep current checkout — ${current_branch} (default)"
  else
    echo "  1) keep current checkout — detached HEAD (${current_ref}) (default)"
  fi
  echo "  2) main"
  echo "  3) dev"
  echo "  4) custom branch"
  echo ""
  prompt "Choose" "1"

  case "$REPLY" in
    1|keep|current)
      REPO_BRANCH="${current_branch:-$previous}"
      ;;
    2|main)
      REPO_BRANCH="main"
      ;;
    3|dev)
      REPO_BRANCH="dev"
      ;;
    4|custom)
      prompt "Repository branch" "$previous"
      requested_branch="${REPLY:-$previous}"
      if ! REPO_BRANCH="$(resolve_repo_branch_spec "$requested_branch")"; then
        warn "Invalid repository branch '$requested_branch', keeping ${previous}"
        REPO_BRANCH="$previous"
      fi
      ;;
    *)
      warn "Invalid choice, keeping ${previous}"
      REPO_BRANCH="$previous"
      ;;
  esac

  info "Repository branch: ${REPO_BRANCH}"
}

select_image_channel() {
  # --image-tag= flag wins. Preset files (web composer) can also pin IMAGE_TAG.
  if [[ "${IMAGE_CHANNEL_PRESET:-false}" == "true" ]]; then
    info "Image tag pre-selected: ${IMAGE_TAG}"
    recompute_image_defaults
    return 0
  fi

  if [[ "${PRESET_LOADED:-false}" == "true" ]] \
    && [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ] \
    && declare -F preset_key_loaded >/dev/null \
    && preset_key_loaded IMAGE_TAG; then
    info "Image tag pre-selected from preset: ${IMAGE_TAG}"
    recompute_image_defaults
    return 0
  fi

  local previous="${IMAGE_TAG:-main}"
  local current_branch=""
  local current_branch_tag=""
  # If IMAGE_TAG isn't recorded in .env but image refs are, infer the channel
  # from those — otherwise upgrading users who already pulled :dev would get
  # silently flipped to :main just because the new key didn't exist yet.
  if ! is_valid_image_tag "$previous" || [[ "$previous" == "main" && "${GPS_IMAGE:-${MOWGLI_ROS2_IMAGE:-}}" == *":dev" ]]; then
    if [[ "${GPS_IMAGE:-${MOWGLI_ROS2_IMAGE:-}}" == *":dev" ]]; then
      previous="dev"
    else
      previous="main"
    fi
  fi

  current_branch="$(current_repo_branch_name "$REPO_DIR" 2>/dev/null || true)"
  if [[ -n "$current_branch" && "$current_branch" != "main" && "$current_branch" != "dev" ]]; then
    current_branch_tag="$(sanitize_image_tag "$current_branch")"
  fi

  if [[ -n "$current_branch_tag" ]] && { is_release_image_channel "$previous" || [[ -z "$previous" ]]; }; then
    previous="$current_branch_tag"
  fi

  if [[ "${NON_INTERACTIVE:-false}" == "true" ]]; then
    IMAGE_TAG="$previous"
    info "Image tag: ${IMAGE_TAG}"
    recompute_image_defaults
    return 0
  fi

  echo ""
  echo -e "${CYAN:-}${BOLD:-}Image tag${NC:-}"
  echo "  1) main — stable published images"
  echo "  2) dev  — integration images"
  if [[ -n "$current_branch_tag" ]]; then
    echo "  3) current branch / custom tag — keep checkout ${current_branch} (default tag: ${current_branch_tag})"
  else
    echo "  3) custom tag — keep the current checkout and choose an explicit image tag"
  fi
  echo ""
  local default_choice="1"
  [[ "$previous" == "dev" ]] && default_choice="2"
  if ! is_release_image_channel "$previous"; then
    default_choice="3"
  fi
  prompt "Choose" "$default_choice"

  case "$REPLY" in
    1|main) IMAGE_TAG="main" ;;
    2|dev)  IMAGE_TAG="dev" ;;
    3|current|custom)
      local custom_default="$previous"
      local requested_tag

      if is_release_image_channel "$custom_default"; then
        custom_default="${current_branch_tag:-$custom_default}"
      fi
      [ -n "$custom_default" ] || custom_default="main"

      prompt "Image tag" "$custom_default"
      requested_tag="${REPLY:-$custom_default}"
      if ! IMAGE_TAG="$(resolve_image_tag_spec "$requested_tag")"; then
        warn "Invalid image tag '$requested_tag', keeping ${previous}"
        IMAGE_TAG="$previous"
      fi
      ;;
    *)
      warn "Invalid choice, keeping ${previous}"
      IMAGE_TAG="$previous"
      ;;
  esac

  info "Image tag: ${IMAGE_TAG}"
  if [[ -n "$current_branch" && "$current_branch" != "$IMAGE_TAG" ]]; then
    info "Repository checkout stays on ${current_branch}; only container images use tag ${IMAGE_TAG}."
  fi
  recompute_image_defaults
}

recompute_image_defaults

CHECK_ONLY=false
# --only=<step> (mowglinext/mowglinext#632): run exactly one named step
# instead of the full install flow, for an operator adding one thing (e.g.
# the host updater) to an already-working install. Empty = full flow, the
# default. See run_only_step()/list_only_steps() in mowglinext.sh for the
# valid step names.
ONLY_STEP=""
# Mode: install (default) | update | repair | check | only (--only=<step>).
INSTALL_MODE="install"
# Set when the mode came from the command line; a bare run on an installed
# robot with a terminal asks instead of silently reinstalling.
MODE_EXPLICIT=false
# --non-interactive: every prompt takes its default. Also switched on
# automatically when no controlling terminal is available (parse_args).
NON_INTERACTIVE=false
# Distinguishes an explicit --yes from a missing tty: only the former may
# confirm a destructive mode (uninstall).
NON_INTERACTIVE_EXPLICIT=false
# --no-updater: install without the host updater service (manual updates only).
INSTALL_UPDATER=true
CLI_PRESET=false
GNSS_RECEIVER_FAMILY_CLI_PRESET=false
GNSS_CONNECTION_CLI_PRESET=false
GNSS_SERIAL_DEVICE_CLI_PRESET=false
GNSS_SERIAL_BAUD_CLI_PRESET=false
GNSS_FRAME_ID_CLI_PRESET=false
GNSS_NTRIP_GGA_ENABLED_CLI_PRESET=false
GNSS_NTRIP_GGA_INTERVAL_S_CLI_PRESET=false
CONFIG_NTRIP_ENABLED_EXPLICIT=false
CONFIG_NTRIP_HOST_EXPLICIT=false
CONFIG_NTRIP_PORT_EXPLICIT=false
CONFIG_NTRIP_USER_EXPLICIT=false
CONFIG_NTRIP_PASSWORD_EXPLICIT=false
CONFIG_NTRIP_MOUNTPOINT_EXPLICIT=false

installer_main_command() {
  printf 'bash %q' "$REPO_DIR/install/mowglinext.sh"
}

rerun_check_command() {
  printf '%s --check' "$(installer_main_command)"
}

compose_restart_services_for_backend() {
  local backend="${1:-${HARDWARE_BACKEND:-mowgli}}"
  local services=()
  local gnss_backend
  local gnss_stack
  local gnss_service

  if ! is_supported_hardware_backend "$backend"; then
    error "Unknown hardware backend: $backend (expected ${SUPPORTED_HARDWARE_BACKENDS// /, })"
    return 1
  fi

  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
  gnss_stack="$(effective_gnss_stack 2>/dev/null || true)"
  if [[ "$gnss_stack" != "disabled" ]] && is_supported_gnss_backend "$gnss_backend"; then
    gnss_service="$(compose_gnss_service_name "$gnss_backend" 2>/dev/null || true)"
    [ -n "$gnss_service" ] && services+=("$gnss_service")
  fi

  if [[ "$backend" == "mavros" ]]; then
    services+=(mavros)
  fi
  # The OpenMower bridge reads mowgli_robot.yaml too, so it restarts with it.
  if [[ "$backend" == "openmower" ]]; then
    services+=(openmower)
  fi
  services+=(mowgli)

  printf '%s\n' "${services[@]}"
}

print_restart_command_for_backend() {
  local backend="${1:-${HARDWARE_BACKEND:-mowgli}}"
  local services=()
  local service

  mapfile -t services < <(compose_restart_services_for_backend "$backend")

  printf 'docker compose -f %q --env-file %q restart' "$FINAL_COMPOSE_FILE" "$FINAL_ENV_FILE"
  for service in "${services[@]}"; do
    printf ' %q' "$service"
  done
  printf '\n'
}

warn_legacy_nmea_backend_once() {
  if [ "${LEGACY_GNSS_NMEA_WARNING_SHOWN:-false}" = "true" ]; then
    return 0
  fi

  warn "Legacy GNSS_BACKEND=nmea detected — normalizing to Universal GNSS with GNSS_RECEIVER_FAMILY=nmea."
  LEGACY_GNSS_NMEA_WARNING_SHOWN=true
}

normalize_gnss_backend() {
  local backend="${1:-}"

  case "${backend,,}" in
    ""|universal|gps|ublox|unicore)
      printf 'universal\n'
      ;;
    nmea)
      warn_legacy_nmea_backend_once
      printf 'universal\n'
      ;;
    legacy)
      printf 'universal\n'
      ;;
    disabled)
      printf 'disabled\n'
      ;;
    *)
      printf '%s\n' "${backend,,}"
      ;;
  esac
}

normalize_gnss_status_source() {
  local status_source="${1:-}"

  case "${status_source,,}" in
    universal)
      printf 'universal\n'
      ;;
    external|disabled|off|false|0)
      printf 'external\n'
      ;;
    legacy|mowgli_local|local|"")
      printf 'universal\n'
      ;;
    *)
      printf '%s\n' "${status_source,,}"
      ;;
  esac
}

default_gnss_status_source() {
  printf 'universal\n'
}

default_gnss_stack() {
  printf 'universal\n'
}

normalize_gnss_stack() {
  local stack="${1:-}"

  case "${stack,,}" in
    "")
      printf '%s\n' "$(default_gnss_stack)"
      ;;
    fallback|legacy)
      printf 'universal\n'
      ;;
    universal|disabled)
      printf '%s\n' "${stack,,}"
      ;;
    *)
      printf '%s\n' "${stack,,}"
      ;;
  esac
}

normalize_gnss_receiver_family() {
  local receiver_family="${1:-}"

  case "${receiver_family,,}" in
    ""|auto)
      printf 'auto\n'
      ;;
    u-blox|ublox)
      printf 'ublox\n'
      ;;
    unicore|nmea)
      printf '%s\n' "${receiver_family,,}"
      ;;
    *)
      printf '%s\n' "${receiver_family,,}"
      ;;
  esac
}

gnss_connection_from_serial_device() {
  local serial_device="${1:-${GNSS_SERIAL_DEVICE:-}}"
  local hinted_connection="${GNSS_CONNECTION_HINT:-}"

  if [[ -n "$serial_device" ]]; then
    case "$serial_device" in
      /dev/serial/by-id/*|/dev/ttyACM*|/dev/ttyUSB*)
        printf 'usb\n'
        return 0
        ;;
      /dev/ttyAMA*|/dev/ttyS*|/dev/ttyTHS*|/dev/ttyHS*)
        printf 'uart\n'
        return 0
        ;;
    esac
  fi

  if [[ -n "$hinted_connection" ]]; then
    printf '%s\n' "${hinted_connection,,}"
    return 0
  fi

  printf 'uart\n'
}

# Every hardware backend the installer can select. Lives HERE, next to the
# other shared predicates, and NOT in backend_choice.sh (which owns only the
# interactive selection flow): docker/stack.sh sources common/config/docker/
# deploy/compose and then calls build_compose_stack, so a guard that compose.sh
# needs must be defined in a lib stack.sh actually sources. When this lived in
# backend_choice.sh, `stack.sh regen` died with "command not found" and — since
# `! <missing command>` is TRUE — reported "Unknown HARDWARE_BACKEND" for every
# backend including the default one.
# Keep in lockstep with SUPPORTED_HARDWARE_BACKENDS in
# ros2/src/mowgli_bringup/launch/mowgli.launch.py (pinned by
# test_hardware_backend_launch.py) and the composer in docs/index.html.
SUPPORTED_HARDWARE_BACKENDS="mowgli mavros openmower"

is_supported_hardware_backend() {
  case "${1:-}" in
    mowgli|mavros|openmower) return 0 ;;
    *) return 1 ;;
  esac
}

list_supported_gnss_backends() {
  printf 'universal disabled\n'
}

is_supported_gnss_backend() {
  local backend="${1:-}"

  case "${backend,,}" in
    universal|disabled)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

effective_gnss_backend() {
  local backend="${1:-${GNSS_BACKEND:-universal}}"
  local stack

  backend="$(normalize_gnss_backend "$backend")"
  stack="$(effective_gnss_stack 2>/dev/null || true)"

  if [[ "$backend" == "disabled" || "$stack" == "disabled" ]]; then
    printf 'disabled\n'
    return 0
  fi

  printf 'universal\n'
  return 0
}

effective_gnss_stack() {
  local stack="${1:-${GNSS_STACK:-}}"
  local status_source
  local raw_backend

  raw_backend="$(normalize_gnss_backend "${GNSS_BACKEND:-}")"

  if [[ "$raw_backend" == "disabled" ]]; then
    printf 'disabled\n'
    return 0
  fi

  if [[ -z "$stack" ]]; then
    status_source="$(normalize_gnss_status_source "${GNSS_STATUS_SOURCE:-$(default_gnss_status_source)}")"
    if [[ "$status_source" == "external" ]]; then
      stack="disabled"
    else
      stack="universal"
    fi
  fi

  stack="$(normalize_gnss_stack "$stack")"
  printf '%s\n' "$stack"

  case "$stack" in
    universal|disabled)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

gnss_receiver_family_from_state() {
  local receiver_family="${GNSS_RECEIVER_FAMILY:-}"

  if [[ -n "$receiver_family" ]]; then
    printf '%s\n' "${receiver_family,,}"
    return 0
  fi

  printf 'auto\n'
}

gnss_transport_from_state() {
  local transport="${GNSS_TRANSPORT:-serial}"
  printf '%s\n' "${transport,,}"
}

# On OpenMower v1 the GPS sits on ttyAMA2 (open_mower_ros, kernel >= 6.1.28);
# ttyAMA4 — the usual GNSS port — is the mow xESC there.
default_gnss_uart_device() {
  if [[ "${HARDWARE_BACKEND:-}" == "openmower" ]]; then
    printf '/dev/ttyAMA2\n'
  else
    printf '/dev/ttyAMA4\n'
  fi
}

gnss_serial_device_from_state() {
  if [[ -n "${GNSS_SERIAL_DEVICE:-}" ]]; then
    printf '%s\n' "$GNSS_SERIAL_DEVICE"
    return 0
  fi

  case "$(gnss_connection_from_serial_device)" in
    usb)  printf '/dev/serial/by-id/usb-stub\n' ;;
    *)    default_gnss_uart_device ;;
  esac
}

gnss_serial_baud_from_state() {
  printf '%s\n' "${GNSS_SERIAL_BAUD:-921600}"
}

existing_yaml_value() {
  local key="${1:?existing_yaml_value: missing key}"
  local yaml_file="${2:-$DOCKER_DIR/config/mowgli/mowgli_robot.yaml}"
  local line value

  [ -f "$yaml_file" ] || return 0

  line="$(grep -E "^[[:space:]]+${key}:" "$yaml_file" 2>/dev/null | head -1 || true)"
  value="${line#*:}"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%%#*}"
  value="${value%"${value##*[![:space:]]}"}"
  value="${value#\"}"
  value="${value%\"}"
  value="${value#\'}"
  value="${value%\'}"
  printf '%s\n' "$value"
}

gnss_installer_key_is_explicit() {
  local key="${1:?gnss_installer_key_is_explicit: missing key}"

  case "$key" in
    GNSS_RECEIVER_FAMILY)
      [[ "${GNSS_RECEIVER_FAMILY_CLI_PRESET:-false}" == "true" ]] && return 0
      ;;
    GNSS_TRANSPORT)
      [[ "${GNSS_CONNECTION_CLI_PRESET:-false}" == "true" ]] && return 0
      ;;
    GNSS_SERIAL_DEVICE)
      if [[ "${GNSS_SERIAL_DEVICE_CLI_PRESET:-false}" == "true" \
        || "${GNSS_CONNECTION_CLI_PRESET:-false}" == "true" ]]; then
        return 0
      fi
      ;;
    GNSS_SERIAL_BAUD)
      [[ "${GNSS_SERIAL_BAUD_CLI_PRESET:-false}" == "true" ]] && return 0
      ;;
    GNSS_FRAME_ID)
      [[ "${GNSS_FRAME_ID_CLI_PRESET:-false}" == "true" ]] && return 0
      ;;
    GNSS_NTRIP_GGA_ENABLED)
      [[ "${GNSS_NTRIP_GGA_ENABLED_CLI_PRESET:-false}" == "true" ]] && return 0
      ;;
    GNSS_NTRIP_GGA_INTERVAL_S)
      [[ "${GNSS_NTRIP_GGA_INTERVAL_S_CLI_PRESET:-false}" == "true" ]] && return 0
      ;;
  esac

  if declare -F preset_key_loaded >/dev/null 2>&1; then
    case "$key" in
      GNSS_RECEIVER_FAMILY|GNSS_TRANSPORT|GNSS_SERIAL_DEVICE|GNSS_SERIAL_BAUD|\
      GNSS_FRAME_ID|GNSS_NTRIP_GGA_ENABLED|GNSS_NTRIP_GGA_INTERVAL_S)
        preset_key_loaded "$key" && return 0
        ;;
    esac
  fi

  return 1
}

preserved_gnss_value() {
  local explicit="${1:-false}"
  local current="${2:-}"
  local previous="${3:-}"
  local default_value="${4:-}"

  if [[ "$explicit" == "true" ]]; then
    printf '%s\n' "$current"
    return 0
  fi

  if [[ -n "$previous" ]]; then
    printf '%s\n' "$previous"
    return 0
  fi

  if [[ -n "$current" ]]; then
    printf '%s\n' "$current"
    return 0
  fi

  printf '%s\n' "$default_value"
}

apply_existing_yaml_gnss_state() {
  local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  [ -f "$yaml_file" ] || return 0

  local prev_receiver_family prev_transport prev_serial_device prev_serial_baud
  local prev_frame_id prev_ntrip_gga_enabled prev_ntrip_gga_interval_s

  prev_receiver_family="$(existing_yaml_value gnss_receiver_family "$yaml_file")"
  prev_transport="$(existing_yaml_value gnss_transport "$yaml_file")"
  prev_serial_device="$(existing_yaml_value gnss_serial_device "$yaml_file")"
  prev_serial_baud="$(existing_yaml_value gnss_serial_baud "$yaml_file")"
  prev_frame_id="$(existing_yaml_value gnss_frame_id "$yaml_file")"
  prev_ntrip_gga_enabled="$(existing_yaml_value gnss_ntrip_gga_enabled "$yaml_file")"
  prev_ntrip_gga_interval_s="$(existing_yaml_value gnss_ntrip_gga_interval_s "$yaml_file")"

  if ! gnss_installer_key_is_explicit GNSS_RECEIVER_FAMILY && [[ -n "$prev_receiver_family" ]]; then
    GNSS_RECEIVER_FAMILY="$prev_receiver_family"
  fi
  if ! gnss_installer_key_is_explicit GNSS_TRANSPORT && [[ -n "$prev_transport" ]]; then
    GNSS_TRANSPORT="$prev_transport"
  fi
  if ! gnss_installer_key_is_explicit GNSS_SERIAL_DEVICE && [[ -n "$prev_serial_device" ]]; then
    GNSS_SERIAL_DEVICE="$prev_serial_device"
  fi
  if ! gnss_installer_key_is_explicit GNSS_SERIAL_BAUD && [[ -n "$prev_serial_baud" ]]; then
    GNSS_SERIAL_BAUD="$prev_serial_baud"
  fi
  if ! gnss_installer_key_is_explicit GNSS_FRAME_ID && [[ -n "$prev_frame_id" ]]; then
    GNSS_FRAME_ID="$prev_frame_id"
  fi
  if ! gnss_installer_key_is_explicit GNSS_NTRIP_GGA_ENABLED && [[ -n "$prev_ntrip_gga_enabled" ]]; then
    GNSS_NTRIP_GGA_ENABLED="$prev_ntrip_gga_enabled"
  fi
  if ! gnss_installer_key_is_explicit GNSS_NTRIP_GGA_INTERVAL_S && [[ -n "$prev_ntrip_gga_interval_s" ]]; then
    GNSS_NTRIP_GGA_INTERVAL_S="$prev_ntrip_gga_interval_s"
  fi
}

compose_gnss_service_name() {
  local backend="${1:-$(effective_gnss_backend)}"

  case "$backend" in
    universal)
      printf 'gps\n'
      ;;
    disabled)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

compose_gnss_container_name() {
  local backend="${1:-$(effective_gnss_backend)}"
  local service_name

  service_name="$(compose_gnss_service_name "$backend" 2>/dev/null || true)"
  if [ -z "$service_name" ]; then
    return 0
  fi

  printf 'mowgli-gps\n'
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      install|update|repair|check|uninstall)
        INSTALL_MODE="$1"
        MODE_EXPLICIT=true
        ;;
      --check)
        INSTALL_MODE="check"
        MODE_EXPLICIT=true
        ;;
      --only=*)
        ONLY_STEP="${1#*=}"
        INSTALL_MODE="only"
        MODE_EXPLICIT=true
        ;;
      --non-interactive|--yes|-y)
        NON_INTERACTIVE=true
        NON_INTERACTIVE_EXPLICIT=true
        ;;
      --no-updater)
        INSTALL_UPDATER=false
        ;;
      --mqtt=*)
        case "${1#*=}" in
          on|true|yes)  ENABLE_MQTT="true" ;;
          off|false|no) ENABLE_MQTT="false" ;;
          *) error "Unknown --mqtt value: ${1#*=} (expected on or off)"; exit 1 ;;
        esac
        ;;
      --help|-h)
        print_usage
        exit 0
        ;;
      --lang=*)
        MOWGLI_LANG="${1#*=}"
        ;;
      --branch=*)
        local branch_spec="${1#*=}"
        if ! REPO_BRANCH="$(resolve_repo_branch_spec "$branch_spec")"; then
          error "Unknown repository branch: $branch_spec"
          exit 1
        fi
        REPO_BRANCH_PRESET=true
        ;;
      --channel=*|--image-tag=*)
        local tag_spec="${1#*=}"
        if [[ "$1" == --channel=* ]]; then
          warn "--channel is deprecated; use --image-tag=<main|dev>."
          if ! is_release_image_channel "$tag_spec"; then
            error "Unknown image channel: $tag_spec (expected main or dev)"
            exit 1
          fi
        fi
        if ! IMAGE_TAG="$(resolve_image_tag_spec "$tag_spec")"; then
          error "Unknown image tag: $tag_spec (expected main, dev, current, or a custom Docker tag)"
          exit 1
        fi
        IMAGE_CHANNEL_PRESET=true
        recompute_image_defaults
        ;;
      --backend=*)
        CLI_PRESET=true
        local backend_spec="${1#*=}"
        case "$backend_spec" in
          mowgli)
            HARDWARE_BACKEND="mowgli"
            MAVROS_BY_ID=""
            ;;
          mavros)
            HARDWARE_BACKEND="mavros"
            ;;
          openmower)
            HARDWARE_BACKEND="openmower"
            MAVROS_BY_ID=""
            ;;
          *)
            error "Unknown hardware backend: $backend_spec (expected mowgli, mavros or openmower)"
            exit 1
            ;;
        esac
        ;;
      --gnss=*)
        CLI_PRESET=true
        GNSS_RECEIVER_FAMILY_CLI_PRESET=true
        local gnss_spec="${1#*=}"
        case "$gnss_spec" in
          auto)
            GNSS_STACK="universal"
            GNSS_STATUS_SOURCE="universal"
            GNSS_BACKEND="universal"
            GNSS_RECEIVER_FAMILY="auto"
            ;;
          gps)
            GNSS_STACK="${GNSS_STACK:-universal}"
            GNSS_STATUS_SOURCE="${GNSS_STATUS_SOURCE:-universal}"
            GNSS_BACKEND="universal"
            GNSS_RECEIVER_FAMILY="auto"
            ;;
          ublox|unicore|nmea)
            GNSS_STACK="${GNSS_STACK:-universal}"
            GNSS_STATUS_SOURCE="${GNSS_STATUS_SOURCE:-universal}"
            GNSS_RECEIVER_FAMILY="$gnss_spec"
            GNSS_BACKEND="universal"
            ;;
          *)
            error "Unknown GNSS backend: $gnss_spec (expected auto|gps|ublox|unicore|nmea)"
            exit 1
            ;;
        esac
        ;;
      --gnss-connection=*)
        CLI_PRESET=true
        GNSS_CONNECTION_CLI_PRESET=true
        case "${1#*=}" in
          usb|USB)
            GNSS_CONNECTION_HINT="usb"
            ;;
          uart|UART)
            GNSS_CONNECTION_HINT="uart"
            ;;
          *)
            error "Unknown GNSS connection: ${1#*=} (expected usb or uart)"
            exit 1
            ;;
        esac
        ;;
      --gnss-device=*)
        CLI_PRESET=true
        GNSS_SERIAL_DEVICE_CLI_PRESET=true
        GNSS_SERIAL_DEVICE="${1#*=}"
        ;;
      --gnss-baud=*)
        CLI_PRESET=true
        GNSS_SERIAL_BAUD_CLI_PRESET=true
        local gnss_baud_spec="${1#*=}"
        case "$gnss_baud_spec" in
          auto)
            GNSS_SERIAL_BAUD=""
            ;;
          ''|*[!0-9]*)
            error "Unknown GNSS baud: $gnss_baud_spec (expected auto or a numeric baud rate)"
            exit 1
            ;;
          *)
            GNSS_SERIAL_BAUD="$gnss_baud_spec"
            ;;
        esac
        ;;
      --gnss-receiver-family=*)
        CLI_PRESET=true
        GNSS_RECEIVER_FAMILY_CLI_PRESET=true
        local receiver_family_spec
        receiver_family_spec="$(normalize_gnss_receiver_family "${1#*=}")"
        case "$receiver_family_spec" in
          auto|ublox|unicore|nmea)
            GNSS_RECEIVER_FAMILY="$receiver_family_spec"
            GNSS_BACKEND="universal"
            ;;
          *)
            error "Unknown GNSS receiver family: ${1#*=} (expected auto|ublox|unicore|nmea)"
            exit 1
            ;;
        esac
        ;;
      --gps=*)
        CLI_PRESET=true
        local gps_spec="${1#*=}"
        # Deprecated legacy alias. Normalize it onto the Universal GNSS
        # receiver-family + serial-connection contract.
        local gps_proto="${gps_spec%%-*}"
        local gps_conn="${gps_spec##*-}"
        case "$gps_proto" in
          ubx)  GNSS_RECEIVER_FAMILY="${GNSS_RECEIVER_FAMILY:-auto}" ;;
          nmea) GNSS_RECEIVER_FAMILY="nmea" ;;
          *)    error "Unknown GPS protocol: $gps_proto (expected ubx or nmea)"; exit 1 ;;
        esac
        case "$gps_conn" in
          usb)  GNSS_CONNECTION_HINT="usb" ;;
          uart) GNSS_CONNECTION_HINT="uart" ;;
          *)    error "Unknown GPS connection: $gps_conn (expected usb or uart)"; exit 1 ;;
        esac
        GNSS_BACKEND="universal"
        ;;
      --gps-uart=*)
        GNSS_SERIAL_DEVICE="${1#*=}"
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
            LIDAR_ENABLED="true"; LIDAR_TYPE="stl27l"; LIDAR_MODEL="LDLiDAR_STL27L"
            LIDAR_CONNECTION="usb"; LIDAR_BAUD="921600"; LIDAR_UART_DEVICE=""
            ;;
          stl27l-uart)
            LIDAR_ENABLED="true"; LIDAR_TYPE="stl27l"; LIDAR_MODEL="LDLiDAR_STL27L"
            LIDAR_CONNECTION="uart"; LIDAR_BAUD="921600"
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
      --tfluna=*|--tfluna-front-uart=*|--tfluna-edge-uart=*)
        warn "TF-Luna rangefinders are no longer configured by the installer; ignoring $1"
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

  # No terminal to ask on (curl | bash without /dev/tty, CI, cron): behave as
  # --non-interactive rather than letting `read` fail or return garbage.
  if [[ "$NON_INTERACTIVE" != "true" ]] && ! { : </dev/tty; } 2>/dev/null; then
    NON_INTERACTIVE=true
  fi
}

# A bare `mowglinext.sh` on a robot that is already installed: ask what the
# operator wants instead of walking them through a full reinstall. Only with
# a terminal — a composer command or a cron job keeps the documented default.
select_mode() {
  [[ "$MODE_EXPLICIT" != "true" ]] || return 0
  [ -f "$FINAL_ENV_FILE" ] || return 0

  # No terminal to ask on: a bare run on an installed robot means "update",
  # exactly what the menu would default to. Hardware flags (a composer
  # command) still mean a reconfiguring install.
  if [[ "${NON_INTERACTIVE:-false}" == "true" ]]; then
    if [[ "${CLI_PRESET:-false}" != "true" ]]; then
      INSTALL_MODE="update"
      info "$MSG_MODE_SELECTED update ($MSG_MODE_NO_TTY)"
    fi
    return 0
  fi

  echo ""
  echo -e "${CYAN:-}${BOLD:-}$MSG_MODE_TITLE${NC:-}"
  echo "  1) $MSG_MODE_UPDATE"
  echo "  2) $MSG_MODE_REPAIR"
  echo "  3) $MSG_MODE_REINSTALL"
  echo "  4) $MSG_MODE_CHECK"
  echo "  5) $MSG_MODE_UNINSTALL"
  echo ""
  prompt "$MSG_CHOICE" "1"
  case "$REPLY" in
    1|update)  INSTALL_MODE="update" ;;
    2|repair)  INSTALL_MODE="repair" ;;
    3|install|reinstall) INSTALL_MODE="install" ;;
    4|check)   INSTALL_MODE="check" ;;
    5|uninstall) INSTALL_MODE="uninstall" ;;
    *) warn "$MSG_MODE_INVALID"; INSTALL_MODE="update" ;;
  esac
  info "$MSG_MODE_SELECTED $INSTALL_MODE"
}

print_usage() {
  cat <<'EOF'
Usage: mowglinext.sh [install|update|repair|check|uninstall] [options]

Modes
  install (default)  Full installation. Interactive when a terminal is attached.
  update             Manual update: sync the checkout to --branch (or the current
                     branch), regenerate docker/.env + compose, pull images, restart.
                     No host updater involved, no readiness or firmware-protocol gate.
  repair             Re-apply udev rules, UART overlays, sysctl, .env, compose and the
                     helper commands from the saved choices. Never asks anything.
  check              Diagnostics only (alias: --check).
  uninstall          Remove containers, images, the host updater, host files and the checkout.
                     Keeps the maps volume and docker/config/mowgli/mowgli_robot.yaml in place.
                     Asks for confirmation; without a terminal only an explicit --yes counts.

Options
  --non-interactive, --yes   Every unset choice takes its default (automatic without a tty)
  --branch=<main|dev|name>   Repository branch to check out (default: keep current)
  --image-tag=<main|dev|tag> Container image tag (default: follows the branch)
  --lang=<en|fr>             Installer language
  --backend=<mowgli|mavros>  Hardware backend (default: mowgli)
  --gnss-connection=<uart|usb>  GNSS serial link (default: uart)
  --gnss-device=<path>       GNSS serial device (default for uart: /dev/ttyAMA4,
                             /dev/ttyAMA2 with --backend=openmower)
  --gnss-baud=<n|auto>       GNSS serial baud (default: keep YAML value or 921600)
  --gnss-receiver-family=<auto|ublox|unicore|nmea>  First-boot receiver family
  --lidar=<none|ldlidar-uart|ldlidar-usb|rplidar-uart|rplidar-usb|stl27l-uart|stl27l-usb>
                             LiDAR (default: ldlidar-uart interactive, none otherwise)
  --lidar-uart=<path>        LiDAR UART device (default: /dev/ttyAMA5)
  --no-updater               Do not install the host updater service
  --mqtt=<on|off>            Run the mosquitto MQTT broker (Home Assistant integrations; default: off)
  --only=<step>              Run one step; see the list printed on an unknown name

Datum, NTRIP, the GNSS receiver profile and LiDAR mounting are configured in
the GUI onboarding wizard after the first start, not here.
EOF
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

  PREV_GNSS_RECEIVER_FAMILY="$(existing_yaml_value gnss_receiver_family "$yaml_file")"
  PREV_GNSS_TRANSPORT="$(existing_yaml_value gnss_transport "$yaml_file")"
  PREV_GNSS_SERIAL_DEVICE="$(existing_yaml_value gnss_serial_device "$yaml_file")"
  PREV_GNSS_SERIAL_BAUD="$(existing_yaml_value gnss_serial_baud "$yaml_file")"
  PREV_GNSS_CONFIG_BAUD="$(existing_yaml_value gnss_config_baud "$yaml_file")"
  PREV_GNSS_FRAME_ID="$(existing_yaml_value gnss_frame_id "$yaml_file")"
  PREV_GNSS_NTRIP_GGA_ENABLED="$(existing_yaml_value gnss_ntrip_gga_enabled "$yaml_file")"
  PREV_GNSS_NTRIP_GGA_INTERVAL_S="$(existing_yaml_value gnss_ntrip_gga_interval_s "$yaml_file")"
}

# Patch a single mowgli/ros__parameters key in-place. Preserves
# indentation, comments, and every other key. If the key is missing
# (only happens when the seeded template is older than the installer)
# we append it under the ros__parameters block.
# Run an in-place python edit of a yaml the installer OWNS (the seed it just
# created, the derived MAVROS copy). Never let a failed edit pass as success.
_yaml_python() {
  local file="${1:?_yaml_python: missing file}"
  if ! python3 - "$@"; then
    error "Could not update $file"
    return 1
  fi
}

_yaml_patch_key() {
  local file="$1" key="$2" value="$3"
  if grep -qE "^[[:space:]]+${key}:" "$file"; then
    # Replace value, preserving leading whitespace and any trailing
    # comment on the same line.
    _yaml_python "$file" "$key" "$value" <<'PY'
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
    _yaml_python "$file" "$key" "$value" <<'PY'
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

write_mavros_runtime_config() {
  local source="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  local target="$DOCKER_DIR/config/mavros/mowgli_robot.yaml"

  mkdir -p "$(dirname "$target")"
  cp "$source" "$target"
  # Universal GNSS is the sole NTRIP owner.
  _yaml_patch_key "$target" ntrip_enabled false || return 1
}

runtime_gnss_config_value() {
  local yaml_file="${1:?runtime_gnss_config_value: missing yaml file}"
  local key="${2:?runtime_gnss_config_value: missing key}"
  local default_value="${3:-}"
  local value

  value="$(existing_yaml_value "$key" "$yaml_file")"
  printf '%s\n' "${value:-$default_value}"
}

regenerate_sidecar_runtime_configs() {
  local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"

  if [ ! -f "$yaml_file" ]; then
    error "Cannot regenerate GNSS runtime config: missing source $yaml_file"
    return 1
  fi

  write_mavros_runtime_config
}

write_config() {
  local yaml_file="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  local template="$INSTALL_DIR/config/mowgli/mowgli_robot.yaml"
  local resolved_receiver_family resolved_transport resolved_serial_device
  local resolved_serial_baud resolved_frame_id

  # mowgli_robot.yaml is the OPERATOR's file: the GUI edits every key in it
  # (GNSS link and profile, LiDAR, datum, NTRIP, ...) and the ROS containers
  # write calibration results into it as root (Invariant 6). The installer
  # therefore writes it exactly once — when it does not exist yet — seeding
  # the sparse template with the hardware wiring just chosen. An existing
  # file is never patched: update/repair leave it alone, and a reinstall that
  # changes the wiring says so instead of overriding the GUI.
  if [ -f "$yaml_file" ]; then
    info "$yaml_file exists — left untouched (GNSS link, LiDAR presence and everything else are edited in the GUI)"
    write_mavros_runtime_config || return 1
    return 0
  fi

  : "${GNSS_RECEIVER_FAMILY:=auto}"
  : "${GNSS_TRANSPORT:=serial}"
  : "${GNSS_SERIAL_DEVICE:=$(default_gnss_uart_device)}"
  : "${GNSS_SERIAL_BAUD:=921600}"
  : "${GNSS_FRAME_ID:=gps_link}"

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

  resolved_receiver_family="$(normalize_gnss_receiver_family "${GNSS_RECEIVER_FAMILY}")"
  resolved_transport="${GNSS_TRANSPORT}"
  resolved_serial_device="${GNSS_SERIAL_DEVICE}"
  resolved_serial_baud="${GNSS_SERIAL_BAUD}"
  resolved_frame_id="${GNSS_FRAME_ID}"

  # Only the hardware wiring the installer just asked for. Datum, NTRIP,
  # the receiver profile, LiDAR pose and dock pose stay at the template
  # placeholders for the GUI onboarding wizard.
  _yaml_patch_key "$yaml_file" gnss_receiver_family "\"$resolved_receiver_family\"" || return 1
  _yaml_patch_key "$yaml_file" gnss_transport "\"$resolved_transport\"" || return 1
  _yaml_patch_key "$yaml_file" gnss_serial_device "\"$resolved_serial_device\"" || return 1
  _yaml_patch_key "$yaml_file" gnss_serial_baud "$resolved_serial_baud" || return 1
  _yaml_patch_key "$yaml_file" gnss_config_baud "$resolved_serial_baud" || return 1
  _yaml_patch_key "$yaml_file" gnss_frame_id "\"$resolved_frame_id\"" || return 1
  # The ROS2 launch reads the stack from the robot config only (no env
  # fallback), so the install-time choice has to land in the yaml.
  _yaml_patch_key "$yaml_file" gnss_stack "\"${GNSS_STACK:-universal}\"" || return 1

  # LiDAR hardware availability gates obstacle detection and scan-to-map localization.
  local lidar_on="false"
  [[ "${LIDAR_ENABLED:-false}" == "true" ]] && lidar_on="true"
  _yaml_patch_key "$yaml_file" lidar_enabled "$lidar_on" || return 1

  # No per-backend seeding: OpenMower's different defaults (xESC hall ticks,
  # the LowLevel board's own lift/tilt/charge values) live in
  # ros2/src/mowgli_bringup/config/backends/openmower.yaml, which every
  # consumer layers between the template and this sparse file. Writing them
  # here would pin them and break the GUI's reset-to-default — which is also
  # why the seed carries no ticks_per_meter.

  # The Universal GNSS sidecar reads mowgli_robot.yaml itself (see
  # install/compose/docker-compose.gps.yml): no derived parameter file.
  write_mavros_runtime_config || return 1

  info "Wrote $yaml_file"
}

run_mower_configuration_step() {
  ensure_default_configs && write_config
}

