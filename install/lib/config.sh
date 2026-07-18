#!/usr/bin/env bash
#interactive_config

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
  GPS_IMAGE_DEFAULT="${prefix}/gps:${IMAGE_TAG}"
  LIDAR_LDLIDAR_IMAGE_DEFAULT="${prefix}/lidar-ldlidar:${IMAGE_TAG}"
  LIDAR_RPLIDAR_IMAGE_DEFAULT="${prefix}/lidar-rplidar:${IMAGE_TAG}"
  LIDAR_STL27L_IMAGE_DEFAULT="${prefix}/lidar-stl27l:${IMAGE_TAG}"
  MAVROS_IMAGE_DEFAULT="${prefix}/mavros:${IMAGE_TAG}"
  GUI_IMAGE_DEFAULT="${prefix}/mowglinext-gui:${IMAGE_TAG}"
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

  if [[ "$backend" == "mavros" ]]; then
    services+=(mavros ntrip mowgli)
  else
    local gnss_backend
    local gnss_stack
    local gnss_service

    gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
    gnss_stack="$(effective_gnss_stack 2>/dev/null || true)"
    if [[ "$gnss_stack" != "disabled" ]] && is_supported_gnss_backend "$gnss_backend"; then
      gnss_service="$(compose_gnss_service_name "$gnss_backend" 2>/dev/null || true)"
      [ -n "$gnss_service" ] && services+=("$gnss_service")
    fi
    services+=(mowgli)
  fi

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

range_services_available() {
  local fragment

  for fragment in \
    "$COMPOSE_SRC_DIR/docker-compose.tfluna-front.yml" \
    "$COMPOSE_SRC_DIR/docker-compose.tfluna-edge.yml"
  do
    if [ ! -f "$fragment" ]; then
      return 1
    fi

    if grep -q 'ghcr.io/\.\.\.' "$fragment" 2>/dev/null; then
      return 1
    fi
  done

  return 0
}

vesc_service_available() {
  # Keep VESC disabled during the installer hardening phase until the
  # runtime/image contract is finalized and tested end-to-end.
  return 1
}

feature_is_available() {
  local feature="${1:-}"

  case "$feature" in
    range|rangefinders|tfluna|tfluna_front|tfluna_edge)
      range_services_available
      ;;
    vesc)
      vesc_service_available
      ;;
    *)
      return 0
      ;;
  esac
}

warn_unavailable_feature_once() {
  local feature="${1:?warn_unavailable_feature_once: missing feature}"
  local message="${2:?warn_unavailable_feature_once: missing message}"
  local flag_name="FEATURE_WARNING_${feature//[^A-Za-z0-9_]/_}"

  if [ "${!flag_name:-false}" = "true" ]; then
    return 0
  fi

  warn "$message"
  printf -v "$flag_name" '%s' "true"
}

effective_tfluna_front_enabled() {
  if [[ "${TFLUNA_FRONT_ENABLED:-false}" != "true" ]]; then
    return 1
  fi

  if feature_is_available tfluna_front; then
    return 0
  fi

  warn_unavailable_feature_once \
    tfluna \
    "TF-Luna rangefinder services are not available on this branch yet; requested TF-Luna options will be skipped."
  return 1
}

effective_tfluna_edge_enabled() {
  if [[ "${TFLUNA_EDGE_ENABLED:-false}" != "true" ]]; then
    return 1
  fi

  if feature_is_available tfluna_edge; then
    return 0
  fi

  warn_unavailable_feature_once \
    tfluna \
    "TF-Luna rangefinder services are not available on this branch yet; requested TF-Luna options will be skipped."
  return 1
}

effective_vesc_enabled() {
  if [[ "${ENABLE_VESC:-false}" != "true" ]]; then
    return 1
  fi

  if feature_is_available vesc; then
    return 0
  fi

  warn_unavailable_feature_once \
    vesc \
    "VESC support is not available on this branch yet; the VESC compose fragment will be skipped."
  return 1
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
  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    printf 'external\n'
  else
    printf 'universal\n'
  fi
}

default_gnss_stack() {
  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    printf 'disabled\n'
  else
    printf 'universal\n'
  fi
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

list_supported_gnss_backends() {
  local backends="universal"
  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]; then
    printf '%s disabled\n' "$backends"
  else
    printf '%s\n' "$backends"
  fi
}

is_supported_gnss_backend() {
  local backend="${1:-}"

  case "${backend,,}" in
    universal)
      return 0
      ;;
    disabled)
      [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" ]]
      return
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

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" || "$backend" == "disabled" || "$stack" == "disabled" ]]; then
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

  if [[ "${HARDWARE_BACKEND:-mowgli}" == "mavros" || "$raw_backend" == "disabled" ]]; then
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

gnss_serial_device_from_state() {
  if [[ -n "${GNSS_SERIAL_DEVICE:-}" ]]; then
    printf '%s\n' "$GNSS_SERIAL_DEVICE"
    return 0
  fi

  case "$(gnss_connection_from_serial_device)" in
    usb)  printf '/dev/serial/by-id/usb-stub\n' ;;
    *)    printf '/dev/ttyAMA4\n' ;;
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
      --check)
        CHECK_ONLY=true
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
          *)
            error "Unknown hardware backend: $backend_spec (expected mowgli or mavros)"
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

  PREV_DATUM_LAT="$(existing_yaml_value datum_lat "$yaml_file")"
  PREV_DATUM_LON="$(existing_yaml_value datum_lon "$yaml_file")"
  PREV_GNSS_RECEIVER_FAMILY="$(existing_yaml_value gnss_receiver_family "$yaml_file")"
  PREV_GNSS_TRANSPORT="$(existing_yaml_value gnss_transport "$yaml_file")"
  PREV_GNSS_SERIAL_DEVICE="$(existing_yaml_value gnss_serial_device "$yaml_file")"
  PREV_GNSS_SERIAL_BAUD="$(existing_yaml_value gnss_serial_baud "$yaml_file")"
  PREV_GNSS_FRAME_ID="$(existing_yaml_value gnss_frame_id "$yaml_file")"
  PREV_GNSS_NTRIP_GGA_ENABLED="$(existing_yaml_value gnss_ntrip_gga_enabled "$yaml_file")"
  PREV_GNSS_NTRIP_GGA_INTERVAL_S="$(existing_yaml_value gnss_ntrip_gga_interval_s "$yaml_file")"
  PREV_NTRIP_ENABLED="$(existing_yaml_value ntrip_enabled "$yaml_file")"
  PREV_NTRIP_HOST="$(existing_yaml_value ntrip_host "$yaml_file")"
  PREV_NTRIP_PORT="$(existing_yaml_value ntrip_port "$yaml_file")"
  PREV_NTRIP_USER="$(existing_yaml_value ntrip_user "$yaml_file")"
  PREV_NTRIP_PASSWORD="$(existing_yaml_value ntrip_password "$yaml_file")"
  PREV_NTRIP_MOUNTPOINT="$(existing_yaml_value ntrip_mountpoint "$yaml_file")"
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
  echo -e "${DIM}Default mountpoint NEAR picks the closest base via NMEA GGA.${NC}"
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
  CONFIG_NTRIP_ENABLED_EXPLICIT=true
  CONFIG_NTRIP_HOST_EXPLICIT=true
  CONFIG_NTRIP_PORT_EXPLICIT=true
  CONFIG_NTRIP_USER_EXPLICIT=true
  CONFIG_NTRIP_PASSWORD_EXPLICIT=true
  CONFIG_NTRIP_MOUNTPOINT_EXPLICIT=true
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
  local resolved_receiver_family resolved_transport resolved_serial_device
  local resolved_serial_baud resolved_frame_id resolved_ntrip_enabled
  local resolved_ntrip_host resolved_ntrip_port resolved_ntrip_user
  local resolved_ntrip_password resolved_ntrip_mountpoint
  local resolved_ntrip_gga_enabled resolved_ntrip_gga_interval_s

  : "${GNSS_RECEIVER_FAMILY:=auto}"
  : "${GNSS_TRANSPORT:=serial}"
  : "${GNSS_SERIAL_DEVICE:=/dev/ttyAMA4}"
  : "${GNSS_SERIAL_BAUD:=921600}"
  : "${GNSS_FRAME_ID:=gps_link}"
  : "${GNSS_NTRIP_GGA_ENABLED:=true}"
  : "${GNSS_NTRIP_GGA_INTERVAL_S:=10}"

  load_existing_config

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

  resolved_receiver_family="$(preserved_gnss_value \
    "$(if gnss_installer_key_is_explicit GNSS_RECEIVER_FAMILY; then printf 'true'; else printf 'false'; fi)" \
    "${GNSS_RECEIVER_FAMILY}" "${PREV_GNSS_RECEIVER_FAMILY:-}" "auto")"
  resolved_transport="$(preserved_gnss_value \
    "$(if gnss_installer_key_is_explicit GNSS_TRANSPORT; then printf 'true'; else printf 'false'; fi)" \
    "${GNSS_TRANSPORT}" "${PREV_GNSS_TRANSPORT:-}" "serial")"
  resolved_serial_device="$(preserved_gnss_value \
    "$(if gnss_installer_key_is_explicit GNSS_SERIAL_DEVICE; then printf 'true'; else printf 'false'; fi)" \
    "${GNSS_SERIAL_DEVICE}" "${PREV_GNSS_SERIAL_DEVICE:-}" "/dev/ttyAMA4")"
  resolved_serial_baud="$(preserved_gnss_value \
    "$(if gnss_installer_key_is_explicit GNSS_SERIAL_BAUD; then printf 'true'; else printf 'false'; fi)" \
    "${GNSS_SERIAL_BAUD}" "${PREV_GNSS_SERIAL_BAUD:-}" "921600")"
  resolved_frame_id="$(preserved_gnss_value \
    "$(if gnss_installer_key_is_explicit GNSS_FRAME_ID; then printf 'true'; else printf 'false'; fi)" \
    "${GNSS_FRAME_ID}" "${PREV_GNSS_FRAME_ID:-}" "gps_link")"
  resolved_ntrip_enabled="$(preserved_gnss_value \
    "${CONFIG_NTRIP_ENABLED_EXPLICIT:-false}" "${CONFIG_NTRIP_ENABLED:-}" "${PREV_NTRIP_ENABLED:-}" "true")"
  resolved_ntrip_host="$(preserved_gnss_value \
    "${CONFIG_NTRIP_HOST_EXPLICIT:-false}" "${CONFIG_NTRIP_HOST:-}" "${PREV_NTRIP_HOST:-}" "crtk.net")"
  resolved_ntrip_port="$(preserved_gnss_value \
    "${CONFIG_NTRIP_PORT_EXPLICIT:-false}" "${CONFIG_NTRIP_PORT:-}" "${PREV_NTRIP_PORT:-}" "2101")"
  resolved_ntrip_user="$(preserved_gnss_value \
    "${CONFIG_NTRIP_USER_EXPLICIT:-false}" "${CONFIG_NTRIP_USER:-}" "${PREV_NTRIP_USER:-}" "centipede")"
  resolved_ntrip_password="$(preserved_gnss_value \
    "${CONFIG_NTRIP_PASSWORD_EXPLICIT:-false}" "${CONFIG_NTRIP_PASSWORD:-}" "${PREV_NTRIP_PASSWORD:-}" "centipede")"
  resolved_ntrip_mountpoint="$(preserved_gnss_value \
    "${CONFIG_NTRIP_MOUNTPOINT_EXPLICIT:-false}" "${CONFIG_NTRIP_MOUNTPOINT:-}" "${PREV_NTRIP_MOUNTPOINT:-}" "NEAR")"
  resolved_ntrip_gga_enabled="$(preserved_gnss_value \
    "$(if gnss_installer_key_is_explicit GNSS_NTRIP_GGA_ENABLED; then printf 'true'; else printf 'false'; fi)" \
    "${GNSS_NTRIP_GGA_ENABLED}" "${PREV_GNSS_NTRIP_GGA_ENABLED:-}" "true")"
  resolved_ntrip_gga_interval_s="$(preserved_gnss_value \
    "$(if gnss_installer_key_is_explicit GNSS_NTRIP_GGA_INTERVAL_S; then printf 'true'; else printf 'false'; fi)" \
    "${GNSS_NTRIP_GGA_INTERVAL_S}" "${PREV_GNSS_NTRIP_GGA_INTERVAL_S:-}" "10")"

  # Patch in only the keys the installer is responsible for.
  _yaml_patch_key "$yaml_file" datum_lat       "$CONFIG_DATUM_LAT"
  _yaml_patch_key "$yaml_file" datum_lon       "$CONFIG_DATUM_LON"
  _yaml_patch_key "$yaml_file" gnss_receiver_family "\"$resolved_receiver_family\""
  _yaml_patch_key "$yaml_file" gnss_transport "\"$resolved_transport\""
  _yaml_patch_key "$yaml_file" gnss_serial_device "\"$resolved_serial_device\""
  _yaml_patch_key "$yaml_file" gnss_serial_baud "$resolved_serial_baud"
  _yaml_patch_key "$yaml_file" gnss_frame_id "\"$resolved_frame_id\""
  _yaml_patch_key "$yaml_file" ntrip_enabled   "$resolved_ntrip_enabled"
  _yaml_patch_key "$yaml_file" ntrip_host      "\"$resolved_ntrip_host\""
  _yaml_patch_key "$yaml_file" ntrip_port      "$resolved_ntrip_port"
  _yaml_patch_key "$yaml_file" ntrip_user      "\"$resolved_ntrip_user\""
  _yaml_patch_key "$yaml_file" ntrip_password  "\"$resolved_ntrip_password\""
  _yaml_patch_key "$yaml_file" ntrip_mountpoint "\"$resolved_ntrip_mountpoint\""
  _yaml_patch_key "$yaml_file" gnss_ntrip_gga_enabled "$resolved_ntrip_gga_enabled"
  _yaml_patch_key "$yaml_file" gnss_ntrip_gga_interval_s "$resolved_ntrip_gga_interval_s"

  # LiDAR enable + the LiDAR-aware localizer flags. When the operator
  # picked a LiDAR in the previous step we also want the GTSAM
  # factor-graph backend on with scan-matching + loop closure, so
  # the GUI doesn't show LiDAR plugged in but ignored.
  local lidar_on="false"
  if [[ "${LIDAR_ENABLED:-false}" == "true" ]]; then
    lidar_on="true"
  fi
  _yaml_patch_key "$yaml_file" lidar_enabled     "$lidar_on"
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

  local gnss_backend
  local gnss_stack
  local restart_services=()

  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
  gnss_stack="$(effective_gnss_stack 2>/dev/null || true)"

  if ! docker_cmd inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    warn "mowgli-ros2 container not running — cannot auto-detect"
    add_issue "Set datum_lat and datum_lon manually in config/mowgli/mowgli_robot.yaml"
    return
  fi

  echo -e "${DIM}Waiting for GPS fix (up to 60s)...${NC}"

  local fix_data="" lat="" lon=""
  local attempt=0
  while [[ $attempt -lt 12 ]]; do
    fix_data=$(docker_cmd exec mowgli-ros2 bash -c "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /gps/fix --once 2>/dev/null" 2>/dev/null || true)
    lat=$(awk '/latitude:/ {print $2; exit}' <<< "$fix_data")
    lon=$(awk '/longitude:/ {print $2; exit}' <<< "$fix_data")

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
  if docker_cmd inspect -f '{{.State.Status}}' mowgli-ros2 2>/dev/null | grep -q running; then
    local status_data
    status_data=$(docker_cmd exec mowgli-ros2 bash -c "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic echo /hardware_bridge/status --once 2>/dev/null" 2>/dev/null || true)
    is_charging=$(awk '/is_charging:/ {print $2; exit}' <<< "$status_data")
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
  mapfile -t restart_services < <(compose_restart_services_for_backend)
  docker_compose_cmd \
    -f "$FINAL_COMPOSE_FILE" \
    --env-file "$FINAL_ENV_FILE" \
    restart "${restart_services[@]}" 2>&1 | tail -3
  sleep 10
}

run_mower_configuration_step() {
  SKIP_WRITE_CONFIG=false
  interactive_config
  if ! $SKIP_WRITE_CONFIG; then
    write_config
  fi
}
