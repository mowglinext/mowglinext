#!/usr/bin/env bash
# =============================================================================
# Mowgli ROS2 v3 — Interactive Install / Upgrade / Diagnose Script
# =============================================================================

set -euo pipefail

# Preserve the original argv so an explicit repository branch switch can
# re-exec the installer with the same flags after checking out the new source.
MOWGLI_INSTALLER_ARGV=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_LIB_DIR="${SCRIPT_DIR}/lib"

source "${INSTALL_LIB_DIR}/common.sh"
source "${INSTALL_LIB_DIR}/i18n.sh"
source "${INSTALL_LIB_DIR}/config.sh"
source "${INSTALL_LIB_DIR}/state.sh"
source "${INSTALL_LIB_DIR}/banner.sh"
source "${INSTALL_LIB_DIR}/progress.sh"
source "${INSTALL_LIB_DIR}/motd.sh"
source "${INSTALL_LIB_DIR}/system.sh"
source "${INSTALL_LIB_DIR}/platform.sh"
source "${INSTALL_LIB_DIR}/docker.sh"
source "${INSTALL_LIB_DIR}/backend_choice.sh"
source "${INSTALL_LIB_DIR}/udev.sh"
source "${INSTALL_LIB_DIR}/deploy.sh"
source "${INSTALL_LIB_DIR}/env.sh"
source "${INSTALL_LIB_DIR}/serial_probe.sh"
source "${INSTALL_LIB_DIR}/unicore_config.sh"
source "${INSTALL_LIB_DIR}/ublox_config.sh"
source "${INSTALL_LIB_DIR}/gps.sh"
source "${INSTALL_LIB_DIR}/lidar.sh"
source "${INSTALL_LIB_DIR}/range.sh"
source "${INSTALL_LIB_DIR}/uart.sh"
source "${INSTALL_LIB_DIR}/rc_local.sh"
source "${INSTALL_LIB_DIR}/checks.sh"
source "${INSTALL_LIB_DIR}/compose.sh"
source "${INSTALL_LIB_DIR}/tools.sh"


load_preset() {
  local preset_file
  preset_file="$(preset_file_path)"
  if [ -f "$preset_file" ]; then
    info "Loading hardware preset from web composer"
    load_preset_file "$preset_file"
    if [ "${STATE_ACTIVE_PRESET_COUNT:-0}" -gt 0 ]; then
      PRESET_LOADED=true
    else
      PRESET_LOADED=false
    fi
  elif [[ "${CLI_PRESET:-false}" == "true" ]]; then
    PRESET_LOADED=true
  else
    PRESET_LOADED=false
  fi
}

load_install_state() {
  local preset_file
  local preset_attempted=false

  preset_file="$(preset_file_path)"

  if ! $CHECK_ONLY && [ -f "$preset_file" ]; then
    preset_attempted=true
    load_preset

    if [ "${PRESET_LOADED:-false}" = "true" ] && [ -n "${STATE_ACTIVE_PRESET_FILE:-}" ]; then
      if [ -f "$REPO_DIR/docker/.env" ]; then
        warn "Web preset detected; existing docker/.env will be backed up and ignored for this install run."
        backup_env_defaults_file "$REPO_DIR/docker/.env"
      fi
      return 0
    fi
  fi

  if [ -f "$REPO_DIR/docker/.env" ]; then
    load_env_defaults_file "$REPO_DIR/docker/.env"
  fi

  if ! $CHECK_ONLY && [ "$preset_attempted" != "true" ]; then
    load_preset
  fi
}

main() {
  show_banner
  load_locale
  init_install_logs
  assert_supported_platform || return 1
  print_platform_summary

  if ! $CHECK_ONLY; then
    # Pre-acquire sudo credentials once for the entire install session
    if command -v sudo >/dev/null 2>&1; then
      echo ""
      sudo -v
    fi
    local TOTAL_STEPS=15

    # Choose the source checkout first. If the user selects another branch,
    # the installer intentionally checks it out, syncs submodules, and
    # re-execs before any further prompts or runtime generation happen.
    select_repo_branch
    sync_repo_branch_to_selected_branch

    # Language selection, then load previous env unless a web preset
    # intentionally starts a fresh runtime configuration.
    select_language
    load_install_state

    # Image refs are tied to the install script version — never inherit
    # stale paths from older installs (e.g. mowgli-docker, openmower-gui).
    unset MOWGLI_ROS2_IMAGE GPS_IMAGE LIDAR_IMAGE MAVROS_IMAGE GUI_IMAGE

    # Image tag selection is independent from the selected repository branch.
    select_image_channel

    progress_run_interactive 1 "$TOTAL_STEPS" "Updating system" \
      run_system_update

    progress_run 2 "$TOTAL_STEPS" "Installing Docker" \
      'install_docker'

    progress_run 3 "$TOTAL_STEPS" "Enabling UARTs" \
      'enable_all_platform_uarts && generate_rc_local'
    
    progress_run_interactive 4 "$TOTAL_STEPS" "Selecting hardware backend" \
      select_hardware_backend

    progress_run_interactive 5 "$TOTAL_STEPS" "Configuring Universal GNSS" \
      run_gps_configuration_step

    progress_run_interactive 6 "$TOTAL_STEPS" "Configuring LiDAR" \
      run_lidar_configuration_step

    progress_run_interactive 7 "$TOTAL_STEPS" "Configuring rangefinders" \
      run_range_configuration_step

    progress_run_interactive 8 "$TOTAL_STEPS" "Preparing repository" \
      setup_directory

    progress_run 9 "$TOTAL_STEPS" "Migrating runtime files" \
      'migrate_runtime_paths'

    progress_run 10 "$TOTAL_STEPS" "Writing environment" \
      'setup_env'

    progress_run 11 "$TOTAL_STEPS" "Installing udev rules" \
      'install_udev_rules'

    progress_run_interactive 12 "$TOTAL_STEPS" "Configuring mower" \
      run_mower_configuration_step

    progress_run_interactive 13 "$TOTAL_STEPS" "Installing optional tools" \
      install_optional_tools

    progress_run 14 "$TOTAL_STEPS" "Installing MOTD" \
      'install_motd'

    progress_run_live 15 "$TOTAL_STEPS" "Starting containers" \
      run_startup_step_live

  else
    load_install_state

    if [ ! -f "$INSTALL_DIR/compose/docker-compose.base.yml" ]; then
      error "No installation sources found at $INSTALL_DIR — run without --check first"
      return 1
    fi

    if [ ! -f "$FINAL_COMPOSE_FILE" ]; then
      error "No generated runtime compose found at $FINAL_COMPOSE_FILE — run installer first"
      return 1
    fi

    cd "$DOCKER_DIR"
    echo -e "${DIM}Running diagnostics on runtime at $DOCKER_DIR${NC}"
  fi
  echo ""
  echo -e "${CYAN}${BOLD}══ System Health Check ══${NC}"

  check_devices
  check_generated_gps_yaml_alignment
  check_containers
  check_firmware || true
  check_gps || true
  check_lidar || true
  check_rangefinders || true
  check_gui || true

  print_summary

  if ! $CHECK_ONLY; then
    mark_preset_consumed
  fi
}

parse_args "$@"
main
