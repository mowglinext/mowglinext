#!/usr/bin/env bash
# =============================================================================
# MowgliNext — install / update / repair / check
#
#   mowglinext.sh [install]            interactive when a terminal is attached,
#                                      every unset choice defaults otherwise
#   mowglinext.sh install --non-interactive [--backend= --gnss-connection= ...]
#   mowglinext.sh update  [--branch=]  manual update: sync the checkout, regenerate
#                                      the runtime files, pull images, restart.
#                                      No host updater, no readiness or firmware gate.
#   mowglinext.sh repair               re-apply host artefacts (udev, UARTs, sysctl,
#                                      .env, compose, helpers) from the SAVED choices
#   mowglinext.sh check                diagnostics only (alias: --check)
#   mowglinext.sh uninstall            remove everything except the maps volume and
#                                      mowgli_robot.yaml (see lib/uninstall.sh)
#   mowglinext.sh --only=<step>        run exactly one step (see list_only_steps)
#
# The installer owns the HOST side only: Docker, UARTs, udev symlinks, docker/.env,
# the merged compose file and the sparse mowgli_robot.yaml seed. Everything the
# GUI can configure (datum, NTRIP, GNSS receiver profile, LiDAR pose, ...) is
# deliberately NOT asked here — the onboarding wizard does that on first boot.
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
source "${INSTALL_LIB_DIR}/sysctl.sh"
source "${INSTALL_LIB_DIR}/deploy.sh"
source "${INSTALL_LIB_DIR}/env.sh"
source "${INSTALL_LIB_DIR}/gps.sh"
source "${INSTALL_LIB_DIR}/lidar.sh"
source "${INSTALL_LIB_DIR}/uart.sh"
source "${INSTALL_LIB_DIR}/rc_local.sh"
source "${INSTALL_LIB_DIR}/checks.sh"
source "${INSTALL_LIB_DIR}/compose.sh"
source "${INSTALL_LIB_DIR}/tools.sh"
source "${INSTALL_LIB_DIR}/updater.sh"
source "${INSTALL_LIB_DIR}/uninstall.sh"


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

  if [[ "$INSTALL_MODE" == "install" ]] && [ -f "$preset_file" ]; then
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

  if [[ "$INSTALL_MODE" == "install" ]] && [ "$preset_attempted" != "true" ]; then
    load_preset
  fi
}

# update/repair/check act on an EXISTING installation: they never invent
# hardware choices, so a missing docker/.env is an error, not a prompt.
require_installed_runtime() {
  if [ ! -f "$FINAL_ENV_FILE" ]; then
    error "No installation found at $DOCKER_DIR (docker/.env missing) — run '$(installer_main_command) install' first"
    return 1
  fi
}

# ── --only=<step> : run one step instead of a full mode (issue #632) ──────────
# Each case mirrors exactly one step of run_install() — same function, same
# progress_run* wrapper — run standalone with current/total 1/1.
list_only_steps() {
  cat <<'EOF'
Available --only= steps (mirrors mowglinext.sh run_install()'s sequence):
  docker         Installing Docker
  backend        Selecting hardware backend
  gps            Configuring the GNSS source / serial link
  lidar          Configuring LiDAR
  uart           Enabling UARTs + rc.local
  directory      Preparing repository
  migrate        Migrating runtime files
  env            Writing environment (docker/.env)
  udev           Installing udev rules + DDS sysctl
  mower          Seeding/patching mowgli_robot.yaml (hardware keys only)
  helpers        Installing the mowgli-* helper commands + MOTD
  updater        Installing/updating the host updater service
  startup        Regenerating compose + (re)starting containers
  system         apt update/upgrade (not part of any mode; opt-in only)
EOF
}

run_only_step() {
  local step="$1"
  case "$step" in
    system)
      progress_run_interactive 1 1 "Updating system" \
        run_system_update
      ;;
    docker)
      progress_run 1 1 "Installing Docker" \
        'install_docker'
      ;;
    uart)
      progress_run 1 1 "Enabling UARTs" \
        'enable_all_platform_uarts && generate_rc_local'
      ;;
    backend)
      progress_run_interactive 1 1 "Selecting hardware backend" \
        select_hardware_backend
      ;;
    gps)
      progress_run_interactive 1 1 "Configuring GNSS source / serial link" \
        run_gps_configuration_step
      ;;
    lidar)
      progress_run_interactive 1 1 "Configuring LiDAR" \
        run_lidar_configuration_step
      ;;
    directory)
      progress_run_interactive 1 1 "Preparing repository" \
        setup_directory
      ;;
    migrate)
      progress_run 1 1 "Migrating runtime files" \
        'migrate_runtime_paths'
      ;;
    env)
      progress_run 1 1 "Writing environment" \
        'setup_env'
      ;;
    udev)
      progress_run 1 1 "Installing udev rules" \
        'install_udev_rules && install_dds_sysctl'
      ;;
    mower)
      progress_run 1 1 "Seeding mower configuration" \
        'run_mower_configuration_step'
      ;;
    helpers|tools|motd)
      progress_run 1 1 "Installing helper commands" \
        'install_mowgli_helpers && install_motd'
      ;;
    updater)
      # install_host_updater() already calls check_updater_hardware() as its
      # own first action — no separate gate needed here.
      install_host_updater
      ;;
    startup)
      progress_run_live 1 1 "Starting containers" \
        run_startup_step_live
      ;;
    *)
      error "Unknown --only= step: '$step'"
      echo "" >&2
      list_only_steps >&2
      return 1
      ;;
  esac
}

# ── Modes ────────────────────────────────────────────────────────────────────

run_install() {
  local TOTAL_STEPS=12

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
  unset MOWGLI_ROS2_IMAGE GPS_IMAGE LIDAR_IMAGE MAVROS_IMAGE UNIVERSAL_GNSS_IMAGE OPENMOWER_IMAGE GUI_IMAGE

  # Image tag selection is independent from the selected repository branch.
  select_image_channel

  progress_run 1 "$TOTAL_STEPS" "Installing Docker" \
    'install_docker'

  progress_run_interactive 2 "$TOTAL_STEPS" "Selecting hardware backend" \
    select_hardware_backend

  progress_run_interactive 3 "$TOTAL_STEPS" "Configuring GNSS source / serial link" \
    run_gps_configuration_step

  progress_run_interactive 4 "$TOTAL_STEPS" "Configuring LiDAR" \
    run_lidar_configuration_step

  # Runs AFTER GPS/LiDAR configuration so required_uart_overlays()
  # (install/lib/uart.sh) only claims the GPIO pins the chosen ports need
  # (issue #631).
  progress_run 5 "$TOTAL_STEPS" "Enabling UARTs" \
    'enable_all_platform_uarts && generate_rc_local'

  check_updater_hardware || return 1

  progress_run_interactive 6 "$TOTAL_STEPS" "Preparing repository" \
    setup_directory

  progress_run 7 "$TOTAL_STEPS" "Migrating runtime files" \
    'migrate_runtime_paths'

  progress_run 8 "$TOTAL_STEPS" "Writing environment" \
    'setup_env'

  progress_run 9 "$TOTAL_STEPS" "Installing udev rules" \
    'install_udev_rules && install_dds_sysctl'

  progress_run 10 "$TOTAL_STEPS" "Seeding mower configuration" \
    'run_mower_configuration_step'

  progress_run 11 "$TOTAL_STEPS" "Installing helper commands" \
    'install_mowgli_helpers && install_motd'

  if [[ "$INSTALL_UPDATER" == "true" ]]; then
    install_host_updater
  else
    info "Host updater skipped (--no-updater); use '$(installer_main_command) update' to update manually."
  fi

  progress_run_live 12 "$TOTAL_STEPS" "Starting containers" \
    run_startup_step_live
}

# Manual update. Deliberately NOT the host updater: no readiness gate, no
# firmware-protocol check, no backup/rollback transaction. It regenerates the
# runtime files from the CURRENT fragments and the SAVED docker/.env — the same
# writers as install, so the compose baseline is re-recorded and a host updater
# that is installed can still adopt the result later (never hand-edit images:
# see docs/UPDATES.md, "Manual update").
run_update() {
  local TOTAL_STEPS=5

  require_installed_runtime || return 1
  load_install_state

  select_repo_branch
  sync_repo_branch_to_selected_branch

  unset MOWGLI_ROS2_IMAGE GPS_IMAGE LIDAR_IMAGE MAVROS_IMAGE UNIVERSAL_GNSS_IMAGE OPENMOWER_IMAGE GUI_IMAGE
  select_image_channel

  # Manual mode: the checkout's fragments and .env image tags decide, not the
  # release the updater installed (which is exactly what a stuck updater
  # cannot move past). See write_compose_merged / installer-stack.
  export MOWGLI_REGENERATE_STACK=true
  if [[ -f "$DOCKER_DIR/.updater-managed" ]]; then
    warn "$MSG_UPDATE_MANUAL_UPDATER"
  fi

  progress_run 1 "$TOTAL_STEPS" "Migrating runtime files" \
    'migrate_runtime_paths'

  progress_run 2 "$TOTAL_STEPS" "Writing environment" \
    'setup_env'

  progress_run 3 "$TOTAL_STEPS" "Patching mower configuration" \
    'run_mower_configuration_step'

  progress_run 4 "$TOTAL_STEPS" "Refreshing helper commands" \
    'install_mowgli_helpers'

  progress_run_live 5 "$TOTAL_STEPS" "Pulling images and restarting containers" \
    run_startup_step_live
}

# Repair re-applies every host artefact from the SAVED choices. It never asks
# a hardware question: a robot whose udev rules, UART overlays, sysctl, .env or
# compose file went missing or stale gets them back exactly as installed.
run_repair() {
  local TOTAL_STEPS=9

  require_installed_runtime || return 1
  NON_INTERACTIVE=true
  export MOWGLI_REGENERATE_STACK=true
  load_install_state
  unset MOWGLI_ROS2_IMAGE GPS_IMAGE LIDAR_IMAGE MAVROS_IMAGE UNIVERSAL_GNSS_IMAGE OPENMOWER_IMAGE GUI_IMAGE
  recompute_image_defaults

  progress_run 1 "$TOTAL_STEPS" "Installing Docker" \
    'install_docker'

  progress_run 2 "$TOTAL_STEPS" "Enabling UARTs" \
    'enable_all_platform_uarts && generate_rc_local'

  progress_run 3 "$TOTAL_STEPS" "Preparing repository" \
    setup_directory

  progress_run 4 "$TOTAL_STEPS" "Migrating runtime files" \
    'migrate_runtime_paths'

  progress_run 5 "$TOTAL_STEPS" "Writing environment" \
    'setup_env'

  progress_run 6 "$TOTAL_STEPS" "Installing udev rules" \
    'install_udev_rules && install_dds_sysctl'

  progress_run 7 "$TOTAL_STEPS" "Patching mower configuration" \
    'run_mower_configuration_step'

  progress_run 8 "$TOTAL_STEPS" "Installing helper commands" \
    'install_mowgli_helpers && install_motd'

  progress_run_live 9 "$TOTAL_STEPS" "Restarting containers" \
    run_startup_step_live
}

run_check_prelude() {
  load_install_state

  if [ ! -f "$INSTALL_DIR/compose/docker-compose.base.yml" ]; then
    error "No installation sources found at $INSTALL_DIR — run '$(installer_main_command) install' first"
    return 1
  fi

  if [ ! -f "$FINAL_COMPOSE_FILE" ]; then
    error "No generated runtime compose found at $FINAL_COMPOSE_FILE — run '$(installer_main_command) install' first"
    return 1
  fi

  cd "$DOCKER_DIR"
  echo -e "${DIM}Running diagnostics on runtime at $DOCKER_DIR${NC}"
}

main() {
  show_banner
  load_locale
  init_install_logs
  assert_supported_platform || return 1
  print_platform_summary
  if [[ "$NON_INTERACTIVE" == "true" ]]; then
    info "Non-interactive mode: unset choices take their defaults."
  fi
  select_mode

  if [[ "$INSTALL_MODE" != "check" ]]; then
    if [[ -f "$DOCKER_DIR/.updater-managed" ]]; then
      exec 9<"$DOCKER_DIR/.deployment.lock"
      flock -n -x 9 || { error "Another deployment operation is running."; return 1; }
      [[ ! -e /var/lib/mowgli-updater/maintenance ]] || { error "$MSG_UPDATER_RECOVERY"; return 1; }
    fi
    # Pre-acquire sudo credentials once for the entire session
    if command -v sudo >/dev/null 2>&1; then
      echo ""
      sudo -v
    fi
  fi

  case "$INSTALL_MODE" in
    only)
      # Issue #632: one step, skipping the branch-switch and language prompts —
      # an operator reaching for --only= already has a checkout and a language
      # they're happy with. Still loads docker/.env so the step has context.
      load_install_state
      run_only_step "$ONLY_STEP" || return 1
      ;;
    install) run_install || return 1 ;;
    update)  run_update  || return 1 ;;
    repair)  run_repair  || return 1 ;;
    check)   run_check_prelude || return 1 ;;
    uninstall)
      run_uninstall || return 1
      return 0
      ;;
    *)
      error "Unknown mode: $INSTALL_MODE"
      return 1
      ;;
  esac

  echo ""
  echo -e "${CYAN}${BOLD}══ System Health Check ══${NC}"

  check_devices
  check_generated_gps_yaml_alignment
  check_containers
  check_firmware || true
  check_mavros || true
  check_gps || true
  check_lidar || true
  check_gui || true

  print_summary

  if [[ "$INSTALL_MODE" == "install" ]]; then
    mark_preset_consumed
  fi
}

parse_args "$@"
main
