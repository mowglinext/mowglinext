#!/usr/bin/env bash
# =============================================================================
# `mowglinext.sh uninstall` — remove MowgliNext from this host.
#
# Removed: the container stack and its images (incl. the GUI-created
# mowgli-remote sidecar and a leftover Watchtower), the host updater
# (service, binary, config, state), udev rules, DDS sysctl, MOTD, the
# mowgli-* helpers, our rc.local, and the whole checkout.
# Kept, on purpose: the maps volume (areas.dat + saved fusion graph — the
# stack is never brought down with -v) and docker/config/mowgli/
# mowgli_robot.yaml, left in place so a later install finds both: the
# installer never overwrites an existing yaml and the Compose project name
# is stable (install/CLAUDE.md). Docker itself and the /boot UART overlays
# stay: they are not ours.
#
# HOST_PREFIX lets the test suite point every host path at a sandbox.
# =============================================================================

uninstall_host_paths() {
  UNINSTALL_HOST_FILES=(
    "${HOST_PREFIX:-}/etc/udev/rules.d/50-mowgli.rules"
    "${HOST_PREFIX:-}/etc/sysctl.d/90-mowgli-dds.conf"
    "${HOST_PREFIX:-}/etc/profile.d/mowgli-motd.sh"
    "${HOST_PREFIX:-}/etc/systemd/system/mowgli-updater.service"
    "${HOST_PREFIX:-}/usr/local/bin/mowgli-updater"
    "${HOST_PREFIX:-}/etc/mowgli-updater.json"
  )
  local helper
  for helper in up down restart logs ps pull check gps-logs lidar-logs shell status; do
    UNINSTALL_HOST_FILES+=("${HOST_PREFIX:-}/usr/local/bin/mowgli-$helper")
  done
  UNINSTALL_HOST_DIRS=("${HOST_PREFIX:-}/var/lib/mowgli-updater")
}

uninstall_print_plan() {
  local maps_volume="${COMPOSE_PROJECT_NAME:-install}_mowgli_maps"
  echo ""
  echo -e "${CYAN:-}${BOLD:-}$MSG_UNINSTALL_TITLE${NC:-}"
  echo "  $MSG_UNINSTALL_REMOVES"
  echo "    - $MSG_UNINSTALL_CONTAINERS"
  echo "    - $MSG_UNINSTALL_UPDATER"
  echo "    - $MSG_UNINSTALL_HOST"
  echo "    - $REPO_DIR"
  echo "  $MSG_UNINSTALL_KEEPS"
  echo "    - $maps_volume ($MSG_UNINSTALL_MAPS)"
  echo "    - $DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  echo "    - $MSG_UNINSTALL_NOT_OURS"
  echo ""
}

uninstall_stop_stack() {
  local name
  if [[ -f "$FINAL_COMPOSE_FILE" && -f "$FINAL_ENV_FILE" ]]; then
    # --rmi all removes the images the services use; volumes are NOT removed
    # (no -v): the maps live there.
    docker_compose_cmd -f "$FINAL_COMPOSE_FILE" --env-file "$FINAL_ENV_FILE" down --remove-orphans --rmi all || \
      warn "$MSG_UNINSTALL_COMPOSE_FAILED"
  fi
  # Containers the Compose file does not know about: the GUI-created remote
  # access sidecar, a Watchtower from an older install, or a stack whose
  # compose file is already gone.
  for name in mowgli-remote mowgli-watchtower mowgli-ros2 mowgli-gui mowgli-gps mowgli-lidar mowgli-mqtt mowgli-mavros mowgli-ntrip mowgli-updater; do
    docker_cmd rm -f "$name" >/dev/null 2>&1 || true
  done
}

uninstall_remove_updater_service() {
  local unit="${HOST_PREFIX:-}/etc/systemd/system/mowgli-updater.service"
  if command -v systemctl >/dev/null 2>&1 && [[ -f "$unit" ]]; then
    $SUDO systemctl disable --now mowgli-updater.service >/dev/null 2>&1 || true
  fi
}

uninstall_remove_host_files() {
  local path
  uninstall_host_paths
  for path in "${UNINSTALL_HOST_FILES[@]}"; do
    [[ -e "$path" ]] || continue
    $SUDO rm -f "$path" && info "$MSG_UNINSTALL_REMOVED $path"
  done
  for path in "${UNINSTALL_HOST_DIRS[@]}"; do
    [[ -d "$path" ]] || continue
    $SUDO rm -rf "$path" && info "$MSG_UNINSTALL_REMOVED $path"
  done
  if command -v systemctl >/dev/null 2>&1; then
    $SUDO systemctl daemon-reload >/dev/null 2>&1 || true
  fi
  if command -v udevadm >/dev/null 2>&1; then
    $SUDO udevadm control --reload-rules >/dev/null 2>&1 || true
  fi
  # rc.local is ours only when it carries our marker; a saved original goes
  # back, otherwise the file is removed together with the compat unit.
  local rclocal="${HOST_PREFIX:-}/etc/rc.local"
  if [[ -f "$rclocal" ]] && grep -q "MOWGLI_UART_INIT" "$rclocal" 2>/dev/null; then
    if [[ -f "$rclocal.bak" ]]; then
      $SUDO mv -f "$rclocal.bak" "$rclocal" && info "$MSG_UNINSTALL_RESTORED $rclocal"
    else
      $SUDO rm -f "$rclocal" "${HOST_PREFIX:-}/etc/systemd/system/rc-local.service" && info "$MSG_UNINSTALL_REMOVED $rclocal"
    fi
  fi
}

# Remove the checkout but keep mowgli_robot.yaml exactly where a reinstall
# looks for it. Refuses anything that is not a MowgliNext checkout.
uninstall_remove_checkout() {
  local yaml="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"
  local keep=""
  if [[ ! -f "$REPO_DIR/install/mowglinext.sh" ]]; then
    error "$MSG_UNINSTALL_NOT_A_CHECKOUT $REPO_DIR"
    return 1
  fi
  if [[ -f "$yaml" ]]; then
    keep="$(mktemp)"
    cp -p "$yaml" "$keep" || return 1
  fi
  # Runtime files written by root inside the containers (calibration
  # write-back, GUI DB) need sudo to go; everything else is the operator's.
  if ! rm -rf "$REPO_DIR" 2>/dev/null; then
    $SUDO rm -rf "$REPO_DIR" || return 1
  fi
  if [[ -n "$keep" ]]; then
    mkdir -p "$DOCKER_DIR/config/mowgli"
    cp -p "$keep" "$yaml" && rm -f "$keep"
    info "$MSG_UNINSTALL_KEPT $yaml"
  fi
  info "$MSG_UNINSTALL_REMOVED $REPO_DIR"
}

run_uninstall() {
  require_root_for "uninstall"
  uninstall_print_plan

  # Destructive: a terminal must confirm, and without one only an explicit
  # --yes/--non-interactive on the command line counts — an auto-detected
  # missing tty (cron, plain `ssh host cmd`) is refused.
  if [[ "${NON_INTERACTIVE:-false}" == "true" ]]; then
    if [[ "${NON_INTERACTIVE_EXPLICIT:-false}" != "true" ]]; then
      error "$MSG_UNINSTALL_NEEDS_YES"
      return 1
    fi
  elif ! confirm "$MSG_UNINSTALL_CONFIRM"; then
    info "$MSG_UNINSTALL_ABORTED"
    return 1
  fi

  progress_run_live 1 3 "Stopping and removing containers" \
    uninstall_stop_stack
  progress_run 2 3 "Removing the host updater and host files" \
    'uninstall_remove_updater_service && uninstall_remove_host_files'
  progress_run 3 3 "Removing the checkout" \
    'uninstall_remove_checkout'

  echo ""
  info "$MSG_UNINSTALL_DONE"
  info "  $MSG_UNINSTALL_MAPS_HINT ${COMPOSE_PROJECT_NAME:-install}_mowgli_maps"
}
