#!/usr/bin/env bash

banner_tools() {
  echo ""
  echo -e "${CYAN}${BOLD}── Optional tools ──${NC}"
  echo ""
}

create_helper_script() {
  local path="$1"
  local content="$2"

  require_root_for "helper script"

  printf '%s\n' "$content" | $SUDO tee "$path" > /dev/null
  $SUDO chmod +x "$path"
}

# The mowgli-* commands are the operator's day-to-day interface (wiki,
# FIRST_BOOT.md reference them), so they are always installed. The former
# optional tools (lazydocker/ctop, mc/ranger, debug packages) are gone: the
# GUI covers the monitoring they provided and apt is one command away.
install_mowgli_helpers() {
  step "$MSG_TOOLS_HELPERS"

  local project_dir="${DOCKER_DIR:-$REPO_DIR/docker}"
  local dc="docker compose --env-file \"$project_dir/.env\""

  create_helper_script "/usr/local/bin/mowgli-up" "#!/usr/bin/env bash
if [[ -f \"$project_dir/.updater-managed\" ]]; then exec bash \"$REPO_DIR/docker/stack.sh\" up \"\$@\"; fi
cd \"$project_dir\" || exit 1
exec $dc up -d \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-down" "#!/usr/bin/env bash
if [[ -f \"$project_dir/.updater-managed\" ]]; then exec bash \"$REPO_DIR/docker/stack.sh\" down \"\$@\"; fi
cd \"$project_dir\" || exit 1
exec $dc down \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-restart" "#!/usr/bin/env bash
if [[ -f \"$project_dir/.updater-managed\" ]]; then exec bash \"$REPO_DIR/docker/stack.sh\" restart \"\$@\"; fi
cd \"$project_dir\" || exit 1
exec $dc restart \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-logs" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc logs -f \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-ps" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc ps"

  create_helper_script "/usr/local/bin/mowgli-pull" "#!/usr/bin/env bash
if [[ -f \"$project_dir/.updater-managed\" ]]; then exec bash \"$REPO_DIR/docker/stack.sh\" pull \"\$@\"; fi
cd \"$project_dir\" || exit 1
$dc pull \"\$@\"
rc=\$?
# Every pull leaves the image it superseded behind, untagged. With a 3.67 GB
# ROS2 image on a 58 GB SD card that fills the disk in about ten updates —
# observed at 100 %, zero bytes free, on 2026-09-06, which stops the robot
# writing logs or recordings and makes the next pull fail. Dangling images are
# by definition superseded, and docker refuses to remove one that any existing
# container uses, so this is safe to run unattended. Tagged images are left
# alone: those are named rollback points and only the operator should drop them.
docker image prune -f >/dev/null 2>&1 || true
df -h / | awk 'NR==2 {printf \"Disk: %s free of %s (%s used)\\n\", \$4, \$2, \$5}'
exit \$rc"

  create_helper_script "/usr/local/bin/mowgli-check" "#!/usr/bin/env bash
cd \"$INSTALL_DIR\" || exit 1
exec bash ./mowglinext.sh --check"

  create_helper_script "/usr/local/bin/mowgli-gps-logs" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc logs -f gps"

  create_helper_script "/usr/local/bin/mowgli-lidar-logs" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc logs -f lidar"

  create_helper_script "/usr/local/bin/mowgli-shell" "#!/usr/bin/env bash
container=\"\${1:-mowgli-ros2}\"
exec docker exec -it \"\$container\" bash"

  create_helper_script "/usr/local/bin/mowgli-status" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
$dc ps
echo
docker stats --no-stream 2>/dev/null || true"

  info "Installed Mowgli helper commands"
  echo ""
  echo "Available commands:"
  echo "  mowgli-up"
  echo "  mowgli-down"
  echo "  mowgli-restart"
  echo "  mowgli-logs"
  echo "  mowgli-ps"
  echo "  mowgli-check"
  echo "  mowgli-gps-logs"
  echo "  mowgli-lidar-logs"
  echo "  mowgli-shell [container]"
  echo "  mowgli-status"
}
