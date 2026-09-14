#!/usr/bin/env bash

banner_tools() {
  echo ""
  echo -e "${CYAN}${BOLD}── Optional tools ──${NC}"
  echo ""
}

create_dockermgr_wrapper() {
  local target_cmd="$1"
  require_root_for "dockermgr wrapper"

  cat <<EOF | $SUDO tee /usr/local/bin/dockermgr > /dev/null
#!/usr/bin/env bash
exec ${target_cmd} "\$@"
EOF
  $SUDO chmod +x /usr/local/bin/dockermgr
  info "Command available: dockermgr -> ${target_cmd}"
}

install_docker_cli_tool() {
  banner_tools
  echo "=== $MSG_TOOLS_DOCKER_CLI ==="
  echo "1) $MSG_TOOLS_DOCKER_LAZY"
  echo "2) $MSG_TOOLS_DOCKER_CTOP"
  echo "3) $MSG_TOOLS_NO"
  prompt "$MSG_YOUR_CHOICE" "3"
  local docker_cli="$REPLY"

  case "$docker_cli" in
    1)
      info "Installing lazydocker..."
      curl -fsSL https://raw.githubusercontent.com/jesseduffield/lazydocker/master/scripts/install_update_linux.sh | bash
      create_dockermgr_wrapper "lazydocker"
      ;;
    2)
      require_root_for "ctop"
      info "Installing ctop..."
      $SUDO apt install -y ctop
      create_dockermgr_wrapper "ctop"
      ;;
    3)
      info "No Docker CLI manager installed"
      ;;
    *)
      warn "Invalid choice. No Docker CLI manager installed."
      ;;
  esac
}

install_file_manager_tool() {
  banner_tools
  echo "=== $MSG_TOOLS_FILE_MANAGER ==="
  echo "1) $MSG_TOOLS_FILE_MC"
  echo "2) $MSG_TOOLS_FILE_RANGER"
  echo "3) $MSG_TOOLS_NO"
  prompt "$MSG_YOUR_CHOICE" "3"
  local fileman_choice="$REPLY"

  require_root_for "file manager"

  case "$fileman_choice" in
    1)
      info "Installing Midnight Commander..."
      $SUDO apt install -y mc
      info "Command available: mc"
      ;;
    2)
      info "Installing ranger..."
      $SUDO apt install -y ranger
      info "Command available: ranger"
      ;;
    3)
      info "No file manager installed"
      ;;
    *)
      warn "Invalid choice. No file manager installed."
      ;;
  esac
}

install_debug_tools() {
  banner_tools
  echo "=== $MSG_TOOLS_DEBUG ==="
  echo "1) $MSG_TOOLS_DEBUG_ALL"
  echo "2) $MSG_TOOLS_DEBUG_ESSENTIAL"
  echo "3) $MSG_TOOLS_DEBUG_NONE"
  prompt "$MSG_YOUR_CHOICE" "2"
  local debug_tools="$REPLY"

  require_root_for "debug tools"

  case "$debug_tools" in
    1)
      info "Installing full debug/tooling set..."
      $SUDO apt install -y \
        htop ncdu lsof strace gdb minicom screen picocom \
        tmux bat fd-find ripgrep jq yq \
        nmap iptraf-ng usbutils i2c-tools
      ;;
    2)
      info "Installing essential debug/tooling set..."
      $SUDO apt install -y \
        htop ncdu git tmux minicom screen jq usbutils
      ;;
    3)
      info "No debug/dev tools installed"
      ;;
    *)
      warn "Invalid choice. No debug/dev tools installed."
      ;;
  esac
}

install_optional_tools() {
  step "Optional tools"

  install_docker_cli_tool
  install_file_manager_tool
  install_debug_tools
  install_mowgli_helpers
}

create_helper_script() {
  local path="$1"
  local content="$2"

  require_root_for "helper script"

  printf '%s\n' "$content" | $SUDO tee "$path" > /dev/null
  $SUDO chmod +x "$path"
}

install_mowgli_helpers() {
  banner_tools
  echo "=== $MSG_TOOLS_HELPERS ==="

  if ! confirm "$MSG_TOOLS_HELPERS_CONFIRM"; then
    info "No Mowgli helpers installed"
    return
  fi

  local project_dir="${DOCKER_DIR:-$REPO_DIR/docker}"
  local dc="docker compose --env-file \"$project_dir/.env\""

  create_helper_script "/usr/local/bin/mowgli-up" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc up -d \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-down" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc down \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-restart" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc restart \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-logs" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc logs -f \"\$@\""

  create_helper_script "/usr/local/bin/mowgli-ps" "#!/usr/bin/env bash
cd \"$project_dir\" || exit 1
exec $dc ps"

  create_helper_script "/usr/local/bin/mowgli-pull" "#!/usr/bin/env bash
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
