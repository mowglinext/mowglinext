#!/usr/bin/env bash
# =============================================================================
# Mock factories — produce shim binaries under a sandbox bin/ directory.
#
# These tests must NEVER:
#   - install OS packages  (apt-get, dnf)
#   - clone from GitHub    (git clone, git fetch)
#   - pull container images (docker pull)
#   - mutate the host udev / systemd
#   - require sudo / be run as root
#
# By prepending $SANDBOX/bin to PATH we route every such call to a shim
# that records the invocation under $SANDBOX/calls.log and returns success.
# Real `docker compose config` is still required for compose validation —
# the docker shim forwards `compose config` to the real binary so we get
# authentic YAML validation while pull/up/restart are no-ops.
# =============================================================================

_MOCKS_BIN_DIR=""

# Create $SANDBOX/bin and prepend to PATH. Idempotent.
install_mocks() {
  : "${SANDBOX:?SANDBOX must be set — call setup_sandbox first}"
  _MOCKS_BIN_DIR="$SANDBOX/bin"
  mkdir -p "$_MOCKS_BIN_DIR"
  export PATH="$_MOCKS_BIN_DIR:$ORIG_PATH"
  : > "$SANDBOX/calls.log"
}

# Helper: write a shim script to $SANDBOX/bin/<name>. Bakes the absolute
# sandbox path into the shim so it doesn't depend on $SANDBOX being
# exported into every subshell.
_make_shim() {
  local name="$1" body="$2"
  local path="$_MOCKS_BIN_DIR/$name"
  printf '#!/usr/bin/env bash\n' > "$path"
  printf 'echo "%s $*" >> "%s/calls.log"\n' "$name" "$SANDBOX" >> "$path"
  printf '%s\n' "$body" >> "$path"
  chmod +x "$path"
}

# ── Generic mocks ──────────────────────────────────────────────────────────

mock_sudo() {
  # Strip leading 'sudo' and exec the remaining command.
  _make_shim sudo 'exec "$@"'
}

mock_apt_get() {
  _make_shim apt-get 'exit 0'
  _make_shim apt 'exit 0'
}

mock_dnf() {
  _make_shim dnf 'exit 0'
}

# Allow `command -v git` to find the host git but intercept actual
# operations that would touch the network.
mock_git() {
  local real_git
  real_git="$(command -v git)"
  local sandbox="$SANDBOX"
  cat > "$_MOCKS_BIN_DIR/git" <<EOF
#!/usr/bin/env bash
echo "git \$*" >> "$sandbox/calls.log"
case "\${1:-}" in
  fetch|pull|push|clone)
    # No-op: we already populated the sandbox repo via sandbox_repo()
    exit 0
    ;;
  -C)
    case "\${3:-}" in
      fetch|pull|push|clone)
        exit 0
        ;;
    esac
    ;;
esac
exec "$real_git" "\$@"
EOF
  chmod +x "$_MOCKS_BIN_DIR/git"
}

# Forward `docker compose ... config` to real docker (compose validation
# needs the engine), no-op for compose subcommands that mutate state or
# pull from a registry. `docker inspect / exec` always returns non-running
# so auto_detect_position bails out quickly.
#
# We scan the full argv (not just $1, $2) because the installer calls
# `docker compose --project-directory X --env-file Y -f Z ... <subcmd>` —
# the subcommand is at the END, not at $2.
mock_docker() {
  local real_docker
  real_docker="$(command -v docker || echo /bin/false)"
  local sandbox="$SANDBOX"
  # Docker discovers the `compose` plugin via $HOME/.docker/cli-plugins/
  # — but our setup_sandbox redirects HOME to the sandbox to keep tests
  # hermetic. Restore the real HOME just for the docker invocation so
  # `docker compose config` can run.
  local real_home="$ORIG_HOME"
  cat > "$_MOCKS_BIN_DIR/docker" <<EOF
#!/usr/bin/env bash
echo "docker \$*" >> "$sandbox/calls.log"

mock_compose_config() {
  quiet=false
  files=()
  while [ "\$#" -gt 0 ]; do
    case "\$1" in
      -q)
        quiet=true
        shift
        ;;
      -f)
        shift
        [ "\$#" -gt 0 ] && files+=("\$1")
        shift
        ;;
      *)
        shift
        ;;
    esac
  done

  [ "\$quiet" = "true" ] && exit 0

  echo "services:"
  idx=0
  for file in "\${files[@]}"; do
    [ -f "\$file" ] || continue
    while IFS= read -r line; do
      case "\$line" in
        *container_name:*)
          name="\${line#*container_name:}"
          name="\${name#" \${name%%[![:space:]]*}"}"
          idx=\$((idx + 1))
          echo "  mock_service_\$idx:"
          echo "    container_name: \$name"
          echo "    environment:"
          echo "      ENABLE_FOXGLOVE: \"true\""
          echo "    volumes:"
          echo "      - type: bind"
          echo "        source: /dev"
          echo "        target: /dev"
          ;;
      esac
    done < "\$file"
  done
  echo "volumes:"
  echo "  mowgli_maps: {}"
}

# Find the compose subcommand (last token that isn't a flag value).
sub=""
if [ "\${1:-}" = "compose" ]; then
  for arg in "\$@"; do
    case "\$arg" in
      pull|up|down|restart|stop|start|ps|build|config|logs|exec|run|kill)
        sub="\$arg"
        ;;
    esac
  done

  case "\$sub" in
    pull|up|down|restart|stop|start|ps|build|logs|kill|exec|run)
      # No-op for everything that would mutate state or hit the network.
      exit 0
      ;;
    config|"")
      if [ "$real_docker" = "/bin/false" ]; then
        mock_compose_config "\$@"
        exit 0
      fi
      HOME="$real_home" exec "$real_docker" "\$@"
      ;;
  esac
fi

case "\${1:-}" in
  inspect)
    echo "stopped"
    exit 1
    ;;
  exec|run)
    exit 1
    ;;
  pull|push)
    exit 0
    ;;
  *)
    if [ "$real_docker" = "/bin/false" ]; then
      exit 0
    fi
    HOME="$real_home" exec "$real_docker" "\$@"
    ;;
esac
EOF
  chmod +x "$_MOCKS_BIN_DIR/docker"
}

# udevadm — used inside install_udev_rules.
mock_udevadm() {
  _make_shim udevadm 'exit 0'
}

# systemctl — used by enable_all_platform_uarts on Linux Pi targets.
mock_systemctl() {
  _make_shim systemctl 'exit 0'
}

# raspi-config / dpkg-reconfigure — Pi-only helpers some scripts call.
mock_raspi() {
  _make_shim raspi-config 'exit 0'
  _make_shim dpkg-reconfigure 'exit 0'
}

# Install the full set of mocks needed for a non-interactive installer run.
install_all_mocks() {
  install_mocks
  mock_sudo
  mock_apt_get
  mock_dnf
  mock_git
  mock_docker
  mock_udevadm
  mock_systemctl
  mock_raspi
}

# Echo the list of recorded calls (useful for assertions).
calls_log() {
  cat "$SANDBOX/calls.log" 2>/dev/null || true
}

# Count occurrences of a command in the call log.
calls_count() {
  local needle="$1"
  grep -cF -- "$needle" "$SANDBOX/calls.log" 2>/dev/null || echo 0
}
