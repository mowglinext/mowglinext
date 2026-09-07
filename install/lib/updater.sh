#!/usr/bin/env bash
# Host updater bootstrap. Download/build selection runs as the project user;
# only installation of host files and service management use sudo.

install_host_updater() (
  local arch repo revision stage binary expected state_dir="/var/lib/mowgli-updater"
  arch="$(detect_cpu_arch)"
  if [[ "$(uname -s)" != Linux || ! "$arch" =~ ^(amd64|arm64)$ ]] || ! command -v systemctl >/dev/null 2>&1; then
    warn "$MSG_UPDATER_UNSUPPORTED"
    return 0
  fi
  if [[ -e "$state_dir/maintenance" ]]; then
    error "$MSG_UPDATER_RECOVERY"
    return 1
  fi
  revision="$(git -C "$REPO_DIR" rev-parse HEAD)"
  repo="${REPO_URL#https://github.com/}"; repo="${repo%.git}"
  if [[ ! "$repo" =~ ^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$ ]]; then
    error "$MSG_UPDATER_SOURCE"
    return 1
  fi
  stage="$(mktemp -d)"
  trap 'rm -rf -- "$stage"' EXIT
  binary="${MOWGLI_UPDATER_BINARY:-}"
  if [[ -z "$binary" ]]; then
    binary="$stage/mowgli-updater-linux-$arch"
    local base="https://github.com/$repo/releases/download/updater-$revision"
    if ! curl --fail --location --proto '=https' --tlsv1.2 --max-time 180 "$base/mowgli-updater-linux-$arch" -o "$binary" ||
       ! curl --fail --location --proto '=https' --tlsv1.2 --max-time 30 "$base/SHA256SUMS" -o "$stage/SHA256SUMS"; then
      warn "$MSG_UPDATER_UNPUBLISHED"
      return 0
    fi
    expected="$(awk -v name="mowgli-updater-linux-$arch" '$2 == name { print $1 }' "$stage/SHA256SUMS")"
    [[ "$expected" =~ ^[a-f0-9]{64}$ && "$(sha256sum "$binary" | awk '{print $1}')" == "$expected" ]] || { error "$MSG_UPDATER_CHECKSUM"; return 1; }
  fi
  chmod +x "$binary"
  "$binary" version >/dev/null || return 1
  if [[ -f /etc/mowgli-updater.json ]]; then
    $SUDO "$binary" installer-check /etc/mowgli-updater.json "$DOCKER_DIR" "${COMPOSE_PROJECT_NAME:-install}" "linux/$arch" || return 1
  fi
  $SUDO install -d -m 0755 "$state_dir" "$state_dir/run"
  $SUDO install -d -m 0700 "$state_dir/bin" "$state_dir/backups"
  # Preserve a known launcher until the new installation has passed its probe.
  $SUDO install -m 0755 "$binary" /usr/local/bin/mowgli-updater.new
  $SUDO mv /usr/local/bin/mowgli-updater.new /usr/local/bin/mowgli-updater
  # The binary emits JSON safely; jq is an optional installer tool, so do not
  # turn it into a dependency of ordinary updates.
  "$binary" installer-config "$DOCKER_DIR" "${COMPOSE_PROJECT_NAME:-install}" "$repo" "linux/$arch" "${IMAGE_TAG:-dev}" "${REPO_BRANCH:-dev}" > "$stage/config.json"
  if [[ ! -f /etc/mowgli-updater.json ]]; then
    $SUDO install -m 0600 "$stage/config.json" /etc/mowgli-updater.json
  fi
  $SUDO install -m 0644 "$INSTALL_DIR/systemd/mowgli-updater.service" /etc/systemd/system/mowgli-updater.service
  $SUDO systemctl daemon-reload
  $SUDO systemctl enable mowgli-updater.service
  $SUDO systemctl restart mowgli-updater.service
  $SUDO touch "$DOCKER_DIR/.deployment.lock"
  $SUDO chmod 0644 "$DOCKER_DIR/.deployment.lock"
  $SUDO touch "$DOCKER_DIR/.updater-managed"
  # Only retire the instance belonging to this Compose project. Never remove
  # an unrelated administrator's updater merely because its name matches.
  local owner
  owner="$(docker_cmd inspect --format '{{ index .Config.Labels "com.docker.compose.project" }}' mowgli-watchtower 2>/dev/null || true)"
  if [[ "$owner" == "${COMPOSE_PROJECT_NAME:-install}" ]]; then
    docker_cmd stop mowgli-watchtower
    docker_cmd rm mowgli-watchtower
  fi
  info "$MSG_UPDATER_INSTALLED"
)

updater_compose_arguments() {
  UPDATER_COMPOSE_ARGS=()
  if [[ -f "$DOCKER_DIR/update-images.json" ]]; then
    UPDATER_COMPOSE_ARGS=(-f "$DOCKER_DIR/update-images.json")
  fi
}
