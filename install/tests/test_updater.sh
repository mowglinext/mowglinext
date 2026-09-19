#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# No host installation/network calls: validate the shipped installer contract.
for file in "$ROOT/install/lib/updater.sh" "$ROOT/docker/stack.sh" "$ROOT/install/lib/compose.sh" "$ROOT/install/lib/tools.sh"; do bash -n "$file"; done
source "$ROOT/install/lib/updater.sh"
sandbox="$(mktemp -d)"
trap 'rm -rf -- "$sandbox"' EXIT
DOCKER_DIR="$sandbox"
updater_compose_arguments
[[ ${#UPDATER_COMPOSE_ARGS[@]} == 0 ]]
printf '{"services":{}}\n' > "$sandbox/update-images.json"
updater_compose_arguments
[[ "${UPDATER_COMPOSE_ARGS[1]}" == "$sandbox/update-images.json" ]]

# Unsupported hardware keeps the legacy path on fresh installs, but must be
# rejected before rewriting any existing managed installation.
effective_tfluna_front_enabled() { [[ "${TFLUNA_FRONT_ENABLED:-false}" == true ]]; }
effective_tfluna_edge_enabled() { [[ "${TFLUNA_EDGE_ENABLED:-false}" == true ]]; }
effective_vesc_enabled() { [[ "${ENABLE_VESC:-false}" == true ]]; }
error() { printf '%s\n' "$*" >&2; }
warn() { printf '%s\n' "$*" >&2; }
info() { :; }
# install_host_updater() now calls require_root_for itself (issue #632's
# --only=updater exposed that it used to rely on an earlier full-flow step
# having already set $SUDO as a side effect) — this test sources only
# updater.sh, not common.sh where the real require_root_for lives, so stub
# it the same way as error/warn/info above rather than pulling in the whole
# file. None of the $SUDO-prefixed lines below are actually reached in this
# test (every path returns before them), so the empty value never matters.
require_root_for() { SUDO=""; }
REPO_DIR="$ROOT"; INSTALL_DIR="$ROOT/install"
source "$ROOT/install/lib/compose.sh"
effective_gnss_backend() { echo disabled; }
effective_gnss_stack() { echo disabled; }
is_supported_gnss_backend() { return 0; }
LIDAR_ENABLED=false
MSG_UPDATER_HARDWARE_MANAGED='unsupported managed hardware'
MSG_UPDATER_HARDWARE_LEGACY='preserving legacy hardware selection'
for choice in mowgli mavros front edge vesc; do
  HARDWARE_BACKEND=mowgli; TFLUNA_FRONT_ENABLED=false; TFLUNA_EDGE_ENABLED=false; ENABLE_VESC=false
  case "$choice" in
    mavros) HARDWARE_BACKEND=mavros;;
    front) TFLUNA_FRONT_ENABLED=true;;
    edge) TFLUNA_EDGE_ENABLED=true;;
    vesc) ENABLE_VESC=true;;
  esac
  rm -f -- "$sandbox/.updater-managed"
  check_updater_hardware
  if [[ "$choice" != mowgli ]]; then
    ! updater_hardware_supported
    install_host_updater
    [[ ! -e "$sandbox/.updater-managed" ]]
    build_compose_stack
    case "$choice" in
      mavros) fragment=docker-compose.mavros.yml;;
      front) fragment=docker-compose.tfluna-front.yml;;
      edge) fragment=docker-compose.tfluna-edge.yml;;
      vesc) fragment=docker-compose.vesc.yml;;
    esac
    [[ " ${COMPOSE_FILES[*]} " == *"/$fragment "* ]]
    touch "$sandbox/.updater-managed"
    ! check_updater_hardware
    ! install_host_updater
  else
    updater_hardware_supported
    touch "$sandbox/.updater-managed"
    check_updater_hardware
  fi
done

# A missing exact-revision asset must stop before any host write/Watchtower
# fallback. These functions intercept network/platform calls only.
rm -f -- "$sandbox/.updater-managed"
HARDWARE_BACKEND=mowgli; ENABLE_VESC=false
REPO_DIR="$ROOT"; REPO_URL=https://github.com/mowglinext/mowglinext.git
MSG_UPDATER_UNPUBLISHED='bootstrap asset unavailable'
detect_cpu_arch() { echo amd64; }
uname() { echo Linux; }
systemctl() { echo 'unexpected systemctl mutation' >&2; return 99; }
curl() { return 22; }
git() { printf '%040d\n' 1; }
! install_host_updater
[[ ! -e "$sandbox/.updater-managed" ]]
grep -q 'ExecStart=/usr/local/bin/mowgli-updater supervise' "$ROOT/install/systemd/mowgli-updater.service"
grep -q 'MOWGLI_UPDATE_MAINTENANCE' "$ROOT/install/compose/docker-compose.updater.yml"
grep -q 'MSG_UPDATER_UNSUPPORTED=' "$ROOT/install/locale/en.sh"
grep -q 'MSG_UPDATER_UNSUPPORTED=' "$ROOT/install/locale/fr.sh"

# Regression (issue #632): install_host_updater must set $SUDO itself before
# reading it, not rely on an earlier full-flow step (e.g. install_docker)
# having already called require_root_for as a side effect — that implicit
# ordering broke under --only=updater, which calls this function with
# nothing having set $SUDO yet ("SUDO: unbound variable" under set -u).
# install_host_updater runs as a subshell (note the "(" not "{" at its
# definition), so an assertion made out here in the parent test script can't
# observe its internal $SUDO — instead, bypass the network download (via
# MOWGLI_UPDATER_BINARY) to run far enough to pass every real $SUDO-prefixed
# line, and assert the captured output never contains "unbound variable".
# It is still expected to fail and return non-zero — at the hardcoded
# /usr/local/bin/mowgli-updater path this sandbox deliberately never creates
# (this test must never write real host files, per the file header) — the
# point is only that it fails there, for that controlled reason, not earlier
# for an unbound-variable crash.
rm -f -- "$sandbox/.updater-managed"
HARDWARE_BACKEND=mowgli; TFLUNA_FRONT_ENABLED=false; TFLUNA_EDGE_ENABLED=false; ENABLE_VESC=false
REPO_DIR="$ROOT"; REPO_URL=https://github.com/mowglinext/mowglinext.git; INSTALL_DIR="$ROOT/install"
detect_cpu_arch() { echo amd64; }
uname() { echo Linux; }
git() { printf '%040d\n' 3; }
systemctl() { [[ "${1:-}" == cat ]] && return 1; return 0; }
install() { :; }
fake_binary="$sandbox/fake-mowgli-updater"
cat > "$fake_binary" <<'FAKEBIN'
#!/usr/bin/env bash
case "${1:-}" in
  version) exit 0 ;;
  installer-config) echo '{}' ;;
  installer-select) exit 0 ;;
  *) exit 0 ;;
esac
FAKEBIN
chmod +x "$fake_binary"
MOWGLI_UPDATER_BINARY="$fake_binary"
unset SUDO
updater_output="$(install_host_updater 2>&1)" && updater_ec=0 || updater_ec=$?
unset MOWGLI_UPDATER_BINARY
[[ "$updater_output" != *"unbound variable"* ]]
[[ "$updater_ec" -ne 0 ]]
[[ ! -e "$sandbox/.updater-managed" ]]

printf 'Updater installer contract passed\n'
