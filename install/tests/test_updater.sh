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
REPO_DIR="$ROOT"; INSTALL_DIR="$ROOT/install"
source "$ROOT/install/lib/compose.sh"
effective_gnss_backend() { echo disabled; }
effective_gnss_stack() { echo disabled; }
is_supported_gnss_backend() { return 0; }
# Same reason as the GNSS stub above: this file exercises compose.sh in
# isolation, so every collaborator it calls has to be provided here. The real
# definition lives in config.sh — the stack.sh source-set guard at the bottom
# is what checks that compose.sh's dependencies are actually reachable in
# production.
is_supported_hardware_backend() { case "${1:-}" in mowgli|mavros|openmower) return 0;; *) return 1;; esac; }
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
printf 'Updater installer contract passed\n'

# ---------------------------------------------------------------------------
# docker/stack.sh sources a NARROWER lib set than the installer: common.sh,
# config.sh, docker.sh, deploy.sh, compose.sh — and notably NOT
# backend_choice.sh. Anything build_compose_stack calls must therefore be
# defined in one of those. A guard that lived in backend_choice.sh once made
# `stack.sh regen` fail for EVERY backend, because `! <missing command>`
# evaluates TRUE and the "Unknown HARDWARE_BACKEND" branch fired on the
# default install too.
#
# Runs in a FRESH bash, not a subshell: a subshell inherits the stubs defined
# above (that is precisely how the first version of this guard passed while
# the regression was still present), which would mask the very definitions it
# is here to prove exist.
# ---------------------------------------------------------------------------
MOWGLI_TEST_ROOT="$ROOT" bash -euo pipefail -s <<'STACK_SOURCE_GUARD'
ROOT="$MOWGLI_TEST_ROOT"
stack_sandbox="$(mktemp -d)"
trap 'rm -rf -- "$stack_sandbox"' EXIT
export MOWGLI_HOME="$ROOT"
# Exactly what docker/stack.sh sources, in its order.
# shellcheck source=/dev/null
source "$ROOT/install/lib/common.sh"
# shellcheck source=/dev/null
source "$ROOT/install/lib/config.sh"
# shellcheck source=/dev/null
source "$ROOT/install/lib/docker.sh"
# shellcheck source=/dev/null
source "$ROOT/install/lib/deploy.sh"
# shellcheck source=/dev/null
source "$ROOT/install/lib/compose.sh"
DOCKER_DIR="$stack_sandbox"
LIDAR_ENABLED=false
for backend in mowgli mavros openmower; do
  HARDWARE_BACKEND="$backend"
  if ! build_compose_stack >/dev/null 2>&1; then
    echo "build_compose_stack failed for HARDWARE_BACKEND=$backend with stack.sh's source set" >&2
    exit 1
  fi
  [[ ${#COMPOSE_FILES[@]} -gt 0 ]]
done
# An unknown backend must still be rejected, not silently composed.
HARDWARE_BACKEND=definitely-not-a-backend
if build_compose_stack >/dev/null 2>&1; then
  echo "build_compose_stack accepted an unknown HARDWARE_BACKEND" >&2
  exit 1
fi
STACK_SOURCE_GUARD
