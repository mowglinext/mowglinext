#!/usr/bin/env bash
# =============================================================================
# Mowgli stack manager — drive the Docker stack straight from this checkout
# without re-running the full host installer (install/mowglinext.sh).
#
# Reuses the installer's compose-assembly machinery (install/lib/compose.sh)
# but skips all host setup (UARTs, udev, docker install, GPS probing). All
# runtime parameters live in docker/.env (gitignored); the generated
# docker/docker-compose.yaml and docker/config/* are gitignored runtime files.
#
#   ./docker/stack.sh regen     # regenerate docker-compose.yaml from .env
#   ./docker/stack.sh up        # regen + (re)create the stack (-d)
#   ./docker/stack.sh down      # stop + remove containers (keeps volumes)
#   ./docker/stack.sh restart   # restart running services
#   ./docker/stack.sh pull      # regen + pull images
#   ./docker/stack.sh update    # pull + up (apply newest images)
#   ./docker/stack.sh logs [-f] [svc]
#   ./docker/stack.sh ps
#   ./docker/stack.sh config    # print the generated compose (validates it)
#   ./docker/stack.sh <other>   # passthrough to `docker compose <other>`
# =============================================================================

set -euo pipefail

# --- Paths -------------------------------------------------------------------
# pwd -P (physical) so a stale $PWD / symlinked invocation can't point us at a
# different checkout — the stack must bind from the checkout holding this file.
DOCKER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
REPO_DIR="$(dirname "$DOCKER_DIR")"
INSTALL_DIR="$REPO_DIR/install"
INSTALL_LIB_DIR="$INSTALL_DIR/lib"
COMPOSE_SRC_DIR="$INSTALL_DIR/compose"
FINAL_COMPOSE_FILE="$DOCKER_DIR/docker-compose.yaml"
FINAL_ENV_FILE="$DOCKER_DIR/.env"

# install/lib/config.sh derives REPO_DIR and friends from MOWGLI_HOME at source
# time (default $HOME/mowglinext). Point it at THIS checkout so the stack binds
# from here and not from a sibling install.
export MOWGLI_HOME="$REPO_DIR"

# Exported so install/lib/compose.sh (which guards `: "${REPO_DIR:?}"`) and
# write_compose_merged pick them up.
export REPO_DIR INSTALL_DIR DOCKER_DIR COMPOSE_SRC_DIR FINAL_COMPOSE_FILE FINAL_ENV_FILE

if [[ ! -f "$FINAL_ENV_FILE" ]]; then
  echo "[x] Missing $FINAL_ENV_FILE — copy docker/.env.example to docker/.env first." >&2
  exit 1
fi

# --- Load runtime parameters BEFORE sourcing libs ----------------------------
# install/lib/config.sh runs recompute_image_defaults at source time, which
# expands ${IMAGE_TAG} under `set -u`; loading .env first makes it available.
set -a
# shellcheck disable=SC1090
source "$FINAL_ENV_FILE"
set +a

: "${IMAGE_TAG:=main}"
: "${COMPOSE_PROJECT_NAME:=install}"
export IMAGE_TAG COMPOSE_PROJECT_NAME

# --- Reuse the installer's compose machinery (no host setup) -----------------
# shellcheck source=/dev/null
source "$INSTALL_LIB_DIR/common.sh"
# shellcheck source=/dev/null
source "$INSTALL_LIB_DIR/config.sh"
# shellcheck source=/dev/null
source "$INSTALL_LIB_DIR/docker.sh"
# deploy.sh provides fix_path_type_conflict / backup_path_if_exists, which
# ensure_default_configs() calls to self-heal bind-mount dir-squatting.
# shellcheck source=/dev/null
source "$INSTALL_LIB_DIR/deploy.sh"
# shellcheck source=/dev/null
source "$INSTALL_LIB_DIR/compose.sh"

# Re-assert our canonical paths: config.sh recomputes these at source time, so
# pin them back to this checkout regardless of what the libs derived.
REPO_DIR="$MOWGLI_HOME"
DOCKER_DIR="$REPO_DIR/docker"
INSTALL_DIR="$REPO_DIR/install"
COMPOSE_SRC_DIR="$INSTALL_DIR/compose"
FINAL_COMPOSE_FILE="$DOCKER_DIR/docker-compose.yaml"
FINAL_ENV_FILE="$DOCKER_DIR/.env"

# Drop optional fragments the operator hasn't opted into. build_compose_stack
# always appends mqtt + watchtower; gate them on .env toggles here so docker/.env
# stays the single source of truth without modifying shared installer code.
filter_optional_fragments() {
  local kept=() f base
  for f in "${COMPOSE_FILES[@]}"; do
    base="$(basename "$f")"
    case "$base" in
      docker-compose.mqtt.yml)
        [[ "${ENABLE_MQTT:-false}" == "true" ]] || continue
        ;;
      docker-compose.watchtower.yml)
        [[ "${ENABLE_WATCHTOWER:-false}" == "true" ]] || continue
        ;;
    esac
    kept+=("$f")
  done
  COMPOSE_FILES=("${kept[@]}")
}

regen() {
  step "Regenerating $FINAL_COMPOSE_FILE from docker/.env"
  ensure_default_configs
  build_compose_stack
  filter_optional_fragments
  write_compose_merged
}

# All lifecycle commands run against the generated file, pinned to the project
# name from .env (preserves the install_mowgli_maps volume) and the repo as
# project directory (resolves the fragments' relative bind paths).
compose() {
  docker_compose_cmd \
    --project-name "$COMPOSE_PROJECT_NAME" \
    --project-directory "$REPO_DIR" \
    -f "$FINAL_COMPOSE_FILE" \
    --env-file "$FINAL_ENV_FILE" \
    "$@"
}

require_compose_file() {
  [[ -f "$FINAL_COMPOSE_FILE" ]] || regen
}

cmd="${1:-}"
[[ $# -gt 0 ]] && shift || true

case "$cmd" in
  regen)
    regen
    ;;
  up)
    regen
    compose up -d --remove-orphans "$@"
    compose ps
    ;;
  down)
    require_compose_file
    compose down "$@"
    ;;
  restart)
    require_compose_file
    compose restart "$@"
    ;;
  pull)
    regen
    compose pull "$@"
    ;;
  update)
    regen
    compose pull "$@"
    compose up -d --remove-orphans
    compose ps
    ;;
  logs)
    require_compose_file
    compose logs "$@"
    ;;
  ps)
    require_compose_file
    compose ps "$@"
    ;;
  config)
    require_compose_file
    compose config "$@"
    ;;
  ""|-h|--help|help)
    sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    ;;
  *)
    require_compose_file
    compose "$cmd" "$@"
    ;;
esac
