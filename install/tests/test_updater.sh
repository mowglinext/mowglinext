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
grep -q 'ExecStart=/usr/local/bin/mowgli-updater supervise' "$ROOT/install/systemd/mowgli-updater.service"
grep -q 'MOWGLI_UPDATE_MAINTENANCE' "$ROOT/install/compose/docker-compose.updater.yml"
grep -q 'MSG_UPDATER_UNSUPPORTED=' "$ROOT/install/locale/en.sh"
grep -q 'MSG_UPDATER_UNSUPPORTED=' "$ROOT/install/locale/fr.sh"
printf 'Updater installer contract passed\n'
