#!/usr/bin/env bash
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/lib/framework.sh"

section "MAVROS / Universal GNSS independence contract"

assert_not_contains "default GNSS stack is not conditional on MAVROS"   'HARDWARE_BACKEND:-mowgli}" == "mavros"'   "$(sed -n '/^default_gnss_stack()/,/^}/p' "$REPO_DIR/install/lib/config.sh")"
assert_not_contains "effective GNSS backend never disables solely for MAVROS"   'HARDWARE_BACKEND:-mowgli}" == "mavros"'   "$(sed -n '/^effective_gnss_backend()/,/^}/p' "$REPO_DIR/install/lib/config.sh")"
assert_not_contains "effective GNSS stack never disables solely for MAVROS"   'HARDWARE_BACKEND:-mowgli}" == "mavros"'   "$(sed -n '/^effective_gnss_stack()/,/^}/p' "$REPO_DIR/install/lib/config.sh")"

env_content="$(<"$REPO_DIR/install/lib/env.sh")"
assert_not_contains "setup_env does not force GNSS_BACKEND=disabled for MAVROS" 'GNSS_BACKEND="disabled"' "$env_content"
assert_not_contains "setup_env does not force GNSS_STACK=disabled for MAVROS" 'GNSS_STACK="disabled"' "$env_content"

compose_content="$(<"$REPO_DIR/install/compose/docker-compose.mavros.yml")"
assert_not_contains "MAVROS compose has no standalone NTRIP service" "container_name: mowgli-ntrip" "$compose_content"
assert_not_contains "MAVROS compose has no standalone NTRIP launch" "mowgli_ntrip_client" "$compose_content"
assert_contains "MAVROS receives generated config copy" "./docker/config/mavros:/ros2_ws/config:ro" "$compose_content"

checks_content="$(<"$REPO_DIR/install/lib/checks.sh")"
assert_not_contains "GPS check does not reject mowgli-gps under MAVROS" "mowgli-gps must not run when HARDWARE_BACKEND=mavros" "$checks_content"
assert_contains "MAVROS has a separate health check" "check_mavros()" "$checks_content"

test_summary
