#!/usr/bin/env bash
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/lib/framework.sh"

section "MAVROS / Universal GNSS source-selection contract"

config_content="$(<"$REPO_DIR/install/lib/config.sh")"
assert_contains "GNSS source has an explicit normalizer" "normalize_gnss_source()" "$config_content"
assert_contains "GNSS source accepts direct" "direct|soc|host|companion" "$config_content"
assert_contains "GNSS source accepts MAVROS" "mavros|pixhawk|fcu" "$config_content"
assert_contains "legacy installs default to direct GNSS" "default_gnss_source()" "$config_content"

env_content="$(<"$REPO_DIR/install/lib/env.sh")"
assert_not_contains "setup_env does not force GNSS_BACKEND=disabled for MAVROS" 'GNSS_BACKEND="disabled"' "$env_content"
assert_not_contains "setup_env does not force GNSS_STACK=disabled for MAVROS" 'GNSS_STACK="disabled"' "$env_content"
assert_contains "MAVROS receiver selects external status source" '"$GNSS_SOURCE" == "mavros"' "$env_content"
assert_contains "installer persists GNSS_SOURCE" '"GNSS_SOURCE" "$GNSS_SOURCE"' "$env_content"

compose_logic="$(<"$REPO_DIR/install/lib/compose.sh")"
assert_contains "direct source selects direct UG fragment" 'docker-compose.gps.yml' "$compose_logic"
assert_contains "MAVROS source selects NTRIP-only UG fragment" 'docker-compose.gps-mavros.yml' "$compose_logic"

mavros_compose="$(<"$REPO_DIR/install/compose/docker-compose.mavros.yml")"
assert_contains "MAVROS receives GNSS source mode" 'GNSS_SOURCE: ${GNSS_SOURCE:-direct}' "$mavros_compose"
assert_contains "MAVROS receives selected receiver index" 'GNSS_MAVROS_SOURCE: ${GNSS_MAVROS_SOURCE:-gps1}' "$mavros_compose"
assert_not_contains "MAVROS compose has no legacy standalone NTRIP service" "container_name: mowgli-ntrip" "$mavros_compose"
assert_not_contains "MAVROS compose has no legacy standalone NTRIP launch" "mowgli_ntrip_client" "$mavros_compose"

mavros_gnss_compose="$(<"$REPO_DIR/install/compose/docker-compose.gps-mavros.yml")"
assert_contains "MAVROS GNSS sidecar consumes UG MAVROS status" '/mavros/universal_gnss/%s/status' "$mavros_gnss_compose"
assert_contains "MAVROS GNSS sidecar publishes /rtcm" '"rtcm:=/rtcm"' "$mavros_gnss_compose"
assert_contains "MAVROS GNSS sidecar runs ntrip_node" '"ntrip_node"' "$mavros_gnss_compose"
assert_not_contains "MAVROS GNSS sidecar never owns a serial receiver" '- /dev:/dev' "$mavros_gnss_compose"

checks_content="$(<"$REPO_DIR/install/lib/checks.sh")"
assert_contains "device check distinguishes direct GNSS" '"$gnss_source" == "direct"' "$checks_content"
assert_contains "MAVROS has a separate health check" "check_mavros()" "$checks_content"

test_summary
