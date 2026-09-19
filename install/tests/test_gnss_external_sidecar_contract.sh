#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

compose_file="$REPO_DIR/install/compose/docker-compose.gps.yml"
config_file="$REPO_DIR/install/lib/config.sh"
stack_file="$REPO_DIR/docker/stack.sh"
env_example="$REPO_DIR/docker/.env.example"

section "External Universal GNSS RC2 sidecar contract"

compose_content="$(<"$compose_file")"
assert_contains "gps service consumes external Universal GNSS image" \
  'image: ${UNIVERSAL_GNSS_IMAGE}' "$compose_content"
assert_not_contains "gps service no longer consumes MowgliNext GPS_IMAGE" \
  'image: ${GPS_IMAGE}' "$compose_content"
assert_contains "receiver is mapped to stable in-container path" \
  '${GNSS_DEVICE}:/dev/gnss-receiver' "$compose_content"
assert_contains "sidecar receives generated parameter file" \
  'parameters.yaml:/etc/universal_gnss/parameters.yaml:ro' "$compose_content"
assert_contains "RC2 combined receiver/NTRIP launch is used" \
  'receiver_and_ntrip.launch.py' "$compose_content"
assert_contains "NTRIP enable state is a launch argument" \
  'ntrip_enabled:=${GNSS_NTRIP_ENABLED:-true}' "$compose_content"
assert_contains "Mowgli fix topic is selected natively" \
  'fix_topic:=/gps/fix' "$compose_content"
assert_contains "bridge status topic stays internal to Universal GNSS" \
  'status_topic:=/universal_gnss_receiver/status' "$compose_content"
assert_contains "bridge RTCM topic stays internal to Universal GNSS" \
  'rtcm_topic:=/universal_gnss_receiver/rtcm' "$compose_content"

config_content="$(<"$config_file")"
assert_contains "MowgliNext pins RC2 Lyrical image independently" \
  'UNIVERSAL_GNSS_IMAGE_DEFAULT="ghcr.io/pepeuch/universal-gnss-ros2-lyrical:v0.1.4-rc2@sha256:24ea6c1c0553463207a4c33b803e920a974989f5891f2f107e8c35cce56f7a95"' "$config_content"
# The installer default and the deployment descriptor name the SAME image; the
# digest is what makes the pin real, so a bump must touch both or fail here.
descriptor_digest="$(python3 -c 'import json,sys; print(next(i["digest"] for i in json.load(open(sys.argv[1]))["components"] if i["name"] == "gps"))' "$REPO_DIR/install/deployment.json" 2>/dev/null || true)"
assert_contains "installer default is pinned to the deployment descriptor digest" \
  "@${descriptor_digest:-MISSING-DIGEST}\"" "$config_content"
assert_contains "derived receiver config uses stable device path" \
  'serial_device: /dev/gnss-receiver' "$config_content"
assert_contains "runtime config regeneration writes Universal GNSS parameters" \
  'write_universal_gnss_parameters' "$config_content"

stack_content="$(<"$stack_file")"
assert_contains "stack regen rebuilds sidecar runtime config" \
  'regenerate_sidecar_runtime_configs' "$stack_content"

env_example_content="$(<"$env_example")"
assert_contains "example uses RC2 Lyrical image" \
  'UNIVERSAL_GNSS_IMAGE=ghcr.io/pepeuch/universal-gnss-ros2-lyrical:v0.1.4-rc2@sha256:24ea6c1c0553463207a4c33b803e920a974989f5891f2f107e8c35cce56f7a95' "$env_example_content"

test_summary
