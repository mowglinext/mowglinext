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

section "External Universal GNSS v0.7.1-rc3 sidecar contract"

compose_content="$(<"$compose_file")"
assert_contains "gps service consumes external Universal GNSS image" \
  'image: ${UNIVERSAL_GNSS_IMAGE:-ghcr.io/pepeuch/universal-gnss-ros2-lyrical:' "$compose_content"
assert_not_contains "gps service no longer consumes MowgliNext GPS_IMAGE" \
  'image: ${GPS_IMAGE}' "$compose_content"
# mowgli_robot.yaml is the ONE GNSS configuration: no derived parameter file on
# the host, no GNSS_* variable the compose file cannot live without. A robot
# that reaches this fragment through the updater has neither.
assert_contains "sidecar reads mowgli_robot.yaml itself" \
  './docker/config/mowgli:/config:ro' "$compose_content"
assert_not_contains "no derived parameter file is bind-mounted from the host" \
  'parameters.yaml:/etc/universal_gnss' "$compose_content"
assert_contains "generated ROS parameters live in memory only" \
  '/run/universal_gnss:uid=1000,gid=1000,mode=0700,size=64m' "$compose_content"
# The host updater refuses a release that ADDS writable storage (stack.go
# validateStackMounts): /dev was already mounted by the previous gps service,
# everything else must be read-only or tmpfs.
writable_mounts="$(awk '/^    volumes:/{v=1;next} v&&/^    [a-z_]+:/{v=0} v&&/^      - /{print}' "$compose_file" | grep -v ':ro$' | grep -v -- '- /dev:/dev$' || true)"
assert_eq "gps fragment adds no writable storage" "" "$writable_mounts"
assert_not_contains "gps fragment declares no named volume" 'universal_gnss_logs' "$compose_content"
assert_contains "receiver port comes from gnss_serial_device" \
  '"serial_device": str(p["gnss_serial_device"])' "$compose_content"
assert_not_contains "sidecar is not privileged" 'privileged: true' "$compose_content"
required_vars="$(grep -oE '\$\{[A-Z_0-9]+\}' "$compose_file" | sort -u | tr '\n' ' ')"
assert_eq "every compose variable of the gps fragment has a default" "" "$required_vars"
assert_contains "fragment image default is pinned to the deployment descriptor digest" \
  "@$(python3 -c 'import json,sys; print(next(i["digest"] for i in json.load(open(sys.argv[1]))["components"] if i["name"] == "gps"))' "$REPO_DIR/install/deployment.json")}" "$compose_content"
assert_contains "v0.7.1-rc3 combined receiver/NTRIP launch is used" \
  'receiver_and_ntrip.launch.py' "$compose_content"
assert_contains "NTRIP enable state is a launch argument derived from the yaml" \
  '"ntrip_enabled:=" + ("true" if ntrip else "false")' "$compose_content"
assert_contains "Mowgli fix topic is selected natively" \
  'fix_topic:=/gps/fix' "$compose_content"
assert_contains "bridge status topic stays internal to Universal GNSS" \
  'status_topic:=/universal_gnss_receiver/status' "$compose_content"
assert_contains "bridge RTCM topic stays internal to Universal GNSS" \
  'rtcm_topic:=/universal_gnss_receiver/rtcm' "$compose_content"

config_content="$(<"$config_file")"
assert_contains "MowgliNext pins v0.7.1-rc3 Lyrical image independently" \
  'UNIVERSAL_GNSS_IMAGE_DEFAULT="ghcr.io/pepeuch/universal-gnss-ros2-lyrical:v0.7.1-rc3@sha256:4e7960132882f2f83fb2b1e7d1430b4dfd00081d4be15d7d8ab20dacf7f22bc5
"' "$config_content"
# The installer default and the deployment descriptor name the SAME image; the
# digest is what makes the pin real, so a bump must touch both or fail here.
descriptor_digest="$(python3 -c 'import json,sys; print(next(i["digest"] for i in json.load(open(sys.argv[1]))["components"] if i["name"] == "gps"))' "$REPO_DIR/install/deployment.json" 2>/dev/null || true)"
# A digest is "sha256:" + 64 hex. The parity checks below compare strings, so a
# typo copied to every file ("ssha256:") satisfied them all while docker refused
# the reference ("unsupported digest algorithm") — the gps container could not
# have been pulled on any robot.
if [[ "${descriptor_digest:-}" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  pass "deployment descriptor gps digest is a well-formed sha256 digest"
else
  fail "deployment descriptor gps digest is a well-formed sha256 digest" "got '${descriptor_digest:-}'"
fi
malformed_refs="$(grep -hoE 'universal-gnss-ros2-lyrical:[A-Za-z0-9._-]+@[^"[:space:]}]+' \
  "$compose_file" "$config_file" "$env_example" | grep -vE '@sha256:[0-9a-f]{64}$' || true)"
assert_eq "every pinned Universal GNSS image reference uses a well-formed digest" "" "$malformed_refs"
assert_contains "installer default is pinned to the deployment descriptor digest" \
  "@${descriptor_digest:-MISSING-DIGEST}\"" "$config_content"
assert_not_contains "installer no longer writes a derived GNSS parameter file" \
  'write_universal_gnss_parameters' "$config_content"

stack_content="$(<"$stack_file")"
assert_contains "stack regen rebuilds sidecar runtime config" \
  'regenerate_sidecar_runtime_configs' "$stack_content"

env_example_content="$(<"$env_example")"
assert_contains "example uses v0.7.1-rc3 Lyrical image" \
  'UNIVERSAL_GNSS_IMAGE=ghcr.io/pepeuch/universal-gnss-ros2-lyrical:v0.7.1-rc3@sha256:4e7960132882f2f83fb2b1e7d1430b4dfd00081d4be15d7d8ab20dacf7f22bc5
' "$env_example_content"

test_summary
