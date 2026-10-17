#!/bin/bash
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
#
# capture_gnss_container_health.sh — companion to mow_session_monitor.py for
# the mowglinext#694 field-evidence set. Everything mow_session_monitor.py
# records comes from ROS topics/services inside mowgli-ros2; the gps
# container's restart count and receiver_node/ntrip_node exit/restart events
# are Docker-level facts that node has no visibility into, so this runs on
# the HOST instead (not via docker exec into mowgli-ros2).
#
# Usage: run this ALONGSIDE mow_session_monitor.py, with the SAME session
# name, so the two output files line up by name for later correlation:
#
#   bash capture_gnss_container_health.sh my-session-name &
#   ... start mow_session_monitor.py, do the mow, stop it ...
#   kill %1   # or let it run — Ctrl-C stops it cleanly
#
# Writes, in the current directory:
#   <session>.container-health.jsonl   RestartCount sampled once per second
#   <session>.receiver-logs.txt        docker logs -f mowgli-gps, unfiltered
set -euo pipefail

SESSION="${1:?usage: $0 <session-name> [container-name]}"
CONTAINER="${2:-mowgli-gps}"
HEALTH_FILE="${SESSION}.container-health.jsonl"
LOGS_FILE="${SESSION}.receiver-logs.txt"

echo "Capturing ${CONTAINER} restart count -> ${HEALTH_FILE}"
echo "Capturing ${CONTAINER} logs          -> ${LOGS_FILE}"
echo "Ctrl-C to stop."

cleanup() {
  jobs -p | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Raw logs, unfiltered — the "process has died [pid ... exit code -2 ...]"
# SIGINT/restart lines and everything around them for context.
docker logs -f -t "$CONTAINER" >"$LOGS_FILE" 2>&1 &

# RestartCount + Health status, once per second. A change in RestartCount
# between two samples means the container was recreated/restarted in that
# window — cross-reference the timestamp against mow_session_monitor.py's
# JSONL (session_elapsed_sec) and the logs file above.
while true; do
  ts="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
  restart_count="$(docker inspect "$CONTAINER" --format '{{.RestartCount}}' 2>/dev/null || echo null)"
  health="$(docker inspect "$CONTAINER" --format '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' 2>/dev/null || echo null)"
  running="$(docker inspect "$CONTAINER" --format '{{.State.Running}}' 2>/dev/null || echo null)"
  printf '{"ts":"%s","restart_count":%s,"health":"%s","running":%s}\n' \
    "$ts" "$restart_count" "$health" "$running" >>"$HEALTH_FILE"
  sleep 1
done
