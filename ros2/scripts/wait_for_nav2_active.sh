#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
  echo "Usage: $0 <launch-pid> <launch-log> [timeout-seconds]" >&2
  exit 2
fi

launch_pid="$1"
launch_log="$2"
timeout_seconds="${3:-180}"
deadline=$((SECONDS + timeout_seconds))
ready_message="Managed nodes are active"

while (( SECONDS < deadline )); do
  if grep -Fq "$ready_message" "$launch_log" 2>/dev/null; then
    echo "Nav2 lifecycle manager reports: $ready_message"
    exit 0
  fi

  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "Simulation launch exited before Nav2 became active" >&2
    tail -n 80 "$launch_log" >&2 || true
    exit 1
  fi

  sleep 1
done

echo "Timed out after ${timeout_seconds}s waiting for Nav2 lifecycle activation" >&2
tail -n 80 "$launch_log" >&2 || true
exit 1
