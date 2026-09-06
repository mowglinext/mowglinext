#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
compose_file="${1:-${script_dir}/docker-compose.simulation.yaml}"
service="${2:-dev-sim}"

# These files describe one execution, not the simulated garden. Preserve
# areas.dat and any other map fixtures so each test runs against its declared
# geometry while localization and coverage progress always start fresh.
docker compose -f "${compose_file}" run --rm --no-deps \
  --entrypoint /bin/rm "${service}" -f \
  /ros2_ws/maps/fusion_graph.graph \
  /ros2_ws/maps/fusion_graph.meta \
  /ros2_ws/maps/fusion_graph.scans \
  /ros2_ws/maps/coverage_resume.txt
