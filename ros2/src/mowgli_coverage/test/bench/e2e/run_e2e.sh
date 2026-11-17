#!/bin/bash
# End-to-end plan_coverage A/B: two real coverage_server processes, one client.
#   run_e2e.sh BASE_EXE CAND_EXE CLIENT_EXE PASSES REPS CPU FIXTURE.json...
# Both servers get the shipped coverage_server parameters (navigation.launch.py
# injection for the template robot) with num_headland_passes = PASSES; both are
# pinned to CPU. Uses ROS_DOMAIN_ID 172, localhost discovery only. Server logs
# (with their per-stage "PlanCoverage timing" lines) go to $E2E_LOGS if set.
set -euo pipefail
base=$1 cand=$2 client=$3 passes=$4 reps=$5 cpu=$6
shift 6
export ROS_DOMAIN_ID=172 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
work=$(mktemp -d)
logs=${E2E_LOGS:-$work}
trap 'kill $(jobs -p) 2>/dev/null || true; wait 2>/dev/null || true; rm -rf "$work"' EXIT
for ns in base cand; do
  cat > "$work/$ns.yaml" <<YAML
/$ns/coverage_server:
  ros__parameters:
    robot_width: 0.45
    operation_width: 0.16
    default_headland_width: 0.18
    num_headland_passes: $passes
    chassis_safety_inset: 0.0
    min_swath_length: 0.15
    ring_direction: 0
    min_turning_radius: 0.20
    connector_turn_radius: 0.20
    connector_max_headland_passes: 0
    obstacle_margin: 0.389
    pivot_sweep_radius: 0.5970971445250765
    boundary_soft_margin: 0.5970971445250765
    footprint_half_width: 0.275
YAML
done
# Each server must load ITS OWN libmowgli_coverage_core.so: LD_LIBRARY_PATH (a
# sourced install) outranks the executables' RUNPATH, so put each one's lib first.
own_lib() { echo "$(cd "$(dirname "$1")/.." && pwd):${LD_LIBRARY_PATH:-}"; }
LD_LIBRARY_PATH=$(own_lib "$base") taskset -c "$cpu" "$base" --ros-args -r __ns:=/base --params-file "$work/base.yaml" \
  > "$logs/base.log" 2>&1 &
LD_LIBRARY_PATH=$(own_lib "$cand") taskset -c "$cpu" "$cand" --ros-args -r __ns:=/cand --params-file "$work/cand.yaml" \
  > "$logs/cand.log" 2>&1 &
servers=$(jobs -p)
sleep 2
for pid in $servers; do  # prove which planner library each server mapped
  child=$(pgrep -P "$pid" || echo "$pid")
  echo "server $child: $(grep -o '/[^ ]*libmowgli_coverage_core.so' /proc/$child/maps | sort -u)"
done
"$client" "$reps" "$@"
