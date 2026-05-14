#!/usr/bin/env bash
# =============================================================================
# GNSS runtime contract — common topics and diagnostics across backends
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

section "Common topic contract is documented in compose fragments"

gps_compose="$(cat "$REPO_ROOT/install/compose/docker-compose.gps.yml")"
nmea_compose="$(cat "$REPO_ROOT/install/compose/docker-compose.nmea.yaml")"
unicore_compose="$(cat "$REPO_ROOT/install/compose/docker-compose.unicore.yaml")"

assert_contains "gps compose documents /gps/fix" "/gps/fix" "$gps_compose"
assert_contains "gps compose documents /gps/azimuth" "/gps/azimuth" "$gps_compose"
assert_contains "gps compose documents /ntrip_client/rtcm" "/ntrip_client/rtcm" "$gps_compose"
assert_contains "gps compose documents /diagnostics" "/diagnostics" "$gps_compose"

assert_contains "nmea compose documents /gps/fix" "/gps/fix" "$nmea_compose"
assert_contains "nmea compose documents /gps/azimuth" "/gps/azimuth" "$nmea_compose"
assert_contains "nmea compose documents /diagnostics" "/diagnostics" "$nmea_compose"

assert_contains "unicore compose keeps package env source of truth" "UNICORE_ROS_PACKAGE" "$unicore_compose"
assert_contains "unicore compose keeps executable env source of truth" "UNICORE_ROS_EXECUTABLE" "$unicore_compose"

section "Startup scripts publish the common ROS graph"

generic_start="$(cat "$REPO_ROOT/sensors/gps/start_gps.sh")"
nmea_start="$(cat "$REPO_ROOT/sensors/nmea/start_nmea.sh")"
unicore_start="$(cat "$REPO_ROOT/sensors/unicore/start_gps.sh")"

assert_contains "generic GPS remaps fix to /gps/fix" "-r /fix:=/gps/fix" "$generic_start"
assert_contains "generic NMEA remaps heading to /gps/azimuth" "-r /heading:=/gps/azimuth" "$generic_start"
assert_contains "generic GPS publishes diagnostics" "/diagnostics" "$(cat "$REPO_ROOT/sensors/gps/gps_health_aggregator.py")"
assert_contains "generic NMEA supports common fix diagnostics" "GNSS_DIAGNOSTIC_BACKEND=\"nmea\"" "$generic_start"
assert_contains "generic GPS gates fix diagnostics behind flag" "GNSS_ENABLE_FIX_DIAGNOSTICS" "$generic_start"

assert_contains "standalone NMEA remaps fix to /gps/fix" "-r /fix:=/gps/fix" "$nmea_start"
assert_contains "standalone NMEA remaps heading to /gps/azimuth" "-r /heading:=/gps/azimuth" "$nmea_start"
assert_contains "standalone NMEA starts common diagnostics" "GNSS_DIAGNOSTIC_BACKEND=\"nmea\"" "$nmea_start"
assert_contains "standalone NMEA defaults fix diagnostics on" 'GNSS_ENABLE_FIX_DIAGNOSTICS="${GNSS_ENABLE_FIX_DIAGNOSTICS:-true}"' "$nmea_start"

assert_contains "unicore fix topic is /gps/fix" "fix_topic:=/gps/fix" "$unicore_start"
assert_contains "unicore heading topic is /gps/azimuth" "heading_topic:=/gps/azimuth" "$unicore_start"
assert_contains "unicore diagnostics topic is /diagnostics" "diagnostics_topic:=/diagnostics" "$unicore_start"
assert_contains "unicore RTCM topic is /ntrip_client/rtcm" "rtcm_topic:=/ntrip_client/rtcm" "$unicore_start"
assert_not_contains "unicore does not launch fix diagnostics by default" "gnss_fix_diagnostics.py" "$unicore_start"
assert_not_contains "unicore default no longer hardcodes um982_node" "ros2 run mowgli_unicore_gnss um982_node" "$unicore_start"

section "Diagnostics ownership"

assert_contains "NMEA launches gnss_fix_diagnostics.py" "python3 /gnss_fix_diagnostics.py" "$nmea_start"
assert_contains "generic NMEA can launch gnss_fix_diagnostics.py" "python3 /gnss_fix_diagnostics.py" "$generic_start"
assert_contains "u-blox path uses detailed diagnostics aggregator" "python3 /gps_health_aggregator.py" "$generic_start"
assert_contains "u-blox diagnostics aggregator publishes /diagnostics" "/diagnostics" "$(cat "$REPO_ROOT/sensors/gps/gps_health_aggregator.py")"
assert_contains "Unicore driver expected to publish /diagnostics" "diagnostics_topic:=/diagnostics" "$unicore_start"

test_summary
