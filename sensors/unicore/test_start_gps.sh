#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

failures=0

assert_contains() {
  local label="${1:?assert_contains: missing label}"
  local needle="${2:?assert_contains: missing needle}"
  local haystack="${3-}"

  if [[ "$haystack" != *"$needle"* ]]; then
    echo "FAIL: $label"
    echo "  missing: $needle"
    echo "  actual:  $haystack"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

make_fake_ros2() {
  mkdir -p "$WORKDIR/bin"
  cat >"$WORKDIR/bin/ros2" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >>"$ROS2_LOG_PATH"
exit 0
EOF
  chmod +x "$WORKDIR/bin/ros2"
}

make_config() {
  cat >"$WORKDIR/mowgli_robot.yaml" <<'EOF'
gps_port: /dev/gps
gps_baudrate: 921600
unicore_auto_configure: false
ntrip_enabled: false
EOF
}

run_profile() {
  local profile="${1:?run_profile: missing profile}"
  local output_format="${2:-ascii}"
  local log_file="$WORKDIR/${profile}.log"
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$log_file" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_PROFILE="$profile" \
  UNICORE_OUTPUT_FORMAT="$output_format" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$log_file"
}

make_fake_ros2
make_config

NORMAL_ARGS="$(run_profile normal)"
assert_contains "normal uses default Unicore package/executable" "run unicore_gnss unicore_node" "$NORMAL_ARGS"
assert_contains "normal disables satellite diagnostics" "enable_satellite_status:=false" "$NORMAL_ARGS"
assert_contains "normal disables RF diagnostics" "enable_rf_status:=false" "$NORMAL_ARGS"
assert_contains "normal disables hardware diagnostics" "enable_hw_status:=false" "$NORMAL_ARGS"
assert_contains "normal disables jamming diagnostics" "enable_jamming_status:=false" "$NORMAL_ARGS"
assert_contains "normal disables raw observation diagnostics" "enable_raw_observation_diag:=false" "$NORMAL_ARGS"
assert_contains "normal keeps binary transport disabled" "enable_unicore_binary:=false" "$NORMAL_ARGS"
assert_contains "normal keeps binary nav disabled" "use_binary_nav:=false" "$NORMAL_ARGS"
assert_contains "normal keeps binary RTK disabled" "use_binary_rtk_diag:=false" "$NORMAL_ARGS"
assert_contains "normal keeps binary RTCM disabled" "use_binary_rtcm_diag:=false" "$NORMAL_ARGS"
assert_contains "normal keeps binary satellite diag disabled" "use_binary_satellite_diag:=false" "$NORMAL_ARGS"

DEBUG_ARGS="$(run_profile debug)"
assert_contains "debug enables satellite diagnostics" "enable_satellite_status:=true" "$DEBUG_ARGS"
assert_contains "debug enables RF diagnostics" "enable_rf_status:=true" "$DEBUG_ARGS"
assert_contains "debug enables hardware diagnostics" "enable_hw_status:=true" "$DEBUG_ARGS"
assert_contains "debug enables jamming diagnostics" "enable_jamming_status:=true" "$DEBUG_ARGS"
assert_contains "debug ascii keeps binary nav disabled" "use_binary_nav:=false" "$DEBUG_ARGS"
assert_contains "debug ascii keeps binary RTK disabled" "use_binary_rtk_diag:=false" "$DEBUG_ARGS"

HYBRID_ARGS="$(run_profile debug hybrid)"
assert_contains "hybrid output enables binary transport" "enable_unicore_binary:=true" "$HYBRID_ARGS"
assert_contains "debug hybrid still keeps binary nav disabled by default" "use_binary_nav:=false" "$HYBRID_ARGS"

DEBUG_HYBRID_BINARY_RTK_ARGS="$(
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$WORKDIR/debug_hybrid_binary_rtk.log" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_PROFILE="debug" \
  UNICORE_OUTPUT_FORMAT="hybrid" \
  UNICORE_USE_BINARY_RTK_DIAG="true" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$WORKDIR/debug_hybrid_binary_rtk.log"
)"
assert_contains "debug hybrid explicit binary RTK is forwarded" "use_binary_rtk_diag:=true" "$DEBUG_HYBRID_BINARY_RTK_ARGS"

SURVEY_HYBRID_ARGS="$(
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$WORKDIR/survey_hybrid.log" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_PROFILE="survey" \
  UNICORE_OUTPUT_FORMAT="hybrid" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$WORKDIR/survey_hybrid.log"
)"
assert_contains "survey hybrid enables binary transport" "enable_unicore_binary:=true" "$SURVEY_HYBRID_ARGS"
assert_contains "survey hybrid keeps nav on ascii by default" "use_binary_nav:=false" "$SURVEY_HYBRID_ARGS"
assert_contains "survey hybrid enables binary RTK diag" "use_binary_rtk_diag:=true" "$SURVEY_HYBRID_ARGS"
assert_contains "survey hybrid enables binary RTCM diag" "use_binary_rtcm_diag:=true" "$SURVEY_HYBRID_ARGS"
assert_contains "survey hybrid enables binary satellite diag" "use_binary_satellite_diag:=true" "$SURVEY_HYBRID_ARGS"
assert_contains "survey hybrid enables binary RF diag" "use_binary_rf_diag:=true" "$SURVEY_HYBRID_ARGS"
assert_contains "survey hybrid enables binary hardware diag" "use_binary_hw_diag:=true" "$SURVEY_HYBRID_ARGS"
assert_contains "survey hybrid enables binary jamming diag" "use_binary_jamming_diag:=true" "$SURVEY_HYBRID_ARGS"

RUNTIME_HYBRID_ARGS="$(
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$WORKDIR/runtime_hybrid.log" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_PROFILE="runtime" \
  UNICORE_OUTPUT_FORMAT="hybrid" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$WORKDIR/runtime_hybrid.log"
)"
assert_contains "runtime hybrid enables binary transport" "enable_unicore_binary:=true" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid keeps nav on ascii by default" "use_binary_nav:=false" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid enables binary RTK diag" "use_binary_rtk_diag:=true" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid enables binary RTCM diag" "use_binary_rtcm_diag:=true" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid enables binary satellite diag" "use_binary_satellite_diag:=true" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid enables binary RF diag" "use_binary_rf_diag:=true" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid enables binary hardware diag" "use_binary_hw_diag:=true" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid enables binary jamming diag" "use_binary_jamming_diag:=true" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid disables raw observation diag" "enable_raw_observation_diag:=false" "$RUNTIME_HYBRID_ARGS"
assert_contains "runtime hybrid disables binary raw observations" "use_binary_raw_observations:=false" "$RUNTIME_HYBRID_ARGS"

HIGH_PRECISION_HYBRID_ARGS="$(
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$WORKDIR/high_precision_hybrid.log" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_PROFILE="high_precision" \
  UNICORE_OUTPUT_FORMAT="hybrid" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$WORKDIR/high_precision_hybrid.log"
)"
assert_contains "high_precision hybrid enables binary nav by default" "use_binary_nav:=true" "$HIGH_PRECISION_HYBRID_ARGS"

DEBUG_BINARY_ARGS="$(
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$WORKDIR/debug_binary.log" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_PROFILE="debug" \
  UNICORE_OUTPUT_FORMAT="binary" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$WORKDIR/debug_binary.log"
)"
assert_contains "debug binary forces binary nav" "use_binary_nav:=true" "$DEBUG_BINARY_ARGS"
assert_contains "debug binary forces binary satellite diag" "use_binary_satellite_diag:=true" "$DEBUG_BINARY_ARGS"
assert_contains "debug binary forces binary RTK diag" "use_binary_rtk_diag:=true" "$DEBUG_BINARY_ARGS"
assert_contains "debug binary forces binary RTCM diag" "use_binary_rtcm_diag:=true" "$DEBUG_BINARY_ARGS"
assert_contains "debug binary forces binary RF diag" "use_binary_rf_diag:=true" "$DEBUG_BINARY_ARGS"
assert_contains "debug binary forces binary hardware diag" "use_binary_hw_diag:=true" "$DEBUG_BINARY_ARGS"
assert_contains "debug binary forces binary jamming diag" "use_binary_jamming_diag:=true" "$DEBUG_BINARY_ARGS"

SURVEY_RAW_ARGS="$(
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$WORKDIR/survey_raw.log" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_PROFILE="survey" \
  UNICORE_OUTPUT_FORMAT="hybrid" \
  UNICORE_ENABLE_RAW_OBSERVATIONS="true" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$WORKDIR/survey_raw.log"
)"
assert_contains "survey hybrid raw enables binary raw diagnostics" "enable_raw_observation_diag:=true" "$SURVEY_RAW_ARGS"
assert_contains "survey hybrid raw uses binary observations" "use_binary_raw_observations:=true" "$SURVEY_RAW_ARGS"

OVERRIDE_ARGS="$(
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$WORKDIR/override.log" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_ROS_PACKAGE="mowgli_unicore_gnss" \
  UNICORE_ROS_EXECUTABLE="um982_node" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$WORKDIR/override.log"
)"
assert_contains "compatibility override honors legacy package/executable" "run mowgli_unicore_gnss um982_node" "$OVERRIDE_ARGS"

if [ "$failures" -ne 0 ]; then
  echo ""
  echo "$failures test(s) failed."
  exit 1
fi

echo ""
echo "All tests passed."
