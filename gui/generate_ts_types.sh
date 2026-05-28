#!/bin/bash
#
# generate_ts_types.sh — Generate TypeScript types from ROS2 .msg files.
#
# The GUI frontend receives ROS messages as raw JSON via rosbridge WebSocket.
# Field names in JSON match the .msg definition (snake_case).  These TS types
# must therefore use snake_case property names — NOT PascalCase.
#
# Generates: gui/web/src/types/ros.generated.ts
#
# Usage:
#   cd gui && ./generate_ts_types.sh
#
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
MOWGLI_MSG_DIR="$SCRIPT_DIR/../ros2/src/mowgli_interfaces/msg"
OUT_FILE="$SCRIPT_DIR/web/src/types/ros.generated.ts"

# ── Embed standard ROS2 message definitions ──────────────────────────────────
STD_MSGS=$(mktemp -d)
trap 'rm -rf "$STD_MSGS"' EXIT

mkdir -p "$STD_MSGS/geometry_msgs"
cat > "$STD_MSGS/geometry_msgs/Point.msg" <<'MSG'
float64 x
float64 y
float64 z
MSG
cat > "$STD_MSGS/geometry_msgs/Point32.msg" <<'MSG'
float32 x
float32 y
float32 z
MSG
cat > "$STD_MSGS/geometry_msgs/Vector3.msg" <<'MSG'
float64 x
float64 y
float64 z
MSG
cat > "$STD_MSGS/geometry_msgs/Quaternion.msg" <<'MSG'
float64 x
float64 y
float64 z
float64 w
MSG
cat > "$STD_MSGS/geometry_msgs/Pose.msg" <<'MSG'
Point position
Quaternion orientation
MSG
cat > "$STD_MSGS/geometry_msgs/PoseWithCovariance.msg" <<'MSG'
Pose pose
float64[36] covariance
MSG
cat > "$STD_MSGS/geometry_msgs/PoseStamped.msg" <<'MSG'
Pose pose
MSG
cat > "$STD_MSGS/geometry_msgs/Polygon.msg" <<'MSG'
Point32[] points
MSG
cat > "$STD_MSGS/geometry_msgs/Twist.msg" <<'MSG'
Vector3 linear
Vector3 angular
MSG
cat > "$STD_MSGS/geometry_msgs/TwistStamped.msg" <<'MSG'
std_msgs/Header header
Twist twist
MSG
cat > "$STD_MSGS/geometry_msgs/TwistWithCovariance.msg" <<'MSG'
Twist twist
float64[36] covariance
MSG

mkdir -p "$STD_MSGS/nav_msgs"
cat > "$STD_MSGS/nav_msgs/Path.msg" <<'MSG'
PoseStamped[] poses
MSG
cat > "$STD_MSGS/nav_msgs/OccupancyGrid.msg" <<'MSG'
OccupancyGridInfo info
int8[] data
MSG

mkdir -p "$STD_MSGS/sensor_msgs"
cat > "$STD_MSGS/sensor_msgs/LaserScan.msg" <<'MSG'
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
MSG
cat > "$STD_MSGS/sensor_msgs/Imu.msg" <<'MSG'
Quaternion orientation
float64[9] orientation_covariance
Vector3 angular_velocity
float64[9] angular_velocity_covariance
Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
MSG

mkdir -p "$STD_MSGS/std_msgs"
cat > "$STD_MSGS/std_msgs/ColorRGBA.msg" <<'MSG'
float32 r
float32 g
float32 b
float32 a
MSG

mkdir -p "$STD_MSGS/visualization_msgs"
cat > "$STD_MSGS/visualization_msgs/Marker.msg" <<'MSG'
string ns
int32 id
int32 type
int32 action
Pose pose
Vector3 scale
ColorRGBA color
float32 lifetime
bool frame_locked
Point[] points
ColorRGBA[] colors
string text
string mesh_resource
bool mesh_use_embedded_materials
MSG
cat > "$STD_MSGS/visualization_msgs/MarkerArray.msg" <<'MSG'
Marker[] markers
MSG

# ── Helpers ──────────────────────────────────────────────────────────────────

# Map a ROS2 field type to a TypeScript type
ros_to_ts() {
    local rostype="$1"
    local is_array=false

    # Strip trailing [] or [N] for arrays
    if [[ "$rostype" =~ \[([0-9]*)\]$ ]]; then
        rostype="${rostype%%\[*}"
        is_array=true
    fi

    local tstype
    case "$rostype" in
        bool)                                    tstype="boolean" ;;
        int8|uint8|int16|uint16|int32|uint32|\
        int64|uint64|float32|float64)            tstype="number" ;;
        string)                                  tstype="string" ;;
        # Cross-package references
        builtin_interfaces/Time)                 tstype="{ sec: number; nanosec: number }" ;;
        std_msgs/Header)                         tstype="{ stamp: { sec: number; nanosec: number }; frame_id: string }" ;;
        std_msgs/ColorRGBA)                      tstype="ColorRGBA" ;;
        geometry_msgs/*)                         tstype="${rostype#geometry_msgs/}" ;;
        nav_msgs/*)                              tstype="${rostype#nav_msgs/}" ;;
        sensor_msgs/*)                           tstype="${rostype#sensor_msgs/}" ;;
        visualization_msgs/*)                    tstype="${rostype#visualization_msgs/}" ;;
        # Bare type name — same package
        *)                                       tstype="$rostype" ;;
    esac

    if $is_array; then
        echo "${tstype}[]"
    else
        echo "$tstype"
    fi
}

# Parse a .msg file and emit TS interface fields (snake_case, matching JSON).
# Skips constants (lines with =).
parse_ts_fields() {
    local file="$1"
    while IFS= read -r line; do
        # Strip comments
        line="${line%%#*}"
        # Trim whitespace
        line="$(echo "$line" | xargs)"
        [ -z "$line" ] && continue
        # Skip constants (contain =)
        [[ "$line" == *"="* ]] && continue

        local rostype field
        rostype="$(echo "$line" | awk '{print $1}')"
        field="$(echo "$line" | awk '{print $2}')"
        [ -z "$field" ] && continue

        local tstype
        tstype="$(ros_to_ts "$rostype")"

        echo "  ${field}?: ${tstype};"
    done < "$file"
}

# Parse constants from a .msg file and emit TS const enum entries
parse_ts_constants() {
    local file="$1"
    local has_constants=false
    while IFS= read -r line; do
        line="${line%%#*}"
        line="$(echo "$line" | xargs)"
        [ -z "$line" ] && continue
        # Only constants (contain =)
        [[ "$line" != *"="* ]] && continue

        local name_val
        # Extract "NAME=value" from "type NAME=value"
        name_val="$(echo "$line" | awk '{print $2}')"
        local name="${name_val%%=*}"
        local val="${name_val#*=}"

        if ! $has_constants; then
            has_constants=true
        fi
        echo "  ${name} = ${val},"
    done < "$file"
}

# ── Generate ─────────────────────────────────────────────────────────────────

{
    echo "// Code generated by generate_ts_types.sh — DO NOT EDIT."
    echo "// Re-run: cd gui && ./generate_ts_types.sh"
    echo "//"
    echo "// Field names are snake_case to match the JSON from rosbridge."
    echo ""

    # Standard geometry types
    for file in "$STD_MSGS/geometry_msgs"/*.msg; do
        [ -f "$file" ] || continue
        name="$(basename "$file" .msg)"
        echo "export type ${name} = {"
        parse_ts_fields "$file"
        echo "};"
        echo ""
    done

    # std_msgs types
    echo "export type ColorRGBA = {"
    parse_ts_fields "$STD_MSGS/std_msgs/ColorRGBA.msg"
    echo "};"
    echo ""

    # nav_msgs types
    echo "export type Path = {"
    echo "  poses?: PoseStamped[];"
    echo "};"
    echo ""

    echo "export type OccupancyGridInfo = {"
    echo "  resolution?: number;"
    echo "  width?: number;"
    echo "  height?: number;"
    echo "  origin?: Pose;"
    echo "};"
    echo ""

    echo "export type OccupancyGrid = {"
    echo "  info?: OccupancyGridInfo;"
    echo "  data?: number[];"
    echo "};"
    echo ""

    # sensor_msgs types
    for file in "$STD_MSGS/sensor_msgs"/*.msg; do
        [ -f "$file" ] || continue
        name="$(basename "$file" .msg)"
        echo "export type ${name} = {"
        parse_ts_fields "$file"
        echo "};"
        echo ""
    done

    # visualization_msgs types
    for file in "$STD_MSGS/visualization_msgs"/*.msg; do
        [ -f "$file" ] || continue
        name="$(basename "$file" .msg)"
        echo "export type ${name} = {"
        parse_ts_fields "$file"
        echo "};"
        echo ""
    done

    # mowgli_interfaces types
    for file in "$MOWGLI_MSG_DIR"/*.msg; do
        [ -f "$file" ] || continue
        name="$(basename "$file" .msg)"

        # Emit constants as const enum if present
        constants="$(parse_ts_constants "$file")"
        if [ -n "$constants" ]; then
            echo "export const enum ${name}Constants {"
            echo "$constants"
            echo "};"
            echo ""
        fi

        echo "export type ${name} = {"
        parse_ts_fields "$file"
        echo "};"
        echo ""
    done

    # Additional hand-maintained types not from .msg files
    echo "// --- Additional types not from .msg files ---"
    echo ""
    echo "export type Joy = {"
    echo "  axes?: number[];"
    echo "  buttons?: number[];"
    echo "};"
    echo ""
    echo "export type Map = {"
    echo "  map_width?: number;"
    echo "  map_height?: number;"
    echo "  map_center_x?: number;"
    echo "  map_center_y?: number;"
    echo "  navigation_areas?: MapArea[];"
    echo "  working_area?: MapArea[];"
    echo "  dock_x?: number;"
    echo "  dock_y?: number;"
    echo "  dock_heading?: number;"
    echo "};"
    echo ""
    echo "export type DockingSensor = {"
    echo "  dock_present?: boolean;"
    echo "  dock_distance?: number;"
    echo "};"

} > "$OUT_FILE"

echo "Generated $OUT_FILE"
