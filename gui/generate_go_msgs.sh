#!/bin/bash
#
# generate_go_msgs.sh — Generate Go structs from ROS2 .msg and .srv files.
#
# The GUI backend communicates with ROS2 via rosbridge (JSON over WebSocket),
# so these structs only need JSON tags — no ROS serialization.
#
# Generates all packages under gui/pkg/msgs/:
#   - geometry   (geometry_msgs, std_msgs/Header, builtin_interfaces/Time)
#   - nav        (nav_msgs)
#   - sensor     (sensor_msgs)
#   - std        (std_msgs)
#   - visualization (visualization_msgs)
#   - mowgli     (mowgli_interfaces)
#
# Usage:
#   ./generate_go_msgs.sh
#
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
MOWGLI_MSG_DIR="$SCRIPT_DIR/../ros2/src/mowgli_interfaces/msg"
MOWGLI_SRV_DIR="$SCRIPT_DIR/../ros2/src/mowgli_interfaces/srv"
MSGS_DIR="$SCRIPT_DIR/pkg/msgs"
MODULE="github.com/mowglinext/mowglinext/pkg/msgs"

# Temporary directory for embedded standard ROS2 msg definitions
STD_MSGS=$(mktemp -d)
trap 'rm -rf "$STD_MSGS"' EXIT

# ── Embed standard ROS2 message definitions ──────────────────────────────────
# These are stable definitions from ros2/common_interfaces.
# We embed them here so the script works without a ROS2 installation.

mkdir -p "$STD_MSGS/builtin_interfaces"
cat > "$STD_MSGS/builtin_interfaces/Time.msg" <<'MSG'
uint32 sec
uint32 nanosec
MSG
cat > "$STD_MSGS/builtin_interfaces/Duration.msg" <<'MSG'
int32 sec
uint32 nanosec
MSG

mkdir -p "$STD_MSGS/std_msgs"
cat > "$STD_MSGS/std_msgs/Header.msg" <<'MSG'
builtin_interfaces/Time stamp
string frame_id
MSG
cat > "$STD_MSGS/std_msgs/String.msg" <<'MSG'
string data
MSG
cat > "$STD_MSGS/std_msgs/Bool.msg" <<'MSG'
bool data
MSG
cat > "$STD_MSGS/std_msgs/ColorRGBA.msg" <<'MSG'
float32 r
float32 g
float32 b
float32 a
MSG

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
std_msgs/Header header
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
std_msgs/Header header
geometry_msgs/PoseStamped[] poses
MSG
cat > "$STD_MSGS/nav_msgs/Odometry.msg" <<'MSG'
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
MSG
cat > "$STD_MSGS/nav_msgs/OccupancyGrid.msg" <<'MSG'
std_msgs/Header header
MapMetaData info
int8[] data
MSG
cat > "$STD_MSGS/nav_msgs/MapMetaData.msg" <<'MSG'
builtin_interfaces/Time map_load_time
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin
MSG

mkdir -p "$STD_MSGS/sensor_msgs"
cat > "$STD_MSGS/sensor_msgs/Imu.msg" <<'MSG'
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
MSG
cat > "$STD_MSGS/sensor_msgs/NavSatFix.msg" <<'MSG'
std_msgs/Header header
NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
MSG
cat > "$STD_MSGS/sensor_msgs/NavSatStatus.msg" <<'MSG'
int8 status
uint16 service
MSG
cat > "$STD_MSGS/sensor_msgs/LaserScan.msg" <<'MSG'
std_msgs/Header header
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

mkdir -p "$STD_MSGS/visualization_msgs"
cat > "$STD_MSGS/visualization_msgs/Marker.msg" <<'MSG'
std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
builtin_interfaces/Duration lifetime
geometry_msgs/Point[] points
MSG
cat > "$STD_MSGS/visualization_msgs/MarkerArray.msg" <<'MSG'
Marker[] markers
MSG

# ── Helpers ──────────────────────────────────────────────────────────────────

# Convert a snake_case ROS field name to Go PascalCase
to_pascal() {
    local result=""
    IFS='_' read -ra parts <<< "$1"
    for part in "${parts[@]}"; do
        result="${result}$(echo "${part:0:1}" | tr '[:lower:]' '[:upper:]')${part:1}"
    done
    echo "$result"
}

# Map a ROS2 field type to a Go type.
# $1 = rostype, $2 = current package's ros name (e.g. "geometry_msgs")
ros_to_go() {
    local rostype="$1"
    local current_pkg="${2:-}"
    local is_array=false
    local fixed_size=""

    # Strip trailing [] or [N] for arrays
    if [[ "$rostype" =~ \[([0-9]*)\]$ ]]; then
        fixed_size="${BASH_REMATCH[1]}"
        rostype="${rostype%%\[*}"
        is_array=true
    fi

    local gotype
    local prefix=""

    case "$rostype" in
        # Primitives
        bool)       gotype="bool" ;;
        int8)       gotype="int8" ;;
        uint8)      gotype="uint8" ;;
        byte)       gotype="uint8" ;;
        int16)      gotype="int16" ;;
        uint16)     gotype="uint16" ;;
        int32)      gotype="int32" ;;
        uint32)     gotype="uint32" ;;
        int64)      gotype="int64" ;;
        uint64)     gotype="uint64" ;;
        float32)    gotype="float32" ;;
        float64)    gotype="float64" ;;
        string)     gotype="string" ;;

        # Fully qualified cross-package references
        builtin_interfaces/Time)            gotype="Stamp"; prefix="geometry." ;;
        builtin_interfaces/Duration)
            gotype="Duration"
            if [[ "$current_pkg" != "visualization_msgs" ]]; then
                prefix="visualization."
            fi ;;
        std_msgs/Header)
            if [[ "$current_pkg" == "geometry_msgs" ]]; then
                gotype="Header"
            else
                gotype="Header"; prefix="geometry."
            fi ;;
        std_msgs/ColorRGBA)
            if [[ "$current_pkg" == "visualization_msgs" ]]; then
                gotype="ColorRGBA"
            else
                gotype="ColorRGBA"; prefix="visualization."
            fi ;;
        std_msgs/String)                    gotype="String"; prefix="std." ;;
        std_msgs/Bool)                      gotype="Bool"; prefix="std." ;;

        geometry_msgs/*)
            gotype="${rostype#geometry_msgs/}"
            if [[ "$current_pkg" != "geometry_msgs" ]]; then
                prefix="geometry."
            fi ;;

        nav_msgs/*)
            gotype="${rostype#nav_msgs/}"
            if [[ "$current_pkg" != "nav_msgs" ]]; then
                prefix="nav."
            fi ;;

        sensor_msgs/*)
            gotype="${rostype#sensor_msgs/}"
            if [[ "$current_pkg" != "sensor_msgs" ]]; then
                prefix="sensor."
            fi ;;

        visualization_msgs/*)
            gotype="${rostype#visualization_msgs/}"
            if [[ "$current_pkg" != "visualization_msgs" ]]; then
                prefix="visualization."
            fi ;;

        mowgli_interfaces/*)
            gotype="${rostype#mowgli_interfaces/}"
            ;;

        # Bare type name — same package
        *)  gotype="$rostype" ;;
    esac

    if $is_array; then
        if [ -n "$fixed_size" ]; then
            echo "[${fixed_size}]${prefix}${gotype}"
        else
            echo "[]${prefix}${gotype}"
        fi
    else
        echo "${prefix}${gotype}"
    fi
}

# Parse a .msg file and emit Go struct fields.
# Skips constants (lines with =) and comments.
# $1 = file, $2 = ros package name
parse_fields() {
    local file="$1"
    local pkg="${2:-}"
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

        local gotype gofield jsontag
        gotype="$(ros_to_go "$rostype" "$pkg")"
        gofield="$(to_pascal "$field")"
        jsontag="$field"

        printf '\t%-25s %-30s `json:"%s"`\n' "$gofield" "$gotype" "$jsontag"
    done < "$file"
}

# Determine which imports a generated file needs
# $1 = go source content
compute_imports() {
    local content="$1"
    local imports=""
    if echo "$content" | grep -q 'geometry\.'; then
        imports="$imports\n\t\"${MODULE}/geometry\""
    fi
    if echo "$content" | grep -q 'nav\.'; then
        imports="$imports\n\t\"${MODULE}/nav\""
    fi
    if echo "$content" | grep -q 'sensor\.'; then
        imports="$imports\n\t\"${MODULE}/sensor\""
    fi
    if echo "$content" | grep -q 'std\.'; then
        imports="$imports\n\t\"${MODULE}/std\""
    fi
    if echo "$content" | grep -q 'visualization\.'; then
        imports="$imports\n\t\"${MODULE}/visualization\""
    fi
    echo -e "$imports"
}

# Generate a package from .msg files
# $1 = ros package name (e.g. "geometry_msgs")
# $2 = go package name (e.g. "geometry")
# $3 = msg directory
# $4 = output file
generate_package() {
    local ros_pkg="$1"
    local go_pkg="$2"
    local msg_dir="$3"
    local out_file="$4"

    mkdir -p "$(dirname "$out_file")"

    # First pass: generate struct bodies to detect imports
    local body=""
    for file in "$msg_dir"/*.msg; do
        [ -f "$file" ] || continue
        local name
        name="$(basename "$file" .msg)"
        local struct_body
        struct_body="$(parse_fields "$file" "$ros_pkg")"
        body="$body
// ${name} matches ${ros_pkg}/msg/${name}.
type ${name} struct {
${struct_body}
}
"
    done

    local imports
    imports="$(compute_imports "$body")"

    {
        echo "// Code generated by generate_go_msgs.sh — DO NOT EDIT."
        echo "package ${go_pkg}"
        if [ -n "$imports" ]; then
            echo ""
            echo "import ($imports"
            echo ")"
        fi
        echo "$body"
    } > "$out_file"

    echo "  $out_file"
}

# ── Main ─────────────────────────────────────────────────────────────────────

echo "Generating Go types from ROS2 message definitions..."

# Standard ROS2 packages (order matters — geometry first since others depend on it)
# Note: builtin_interfaces types (Time, Duration) are folded into geometry and visualization

# geometry package: geometry_msgs + std_msgs/Header + builtin_interfaces/Time
# We generate geometry_msgs first, then manually prepend Header and Stamp
GEOM_BODY=""
for file in "$STD_MSGS/geometry_msgs"/*.msg; do
    name="$(basename "$file" .msg)"
    fields="$(parse_fields "$file" "geometry_msgs")"
    GEOM_BODY="$GEOM_BODY
// ${name} matches geometry_msgs/msg/${name}.
type ${name} struct {
${fields}
}
"
done

{
    echo "// Code generated by generate_go_msgs.sh — DO NOT EDIT."
    echo "package geometry"
    echo ""
    echo "// Stamp matches builtin_interfaces/msg/Time."
    echo "type Stamp struct {"
    echo '	Sec                       uint32                         `json:"sec"`'
    echo '	Nanosec                   uint32                         `json:"nanosec"`'
    echo "}"
    echo ""
    echo "// Header matches std_msgs/msg/Header."
    echo "type Header struct {"
    echo '	Stamp                     Stamp                          `json:"stamp"`'
    echo '	FrameId                   string                         `json:"frame_id"`'
    echo "}"
    echo "$GEOM_BODY"
} > "$MSGS_DIR/geometry/types_generated.go"
echo "  $MSGS_DIR/geometry/types_generated.go"

# nav package
generate_package "nav_msgs" "nav" "$STD_MSGS/nav_msgs" "$MSGS_DIR/nav/types_generated.go"

# sensor package
generate_package "sensor_msgs" "sensor" "$STD_MSGS/sensor_msgs" "$MSGS_DIR/sensor/types_generated.go"

# std package
generate_package "std_msgs" "std" "$STD_MSGS/std_msgs" "$MSGS_DIR/std/types_generated.go"

# visualization package
VIS_BODY=""
# Duration (from builtin_interfaces) goes here since only visualization uses it
VIS_BODY="
// Duration matches builtin_interfaces/msg/Duration.
type Duration struct {
$(printf '\t%-25s %-30s `json:"%s"`\n' "Sec" "int32" "sec")
$(printf '\t%-25s %-30s `json:"%s"`\n' "Nanosec" "uint32" "nanosec")
}

// ColorRGBA matches std_msgs/msg/ColorRGBA.
type ColorRGBA struct {
$(printf '\t%-25s %-30s `json:"%s"`\n' "R" "float32" "r")
$(printf '\t%-25s %-30s `json:"%s"`\n' "G" "float32" "g")
$(printf '\t%-25s %-30s `json:"%s"`\n' "B" "float32" "b")
$(printf '\t%-25s %-30s `json:"%s"`\n' "A" "float32" "a")
}
"
for file in "$STD_MSGS/visualization_msgs"/*.msg; do
    name="$(basename "$file" .msg)"
    fields="$(parse_fields "$file" "visualization_msgs")"
    VIS_BODY="$VIS_BODY
// ${name} matches visualization_msgs/msg/${name}.
type ${name} struct {
${fields}
}
"
done

{
    echo "// Code generated by generate_go_msgs.sh — DO NOT EDIT."
    echo "package visualization"
    echo ""
    echo "import ("
    echo "	\"${MODULE}/geometry\""
    echo ")"
    echo "$VIS_BODY"
} > "$MSGS_DIR/visualization/types_generated.go"
echo "  $MSGS_DIR/visualization/types_generated.go"

# mowgli package — messages
generate_package "mowgli_interfaces" "mowgli" "$MOWGLI_MSG_DIR" "$MSGS_DIR/mowgli/types_generated.go"

# mowgli package — services
SRV_BODY=""
for file in "$MOWGLI_SRV_DIR"/*.srv; do
    name="$(basename "$file" .srv)"

    reqfile=$(mktemp)
    resfile=$(mktemp)
    in_response=false
    while IFS= read -r line; do
        if [[ "$line" == "---" ]]; then
            in_response=true
            continue
        fi
        if $in_response; then
            echo "$line" >> "$resfile"
        else
            echo "$line" >> "$reqfile"
        fi
    done < "$file"

    req_fields="$(parse_fields "$reqfile" "mowgli_interfaces")"
    if [ -n "$req_fields" ]; then
        SRV_BODY="$SRV_BODY
// ${name}Req for mowgli_interfaces/srv/${name} request.
type ${name}Req struct {
${req_fields}
}
"
    else
        SRV_BODY="$SRV_BODY
// ${name}Req for mowgli_interfaces/srv/${name} request (empty).
type ${name}Req struct{}
"
    fi

    res_fields="$(parse_fields "$resfile" "mowgli_interfaces")"
    if [ -n "$res_fields" ]; then
        SRV_BODY="$SRV_BODY
// ${name}Res for mowgli_interfaces/srv/${name} response.
type ${name}Res struct {
${res_fields}
}
"
    else
        SRV_BODY="$SRV_BODY
// ${name}Res for mowgli_interfaces/srv/${name} response (empty).
type ${name}Res struct{}
"
    fi

    rm -f "$reqfile" "$resfile"
done

SRV_IMPORTS="$(compute_imports "$SRV_BODY")"
{
    echo "// Code generated by generate_go_msgs.sh — DO NOT EDIT."
    echo "package mowgli"
    if [ -n "$SRV_IMPORTS" ]; then
        echo ""
        echo "import ($SRV_IMPORTS"
        echo ")"
    fi
    echo "$SRV_BODY"
} > "$MSGS_DIR/mowgli/services_generated.go"
echo "  $MSGS_DIR/mowgli/services_generated.go"

echo ""
echo "Done. Now remove the hand-written types.go files that are replaced by *_generated.go."
