#!/bin/bash
# =============================================================================
# check_vendor_overlay.sh
#
# Fails when the dynamic loader resolves a library that /opt/lyrical_vendor
# overrides to the stock copy in /opt/ros/lyrical/lib instead (issue #1121).
#
# The overlay carries the patched nav2_costmap_2d of #1058 (selective clear also
# resets the filter layers). Packages are built against it, but which copy is
# LOADED is decided at runtime by LD_LIBRARY_PATH order, which no compile step
# checks. Run it in the environment the stack runs in, i.e. through the
# entrypoint:   /ros2_entrypoint.sh /ros2_ws/scripts/check_vendor_overlay.sh
#
# Arguments: binaries / shared objects to inspect (defaults below).
# =============================================================================
set -euo pipefail

readonly VENDOR_LIB=/opt/lyrical_vendor/lib
readonly STOCK_LIB=/opt/ros/lyrical/lib
# The overlay library whose absence would mean #1058's fix silently vanished.
readonly REQUIRED_OVERRIDE=libnav2_costmap_2d_core.so

readonly DEFAULT_CONSUMERS=(
    "${STOCK_LIB}/nav2_controller/controller_server"
    "${STOCK_LIB}/nav2_planner/planner_server"
    /ros2_ws/install/mowgli_nav2_plugins/lib/libmowgli_nav2_plugins.so
)

shadowed=()
for vendor_so in "${VENDOR_LIB}"/*.so; do
    name="$(basename "${vendor_so}")"
    if [ -e "${STOCK_LIB}/${name}" ]; then
        shadowed+=("${name}")
    fi
done

if ! printf '%s\n' "${shadowed[@]}" | grep -qx "${REQUIRED_OVERRIDE}"; then
    echo "check_vendor_overlay: ${REQUIRED_OVERRIDE} is not overridden by ${VENDOR_LIB}" >&2
    exit 1
fi

consumers=("$@")
if [ "${#consumers[@]}" -eq 0 ]; then
    consumers=("${DEFAULT_CONSUMERS[@]}")
fi

failures=0
for consumer in "${consumers[@]}"; do
    if [ ! -e "${consumer}" ]; then
        echo "check_vendor_overlay: missing consumer ${consumer}" >&2
        failures=$((failures + 1))
        continue
    fi
    resolved="$(ldd "${consumer}")"
    for name in "${shadowed[@]}"; do
        # "\tlibfoo.so => /path/libfoo.so (0x...)"; absent when not a dependency.
        path="$(awk -v lib="${name}" '$1 == lib { print $3 }' <<<"${resolved}")"
        if [ -n "${path}" ] && [ "${path#"${VENDOR_LIB}"/}" = "${path}" ]; then
            echo "check_vendor_overlay: ${consumer} loads ${name} from ${path}, expected ${VENDOR_LIB}" >&2
            failures=$((failures + 1))
        fi
    done
done

if [ "${failures}" -ne 0 ]; then
    echo "check_vendor_overlay: LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}" >&2
    exit 1
fi
echo "check_vendor_overlay: ${#shadowed[@]} overridden libraries resolve to ${VENDOR_LIB}"
