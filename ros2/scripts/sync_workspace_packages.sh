#!/bin/bash
# =============================================================================
# sync_workspace_packages.sh — Symlink package roots into /ros2_ws/src
#
# Keeps colcon discovery limited to the intended package roots instead of
# letting nested package.xml files leak in from the monorepo or external repos.
#
# Usage:
#   ./scripts/sync_workspace_packages.sh
#   ./scripts/sync_workspace_packages.sh --print-base-paths
#
# Environment:
#   INCLUDE_OPENNAV_COVERAGE_STACK=1
#     Link every package from the upstream opennav_coverage submodule. By
#     default only opennav_coverage_msgs is linked because the server packages
#     are optional full-stack dependencies and require Fields2Cover.
#   UNIVERSAL_GNSS_PATH=/path/to/universal-gnss
#     Optional override for the Universal GNSS repo root. By default the
#     vendored ros2/src/external/universal-gnss submodule is used; the legacy
#     /workspaces/universal-gnss mount remains a fallback for local GNSS
#     development outside this monorepo.
#   The sidecar tools/motor package is also linked into the workspace as the
#   ROS 2 package mowgli_tools so operator utilities are available via
#   `ros2 run mowgli_tools ...`.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MONOREPO_ROOT="${MONOREPO_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-/ros2_ws}"
WORKSPACE_SRC="${WORKSPACE_ROOT}/src"
UNIVERSAL_GNSS_PATH="${UNIVERSAL_GNSS_PATH:-}"
VENDORED_UNIVERSAL_GNSS_PATH="${MONOREPO_ROOT}/ros2/src/external/universal-gnss"
LEGACY_MOUNTED_UNIVERSAL_GNSS_PATH="/workspaces/universal-gnss"
INCLUDE_OPENNAV_COVERAGE_STACK="${INCLUDE_OPENNAV_COVERAGE_STACK:-0}"
PRINT_BASE_PATHS="${1:-}"

if [ "${PRINT_BASE_PATHS}" != "" ] && [ "${PRINT_BASE_PATHS}" != "--print-base-paths" ]; then
    echo "Unsupported argument: ${PRINT_BASE_PATHS}" >&2
    exit 1
fi

quiet_mode=false
if [ "${PRINT_BASE_PATHS}" = "--print-base-paths" ]; then
    quiet_mode=true
fi

log() {
    if [ "${quiet_mode}" = false ]; then
        echo "$@"
    fi
}

warn() {
    echo "$@" >&2
}

find_universal_gnss_repo() {
    local candidate
    local -a candidates=()

    if [ -n "${UNIVERSAL_GNSS_PATH}" ]; then
        candidates+=("${UNIVERSAL_GNSS_PATH}")
    fi

    candidates+=(
        "${VENDORED_UNIVERSAL_GNSS_PATH}"
        "${LEGACY_MOUNTED_UNIVERSAL_GNSS_PATH}"
    )

    for candidate in "${candidates[@]}"; do
        [ -n "${candidate}" ] || continue

        if [ -f "${candidate}/gnss_ros2/package.xml" ]; then
            printf '%s\n' "${candidate}"
            return 0
        fi

        if [ -d "${candidate}" ]; then
            warn "Universal GNSS source candidate exists at ${candidate}, but gnss_ros2/package.xml was not found."
        fi
    done

    return 1
}

package_name_from_xml() {
    local package_xml="${1:?package_name_from_xml: missing package.xml path}"

    sed -n 's:.*<name>[[:space:]]*\([^<][^<]*\)[[:space:]]*</name>.*:\1:p' "$package_xml" | head -n 1
}

link_workspace_package() {
    local source_dir="${1:?link_workspace_package: missing source dir}"
    local link_name="${2:?link_workspace_package: missing link name}"
    local link_path="${WORKSPACE_SRC}/${link_name}"

    if [ -e "${link_path}" ] && [ ! -L "${link_path}" ]; then
        warn "Refusing to overwrite non-symlink workspace entry: ${link_path}"
        return 1
    fi

    ln -sfnT "${source_dir}" "${link_path}"
    BUILD_PATHS+=("${link_path}")
    log "  Linked: ${link_name}"
}

unlink_workspace_symlink() {
    local link_name="${1:?unlink_workspace_symlink: missing link name}"
    local link_path="${WORKSPACE_SRC}/${link_name}"

    if [ -L "${link_path}" ]; then
        rm -f "${link_path}"
        log "  Unlinked: ${link_name}"
    fi
}

mkdir -p "${WORKSPACE_SRC}"

BUILD_PATHS=()

log "Linking ROS2 packages into workspace..."

shopt -s nullglob

for pkg_dir in "${MONOREPO_ROOT}"/ros2/src/mowgli_*/; do
    [ -d "${pkg_dir}" ] || continue
    pkg_name="$(basename "${pkg_dir}")"
    link_workspace_package "${pkg_dir}" "${pkg_name}"
done

opennav_msgs_dir="${MONOREPO_ROOT}/ros2/src/opennav_coverage/opennav_coverage_msgs"
if [ -f "${opennav_msgs_dir}/package.xml" ]; then
    link_workspace_package "${opennav_msgs_dir}" "opennav_coverage_msgs"
fi

if [ "${INCLUDE_OPENNAV_COVERAGE_STACK}" = "1" ]; then
    for pkg_dir in "${MONOREPO_ROOT}"/ros2/src/opennav_coverage/*/; do
        if [ -f "${pkg_dir}/package.xml" ]; then
            pkg_name="$(basename "${pkg_dir}")"
            [ "${pkg_name}" = "opennav_coverage_msgs" ] && continue
            link_workspace_package "${pkg_dir}" "${pkg_name}"
        fi
    done
else
    for pkg_name in \
        opennav_coverage \
        opennav_coverage_bt \
        opennav_coverage_demo \
        opennav_coverage_navigator \
        opennav_row_coverage
    do
        unlink_workspace_symlink "${pkg_name}"
    done
    log "  Skipped: opennav_coverage full stack (set INCLUDE_OPENNAV_COVERAGE_STACK=1 to link it)"
fi

if [ -d "${MONOREPO_ROOT}/ros2/src/fusion_graph" ]; then
    link_workspace_package "${MONOREPO_ROOT}/ros2/src/fusion_graph" "fusion_graph"
fi

if [ -f "${MONOREPO_ROOT}/tools/motor/package.xml" ]; then
    link_workspace_package "${MONOREPO_ROOT}/tools/motor" "mowgli_tools"
fi

if universal_gnss_repo="$(find_universal_gnss_repo)"; then
    universal_gnss_ros2_dir="${universal_gnss_repo}/gnss_ros2"
    universal_gnss_ros2_xml="${universal_gnss_ros2_dir}/package.xml"
    universal_pkg_name="$(package_name_from_xml "${universal_gnss_ros2_xml}")"
    if [ "${universal_pkg_name}" != "universal_gnss_ros2" ]; then
        warn "Universal GNSS package name mismatch at ${universal_gnss_ros2_xml}: ${universal_pkg_name}"
        exit 1
    fi

    link_workspace_package "${universal_gnss_ros2_dir}" "${universal_pkg_name}"
else
    warn "Universal GNSS source not found. Checked vendored submodule at ${VENDORED_UNIVERSAL_GNSS_PATH} and fallback mount at ${LEGACY_MOUNTED_UNIVERSAL_GNSS_PATH}${UNIVERSAL_GNSS_PATH:+, plus UNIVERSAL_GNSS_PATH=${UNIVERSAL_GNSS_PATH}}."
fi

shopt -u nullglob

if [ "${quiet_mode}" = true ]; then
    printf '%s\n' "${BUILD_PATHS[@]}"
fi
