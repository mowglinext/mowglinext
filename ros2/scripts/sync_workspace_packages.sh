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
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MONOREPO_ROOT="${MONOREPO_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-/ros2_ws}"
WORKSPACE_SRC="${WORKSPACE_ROOT}/src"
UNIVERSAL_GNSS_PATH="${UNIVERSAL_GNSS_PATH:-/workspaces/universal-gnss}"
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

mkdir -p "${WORKSPACE_SRC}"

BUILD_PATHS=()

log "Linking ROS2 packages into workspace..."

shopt -s nullglob

for pkg_dir in "${MONOREPO_ROOT}"/ros2/src/mowgli_*/; do
    [ -d "${pkg_dir}" ] || continue
    pkg_name="$(basename "${pkg_dir}")"
    link_workspace_package "${pkg_dir}" "${pkg_name}"
done

for pkg_dir in "${MONOREPO_ROOT}"/ros2/src/opennav_coverage/*/; do
    if [ -f "${pkg_dir}/package.xml" ]; then
        pkg_name="$(basename "${pkg_dir}")"
        link_workspace_package "${pkg_dir}" "${pkg_name}"
    fi
done

if [ -d "${MONOREPO_ROOT}/ros2/src/fusion_graph" ]; then
    link_workspace_package "${MONOREPO_ROOT}/ros2/src/fusion_graph" "fusion_graph"
fi

universal_gnss_ros2_dir="${UNIVERSAL_GNSS_PATH}/gnss_ros2"
universal_gnss_ros2_xml="${universal_gnss_ros2_dir}/package.xml"

if [ -f "${universal_gnss_ros2_xml}" ]; then
    universal_pkg_name="$(package_name_from_xml "${universal_gnss_ros2_xml}")"
    if [ "${universal_pkg_name}" != "universal_gnss_ros2" ]; then
        warn "Universal GNSS package name mismatch at ${universal_gnss_ros2_xml}: ${universal_pkg_name}"
        exit 1
    fi

    link_workspace_package "${universal_gnss_ros2_dir}" "${universal_pkg_name}"
elif [ -d "${UNIVERSAL_GNSS_PATH}" ]; then
    warn "Universal GNSS source is mounted at ${UNIVERSAL_GNSS_PATH}, but gnss_ros2/package.xml was not found."
else
    warn "Universal GNSS source not mounted at ${UNIVERSAL_GNSS_PATH}."
fi

shopt -u nullglob

if [ "${quiet_mode}" = true ]; then
    printf '%s\n' "${BUILD_PATHS[@]}"
fi
