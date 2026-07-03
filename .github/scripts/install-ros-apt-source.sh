#!/usr/bin/env bash

set -euo pipefail

print_usage() {
  cat <<'EOF'
Usage: install-ros-apt-source.sh [--print-url] [PACKAGE_NAME] [UBUNTU_CODENAME]

PACKAGE_NAME defaults to ros2-apt-source.
UBUNTU_CODENAME defaults to the current host codename.

The script resolves the exact release asset from the GitHub release metadata
instead of synthesizing the filename locally. That keeps CI working if the
asset naming convention changes.
EOF
}

print_url_only=0

if [[ "${1:-}" == "--help" ]]; then
  print_usage
  exit 0
fi

if [[ "${1:-}" == "--print-url" ]]; then
  print_url_only=1
  shift
fi

package_name="${1:-ros2-apt-source}"
ubuntu_codename="${2:-}"

release_api_url="https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest"
tmp_deb="/tmp/${package_name}.deb"
expected_repo_url="http://packages.ros.org/${package_name%-apt-source}/ubuntu"

if [[ -z "${ubuntu_codename}" ]]; then
  ubuntu_codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
fi

if [[ "${print_url_only}" -eq 0 ]]; then
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends ca-certificates curl jq lsb-release
else
  command -v curl >/dev/null 2>&1 || {
    echo "curl is required for --print-url mode" >&2
    exit 1
  }
  command -v jq >/dev/null 2>&1 || {
    echo "jq is required for --print-url mode" >&2
    exit 1
  }
fi

if [[ "${print_url_only}" -eq 0 ]] && apt-cache policy | grep -Fq "${expected_repo_url}"; then
  echo "ROS apt source already configured for ${expected_repo_url}"
  exit 0
fi

release_json="$(curl -fsSL "${release_api_url}")"
asset_url="$(
  jq -r \
    --arg package_name "${package_name}" \
    --arg ubuntu_codename "${ubuntu_codename}" \
    '
      .assets[]
      | select(.name | startswith($package_name + "_"))
      | select(.name | endswith($ubuntu_codename + "_all.deb"))
      | .browser_download_url
    ' <<<"${release_json}" \
    | head -n 1
)"

if [[ -z "${asset_url}" || "${asset_url}" == "null" ]]; then
  echo "Unable to find a ${package_name} asset for Ubuntu ${ubuntu_codename} in ${release_api_url}" >&2
  jq -r '.assets[].name' <<<"${release_json}" >&2
  exit 1
fi

if [[ "${print_url_only}" -eq 1 ]]; then
  echo "${asset_url}"
  exit 0
fi

echo "Installing ${package_name} from ${asset_url}"
curl --fail --location --retry 3 --retry-delay 2 -o "${tmp_deb}" "${asset_url}"
sudo dpkg -i "${tmp_deb}"
rm -f "${tmp_deb}"
sudo apt-get update
