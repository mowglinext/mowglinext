#!/usr/bin/env bash
# Format all C++ files in ros2/src/ using the project .clang-format config.
# Usage: ./ros2/scripts/format.sh [--check]
#   --check   Dry-run mode: exit 1 if any file needs formatting (CI mode)
#
# Requires clang-format 18.x. Install: pip install clang-format==18.1.8
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_DIR="$(dirname "$SCRIPT_DIR")"
STYLE="file:${ROS2_DIR}/.clang-format"
REQUIRED_MAJOR=18

if ! command -v clang-format &>/dev/null; then
  echo "ERROR: clang-format not found. Install: pip install clang-format==18.1.8"
  exit 1
fi

VERSION=$(clang-format --version | grep -oE '[0-9]+\.[0-9]+' | head -1)
MAJOR=${VERSION%%.*}
if [ "$MAJOR" != "$REQUIRED_MAJOR" ]; then
  echo "WARNING: clang-format $VERSION detected, CI uses 18.x."
  echo "         Install matching version: pip install clang-format==18.1.8"
fi

# external/ holds vendored submodules (universal-gnss). They carry their own
# clang-format policy and are not ours to reformat: doing so left the
# universal-gnss worktree permanently dirty (155 files on 2026-09-04, purely
# reflow + include sorting from a local clang-format 22), which then blocks
# re-pinning the submodule. CI only ever formats lines touched in THIS repo
# (git-clang-format --diff against the base), so excluding them here matches
# what CI checks rather than diverging from it.
FILES=$(find "${ROS2_DIR}/src/" \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
  -not -path "*/opennav_coverage/*" \
  -not -path "*/external/*")

if [ -z "$FILES" ]; then
  echo "No C++ files found."
  exit 0
fi

if [ "${1:-}" = "--check" ]; then
  echo "$FILES" | xargs clang-format --dry-run --Werror -style="$STYLE"
  echo "All files are correctly formatted."
else
  echo "$FILES" | xargs clang-format -i -style="$STYLE"
  echo "Formatted $(echo "$FILES" | wc -l | tr -d ' ') files."
fi
