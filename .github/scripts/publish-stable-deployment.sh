#!/usr/bin/env bash
set -euo pipefail
: "${REPO:?GitHub repository required}" "${RELEASE:?Release tag required}"
[[ "$RELEASE" =~ ^v(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$ ]] || exit 1

# Firmware and software publish independent assets to the same tagged release.
# Either may finish first; tolerate the other publisher winning creation.
if ! gh release view "$RELEASE" --repo "$REPO" >/dev/null 2>&1; then
  if ! gh release create "$RELEASE" --repo "$REPO" --verify-tag --title "$RELEASE" --notes "MowgliNext $RELEASE"; then
    gh release view "$RELEASE" --repo "$REPO" >/dev/null
  fi
fi

# Make the descriptor discoverable only after its referenced files are present.
# Do not overwrite an existing publication or firmware-owned assets.
gh release upload "$RELEASE" mowgli-compose.json mowgli-updater-linux-* updater-SHA256SUMS --repo "$REPO"
gh release upload "$RELEASE" mowgli-deployment.json --repo "$REPO"
