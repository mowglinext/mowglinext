#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
sandbox="$(mktemp -d)"
trap 'rm -rf -- "$sandbox"' EXIT
export PATH="$sandbox:$PATH" REPO=example/mowgli RELEASE=v1.3.0
export PUBLICATION_SANDBOX="$sandbox"
cat > "$sandbox/gh" <<'SH'
#!/usr/bin/env bash
set -eu
printf '%s\n' "$*" >> "$PUBLICATION_SANDBOX/calls"
case "$2" in
  view) [[ -f "$PUBLICATION_SANDBOX/release" ]];;
  create)
    [[ "$SCENARIO" != failure ]] || exit 1
    touch "$PUBLICATION_SANDBOX/release"
    [[ "$SCENARIO" != race ]];;
  upload) [[ -f "$PUBLICATION_SANDBOX/release" ]];;
  *) exit 2;;
esac
SH
chmod +x "$sandbox/gh"
cd "$sandbox"
touch mowgli-compose.json mowgli-deployment.json mowgli-updater-linux-amd64 mowgli-updater-linux-arm64 updater-SHA256SUMS
for SCENARIO in existing missing race failure; do
  export SCENARIO
  rm -f -- "$sandbox/release" "$sandbox/calls"
  if [[ "$SCENARIO" == existing ]]; then touch "$sandbox/release"; fi
  if bash "$ROOT/.github/scripts/publish-stable-deployment.sh"; then
    [[ "$SCENARIO" != failure ]]
    [[ "$(tail -n 1 "$sandbox/calls")" == 'release upload v1.3.0 mowgli-deployment.json --repo example/mowgli' ]]
    [[ "$(grep -c 'release upload' "$sandbox/calls")" == 2 ]]
  else
    [[ "$SCENARIO" == failure ]]
    ! grep -q 'release upload' "$sandbox/calls"
  fi
done
echo 'Deployment publication ordering passed'
