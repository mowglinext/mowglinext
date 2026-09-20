#!/usr/bin/env bash
# =============================================================================
# Compose baseline + legacy adoption — field report 2026-09-20.
#
# Re-running the installer on an untouched pre-updater install died with
# "legacy Compose differs in mowgli.environment": the host updater compared the
# installed docker-compose.yaml against the NEW target, so the release's own
# fragment changes (#625 added GNSS_STACK to the ROS2 container) looked like a
# hand edit. Two halves are pinned here:
#
#   1. The plain installer records the checksum of every file it generates
#      (docker/stack-definition.sha256, the updater's own baseline format), so
#      a later adoption compares against what the file was GENERATED from.
#   2. For a file with no baseline, `installer-stack` exits 3 and the installer
#      asks the operator instead of dying; nothing is adopted without consent.
#
# The updater itself is covered by gui/pkg/updater/installer_stack_test.go; a
# stub binary stands in for it here.
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

setup_sandbox
install_all_mocks

SANDBOX_REPO="$SANDBOX/repo"
sandbox_repo "$SANDBOX_REPO"
harness_init "$SANDBOX_REPO"
harness_set_preset gnss=auto gnss_connection=uart lidar=none tfluna=none

file_digest() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}

section "plain installer records the baseline of what it generated"

if harness_run; then pass "installer run"; else fail "installer run" "non-zero exit"; fi
BASELINE="$DOCKER_DIR/stack-definition.sha256"
assert_file_exists "baseline recorded" "$BASELINE"
assert_eq "baseline is the updater's sha256:<hex> of the generated file" \
  "sha256:$(file_digest "$FINAL_COMPOSE_FILE")" "$(cat "$BASELINE")"

printf '\n# hand edit\n' >> "$FINAL_COMPOSE_FILE"
assert_neq "a hand edit no longer matches the baseline" \
  "sha256:$(file_digest "$FINAL_COMPOSE_FILE")" "$(cat "$BASELINE")"

if harness_run; then pass "installer rerun"; else fail "installer rerun" "non-zero exit"; fi
assert_eq "regeneration refreshes the baseline" \
  "sha256:$(file_digest "$FINAL_COMPOSE_FILE")" "$(cat "$BASELINE")"

section "updater-managed: a baseline-less file needs the operator's consent"

STUB="$SANDBOX/mowgli-updater-stub"
STUB_LOG="$SANDBOX/stub.log"
cat > "$STUB" <<EOF
#!/usr/bin/env bash
printf '%s adopt=%s\n' "\$1" "\${MOWGLI_ADOPT_LEGACY_COMPOSE:-unset}" >> "$STUB_LOG"
if [[ -n "\${STUB_FORCE_EXIT:-}" ]]; then exit "\$STUB_FORCE_EXIT"; fi
if [[ "\${MOWGLI_ADOPT_LEGACY_COMPOSE:-}" == "true" ]]; then exit 0; fi
echo "legacy Compose has no recorded baseline and differs in mowgli.environment" >&2
exit 3
EOF
chmod +x "$STUB"
export MOWGLI_UPDATER_STACK_BINARY="$STUB"
touch "$DOCKER_DIR/.updater-managed"
build_compose_stack >/dev/null 2>&1
compose_before="$(cat "$FINAL_COMPOSE_FILE")"

run_managed() {
  : > "$STUB_LOG"
  write_compose_merged >"$SANDBOX/managed.out" 2>&1
}

confirm() { return 1; }
if run_managed; then
  fail "declined adoption stops the installer" "exit 0"
else
  pass "declined adoption stops the installer"
fi
assert_eq "declined: updater asked once, without consent" \
  "installer-stack adopt=false" "$(cat "$STUB_LOG")"
assert_contains "declined: the operator is told why" "no checksum of it was recorded" "$(cat "$SANDBOX/managed.out")"
assert_contains "declined: the operator is told where edits belong" "stack-overrides.yaml" "$(cat "$SANDBOX/managed.out")"
assert_eq "declined: installed file untouched" "$compose_before" "$(cat "$FINAL_COMPOSE_FILE")"

confirm() { return 0; }
if run_managed; then
  pass "confirmed adoption continues"
else
  fail "confirmed adoption continues" "$(cat "$SANDBOX/managed.out")"
fi
assert_eq "confirmed: second call carries the consent" \
  "installer-stack adopt=false
installer-stack adopt=true" "$(cat "$STUB_LOG")"

section "non-interactive consent and unrelated failures"

confirm() { echo "UNEXPECTED_PROMPT"; return 0; }
if MOWGLI_ADOPT_LEGACY_COMPOSE=true run_managed; then
  pass "MOWGLI_ADOPT_LEGACY_COMPOSE=true adopts without a prompt"
else
  fail "MOWGLI_ADOPT_LEGACY_COMPOSE=true adopts without a prompt" "$(cat "$SANDBOX/managed.out")"
fi
assert_eq "env consent: a single consenting call" "installer-stack adopt=true" "$(cat "$STUB_LOG")"
assert_not_contains "env consent: no prompt" "UNEXPECTED_PROMPT" "$(cat "$SANDBOX/managed.out")"

# Every other updater refusal (hand-edited generated file, recovery pending,
# invalid bundle, ...) must stay a hard stop: consent is never offered.
if STUB_FORCE_EXIT=1 run_managed; then
  fail "other updater failures stay fatal" "exit 0"
else
  pass "other updater failures stay fatal"
fi
assert_eq "other failures: no retry with consent" "installer-stack adopt=false" "$(cat "$STUB_LOG")"
assert_not_contains "other failures: no prompt" "UNEXPECTED_PROMPT" "$(cat "$SANDBOX/managed.out")"

test_summary
