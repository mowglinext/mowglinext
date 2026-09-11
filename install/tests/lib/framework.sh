#!/usr/bin/env bash
# =============================================================================
# Test framework — assertions, sandboxing, summary output
#
# Sourced by every test_*.sh file under install/tests/. Provides the same
# helper API style as the pre-existing install/test_mowglinext.sh and
# docs/test_install.sh (hand-rolled bash assertions — no bats/pytest dep),
# extended with sandbox helpers for tests that drive the installer
# end-to-end against a temporary repo + temporary HOME.
# =============================================================================

# Counters scoped to the current test file
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_LABELS=()

# ANSI colours (no-op if NO_COLOR is set or not a TTY)
if [ -n "${NO_COLOR:-}" ] || [ ! -t 1 ]; then
  C_RED=""; C_GREEN=""; C_YELLOW=""; C_DIM=""; C_BOLD=""; C_RESET=""
else
  C_RED="\033[0;31m"; C_GREEN="\033[0;32m"; C_YELLOW="\033[1;33m"
  C_DIM="\033[2m"; C_BOLD="\033[1m"; C_RESET="\033[0m"
fi

# NOTE: pass()/fail() are ALSO defined by install/lib/common.sh (the
# installer's status helpers — green "OK", red "FAIL"). When a test
# sources the harness it indirectly re-imports common.sh, which would
# overwrite our counters. Define them via a function that is called
# initially AND can be re-applied via reapply_test_assertions() after
# harness_init().
_define_test_assertions() {
  pass() {
    TESTS_PASSED=$((TESTS_PASSED + 1))
    TESTS_RUN=$((TESTS_RUN + 1))
    echo -e "  ${C_GREEN}PASS${C_RESET}  $1"
  }

  fail() {
    TESTS_FAILED=$((TESTS_FAILED + 1))
    TESTS_RUN=$((TESTS_RUN + 1))
    FAILED_LABELS+=("$1")
    echo -e "  ${C_RED}FAIL${C_RESET}  $1"
    if [ -n "${2:-}" ]; then
      echo "        $2"
    fi
  }
}

reapply_test_assertions() {
  _define_test_assertions
}

_define_test_assertions

section() {
  echo ""
  echo -e "${C_BOLD}── $* ──${C_RESET}"
}

assert_eq() {
  local label="$1" expected="$2" actual="$3"
  if [ "$expected" = "$actual" ]; then
    pass "$label"
  else
    fail "$label" "expected='$expected' got='$actual'"
  fi
}

assert_neq() {
  local label="$1" forbidden="$2" actual="$3"
  if [ "$forbidden" != "$actual" ]; then
    pass "$label"
  else
    fail "$label" "value should differ from '$forbidden'"
  fi
}

assert_contains() {
  local label="$1" needle="$2" haystack="$3"
  if printf '%s' "$haystack" | grep -qF -- "$needle"; then
    pass "$label"
  else
    fail "$label" "expected to contain '$needle'"
  fi
}

assert_not_contains() {
  local label="$1" needle="$2" haystack="$3"
  if ! printf '%s' "$haystack" | grep -qF -- "$needle"; then
    pass "$label"
  else
    fail "$label" "expected NOT to contain '$needle'"
  fi
}

assert_match() {
  local label="$1" pattern="$2" haystack="$3"
  if printf '%s' "$haystack" | grep -qE -- "$pattern"; then
    pass "$label"
  else
    fail "$label" "expected to match regex '$pattern'"
  fi
}

assert_file_exists() {
  local label="$1" file="$2"
  if [ -f "$file" ]; then
    pass "$label"
  else
    fail "$label" "file not found: $file"
  fi
}

assert_file_not_exists() {
  local label="$1" file="$2"
  if [ ! -e "$file" ]; then
    pass "$label"
  else
    fail "$label" "file should not exist: $file"
  fi
}

assert_dir_exists() {
  local label="$1" dir="$2"
  if [ -d "$dir" ]; then
    pass "$label"
  else
    fail "$label" "directory not found: $dir"
  fi
}

assert_exit_nonzero() {
  local label="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    fail "$label" "command unexpectedly succeeded: $*"
  else
    pass "$label"
  fi
}

assert_exit_zero() {
  local label="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    pass "$label"
  else
    fail "$label" "command unexpectedly failed: $*"
  fi
}

# =============================================================================
# Repo paths — REPO_ROOT is the actual MowgliNext checkout we're testing.
# =============================================================================

# install/tests/lib/framework.sh -> repo root is three levels up.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
INSTALL_ROOT="$REPO_ROOT/install"
INSTALLER_MAIN="$INSTALL_ROOT/mowglinext.sh"
BOOTSTRAP_SH="$REPO_ROOT/docs/install.sh"

# =============================================================================
# Sandboxing — every test gets an isolated SANDBOX dir under TMPDIR.
# Cleaned up automatically on EXIT.
# =============================================================================

setup_sandbox() {
  SANDBOX="$(mktemp -d -t mowgli-install-test.XXXXXX)"
  trap 'cleanup_sandbox' EXIT
  mkdir -p "$SANDBOX/home" "$SANDBOX/bin"
  export ORIG_PATH="$PATH"
  export ORIG_HOME="$HOME"
  export HOME="$SANDBOX/home"
}

cleanup_sandbox() {
  if [ -n "${SANDBOX:-}" ] && [ -d "$SANDBOX" ]; then
    rm -rf "$SANDBOX"
  fi
  if [ -n "${ORIG_PATH:-}" ]; then
    export PATH="$ORIG_PATH"
  fi
  if [ -n "${ORIG_HOME:-}" ]; then
    export HOME="$ORIG_HOME"
  fi
}

# Copy the actual repo into a sandboxed location so the installer can run
# against a real (but throwaway) tree. We only copy the directories the
# installer touches. The bin shims under lib/mocks.sh prevent any real
# git/docker network or system mutation.
sandbox_repo() {
  local target="$1"
  mkdir -p "$target"
  cp -R "$REPO_ROOT/install" "$target/"
  cp -R "$REPO_ROOT/docker" "$target/"
  # config.sh sources the externally pinned MAVROS image contract from this
  # integration directory. Keep the sandbox source set as narrow as the
  # installer itself: it needs this contract, not any sensor implementation.
  mkdir -p "$target/sensors"
  cp -R "$REPO_ROOT/sensors/mavros" "$target/sensors/"
  cp -R "$REPO_ROOT/docs" "$target/"
  if [ -f "$REPO_ROOT/CLAUDE.md" ]; then
    cp "$REPO_ROOT/CLAUDE.md" "$target/"
  fi
  # Initialise as a git repo so any `git -C ... rev-parse` succeeds even
  # though our mocked git intercepts fetch/clone/reset.
  (
    cd "$target"
    git init -q -b main >/dev/null 2>&1 || git init -q >/dev/null 2>&1
    git -c user.email=test@example.com -c user.name=test add -A >/dev/null 2>&1 || true
    git -c user.email=test@example.com -c user.name=test \
      commit -q -m "test sandbox" >/dev/null 2>&1 || true
  )
}

test_summary() {
  echo ""
  echo "════════════════════════════════════════════"
  if [ "$TESTS_FAILED" -eq 0 ]; then
    echo -e "  ${C_GREEN}${C_BOLD}ALL ${TESTS_RUN} TESTS PASSED${C_RESET}"
  else
    echo -e "  ${C_RED}${C_BOLD}${TESTS_FAILED} of ${TESTS_RUN} TESTS FAILED${C_RESET}"
    for l in "${FAILED_LABELS[@]}"; do
      echo -e "    ${C_RED}-${C_RESET} $l"
    done
  fi
  echo "════════════════════════════════════════════"
  [ "$TESTS_FAILED" -eq 0 ]
}
