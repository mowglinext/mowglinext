#!/usr/bin/env bash
# Fails when a submodule gitlink differs from .github/submodule-pins.txt, or
# when a submodule has no recorded pin. Needs no submodule checkout, and no
# bash 4 features (macOS ships 3.2).
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
pins_file="${1:-$repo_root/.github/submodule-pins.txt}"
pins_name="${pins_file#"$repo_root"/}"
status=0

recorded_for() {
  awk -v want="$1" '$1 == want && $1 !~ /^#/ {print $2; exit}' "$pins_file"
}

gitlinks="$(git -C "$repo_root" ls-tree -r HEAD | awk '$2 == "commit" {print $3, $4}')"

while read -r commit path; do
  [[ -n "${path:-}" ]] || continue
  expected="$(recorded_for "$path")"
  if [[ -z "$expected" ]]; then
    echo "::error::submodule $path has no entry in $pins_name"
    status=1
  elif [[ "$expected" != "$commit" ]]; then
    echo "::error::submodule $path points at $commit but $pins_name records $expected." \
      "A deliberate re-pin must update that file in the same commit. Otherwise a stale branch" \
      "rolled the gitlink back; restore it with:" \
      "git update-index --cacheinfo 160000,$expected,$path"
    status=1
  else
    echo "ok  $path @ ${commit:0:12}"
  fi
done <<<"$gitlinks"

while read -r path _; do
  [[ -z "${path:-}" || "$path" == \#* ]] && continue
  if ! grep -q " $path\$" <<<"$gitlinks"; then
    echo "::error::$pins_name lists $path, which is not a submodule in this tree"
    status=1
  fi
done < "$pins_file"

exit "$status"
