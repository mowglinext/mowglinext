# AGENTS.md

Repository-local working rules for code agents.

## Where the repository index lives

These files are the maintained map of the tree — read the relevant one before changing code, instead of grepping blind. They apply to every agent, not only Claude.

- [`CLAUDE.md`](CLAUDE.md) — safety rules, monorepo layout, architecture invariants, "what NOT to do".
- Per-tree notes: [`ros2/CLAUDE.md`](ros2/CLAUDE.md), [`gui/CLAUDE.md`](gui/CLAUDE.md), [`firmware/CLAUDE.md`](firmware/CLAUDE.md), [`install/CLAUDE.md`](install/CLAUDE.md), [`docker/CLAUDE.md`](docker/CLAUDE.md), [`sensors/CLAUDE.md`](sensors/CLAUDE.md).
- [`docs/claude/doc-index.md`](docs/claude/doc-index.md) — which document is authoritative and which is a historical record.
- [`docs/claude/codemaps/`](docs/claude/codemaps/) (per-package file maps), [`ros-interfaces.md`](docs/claude/ros-interfaces.md) (every topic/service/action/TF), [`parameters.md`](docs/claude/parameters.md) (every config key and its default), [`testing-ci.md`](docs/claude/testing-ci.md) (every test and the CI job that gates it).

## ROS2 formatting

- Any change under `ros2/` that touches C++ source or headers (`.cpp`, `.hpp`, `.h`) must be `clang-format` clean before finishing.
- Use **clang-format 18** and the repository style file. CI installs `clang-format-18`, `ros2/scripts/format.sh` expects major 18 (it only *warns* on a mismatch) and `ros2/.pre-commit-config.yaml` pins `v18.1.8` — any other major reformats lines CI never asked about, and the opt-in `.githooks/pre-push` hook will amend them into a `chore: auto-format C++ files` commit.

```bash
clang-format -i -style=file:ros2/.clang-format <touched-files>
```

- Before concluding work on `ros2/` files, verify formatting explicitly. Preferred check — this mirrors the CI job `Formatting (clang-format)`, which gates only the **changed lines** against the merge-base with `origin/main` (yes, `main`, even though PRs target `dev`):

```bash
git clang-format --style=file:ros2/.clang-format \
  --diff "$(git merge-base origin/main HEAD)" -- <touched-files>
```

- If `origin/main` is unavailable locally, at minimum run:

```bash
clang-format -n -style=file:ros2/.clang-format <touched-files>
```

## Non-root builds and tools

- Never build as `root`.
- Never run `pio`, `platformio`, `colcon`, `cmake`, `make`, `ninja`, `go`, `yarn`, `npm`, formatters, or generated-file workflows as `root`.
- Always use the normal project user so the repository does not accumulate root-owned files.
- If a previous run created root-owned artifacts, fix ownership or remove those artifacts before continuing.
