# Contributing

## Quick Summary

1. Fork the repo
2. Create a feature branch off `dev`: `git checkout -b feat/my-feature`
3. Make changes, following conventional commits
4. Open a PR **against `dev`** (`gh pr create --base dev`) — `dev` is where feature work lands, `main` is the release branch
5. Address CI + maintainer feedback

Branch prefixes matter: use `feat/`, `fix/`, `refactor/`, `chore/` or `perf/`. The CI push
triggers listen for exactly those names, so a branch called anything else silently skips
some checks.

## Development Environment

### Option A: GitHub Codespaces (recommended)

Click **Code → Codespaces → Create codespace** on the [repo page](https://github.com/mowglinext/mowglinext) (pick `dev`). You get a full ROS2 Kilted environment with Nav2, GTSAM, and all dev tools — ready in minutes, no local setup.

The Codespace image does **not** ship Webots or Fields2Cover 3.0.0, so the simulator and `mowgli_coverage` cannot be run or built there. Use Option C for simulation.

### Option B: VS Code DevContainer (local)

1. Install [Docker](https://docs.docker.com/get-docker/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
2. Clone and open the repo — VS Code will prompt to reopen in container
3. The devcontainer includes Claude Code CLI, GitHub CLI, and all build tools

### Option C: Docker only

```bash
cd docker
docker compose -f docker-compose.simulation.yaml up dev-sim
```

See [Getting Started](Getting-Started#development-with-github-codespaces--devcontainer) for full details.

## What to Contribute

- Bug fixes
- New sensor drivers (`sensors/`)
- Behavior tree improvements (`ros2/src/mowgli_behavior/`)
- GUI features (`gui/`)
- Documentation (this wiki!)
- Test coverage
- E2E test improvements (`ros2/src/e2e_test.py`, `ros2/src/e2e_test_no_lidar.py`)

## Finding Your Way Around

The repo carries a generated index of itself — the quickest way to locate something before you change it:

- [`CLAUDE.md`](https://github.com/mowglinext/mowglinext/blob/main/CLAUDE.md) — safety rules, monorepo layout, architecture invariants. Each top-level component (`ros2/`, `gui/`, `firmware/`, `install/`, `docker/`, `sensors/`) has its own `CLAUDE.md` entry point.
- [`docs/claude/codemaps/`](https://github.com/mowglinext/mowglinext/tree/main/docs/claude/codemaps) — one map per package: files, runtime surface, known pitfalls.
- [`docs/claude/ros-interfaces.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/ros-interfaces.md) — every topic, service, action and TF frame, and the node that creates it.
- [`docs/claude/parameters.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/parameters.md) — every config key, its default and its consumers.
- [`docs/claude/testing-ci.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/testing-ci.md) — every test, the CI job that gates it, and a "before opening a PR" checklist.
- [`docs/claude/doc-index.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/doc-index.md) — which document is authoritative and which is history.

## Code Style

- **C++:** `.clang-format` in `ros2/` — CI pins **clang-format 18** (`pip install clang-format==18.1.8`); run `make -C ros2 format` before committing
- **Go:** `gofmt` (nothing in CI enforces it)
- **TypeScript:** `eslint` — `cd gui/web && yarn lint` (no Prettier config in the repo)
- **Python:** PEP 8 (`ruff`, via `ros2/.pre-commit-config.yaml`)
- **Commits:** `feat:`, `fix:`, `refactor:`, `docs:`, `test:`, `chore:`, `perf:`, `ci:`

## Testing

Before submitting a PR (the `make` targets run inside the devcontainer, where the repo is linked into `/ros2_ws`):

```bash
# Format, lint, build and test the ROS2 workspace
cd ros2
make format        # clang-format in-place
make lint          # cppcheck + cpplint
make build         # colcon build (Release)
make test          # colcon test + colcon test-result

# Run E2E test in simulation (optional but recommended)
cd ../docker
docker compose -f docker-compose.simulation.yaml up -d dev-sim
docker compose -f docker-compose.simulation.yaml exec dev-sim \
  bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
```

If you touched the GUI web app, also run `cd gui/web && npx tsc --noEmit && yarn lint && yarn test` —
all three run in CI. See [Simulation — E2E Test](Simulation#end-to-end-e2e-test) for details on what
the E2E test validates, and
[`docs/claude/testing-ci.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/testing-ci.md)
for the full per-area checklist (including the code-generation steps you owe after editing a `.msg`
or `.srv`, and the COBS protocol-version bump).

## What CI Checks

- **ROS2 — CI** (`ros2/**`, `tools/motor/**`, `install/config/mowgli/**`) — `colcon build` + `colcon test`, the `mowgli_robot.yaml` config-drift gate, `clang-format` on the lines you changed, and cppcheck. `Build & Test (ROS2 kilted)` is the required status check on `dev`.
- **GUI — CI** (`gui/web/**`) — vitest unit tests + `tsc`.
- **Firmware CI** (`firmware/**`) — PlatformIO builds for `Yardforce500` and `Yardforce500B`, plus the `board_defaults.h` single-source guard.
- **Msg Codegen Drift** — fails if a `mowgli_interfaces` `.msg`/`.srv` changed without re-running the Go, TypeScript and firmware generators.
- **Protocol Version Drift** — fails if the COBS wire layout changed without bumping `MOWGLI_PROTOCOL_VERSION` and its host mirror.

Docker image builds (`ros2/`, `gui/`, `sensors/`) run on pushes to `main`, `dev` and the branch
prefixes above, tagging the image with the branch name — pushing a feature branch is how you get an
image built for field testing.

## AI Assistance

There is no AI bot on the GitHub side: the `@claude` responder, the automatic PR review and the
weekly "AI proposes improvements" cron were all removed in July 2026. PRs are gated by CI and
reviewed by the maintainer.

AI assistance happens on *your* machine instead. If you use Claude Code, the repo config loads
automatically — `CLAUDE.md` (safety rules and architecture invariants), the per-component
`CLAUDE.md` files, `.claude/rules/ros2.md`, and the reference material under `docs/claude/` listed
in [Finding Your Way Around](#finding-your-way-around). See
[AI-Assisted Contributing](AI-Assisted-Contributing) for guidelines and the AI mistakes we keep
seeing in this codebase.
