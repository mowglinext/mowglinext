# Contributing

## Quick Summary

1. Fork the repo
2. Create a feature branch: `git checkout -b feat/my-feature`
3. Make changes, following conventional commits
4. Open a PR — Claude will auto-review
5. Address feedback from AI + maintainers

## Development Environment

### Option A: GitHub Codespaces (recommended)

Click **Code → Codespaces → Create codespace on main** on the [repo page](https://github.com/mowglinext/mowglinext). You get a full ROS2 Kilted environment with Nav2, Gazebo, and all dev tools — ready in minutes, no local setup.

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
- E2E test improvements (`ros2/src/e2e_test.py`)

## Code Style

- **C++:** `.clang-format` in `ros2/` (clang-format 18)
- **Go:** `gofmt`
- **TypeScript:** `prettier` + `eslint`
- **Commits:** `feat:`, `fix:`, `refactor:`, `docs:`, `test:`, `chore:`

## Testing

Before submitting a PR:

```bash
# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run unit tests
colcon test

# Run E2E test in simulation (optional but recommended)
cd docker
docker compose -f docker-compose.simulation.yaml up -d dev-sim
docker compose -f docker-compose.simulation.yaml exec dev-sim \
  bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
```

See [Simulation — E2E Test](Simulation#end-to-end-e2e-test) for details on what the test validates.

## AI Assistance

- **@claude** — mention in any issue or PR for AI help
- PRs are automatically reviewed by Claude
- Claude proposes improvements weekly as new issues
- See [AI-Assisted Contributing](AI-Assisted-Contributing) for guidelines
