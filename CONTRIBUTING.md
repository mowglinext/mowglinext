# Contributing to MowgliNext

Thanks for your interest in contributing! MowgliNext is a community-driven project and we welcome contributions of all kinds.

## Getting Started

1. Fork the repository
2. Clone your fork: `git clone https://github.com/YOUR_USERNAME/mowglinext.git`
3. Create a feature branch: `git checkout -b feat/my-feature`
4. Make your changes
5. Push and open a Pull Request

## What to Contribute

- **Bug fixes** — found something broken? Fix it and send a PR
- **New sensor drivers** — add support for your GPS or LiDAR in `sensors/`
- **Behavior tree improvements** — new BT nodes or tree logic in `ros2/src/mowgli_behavior/`
- **GUI features** — React frontend or Go backend improvements in `gui/`
- **Documentation** — fixes, clarifications, guides in `docs/`
- **Test coverage** — we need more tests across all packages

## Development Setup

### ROS2 Stack

```bash
# Requires ROS2 Kilted on Ubuntu 24.04
cd ros2
source /opt/ros/kilted/setup.bash
rosdep install --from-paths src --ignore-src \
  --skip-keys universal_gnss_ros2 -y
colcon build
colcon test
```

### GUI

```bash
cd gui
go build -o openmower-gui
cd web && yarn && yarn build
```

### Firmware

```bash
cd firmware/stm32/ros_usbnode
pio run
```

## Code Style

- **C++**: Follow `.clang-format` in `ros2/`. Run `clang-format` before committing
- **Go**: Run `gofmt`
- **TypeScript/React**: Run `prettier` and `eslint`
- **Python**: Follow PEP 8
- **Commits**: Use [conventional commits](https://www.conventionalcommits.org/) — `feat:`, `fix:`, `refactor:`, `docs:`, `test:`, `chore:`, `ci:`

## Pull Request Guidelines

- Keep PRs focused — one feature or fix per PR
- Include a description of **what** and **why**
- Add tests for new functionality when possible
- Update documentation if your change affects user-facing behavior
- PRs are reviewed by Claude (AI) and maintainers — address both

## Reporting Bugs

Use the [bug report template](https://github.com/mowglinext/mowglinext/issues/new?template=bug_report.yml) and include:
- Hardware (board, GPS, LiDAR model)
- Steps to reproduce
- Expected vs actual behavior
- Relevant logs

## Proposing Features

Use the [feature request template](https://github.com/mowglinext/mowglinext/issues/new?template=feature_request.yml). The community and AI will help evaluate and refine proposals.

## License

By contributing, you agree that your contributions will be licensed under the [GPLv3 License](LICENSE).
