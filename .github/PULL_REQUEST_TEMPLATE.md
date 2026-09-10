## Summary

<!-- What does this PR do? Keep it brief. -->

## Changes

<!-- Bullet list of what changed and why -->

-

## Component

<!-- Check all that apply -->

- [ ] ROS2 Stack (`ros2/`)
- [ ] GUI (`gui/`)
- [ ] Docker (`docker/`)
- [ ] Sensors (`sensors/`)
- [ ] Firmware (`firmware/`)
- [ ] Installer (`install/`)
- [ ] Documentation (`docs/`, `wiki/`)
- [ ] CI/CD (`.github/`)

## Testing

<!-- How was this tested? -->

- [ ] Built successfully
- [ ] Tested on hardware
- [ ] Tested in simulation
- [ ] Unit tests pass
- [ ] Manual testing

## Checklist

<!-- Base branch is `dev` (feature work lands there; `main` is the release branch).
     Full "what fails, in what order" pre-PR list: docs/claude/testing-ci.md § "Before opening a PR".
     Area file maps / topics / config keys: docs/claude/codemaps/, docs/claude/ros-interfaces.md,
     docs/claude/parameters.md. Leave the conditional boxes below unticked if they don't apply. -->

- [ ] Follows conventional commit format
- [ ] Documentation updated (if user-facing)
- [ ] No hardcoded secrets or credentials
- [ ] No unrelated changes
- [ ] Physical behavior (blades, motion, e-stop, safety gates) — flagged as **safety-critical** in the summary above, or unaffected
- [ ] C++ touched → changed lines are clang-format **18** clean (CI installs `clang-format-18`; pre-commit pins `v18.1.8`)
- [ ] `ros2/src/mowgli_interfaces/` touched → regenerated all three consumers (`firmware/scripts/sync_ros_lib.py`, `gui/generate_go_msgs.sh`, `gui/generate_ts_types.sh`) — gated by `msg-codegen-drift`
- [ ] COBS wire touched (`mowgli_protocol.h` / `ll_datatypes.hpp`) → bumped `MOWGLI_PROTOCOL_VERSION` **and** its host mirror `kMowgliProtocolVersion` — gated by `protocol-version-drift`
- [ ] `mowgli_robot.yaml` touched → `python3 ros2/scripts/check_config_drift.py` passes (installed file stays sparse over the template)
