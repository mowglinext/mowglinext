# Universal GNSS downstream validation checkpoint

## Baseline

- Parent: `/ros2_ws/src/mowglinext`, branch `fix/gnss-downstream`, HEAD
  `5bd4e6af37773ce6f44330cdf66362c9fe0a04e6`, clean at initial inspection.
- Parent remotes: `origin=https://github.com/mowglinext/mowglinext.git`,
  `french=https://github.com/mowglifrenchtouch/mowglinext.git`.
- GNSS submodule: `ros2/src/external/universal-gnss`; parent gitlink and
  detached worktree HEAD `5281472116669972ae12b9d1997d66b064671cf5`.
- `.gitmodules` URL is `https://github.com/Pepeuch/universal-gnss.git`; it now
  configures branch `main`.

## Established facts — DO NOT REDISCOVER

- A fresh `git fetch origin main` resolved upstream `origin/main` to
  `f974b565100b4f2aa3d522cd9a292f30f905e8bc`.
- The prior pin and upstream main are divergent: `origin/main` is an ancestor
  of neither the prior pin nor vice versa.
- The submodule worktree is detached at `f974b565100b4f2aa3d522cd9a292f30f905e8bc`.
  The parent gitlink and `.gitmodules` are intentionally modified but not
  committed.
- The user-specified canonical clone path
  `/root/Documents/vscode/tondeuse/mowglinext-official` does not exist in this
  environment. Do not silently substitute another clone for that requested
  comparison.

## Semantic decisions — DO NOT RE-LITIGATE WITHOUT NEW EVIDENCE

- Authorized scope is downstream compatibility validation only. No redesign,
  no `frame_writer` restoration, no commit/push.
- Only this submodule gitlink and its branch metadata are in scope for changes.

## Validation evidence — DO NOT RERUN UNLESS INVALIDATED

- `git diff --submodule` and `git diff --check` were clean before changes.
- Direct focused build PASS: `PACKAGES_MODE=select PACKAGES="universal_gnss_ros2
  mowgli_localization fusion_graph mowgli_hardware mowgli_behavior mowgli_bringup"
  ./scripts/build.sh` as `ubuntu`.
- The project-normal dependency-complete `--packages-up-to` attempt reached
  configuration but is BLOCKED by an environment-only Fields2Cover mismatch:
  `mowgli_coverage` requires 3.0.0 while only 2.0.0 is installed. It is not a
  Universal GNSS compilation failure.
- GNSS-specific test evidence PASS: Universal GNSS status/NavSat/receiver/NTRIP
  tests, Mowgli localization/hardware/fusion tests, behavior tests, and
  Mowgli bringup's `test_gnss_launch_config` + NavSat universal launch test.
- `test_tf_ownership` fails only under Colcon's `/ros2_ws` working directory
  because it expects paths relative to `ros2`; exact rerun from
  `/ros2_ws/src/mowglinext/ros2` PASS (3 tests). This is unrelated to GNSS.
- `sensors/gps/Dockerfile` copies Universal GNSS from the vendored submodule;
  `universal_gnss_topic_bridge.py` maps receipt stamp, observation sequence,
  COG/heading, capability flags, and correction diagnostics onto the Mowgli
  public status contract. An image built from this parent checkout therefore
  consumes `f974b565100b4f2aa3d522cd9a292f30f905e8bc`.

## Open questions

- Current remote `origin/main` target and whether it is the validated work.
- Downstream API/config/deployment compatibility and focused ROS2 test/build
  results.

## Exact next step

Validation complete. Do not commit/push. Current parent changes are only
`.gitmodules` and the Universal GNSS gitlink; `git diff --check` passes.
