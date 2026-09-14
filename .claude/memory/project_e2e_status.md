---
name: E2E simulation status and remaining work
description: SUPERSEDED — the 2026-04-04 Gazebo/SLAM-era E2E notes no longer apply; current reference is docs/claude/testing-ci.md § Simulation & E2E and docs/WEBOTS_SIM.md
type: project
---

> **Superseded** — the original body (2026-04-04) described the Gazebo +
> slam_toolbox + `ExecuteFullCoveragePath` + RPP-coverage stack. None of that
> exists any more, so the notes were removed rather than kept as a trap.

Why every item in the old note is dead:

- **Simulator is Webots, not Gazebo.** `ros2/src/mowgli_simulation/` ships
  `worlds_webots/`, `urdf_webots/`, `protos/` and one launch file
  (`launch/webots_minimal.launch.py`) — there is no Gazebo world, SDF, or
  `ros_gz_bridge` in the package.
- **`ExecuteFullCoveragePath` no longer exists.** Coverage is `PlanCoverageArea`
  (calls `mowgli_coverage`'s `plan_coverage` action, Fields2Cover v3) plus
  `FollowStrip` driving each continuous `drivable_subpath` as one goal — see
  `ros2/src/mowgli_behavior/trees/main_tree.xml:686-716`. Path subsampling and
  the "first-run pose matching" cursor logic were replaced along with it.
- **Coverage is not tracked by RPP.** The `FollowCoveragePath` slot is
  `mowgli_nav2_plugins/FTCController`; RotationShim + RPP only drive transit
  (`ros2/src/mowgli_bringup/config/nav2_params_base.yaml:11-12`, `:339`).
- **SLAM is gone.** `fusion_graph_node` is the sole, unconditional localizer and
  owns both `map→odom` and `odom→base_footprint` (root `CLAUDE.md` Invariant 1).

**Current reference:**

- [`docs/claude/testing-ci.md`](../../docs/claude/testing-ci.md) § Simulation &
  E2E — `make sim`, `make sim-stop`, `make e2e-test`, `make e2e-test-no-lidar`
  (`ros2/Makefile:78-129`) and what each harness scores.
- [`docs/WEBOTS_SIM.md`](../../docs/WEBOTS_SIM.md) — Webots/ODE quirks and the
  load-bearing workarounds.
- [`docs/claude/codemaps/mowgli_simulation.md`](../../docs/claude/codemaps/mowgli_simulation.md)
  — file map plus the known-dead subscriptions in `e2e_test.py`.
