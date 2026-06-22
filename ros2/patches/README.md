# Vendored upstream patches

Patches we carry against **upstream** ROS 2 / Nav2 packages that ship in
`/opt/ros/kilted` but are not yet fixed in the version Kilted provides. Each
patch is applied at **image-build time** by a dedicated builder stage in
`ros2/Dockerfile` that:

1. clones the upstream package at a pinned ref,
2. applies the `.patch` from this directory, and
3. `colcon build`s it into a standalone overlay prefix (e.g.
   `/opt/nav2_mppi_patched`) that the runtime image sources **after**
   `/opt/ros/kilted` but **before** the workspace overlay — so the patched
   build shadows the apt-installed system package.

These are NOT forks of our own packages — they are minimal, removable overlays
over upstream code. Keep each patch as small as possible (a single hunk where
feasible) so it re-bases trivially when the upstream ref is bumped.

---

## `nav2_mppi_controller-turn-in-place-injection.patch`

- **Upstream:** same pinned ref as below (`71a1d84`, nav2_mppi_controller 1.4.2).
  Applied AFTER the arc-length patch in the same builder stage.
- **STATUS: EXPERIMENTAL SPIKE — off by default.** With
  `turn_in_place_samples: 0` (the default) the controller is byte-for-byte
  stock MPPI. This is NOT carried because upstream lacks a fix; it is a local
  experiment to see whether MPPI can do the hard ~180° coverage swath flips
  itself instead of via per-segment RotationShim re-dispatch.
- **WHY this is a patch, not a plugin:** upstream PR #6076 pluginized only the
  *motion models*; for a diff-drive the DiffDrive model already permits a pivot
  (vx=0, wz≠0) — the blocker is that Gaussian control sampling (`NoiseGenerator`)
  almost never *proposes* a near-stationary pivot, and that path is not
  pluginizable. So the only MPPI-native route is to inject candidates directly.
  See nav2 issue #6032 (control-set generation) — the maintainer's recommended
  alternative is RotationShim / Spin, which is why this stays a spike.
- **WHAT:** Adds `Optimizer::injectTurnInPlacePrimitives()`, called in
  `generateNoisedTrajectories()` between `setNoisedControls()` and the motion
  rollout. When `turn_in_place_samples (K) > 0` AND the robot is near-stationary
  (`< turn_in_place_max_speed`) AND grossly mis-headed vs the path
  `turn_in_place_lookahead` m ahead (`|yaw_err| > turn_in_place_min_yaw`), it
  overwrites the last K of the `batch_size` sampled trajectories with a
  deterministic **pivot-then-advance** primitive (spin toward the path heading,
  then drive forward along it). The critics then score it like any other
  candidate — a pivot is only chosen if it actually wins the softmax. A pure
  spin is deliberately NOT used (it makes no progress and the path-follow critic
  would reject it).
- **Params** (under the MPPI controller, e.g. `FollowCoveragePath.*`):
  `turn_in_place_samples` (int, **0=disabled**), `turn_in_place_min_yaw` (rad,
  1.2), `turn_in_place_max_speed` (m/s, 0.1), `turn_in_place_lookahead` (m, 0.5).
- **OPEN QUESTION the spike answers:** whether the critics actually select the
  injected pivot at a swath flip, or whether forward candidates still out-score
  it (Steve Macenski's caveat in #6032). Measure with the monitor's
  `cross_checks.rotation` / `pivot_stall` telemetry on a coverage run with
  `turn_in_place_samples: ~100` vs `0`.
- **REMOVE** if the spike fails or once the per-segment RotationShim path is
  chosen: delete this `.patch`, the second `COPY`/`patch`/`grep` lines in the
  `nav2-mppi-patched-builder` Dockerfile stage.

---

## `nav2_mppi_controller-findPathFurthestReachedPoint-arclength.patch`

- **Upstream:** `ros-navigation/navigation2`, branch `kilted`
  (pinned in the Dockerfile to `71a1d84` = `nav2_mppi_controller` **1.4.2**,
  the version Kilted's `ros-kilted-nav2-mppi-controller` apt package ships).
- **WHAT:** Replaces the body of
  `utils::findPathFurthestReachedPoint()` in
  `nav2_mppi_controller/include/nav2_mppi_controller/tools/utils.hpp`
  with the **arc-length-bounded** version, and adds `#include <cmath>`.
- **WHY:** Kilted 1.4.2 still has the old **Euclidean** "furthest reached
  point" search. On coverage paths with tight reversals (the ~180° flip
  between adjacent serpentine swaths, concentric headland-ring reversals) the
  return leg of a U-turn is *spatially* close to the start leg, so the
  Euclidean nearest-point match jumps the progress index across the turn. MPPI
  then cuts the corner / "loses" the swath. The arc-length bound caps each
  candidate trajectory's match by its own integrated path length, so progress
  can't teleport across a curvature fold. This is upstream
  **PR #6055** ("MPPI: add arc-length path progress"), which fixes upstream
  **issue #5925**. Identical behaviour on straight / gently-curved paths.
- **Builder stage:** `nav2-mppi-patched-builder` in `ros2/Dockerfile`; the
  built tree lands in `/opt/nav2_mppi_patched` and is sourced by
  `scripts/ros2_entrypoint.sh`.

### REMOVE THIS once Kilted ships the fix

When `ros-kilted-nav2-mppi-controller` is updated to a version that contains
PR #6055 (check the upstream `kilted` branch changelog / the installed apt
version), DELETE:

- this `.patch` file,
- the `nav2-mppi-patched-builder` Dockerfile stage and the
  `COPY --from=nav2-mppi-patched-builder ...` line in the runtime stage,
- the `/opt/nav2_mppi_patched/setup.bash` source block in
  `scripts/ros2_entrypoint.sh`.

Leaving the overlay in place after the upstream fix lands would silently
**shadow** the system package with our pinned, stale 1.4.2 build — including
any future bug fixes or security patches Kilted makes to `nav2_mppi_controller`.

### Verify the patch still applies (after an upstream ref bump)

```bash
git clone --depth 1 --filter=blob:none --sparse -b kilted \
  https://github.com/ros-navigation/navigation2.git /tmp/nav2_verify
cd /tmp/nav2_verify && git sparse-checkout set nav2_mppi_controller
cd nav2_mppi_controller
patch -p1 --dry-run \
  < /path/to/ros2/patches/nav2_mppi_controller-findPathFurthestReachedPoint-arclength.patch
```

> Use `patch`, **not** `git apply`, here and in the Dockerfile. In a
> blob-filtered + sparse clone (which is what the Dockerfile and this snippet
> use to avoid pulling the whole navigation2 tree) `git apply` silently
> NO-OPs — it returns exit 0 and even `git apply --check` passes, but it
> leaves the file untouched because it can't reconcile the recorded index
> blob hashes. `patch` is content-based and applies reliably. The Dockerfile
> follows the `patch` with a `grep` guard so a silent miss fails the build.
