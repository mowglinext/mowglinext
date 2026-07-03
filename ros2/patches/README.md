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
