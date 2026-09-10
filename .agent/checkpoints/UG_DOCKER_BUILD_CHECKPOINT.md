# Universal GNSS Docker build investigation checkpoint

## Baseline

- Parent: `/ros2_ws/src/mowglinext`, branch `fix/gnss-downstream`, HEAD
  `01bde9beff6a0f2778cfa16ceb942f198963f493`.
- UG submodule: detached at the parent-pinned upstream `main` SHA
  `f974b565100b4f2aa3d522cd9a292f30f905e8bc`.
- User requested no commit/push and no reversion of the validated submodule pin.

## Established facts — DO NOT REDISCOVER

- A non-root `docker buildx build --progress=plain` cannot access this
  environment's Docker socket; no unprivileged Podman/Buildah/etc. is
  installed. Do not use root Docker or grant Docker-group access without user
  authority.
- UG `main` newly unconditionally has `add_subdirectory(gnss_runtime)` at its
  top level.
- Mowgli's `sensors/gps/Dockerfile` Stage B copied/symlinked six former UG
  subdirectories but omitted `gnss_runtime`.
- Exact disposable-layout reproduction as `ubuntu` failed at the first Stage B
  command: `cmake -S ... -DUNIVERSAL_GNSS_BUILD_ROS2=OFF`, with
  `add_subdirectory given source "gnss_runtime" which is not an existing
  directory.`

## Semantic decisions — DO NOT RE-LITIGATE WITHOUT NEW EVIDENCE

- This is a downstream Mowgli Docker integration defect, not evidence to
  revert UG or alter its standalone build semantics.
- Authorized fix is only to copy and link `gnss_runtime` in the pre-existing
  Stage B layout.

## Validation evidence — DO NOT RERUN UNLESS INVALIDATED

- The source-level reproduced Stage B failure is deterministic on this exact
  pinned UG source layout.
- After adding `gnss_runtime` to the Docker copy/symlink set, the complete
  Dockerfile-equivalent temporary workflow PASSed as `ubuntu`: Colcon built
  `mowgli_interfaces`, `universal_gnss_ros2`, and `mowgli_gnss_bridge`; the
  standalone top-level CMake configure/build/install PASSed with
  `UNIVERSAL_GNSS_BUILD_ROS2=OFF`.
- The temporary install contains receiver, bridge, and all expected GNSS tools
  (including `universal_gnss_supervisor`), with no duplicate library basenames.
- `git diff --check` PASSes. Parent gitlink and submodule worktree both remain
  `f974b565100b4f2aa3d522cd9a292f30f905e8bc`.

## Open questions

- Final Docker result remains externally blocked by non-root Docker socket
  access unless user grants authority.

## Exact next step

Do not commit/push. A real `docker buildx build` must be run by a user with
Docker socket access; project policy forbids running it as root or silently
granting Docker-group access to `ubuntu`.
