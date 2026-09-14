# Quick Commands & Code Generation

> Build/test/dev commands and the code-generation workflow. Loaded on demand from [`../../CLAUDE.md`](../../CLAUDE.md).
> For *which* suite each command runs and which CI workflow gates it, see [`testing-ci.md`](testing-ci.md).

## Quick Commands

All ROS2 commands assume you are inside the devcontainer.

```bash
# Build ROS2 workspace
cd ros2 && make build

# Build a single package
cd ros2 && make build-pkg PKG=mowgli_behavior

# Run headless simulation (Webots; runs sim-stop first, Foxglove on ws://localhost:8765)
cd ros2 && make sim

# Run E2E tests — SELF-CONTAINED: sim-stop + build, launches the sim itself, waits 90 s,
# runs src/e2e_test.py, then stops the sim. Do NOT start a sim first — it gets killed.
cd ros2 && make e2e-test
cd ros2 && make e2e-test-no-lidar   # GPS-only variant (src/e2e_test_no_lidar.py)

# Format C++ code (all of ros2/src, including the vendored opennav_coverage tree;
# ./scripts/format.sh skips it and warns unless clang-format is 18.x — CI pins 18)
cd ros2 && make format

# Run unit tests — `make test` is BROKEN: scripts/test.sh reads ${PACKAGES} under
# `set -u` and never defaults it (build.sh does), so it dies "PACKAGES: unbound variable".
cd ros2 && PACKAGES="" ./scripts/test.sh                 # whole workspace
cd ros2 && PACKAGES="mowgli_behavior" ./scripts/test.sh  # one package

# Build firmware (default env Yardforce500 / STM32F103VC)
cd firmware/stm32/ros_usbnode && pio run
cd firmware/stm32/ros_usbnode && pio run -e Yardforce500B   # STM32F401VC board

# GUI development — run the backend FROM gui/ (it opens asserts/*.json relative to CWD)
cd gui && make run-backend                # CGO_ENABLED=0 go run main.go, serves :4006
cd gui/web && yarn install && yarn dev    # vite :5173, /api proxied to the backend

# --- Code generation (run after changing .msg/.srv files) ---
# Prefix the two GUI generators with LC_ALL=C on macOS: they iterate `*.msg` shell globs,
# and a different collation order fabricates ~20 lines of phantom drift.

# Regenerate firmware rosserial C++ headers from ROS2 .msg files
python3 firmware/scripts/sync_ros_lib.py          # writes to firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/
python3 firmware/scripts/sync_ros_lib.py --check   # diff-only, no writes (CI)

# Regenerate Go message/service structs from ROS2 .msg/.srv files
cd gui && LC_ALL=C ./generate_go_msgs.sh           # writes gui/pkg/msgs/*/types_generated.go + mowgli/services_generated.go

# Regenerate TypeScript ROS types (snake_case fields matching rosbridge JSON)
cd gui && LC_ALL=C ./generate_ts_types.sh          # writes to gui/web/src/types/ros.generated.ts
```

## Code Generation Workflow

When you modify `ros2/src/mowgli_interfaces/msg/*.msg` or `srv/*.srv`:
1. **Firmware headers:** `python3 firmware/scripts/sync_ros_lib.py` — regenerates rosserial C++ headers
2. **Go types:** `cd gui && LC_ALL=C ./generate_go_msgs.sh` — regenerates Go structs with JSON tags for rosbridge
3. **TypeScript types:** `cd gui && LC_ALL=C ./generate_ts_types.sh` — regenerates `gui/web/src/types/ros.generated.ts` with snake_case fields matching rosbridge JSON (`ros.ts` is a hand-written one-line re-export of it — leave it alone)
4. **Protocol constants:** the high-level mode numbers are hand-mirrored in THREE live places — `HighLevelStatus.msg` (`HIGH_LEVEL_STATE_*`), `firmware/stm32/ros_usbnode/include/mowgli_protocol.h:418-422` (`HL_MODE_*`, authoritative) and the local `static constexpr HL_MODE_*` block in `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp:75-79` (the copy the bridge actually compiles). `ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` is a non-built C reference copy stuck at `MOWGLI_PROTOCOL_VERSION 3` — do not treat it as the host mirror.
5. **Wire protocol:** if a `pkt_*_t` struct or a `PKT_ID_*` id changed, bump `MOWGLI_PROTOCOL_VERSION` (`firmware/stm32/ros_usbnode/include/mowgli_protocol.h:59`) AND `kMowgliProtocolVersion` (`ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp:51`) in lockstep, then refresh the baseline with `python3 firmware/scripts/protocol_version_guard.py` (no `--check`).

CI gates both: `msg-codegen-drift.yml` runs `sync_ros_lib.py --check` and re-runs the two GUI generators with `git diff --exit-code`; `protocol-version-drift.yml` runs `protocol_version_guard.py --check`. Commit every generated file in the same change. Both have `pull_request: branches: [main]` only, so a PR into `dev` gets them via the `push` trigger — which matches `feat|fix|refactor|chore|perf/**` branch names only.

Do NOT hand-edit `*_generated.go`, `ros_lib/mower_msgs/*.h`, or `gui/web/src/types/ros.generated.ts` — re-run the scripts instead.
