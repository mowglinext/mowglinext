# GUI

MowgliNext web interface -- React frontend + Go backend for mower monitoring and control.

## Access

Default: `http://<mower-ip>:4006`

## Dashboard

![Dashboard — mowing state](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/dashboard-mowing.png)

The dashboard adapts to the mower's current state with a **hero card**. It always carries the active zone, a battery ring, and -- once a map is loaded -- a "coverage today" ribbon with the mowed / total area in m². The headline and the big primary button change with the state:

| State | Headline | Primary button |
|-------|----------|----------------|
| **Mowing / recording / manual** | Minutes left before the robot heads home (live ETA from the remaining un-mowed cells) | **Pause** -- stop in place (`COMMAND_STOP`); Nav2 stays up so `COMMAND_START` resumes the mission |
| **Charging** | Battery percentage | **Start mowing** (`COMMAND_START`) |
| **Emergency (latched)** | Emergency stop | **Re-arm** -- asks firmware to clear the latch (it only clears once the physical trigger is released) |
| **Idle / Docked** | Idle greeting | **Start mowing** (`COMMAND_START`) |

Two secondary buttons flank the primary in every state: an **emergency stop** (behind a confirm dialog -- it latches the firmware emergency) and **Send home** (`COMMAND_HOME`).

Next to the hero sit a **live mini-map** (areas, obstacles, the mowed-cell overlay and the robot) and a 2x2 grid of **telemetry tiles**:
- **GPS** -- quality percentage, RTK status (Fixed/Float/GPS)
- **Blades** -- RPM + ESC current draw
- **Motor** -- motor temperature + ESC temperature
- **Rain** -- detected / dry

Battery lives in the hero's ring rather than in a tile. A **Health check** card lists GPS, rain, alerts, motor temperature and firmware compatibility as status rows, with the current weather as a chip in its header; an **IrriSense** banner appears above the hero when the soil is too wet to mow.

### Mobile

![Dashboard — mobile](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/dashboard-mobile.png)

On mobile, the dashboard stacks vertically: compact hero card, live mini-map, 2x2 tile grid, then the health card. The bottom tab bar provides quick access to Home, Map, Schedule and Diagnostics, plus a **More** sheet for Statistics, Settings, Parameters, Logs and Onboarding.

## Pages

| | |
|---|---|
| ![Map](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/map.png) | ![Schedule](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/schedule.png) |
| ![Statistics](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/stats.png) | ![Dashboard idle](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/dashboard-idle.png) |

| Page | Description |
|------|-------------|
| **Dashboard** | State-adaptive hero + live mini-map + telemetry tiles + health check |
| **Map** | Mapbox GL map editor -- define mowing areas, navigation zones and obstacles, place the dock (position + heading), OpenMower map import, live robot position, joystick for manual mowing |
| **Schedule** | Weekly grid view with color-coded schedule blocks, schedule cards with day toggles and time picker, IrriSense soil chip |
| **Statistics** | Hero stat cards (distance, hours, completion rate, runs), weekly bar chart, a year-of-mowing heatmap, zone coverage bars, session history table |
| **Settings** | Grouped configuration editor (Appearance, Hardware, Drive Motor, NTRIP Corrections, GPS & Positioning, Sensors, **Localization**, Mowing, Docking, Battery, Safety, Obstacles, Navigation, Rain, Status LEDs, IrriSense, Advanced) |
| **Parameters** | Live ROS2 parameter editor -- read and write running-node parameters without a restart, with a basic/middle/expert tier filter and a confirm step on dangerous keys |
| **Onboarding** | First-time setup wizard (9 steps: welcome, robot model, firmware, NTRIP, GPS, datum, sensors, calibration, done) |
| **Diagnostics** | Health hero + alert list, then tabs: System (containers, CPU temp, rosbag, raw `/diagnostics`), Localization (filtered pose, **Fusion Graph (iSAM2)**, heading sources), Robot (behavior tree + coverage, sensors), Calibration (config cross-checks, calibration status) |
| **Logs** | Live container log viewer -- pick any container on the host (the `mowgli-*` ones carry an app label), tail it with a severity filter |

### Settings: Localization section

The map-frame localizer is **not** selectable: `fusion_graph_node` (GTSAM iSAM2) is the sole, unconditional localizer and owns both `map→odom` and `odom→base_footprint`. The section opens with a note saying so, then gathers the flags that tune what the graph fuses:

- **LiDAR for obstacle avoidance** — a read-only status card: whether `lidar_enabled` is on and what the LiDAR driver's latest `/diagnostics` entry says. The toggle itself lives in *Sensors → lidar_enabled*.
- **LiDAR scan matching** (`use_scan_matching`) — adds ICP between-factors from `/scan` to the graph.
- **Loop closure** (`use_loop_closure`) — searches past scans for revisits and adds loop-closure factors.
- **Magnetometer yaw** (`use_magnetometer`) — fuses tilt-compensated magnetometer yaw as a unary factor. Off by default — enable only after running mag calibration with motors-off.
- **Magnetometer calibration & tuning** — `enable_mag_cal` (collect calibration samples) plus `declination_deg`, `min_horizontal_uT` and `mag_yaw_variance`.

The dock pose (`dock_pose_x`, `dock_pose_y`, `dock_pose_yaw`) lives in `mowgli_robot.yaml` — there is no `dock_calibration.yaml`, it was removed. It is normally written for you by the one-click dock calibration (*Settings → Docking*), by the map editor's "Set docking point" action, and by the "Set dock from current pose" action in *Settings → Sensors* (which captures the GPS-averaged position while the robot is physically charging on the dock). The same *Sensors* card also exposes the dock heading as a compass bearing for a manual override when calibration is unavailable or wrong.

**Overridden dot + reset to default.** Across the Settings sections, any field whose value differs from its shipped default shows a small **dot** and a **reset (undo) button** that reverts it to the default. This works with the [sparse installed config](https://github.com/mowglinext/mowglinext/wiki/Configuration#sparse-robot-config-model): reset puts the shipped default back in the editor, and on Save the backend prunes every key whose value equals its default — so the key disappears from the installed `mowgli_robot.yaml` and falls back to the in-package template. Save only sends the fields you actually changed, so concurrent writers (dock calibration, the map editor's "Set docking point") are never clobbered.

### Diagnostics: Fusion Graph panel

The Diagnostics page's *Localization* tab always carries a dedicated **Fusion Graph (iSAM2)** card — `fusion_graph_node` is the only map-frame localizer, so there is nothing to gate it on. It shows:

- **Nodes in graph** — `total_nodes` from `/fusion_graph/diagnostics`, with the count of nodes that have a stored scan attached.
- **Loop closures** — successful loop-closure factors added since boot.
- **ICP success rate** — `scan_matches_ok / (ok + fail)` over the session.
- **Pose σ** — `√((cov_xx + cov_yy)/2)` in centimetres, with the yaw σ in degrees underneath. Colour-coded green / amber / red.
- **ICP keyframes** and **ICP rejects** — keyframe count with its match rate, and the reject breakdown (RMSE / inliers / sanity / divergence).
- **Attach rate** — the share of received scans that actually became graph factors — and **hand push** (wheels stationary but the gyro disagrees), with the count of GPS fixes rejected as wrong-fix.
- **Save graph** / **Clear graph** buttons — call the corresponding `~/save_graph` / `~/clear_graph` services on `fusion_graph_node`. Save persists the graph to `/ros2_ws/maps/fusion_graph.{graph,scans,meta}`; Clear wipes iSAM2 and waits for the next pose seed to re-initialize.

The card tags itself *stale* when the last `/fusion_graph/diagnostics` sample is more than 5 s old.

## Design System

- **Font:** Satoshi (body), Instrument Serif (display headlines), Space Grotesk / JetBrains Mono (telemetry values)
- **Color palette:** Green-tinted dark theme (`#02110D` base, `#7CFFB2` lime accent, `#45D6E8` aurora cyan, `#F3A85C` amber, `#FF6B7A` rose). The app is **dark-only** — there is no light-mode switch
- **Cards:** 18px border-radius, 1px subtle border, panel surface color
- **Icons:** Custom stroke-1.6 SVG icon set (mower, battery, signal, blades, thermometer, etc.) alongside the Lucide and Ant Design sets
- **Animations:** State pill pulse, boundary violation glow (respects `prefers-reduced-motion`)
- **Display modes:** *Settings → Appearance* offers **Visual** (richer glass surfaces and ambient motion), **Balanced** (the default) and **Efficient** (lower compositing, slower visual update rates). The choice is per browser (`localStorage`), affects rendering only, and never changes what the robot does or how fast it publishes

## Architecture

- **Frontend:** React (TypeScript) + Ant Design + styled-components, translated into English and French
- **Backend:** Go, connects to ROS2 via foxglove_bridge (`ws://localhost:8765`) using the foxglove WebSocket protocol, plus a teleop relay on `ws://localhost:8766` for the joystick
- **Real-time data:** one multiplexed WebSocket (`/api/mowglinext/multiplex`, msgpack frames) carries every live topic — status, power, GPS, pose, emergency, map, mow progress, LiDAR, diagnostics. The browser never talks to foxglove directly; it only ever calls the Go backend under `/api`
- **State management:** Custom hooks (`useHighLevelStatus`, `usePower`, `useStatus`, `useGPS`, `useEmergency`, `useSettings`)
- **System actions:** the header power menu restarts the whole Mowgli stack, and behind an *Advanced* submenu can reset the STM32 board, reboot the Pi or shut it down
- **Optional bridges:** an embedded MQTT broker and a HomeKit switch accessory, both **off by default** (`system.mqtt.enabled` / `system.homekit.enabled` in the GUI's key-value DB, or the `MQTT_ENABLED` / `HOMEKIT_ENABLED` env vars)

## Development

```bash
cd gui

# Backend (run it from gui/ — it loads asserts/ relative to the working dir)
CGO_ENABLED=0 go build -o mowglinext .
./mowglinext
go test ./...          # Go unit tests; no CI job runs these, so run them locally

# Frontend
cd web
yarn install
yarn dev    # http://localhost:5173 with hot reload
MOWGLI_API_TARGET=http://<mower-ip>:4006 yarn dev   # dev UI against a live robot
npx tsc --noEmit && yarn lint && yarn test          # what CI checks
```

Developer notes live with the code: [`gui/CLAUDE.md`](https://github.com/mowglinext/mowglinext/blob/main/gui/CLAUDE.md), with per-half indexes in [`docs/claude/codemaps/gui_backend.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/codemaps/gui_backend.md) and [`docs/claude/codemaps/gui_frontend.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/codemaps/gui_frontend.md).

## Docker

```bash
docker build -t mowglinext gui/
```

## Configuration

The GUI reads and writes the **installed** `mowgli_robot.yaml` (`/mowgli_config/mowgli_robot.yaml` in the container, bind-mounted from `docker/config/mowgli/` on the host and seeded at install time from `install/config/mowgli/`). It keeps that file [sparse](https://github.com/mowglinext/mowglinext/wiki/Configuration#sparse-robot-config-model): on write it drops any key whose value equals the shipped default, so absent keys fall through to their template defaults at launch. (The GUI's notion of "default" is the settings JSON schema `gui/asserts/mower_config.schema.json`, which mirrors the in-package template `mowgli_bringup/config/mowgli_robot.yaml` — the true default source the launch-time deep-merge consumes.) Changes take effect after restarting the ROS2 container (`mowgli-ros2`), which the Settings page offers to do for you.
