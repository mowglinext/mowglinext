# GUI

MowgliNext web interface -- React frontend + Go backend for mower monitoring and control.

## Access

Default: `http://<mower-ip>:4006`

## Dashboard

![Dashboard — mowing state](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/dashboard-mowing.png)

The dashboard adapts to the mower's current state with a **hero card** that shows contextual information and actions:

| State | Hero card | Actions |
|-------|-----------|---------|
| **Mowing** | Zone name, progress bar, elapsed time, GPS quality | Pause, Send home, Emergency |
| **Idle / Docked** | Battery level, mower icon, next scheduled run | Start mowing, Emergency |
| **Charging** | Radial gauge with battery %, charge current, ETA | Mow anyway |
| **Emergency** | Alert with description and instructions | Reset emergency |
| **Boundary violation** | Warning with map context | Send home, Resume |
| **Rain detected** | Weather pause notice | Mow anyway |
| **Low battery docking** | Battery warning, auto-resume notice | Keep mowing |

Below the hero card, four **telemetry tiles** show live sparklines:
- **Battery** -- percentage + voltage, charging indicator
- **GPS** -- quality percentage, RTK status (Fixed/Float/GPS)
- **Blades** -- RPM + current draw
- **Motor** -- temperature + ESC temperature

The bottom row shows **Today's work** (zone progress, active zones), **Next up** (schedule link), and **Health check** (GPS, rain, emergency, motor temp status dots).

### Mobile

![Dashboard — mobile](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/dashboard-mobile.png)

On mobile, the dashboard stacks vertically: compact hero card, 2x2 tile grid, and collapsible sensor/system panels. All buttons expand to full width. The bottom tab bar provides quick access to Home, Map, Stats, and Settings.

## Pages

| | |
|---|---|
| ![Map](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/map.png) | ![Schedule](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/schedule.png) |
| ![Statistics](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/stats.png) | ![Dashboard idle](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/screenshots/dashboard-idle.png) |

| Page | Description |
|------|-------------|
| **Dashboard** | State-adaptive hero + live telemetry tiles + health check |
| **Map** | Mapbox GL map editor -- define mowing areas, navigation zones, dock position (read-only display), live robot position |
| **Schedule** | Weekly grid view with color-coded schedule blocks, schedule cards with day toggles and time picker |
| **Statistics** | Hero stat cards (distance, hours, completion rate, runs), weekly bar chart, zone coverage bars, session history table |
| **Settings** | Grouped configuration editor (Hardware, GPS & Positioning, Sensors, **Localization**, Mowing, Docking, Battery, Safety, Navigation, Rain, Advanced) |
| **Onboarding** | First-time setup wizard |
| **Diagnostics** | System health, **Filtered Pose**, GPS, **Fusion Graph (iSAM2)** when opt-in is enabled, IMU, wheel ticks, calibration status, ROS DiagnosticsArray |
| **Logs** | Live ROS2 log viewer |

### Settings: Localization section

The *Localization* section gathers the four flags that pick the map-frame fusion strategy in one place:

- **Fusion Graph (iSAM2)** (`use_fusion_graph`) — switch between the default robot_localization EKF and the GTSAM iSAM2 factor-graph node. Requires a ROS2 restart.
- **LiDAR scan matching** (`use_scan_matching`) — adds ICP between-factors from `/scan` to the graph. Greyed out until *Fusion Graph* is enabled.
- **Loop closure** (`use_loop_closure`) — searches past scans for revisits and adds loop-closure factors. Greyed out until *Fusion Graph* is enabled.
- **Magnetometer yaw** (`use_magnetometer`) — fuses tilt-compensated magnetometer yaw as a unary factor. Off by default — enable only after running mag calibration with motors-off.

Editable yaw / position fields for the dock (`dock_pose_x`, `dock_pose_y`, `dock_pose_yaw`) are intentionally **not** in the UI: those are auto-captured by `dock_yaw_to_set_pose` into `dock_calibration.yaml` on every dock arrival, and surfacing them as editable was misleading.

### Diagnostics: Fusion Graph panel

When `use_fusion_graph:=true`, the Diagnostics page surfaces a dedicated **Fusion Graph (iSAM2)** card showing:

- **Nodes in graph** — `total_nodes` from `/fusion_graph/diagnostics`, with the count of nodes that have a stored scan attached.
- **Loop closures** — successful loop-closure factors added since boot.
- **ICP success rate** — `scan_matches_ok / (ok + fail)` over the session.
- **Pose σ** — `√((cov_xx + cov_yy)/2)` in centimetres, with the yaw σ in degrees underneath. Colour-coded green / amber / red.
- **Save graph** / **Clear graph** buttons — call the corresponding `~/save_graph` / `~/clear_graph` services on `fusion_graph_node`. Save persists the graph to `/ros2_ws/maps/fusion_graph.{graph,scans,meta}`; Clear wipes iSAM2 and waits for the next pose seed to re-initialize.

The card itself is hidden when `use_fusion_graph:=false` (i.e., when the legacy EKF is the active map-frame localizer) — there's nothing useful to show.

## Design System

- **Font:** Manrope (headings/body), JetBrains Mono (telemetry values)
- **Color palette:** Green-tinted dark theme (`#0F1210` base, `#3EE084` accent, `#7BC6FF` sky, `#FFC567` amber), light theme available
- **Cards:** 18px border-radius, 1px subtle border, panel surface color
- **Icons:** Custom stroke-1.6 SVG icon set (mower, battery, signal, blades, thermometer, etc.)
- **Animations:** State pill pulse, boundary violation glow (respects `prefers-reduced-motion`)

## Architecture

- **Frontend:** React (TypeScript) + Ant Design + styled-components
- **Backend:** Go, connects to ROS2 via foxglove_bridge (`ws://localhost:8765`) using the foxglove WebSocket protocol
- **Real-time data:** WebSocket streams for status, power, GPS, emergency, map updates
- **State management:** Custom hooks (`useHighLevelStatus`, `usePower`, `useStatus`, `useGPS`, `useEmergency`, `useSettings`)

## Development

```bash
cd gui

# Backend
go build -o openmower-gui
./openmower-gui

# Frontend
cd web
yarn install
yarn dev    # http://localhost:5173 with hot reload
```

## Docker

```bash
docker build -t openmower-gui gui/
```

## Configuration

The GUI reads and writes `docker/config/mowgli/mowgli_robot.yaml`. Changes take effect after restarting the `mowgli` container.
