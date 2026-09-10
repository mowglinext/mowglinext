# MowgliNext User Guide

This guide walks you through the GUI of a live MowgliNext robot mower (the web interface served on port `4006` of the robot). It is built from a real-robot session — the screenshots in `docs/gui-walkthrough/screenshots/` were captured against a running mower with RTK-Fixed GPS, so the values you see (89% battery, 13,092 fusion-graph nodes, etc.) are real.

If you only need the technical reference, head to the [Wiki](https://github.com/mowglinext/mowglinext/wiki) — it documents the ROS2 stack in depth. **This guide is for operators**, not roboticists.

---

## Table of contents

1. [Quickstart](#1-quickstart) — first time the robot powers on
2. [GUI tour](#2-gui-tour) — page by page
3. [Configuring the robot](#3-configuring-the-robot) — what every setting does and when to retune
4. [Troubleshooting](#4-troubleshooting) — RTK isn't Fixed, robot drifts, dock is misaligned

---

## 1. Quickstart

> **Reference:** [`docs/FIRST_BOOT.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/FIRST_BOOT.md) is the authoritative low-level checklist. This section is the operator-friendly version.

### Hardware prerequisites

- Robot built and wired (motor, blade, IMU, GPS receiver, optional LiDAR).
- STM32 firmware flashed with the Mowgli build (the Onboarding wizard can flash it for you — see [Step 4](#step-4--firmware) below).
- GPS antenna with a clear sky view. RTK-Fixed needs ≤ ~20° of obstruction, ideally none.
- A NTRIP correction source (free networks: Centipede in France, SAPOS in Germany, RTK2GO worldwide; or your own base station).
- Dock with charger.

### First power-on

1. Power the dock and place the robot on it. Wait for the green charging LED.
2. From any device on the same network, open `http://<robot-ip>:4006`. The IP is printed by the install script and is also visible from the dock's Wi-Fi access point label.
3. The GUI auto-redirects you to **Onboarding** the very first time (`gui/web/src/components/AppShell.tsx:97-110`). Follow the 9-step wizard (Welcome → Robot Model → Firmware → NTRIP → GPS → Datum → Sensors → Calibration → Complete) — see [§2 Onboarding](#23-onboarding-page).
4. After onboarding, ROS2 restarts so the new `mowgli_robot.yaml` is reloaded. You land on the Dashboard.

### What "ready to mow" actually requires

Even after the wizard finishes, several calibration artefacts must exist before the robot will mow well. The wizard does **not** produce all of them today (see the post-onboarding checklist in [`docs/FIRST_BOOT.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/FIRST_BOOT.md)). Concretely you need:

| Artefact | Where to confirm | How to produce it today |
|---|---|---|
| `mowgli_robot.yaml` exists with `datum_lat/lon` set | Settings → GPS & Positioning, lat/lon non-zero | Onboarding **Datum** step, **OR** Settings → GPS → "Use current GPS position" |
| `imu_calibration.txt` (gyro/accel bias) | Diagnostics → IMU bias calibration shows **Present** | Automatic — runs every time the robot returns to dock |
| `imu_yaw` solved + persisted to `mowgli_robot.yaml` | Sensors editor IMU Yaw value matches reality (typically 90° ± 90°) | Onboarding **Calibration** step, **OR** Diagnostics → "Run calibration" on IMU panel, **OR** the compass icon next to IMU Yaw in the Sensors editor |
| `dock_pose_x/y/yaw` non-zero in `mowgli_robot.yaml` | Diagnostics → Dock calibration shows **Present** with a yaw value | Settings → Docking → **Start dock calibration** (one-click: reverse, check COG heading, re-dock, persist); **OR** the Onboarding **Calibration** step run while charging; **OR** map editor → pin/environment icon "Set docking point" with the robot manually placed on the dock |
| At least one mowing area | Map page → Areas list shows ≥ 1 area | Map → "More" → "Area Recording", drive boundary, "Record finish" |
| RTK-Fixed (Fix type = `RTK FIX`) | Dashboard → GPS card says "RTK fix" / Diagnostics → GPS panel | Working NTRIP credentials + clear sky |

If any of these is missing, the robot may run but coverage will be drifty between sessions and docking will arrive misaligned. The Diagnostics page now exposes all three calibration artefacts (`gui/web/src/pages/DiagnosticsPage.tsx:1373-1606`) — make it a habit to glance at the page after onboarding. The wizard's last step also runs this list as a live readiness check (see [§2.3 Onboarding](#23-onboarding-page)).

---

## 2. GUI tour

The desktop layout uses a left rail (`AppShell` in `gui/web/src/components/AppShell.tsx`); on mobile the left rail collapses and a bottom tab bar exposes Home / Map / Schedule / Diag plus a **More** sheet for everything else (`AppShell.tsx:46-56`). Available routes (declared at `gui/web/src/main.tsx:26-73`):

`/mowglinext` Dashboard · `/map` Map · `/schedule` Schedule · `/onboarding` Onboarding · `/settings` Settings · `/parameters` Parameters (live ROS2 tuning) · `/logs` Logs · `/diagnostics` Diagnostics · `/statistics` Statistics

### 2.1 Dashboard

![Dashboard overview](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/dashboard/01-overview.png)

**What you can do here**

- See the robot's current high-level state (Idle, Autonomous, Recording, Manual Mowing) — the pill in the top-right reflects `HighLevelStatus.msg` values from `CLAUDE.md`.
- Read live battery %, GPS quality, blade state, motor temperature.
- Glance at "Today's work" (zone progress), "Next up" (pulled from the schedule), and "Health check" (RTK status, rain, emergency, motor temp).
- Tap **Start mowing** to begin. While mowing, the primary button becomes **Pause** — a true stop-in-place hold (blade off, halt where it stands, no dock trip) via `COMMAND_STOP` (8); tap the same button (back to ▶) to resume. **Home** — the right-hand secondary — drives the robot back to the dock (`COMMAND_HOME`); the left-hand ⚠️ secondary *triggers* an emergency stop (with a confirmation). When an emergency is latched the primary button turns into a **Re-arm** button that clears it.
- The **IrriSense** banner appears above the layout when the soil-moisture provider says your garden is still wet (see [§3.4](#34-schedule-blade-rain)).
- Expand **System Info** and **Sensors & Diagnostics** at the bottom to see the live IMU / GPS / wheel-tick stream.

![Dashboard with sensors expanded](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/dashboard/03-sensors-detail.png)

**Common tasks**

- *Start a mowing run:* tap **Start mowing**. Behavior tree clears the emergency latch (if needed), undocks via Nav2 BackUp, and iterates through every recorded area. See `CLAUDE.md` invariant 7 ("Multi-area coverage = explicit segments from `plan_coverage`").
- *Reset emergency:* while an emergency is latched the big primary button becomes **Re-arm**; it calls `/hardware_bridge/emergency_stop` with `Emergency = 0`. The firmware is the safety authority; it will only un-latch if the physical trigger is no longer asserted.
- *Verify RTK before mowing:* Sensors & Diagnostics → GPS → "Fix type". Should read **RTK FIX**, with **Accuracy** below 0.05 m on a good day.

**Mobile**

![Dashboard on mobile](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/mobile/01-dashboard.png)

The hero card and KPI tiles stack vertically; bottom tabs replace the sidebar.

### 2.2 Map page

![Map overview](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/map/01-map-overview.png)

The map is a Mapbox satellite layer with the robot icon, dock marker (`DOCK`), and the mowing-area polygons (`Area 1`, `Area 2`) drawn in green over the user's actual yard. The right-hand panel lists every area and exposes the **Map Offset** controls (X/Y nudge in metres, applied client-side for visual alignment when the satellite imagery is mis-georeferenced).

**Bottom toolbar (read-only mode)** — buttons defined in `gui/web/src/pages/map/components/MapToolbar.tsx`:

| Button | Action |
|---|---|
| **Edit Map** | Enter polygon edit mode (see below) |
| **Start** | Begin mowing (same as Dashboard's button; shown only while idle) |
| **Home** | Send the robot back to the dock (`COMMAND_HOME`) |
| **Emergency On / Off** | Toggle latched emergency |
| **Mow area** | Start mowing one area — a dropdown of the recorded areas |
| **Manual Mow / Stop Manual** | Enter or leave manual mowing |
| **More** | Opens a dropdown — see screenshot below |

While an area recording is in progress, **Start** / **Home** are replaced by **Finish recording** and **Cancel recording**.

![More menu](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/map/02-more-menu.png)

The **More** menu contains everything that doesn't fit on the bottom bar:

- *Dark map / Satellite* — toggle Mapbox satellite vs dark style; *Tilt 3D view / Flatten map* pitches the camera.
- *Area Recording* — sends `COMMAND_RECORD_AREA` (3). Drive the robot along the boundary; finish via the same menu when you're done.
- *Mow Next Area* — advances the multi-area outer loop without restarting.
- *Pause / Stop* — sends `COMMAND_STOP` (8): a true **stop-in-place hold** (blade off, halt where it stands, stays put — it does **not** drive to the dock, that is *Home*/`COMMAND_HOME`). Resumable.
- *Continue* — resumes a paused run (re-issues `COMMAND_START`, picking up from the persisted mow-progress).
- *Manual Mowing* — `COMMAND_MANUAL_MOW` (7); blade managed from the GUI, motion via teleop. **Note:** teleop publishes to `/cmd_vel_teleop`, which enters twist_mux at priority 20 — above the navigation lane, and it never passes through `collision_monitor` (that only filters the navigation lane, `nav2_params_base.yaml:1101-1104`). The LiDAR stop-zones therefore do **not** filter your joystick. The STM32 firmware remains the sole blade-safety authority (`CLAUDE.md` § Safety); drive carefully.
- *Blade Forward / Backward / Off* — fire-and-forget commands; firmware decides whether to honour them.
- *Backup Map / Restore Map / Import from OpenMower / Download GeoJSON* — full-map import/export.
- *Reset mowing progress* — clears the saved coverage progress for the current map (your areas are **not** deleted) so the next run re-mows everything.

**Edit mode**

![Map in edit mode](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/map/03-edit-mode.png)

Click **Edit Map** and the bottom bar is replaced by a vertical toolbar with: save / close / undo / redo / draw rectangle (border) / draw polygon (plus) / delete / merge cells / minus-square (subtract) / split-cells / form (open `EditAreaModal` for the selected area) / aim (centre on robot) / **environment pin (Set docking point)** / rotation degree input.

> **Important:** The "environment pin" button writes the robot's *current* `map`-frame pose into `mowgli_robot.yaml` as the new `dock_pose_x/y/yaw`. This calls `/map_server_node/set_docking_point` (see `gui/pkg/api/mowglinext.go:238`). The service **refuses** unless the firmware reports charging, the GPS sample is fresh and RTK-accurate, and the fused yaw has converged (`ros2/src/mowgli_map/src/area_manager.cpp:640-770`) — so use it with the robot physically sitting on the dock, contacts engaged, and RTK-Fixed.

If you try to leave edit mode with unsaved changes you get a confirmation:

![Discard unsaved changes](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/map/04-save-confirmation-modal.png)

**Common tasks**

- *Record a mowing area:* dock the robot, **More → Area Recording**, drive it along the boundary at moderate speed (joystick appears in the Foxglove pane). Finish via **More → Area Recording** again — the trajectory is Douglas-Peucker-simplified and saved via `/map_server_node/add_area`. To cancel, send `COMMAND_RECORD_CANCEL` (6).
- *Set the dock pose precisely:* prefer **Settings → Docking → Start dock calibration** (one-click; it also solves the dock yaw). To pin the pose by hand instead, place the robot on the dock (chargers contacting), wait for RTK-Fixed, then **Edit Map → environment pin → confirm**.
- *Edit an existing polygon:* **Edit Map**, click an area, drag vertices; tap the form icon to rename / change type (work area / navigation / obstacle) / change mowing order.
- *Reorder mowing sequence:* in the right-hand Areas list, use the up/down arrows (visible in `EditAreaModal` mode).
- *Make a detected obstacle permanent:* while the LiDAR obstacle tracker is publishing, the right-hand panel grows a **Tracked obstacles** list (hovering a row highlights the polygon on the map). **Promote** turns that transient observation into a permanent keepout inside the containing area via `/map_server_node/promote_obstacle`; it is disabled for an obstacle that falls outside every mowing area, because there is nothing to attach it to. Nothing is auto-promoted — promotion is always your call.
- *After the robot digs a hole:* if the wheels spin without the GNSS-anchored pose moving, the hardware bridge hard-stops, reverses out, and `map_server_node` stamps a **pending** 0.60 m square keepout on the spot inside the containing area (`ros2/src/mowgli_map/src/area_manager.cpp:1120-1200`). It shows up as an obstacle for the rest of the session so coverage routes around it, but it is **not** written to `areas.dat` — restart the stack and it is gone unless you promote it. A dig outside every mowing area is only logged.

### 2.3 Onboarding page

The wizard (`gui/web/src/pages/OnboardingPage.tsx`) now runs **9 steps**: Welcome → Robot Model → Firmware → NTRIP → GPS → Datum → Sensors → **Calibration** → Complete. GPS/datum/NTRIP were split into separate steps and IMU/dock calibration was promoted from a compass icon inside Sensors to its own **Calibration** step. The screenshots and sub-sections below predate that reorganization and are indicative of each panel's content rather than the exact step order:

#### Step 0 — Welcome
![Welcome](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/onboarding/01-welcome.png)
Three info cards summarise what the wizard will do.

#### Step 1 — Robot Model
![Robot Model](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/onboarding/02-robot-model.png)
Pick from YardForce Classic 500 / 500B / SA650 / 900 ECO / LUV1000RI, Sabo MOWiT 500F, or Custom. Selecting a preset auto-fills `wheel_radius`, `wheel_track`, `blade_radius`, battery thresholds, encoder ticks/rev (see `gui/web/src/constants/mowerModels.ts`).

#### Step 2 — GPS & Positioning
![GPS step](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/onboarding/03-gps.png)

Three panels:

1. **Map Origin (Datum)** — latitude/longitude. Either type a value (right-click on Google Maps over your dock and copy "lat, lon") or click **Use current GPS position** (calls `set_datum` → `/navsat_to_absolute_pose/set_datum`, written by `gui/pkg/api/mowglinext.go:391-397`). Requires the GPS to be in RTK-Fixed mode at the moment you click.
2. **GPS Receiver** — protocol (UBX/NMEA) and serial port (`/dev/gps` is the default udev symlink).
3. **NTRIP Corrections** — host, port, mountpoint, username, password.

> **Operator tip:** the dock should be physically positioned where you want the map origin. Stand the robot on the dock, wait for "RTK FIX" in the Diagnostics page, and only *then* click **Use current GPS position**.

#### Step 3 — Sensors
![Sensor placement](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/onboarding/04-sensors.png)

Visual robot editor (`gui/web/src/components/RobotComponentEditor.tsx`) with drag-to-place LiDAR / IMU / GPS markers on a top-down rectangle representing your chassis. Numeric inputs on the right side give precision for X (forward), Y (left), Z (height), Yaw.

> **Critical:** IMU/dock calibration is now its own **Calibration** step (after Sensors), not a compass icon buried in the Sensors editor. Its **Start IMU Yaw Calibration** button triggers `POST /api/calibration/imu-yaw` (`gui/pkg/api/calibration.go`). The robot drives a short distance forward then back — do **not** start it indoors, on a slope, or near furniture. The service blocks for up to 150 s and writes `imu_yaw` (and pitch/roll if the stationary baseline is good enough) into `mowgli_robot.yaml`; if the robot is on the dock and charging it also captures the dock pose. The same compass-icon control still exists next to IMU Yaw in Settings → Sensors for re-runs.

#### Step 4 — Firmware
![Firmware step](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/onboarding/05-firmware.png)

If you skipped this earlier, you can flash the STM32 from here. The flash UI (`FlashBoardComponent.tsx`) lets you pick the board variant, repository, branch, panel layout, and debug type:

![Flash board](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/onboarding/06-flash-firmware.png)

If your firmware is already up-to-date, click **Skip — Already Flashed**.

#### Step 5 — Complete
The final step is a live **readiness check** (`gui/web/src/components/onboarding/ReadinessStep.tsx`, rules in `readinessChecks.ts`). It runs the same list as the [ready-to-mow table](#what-ready-to-mow-actually-requires) against the live robot — RTK fix, NTRIP corrections, datum, localizer alive, localizer confidence, firmware compatibility, IMU yaw, magnetometer, at least one mowing area — and marks each **pass / pending / fail**, with a button that deep-links back to the wizard step (or to Map / Diagnostics) that fixes it. Required checks that are not passing **gate** the Finish button: you can still choose *Finish anyway*, but only through a confirmation that names what is missing. Finishing marks `onboarding_completed=true` in the GUI's SQLite DB (`POST /api/settings/status`) and then triggers `restartRos2()` + `restartGui()` so the new YAML is reloaded.

> **Still not produced by the wizard:** magnetometer calibration and drive/feed-forward tuning. Both are flagged by the readiness check (mag as a *recommended*, not required, item) but you have to run them yourself — see the post-onboarding checklist in [`docs/FIRST_BOOT.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/FIRST_BOOT.md).

### 2.4 Settings page

The Settings page (`gui/web/src/pages/SettingsPage.tsx`) groups parameters into 17 sections listed down the left (`gui/web/src/hooks/useSettingsManager.ts:48-250`). Hardware is the default landing section. The screenshot column below only covers the sections that existed when the walkthrough was captured.

| Section | Screenshot | Covers |
|---|---|---|
| Appearance | — | Visual / Balanced / Efficient display mode and timestamp format (per device, not per robot) |
| Hardware | `settings/01-settings-overview.png` | Robot model picker + wheel/blade dimensions + chassis geometry + `tool_width` |
| Drive Motor | — | Firmware wheel-velocity PID gains and feed-forward (applied live) |
| NTRIP Corrections | — | RTK correction network / base station — set this **before** GPS |
| GPS & Positioning | `settings/02-gps-positioning.png` | Datum lat/lon, GNSS receiver family, serial device / baud, signal profile |
| Sensors | `settings/03-sensors.png` | LiDAR enable, sensor placement (drag editor), IMU calibration cadence |
| Localization | `settings/04-localization.png` | LiDAR scan matching, loop closure, magnetometer yaw + mag calibration/tuning |
| Mowing | `settings/05-mowing.png` | Speeds, headland width / passes, swath overlap, safety inset, turning radius, mow angle & direction |
| Docking | `settings/06-docking.png` | One-click dock calibration, undock distance / speed, approach distance, max retries, charger detection |
| Battery | `settings/07-battery.png` | Full / empty / critical voltage, percentage thresholds for resume / dock |
| Safety | `settings/08-safety.png` | **Read-only.** Lift, tilt, the emergency latch and the blade cut-out all live in the STM32 firmware, so there is nothing to edit — the panel just says so |
| Obstacles | — | Avoidance margins, drawn-obstacle buffers, detection range, wait timeout, approach slowdown |
| Navigation | `settings/09-navigation.png` | Goal tolerances (transit XY, yaw, coverage XY), progress timeout |
| Rain | `settings/10-rain.png` | Behaviour (Ignore / Dock / Dock Until Dry / Pause Auto), resume delay, debounce |
| Status LEDs | — | WS2812 status ring: enable, LED count, brightness, refresh, low-battery / charge-full thresholds |
| IrriSense | — | Connect your IrriSense Cloud garden and skip scheduled mows while the soil is wet |
| Advanced | `settings/11-advanced.png` | Raw key/value editor for `mowgli_robot.yaml` parameters not covered elsewhere |

Live ROS2 parameters (as opposed to YAML settings) are edited on the separate **Parameters** page, which groups them by basic / middle / expert tier and flags the motion- and battery-affecting ones.

Each section persists changes to the GUI's settings store; the **Restart ROS2** button at the bottom-right (visible in every section) reloads the ROS2 container so YAML-backed parameters are picked up.

**Overridden dot + reset to default.** The installed `mowgli_robot.yaml` is [sparse](https://github.com/mowglinext/mowglinext/wiki/Configuration#sparse-robot-config-model) — it stores only values that differ from the shipped defaults. Any field you have pinned to a non-default value shows a small **dot** before its label; a subtle **reset (undo) button** next to it reverts the field to its default. Reset deletes the key from the installed config so it falls back to the in-package template, and the backend prunes any saved value that already equals its default — so the installed file stays sparse and a later software update that changes a default automatically flows through to fields you never overrode.

The **Localization** tab is worth a closer look:

![Localization tab](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/settings/04-localization.png)

There is **no "use fusion graph" switch**: `fusion_graph_node` (GTSAM iSAM2) is the one and only map-frame localizer and it also publishes the local `odom → base_footprint` transform, so the page opens with an info banner saying exactly that. What you toggle here are the *optional factors it can add* (`gui/web/src/components/settings/LocalizationSection.tsx:32-52`):

- **LiDAR — obstacle avoidance** — not a toggle, just a status card with a live badge. The Nav2 obstacle layer and `collision_monitor` consume `/scan` whenever the LiDAR driver runs; you turn the driver itself on or off under **Sensors → lidar_enabled**.
- **LiDAR scan matching** (`use_scan_matching`) — adds per-tick ICP between-factors. **On by default**, but the launch file ANDs it with `use_lidar`, so on a GPS-only robot it is silently inert.
- **Loop closure** (`use_loop_closure`) — searches earlier graph nodes within 5 m that are at least 30 s old and snaps the trajectory back when one matches. Also **on by default** and also ANDed with `use_lidar`. Rate-limited (one accepted closure per node, and none until 2 s and 1 m of travel have passed) and skipped entirely while RTK is Fixed.
- **Magnetometer yaw** (`use_magnetometer`) — fuses tilt-compensated mag yaw as a unary factor. **Off by default**, the in-app help text says: *"motor-induced bias makes the magnetometer unreliable on most chassis. Enable only after running mag calibration with motors-off and validating a stable |B|."*
- Below those, a **Magnetometer calibration & tuning** card: a *Collect calibration samples* switch (`enable_mag_cal`, off by default — turn it on with the motors off, then turn it back off) plus magnetic declination (1.5° default), minimum horizontal field and mag-yaw variance.

Restart ROS2 from the page footer after changing any of these.

### 2.5 Diagnostics page

This is the most information-dense page in the app. It is a near-superset of what the Wiki [Architecture](https://github.com/mowglinext/mowglinext/wiki/Architecture) describes, but rendered live.

![Diagnostics top](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/diagnostics/01-overview.png)

**Top of page:** Health pills (Containers OK, GPS: RTK FIX, Battery 88%, No Emergency, CPU 53.6 °C), Alerts panel, Containers table (mowgli-ros2 / mowgli-gui / mowgli-gps / mowgli-lidar / mowgli-mqtt with state + uptime), CPU temperature card.

![Pose + GPS + Fusion Graph](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/diagnostics/02-pose-gps.png)

**Filtered Pose / GPS / Fusion Graph (iSAM2) / Heading sources / BT State / Coverage**:

- *Fusion Graph (iSAM2)*: nodes in graph (13,092 in our session), loop closures (491), ICP success rate (100% / 15540 of 15573), **Pose σ** (0.0 cm — yaw ±1.15°). Two action buttons: **Save graph** (calls `~/save_graph` `Trigger`, which writes `<graph_save_prefix>.{graph,scans,meta}`) and **Clear graph** (`~/clear_graph`, which drops the in-memory graph, its keyframes and the dead-reckoning frame and waits for re-initialisation — use it after relocating the robot to a new garden).
- *Heading sources*: side-by-side comparison of the active filter yaw, GPS course-over-ground (`/imu/cog_heading`), and magnetometer yaw (`/imu/mag_yaw`). When mag is "Stale" the unary factor is not being fused.
- *Coverage*: per-area progress (cells mowed / total cells, obstacles, strips left).

![Calibration panels](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/diagnostics/03-bt-coverage-network.png)

**Configuration cross-checks + three calibration panels** (`DiagnosticsPage.tsx:1282-1606`):

- *Configuration cross-checks*: the warning bar across the top — checks that the dock pose isn't all-zero, that the datum lat/lon is set, etc.
- *Dock calibration* — Present/Missing tag, `dock_pose_x/y/yaw_rad` from `mowgli_robot.yaml`. **Run calibration** kicks off the IMU yaw calibration service (which includes a dock pre-phase if the robot is charging).
- *IMU bias calibration* — read from `/ros2_ws/maps/imu_calibration.txt`. Shows calibrated-at timestamp, sample count (1000 in the live session), gyro bias vector, and implied pitch/roll. The hardware bridge auto-runs this every time the robot returns to the dock.
- *Magnetometer calibration* — read from `/ros2_ws/maps/mag_calibration.yaml`. Shows |B| mean / std / sample count. **Enable & run** is now a real one-click action: it asks for confirmation, then `POST /api/calibration/magnetometer` makes the robot **drive a figure-8** (0.20 m/s around a 0.60 m radius, ~1.5 loops per side) to collect hard/soft-iron samples — so give it a couple of clear metres, not a corner. Note the in-app confirmation still says "rotate in place"; the motion is the figure-8. Both calibration buttons share one busy flag, so a second drive command cannot fire while one is running.

![ROS Diagnostics](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/diagnostics/04-imu-wheel-bottom.png)

**Hardware Status + ROS Diagnostics**: the bottom is the canonical `diagnostic_msgs/DiagnosticArray` view. Click any row to expand the per-key/value detail. The localizer is `fusion_graph` (the sole map+odom localizer — the old `ekf_map_node`/`ekf_odom_node` dual-EKF was removed, and there is no `use_fusion_graph` toggle); expect `fusion_graph` diagnostics here rather than any `ekf_*` rows.

### 2.6 Schedule

![Schedule](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/schedule/01-overview.png)

Weekly grid (Mon–Sun, 6:00–19:00). Bottom panels show:
- **This week** (count of active schedules + per-day chips).
- **Schedules** list + **+ New run** button. With no schedules yet you get a set of starter templates to pick from.
- **Rules**: Rain-aware toggle, Auto-dock low (return at <20% battery).

An **IrriSense** chip sits under the page title when the soil-moisture integration is configured. Each schedule card also shows its last run and, when a run was skipped, *why* — a wet-soil skip is reported there.

Empty in our test session — schedules are user-defined.

### 2.7 Logs

![Logs](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/logs/01-overview.png)

Live tail of any selected container's stdout/stderr. Picker defaults to `mowgli-ros2`; **Restart** / **Stop** buttons at the top-right are container-level controls.

### 2.8 Statistics

![Statistics](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/statistics/01-overview.png)

Lifetime KPIs: Total Distance, Hours Active, Completion Rate, Runs Completed. Below: distance-per-week chart (12-week window), per-zone coverage (cells mowed / total), session history table with date / duration / area / coverage / status.

> Note: in our live session there are 242 sessions all marked "error" with 0 m distance — this is a known artefact when sessions never reach autonomous mode (e.g. emergency held, RTK never fixed). The Statistics page does not yet filter these out.

---

## 3. Configuring the robot

### 3.1 Localization (the "is the robot lost?" stack)

Localization is the part the user feels is most under-explained today. Here is the canonical truth, sourced from `CLAUDE.md` § "Architecture Invariants" and the GUI source.

**One localizer, not a choice.** `fusion_graph_node` — a GTSAM iSAM2 factor graph — is launched unconditionally and owns **both** the `map → odom` and the `odom → base_footprint` transforms, plus `/odometry/filtered_map`. There is nothing to pick and no fallback: the robot_localization dual EKF (`ekf_map_node` + `ekf_odom_node`) that older versions of this guide described was removed, and so was the `use_fusion_graph` switch. Its inputs are the GPS pose (with the antenna lever arm), the wheels, the gyro, GPS course-over-ground yaw and — when you enable it — magnetometer yaw. See [Architecture → Factor-graph localizer](https://github.com/mowglinext/mowglinext/wiki/Architecture#optional-factor-graph-localizer-fusion_graph) for the internals.

**The calibration artefacts the localizer needs:**

| Artefact | Stored in | Written by | When to retune |
|---|---|---|---|
| `datum_lat`, `datum_lon` | `mowgli_robot.yaml` | The onboarding **Datum** step ("Use current GPS position"), or Settings → GPS & Positioning → the same button | Once at install. Re-run if you physically move the dock. |
| `imu_yaw` (mounting yaw) | `mowgli_robot.yaml` | Diagnostics → Run calibration / Sensors editor → compass icon | Once at install. Re-run if you re-mount the IMU. |
| `imu_pitch`, `imu_roll` (mounting tilt) | `mowgli_robot.yaml` | Same calibration service, only persisted when `stationary_samples_used ≥ 150` (`gui/web/src/hooks/useImuYawCalibration.ts:132-137`) | Once at install. Re-run if you re-mount the IMU or move it relative to the chassis. |
| `dock_pose_x/y/yaw` | `mowgli_robot.yaml` | (a) Settings → Docking → **Start dock calibration** (one-click), (b) the onboarding Calibration step when started while docked, **OR** (c) map editor → "Set docking point" with robot manually positioned on dock | Once at install. Re-run if you physically move the dock. |
| `imu_calibration.txt` (gyro/accel bias) | `/ros2_ws/maps/imu_calibration.txt` | Auto, every dock arrival, `hardware_bridge_node` | Continuous. Look at Diagnostics → IMU bias panel; if `Implied pitch/roll` is > ~1° you should bake those into `imu_pitch/roll` (see FIRST_BOOT.md §3). |
| `mag_calibration.yaml` (magnetometer) | `/ros2_ws/maps/mag_calibration.yaml` | `calibrate_imu_yaw_node`'s figure-8 phase, run on its own from Diagnostics → Magnetometer calibration → **Enable & run**. Never part of an ordinary IMU-yaw calibration | Optional — only needed if you toggle Magnetometer yaw on. |

> The `mowgli_robot.yaml` values above are exactly the per-robot calibration outputs that the **sparse** installed config is meant to carry (alongside the install-time datum/NTRIP/hardware choices). Everything else is a template default — see [Configuration → Sparse robot config model](https://github.com/mowglinext/mowglinext/wiki/Configuration#sparse-robot-config-model).

**What the LiDAR factors change:**

- No LiDAR (or `lidar_enabled` off) → the scan-matching and loop-closure switches are inert whatever they say, and the graph runs on GPS + wheels + gyro + COG yaw alone. It tolerates RTK-Float windows of seconds, not minutes.
- LiDAR mounted, both switches on (the default) → the robot rides through multi-minute RTK-Float windows on scan matching, and loop closures pull accumulated drift back onto previously-mapped ground mid-session.
- Watch the *ICP success rate* on Diagnostics → Fusion Graph: a low rate means the LiDAR mount or `lidar_yaw` is wrong, not that the graph is broken.

### 3.2 Sensors

The drag-to-place editor in Settings → Sensors (or the onboarding **Sensors** step) writes:

- `lidar_x/y/z/yaw` — LiDAR mount, in metres + radians, base_link frame.
- `imu_x/y/z/yaw` (and `imu_pitch/imu_roll` written by calibration) — IMU mount.
- `gps_x/y/z` — GPS antenna mount (no yaw — antenna is point-symmetric). This is the lever arm the localizer rotates with the robot's heading, so measure it carefully.

The robot rectangle uses dimensions from `/robot_description` (URDF). You can verify what the Nav2 stack thinks your robot looks like by comparing the rectangle to your physical chassis.

### 3.3 Navigation tuning

The Settings → Navigation panel exposes:

- **Transit XY Tolerance** (`xy_goal_tolerance`, default 0.10 m from the shipped template; `navigation.launch.py`'s own fallback, used only if the key is missing entirely, is 0.30 m) — the "we arrived" radius for transit, home and the dock approach. Those use the `FollowPath` slot, which is Nav2's RotationShim wrapping Regulated Pure Pursuit, checked by `StoppedGoalChecker` — *not* FTC, which only drives coverage paths.
- **Yaw Tolerance** (`yaw_goal_tolerance`, default 0.10 rad ≈ 5.7°) — final heading at those same goals. It is deliberately tight: with a looser value the robot used to enter the 1.5 m dock corridor "en biais" and miss the cradle.
- **Coverage XY Tolerance** (`coverage_xy_tolerance`, default 0.50 m) — the `PathProgressGoalChecker` xy tolerance for the end of a coverage path. **Do not tighten it.** FTC zeroes its forward speed as soon as it leaves FOLLOWING, so it parks up to `max_goal_distance_error` (0.50 m) short of the last pose; a tighter gate is never satisfied, the goal never succeeds, the progress checker fires "failed to make progress", and the behaviour tree re-mows the whole area. `navigation.launch.py` therefore **raises** any smaller value back to 0.50 m at launch and prints a warning. Completion is really gated on monotonic path-pose tracking ≥ 95 %, which is what guarantees the area was actually mowed — the loose radius costs nothing because the perpendicular swaths already cover that last half-metre.
- **Progress Timeout** (default 30 s, operator-tunable via `progress_timeout_sec`) — Nav2 fails the action if `PoseProgressChecker` hasn't seen 0.15 m of translation OR 0.5 rad of rotation in this window. The angle gate keeps headland pivots from tripping "no progress".

### 3.4 Schedule, blade, rain

These are operator-facing and self-explanatory in the UI:

- **Schedule** — add weekly recurrences with start time + duration. Rain-aware and auto-dock-low rules apply globally.
- **IrriSense** (settings) — optional. Point the GUI at your IrriSense Cloud garden with a read-only token, pick the zones that cover the lawn, and a **scheduled** run is skipped while those zones read wet. It fails open by design: not configured, the service unreachable, or a stale reading all count as "unknown" and the mow goes ahead. Manual **Start mowing** is never blocked — the Dashboard just shows a wet-soil banner.
- **Mowing** (settings) — `mowing_speed` / `transit_speed` (0.2 m/s default for both), `mow_angle_deg` (auto by default; flip the switch to force a fixed swath angle), `num_headland_passes` (concentric perimeter rings: negative = none, 0 = auto, >0 = exactly that many — 2 by default), `swath_overlap` (0.02 m: adjacent swaths deliberately overlap by this much) and `chassis_safety_inset` (0.20 m, how far inside the recorded boundary the outermost swath runs). `mowing_enabled` off is a dry run — the robot drives the whole path with the blade never spinning. `tool_width` (0.18 m, the single source for blade cut width and, minus `swath_overlap`, the swath spacing) lives one section up, under **Hardware**.
- **Battery** — voltage thresholds with hysteresis (Low Dock 20% / Resume Above 95%) — this is also enforced firmware-side.
- **Rain** — choose Dock / Dock Until Dry / Pause Auto / Ignore behaviour, plus debounce (10 s default) and resume delay (30 min default).

---

## 4. Troubleshooting

The Wiki [FAQ](https://github.com/mowglinext/mowglinext/wiki/FAQ) is the long version. Below are the ones the GUI directly surfaces.

### "RTK is not Fixed"

**Symptoms:** Dashboard GPS card says `RTK float` or `3D fix`, Diagnostics shows Accuracy > 0.05 m, Statistics sessions all error out.

**Diagnose:**

1. Diagnostics → ROS Diagnostics → expand `GPS`. Does it say `GPS fix OK lat=… lon=…` with a status code of 2 (RTK Fixed)?
2. Logs → `mowgli-gps` container — are RTCM messages flowing? Search for "rtcm" or "ntrip".
3. Settings → GPS & Positioning → confirm Host, Port, Mountpoint, Username, Password are set. Empty fields are a red flag (we observed an empty Host on the live unit).

**Fix:** correct NTRIP credentials, restart ROS2 from the Settings footer, wait 60 s, recheck. If RTCM rate is 0, investigate firewall/internet. If it's flowing but fix won't go to RTK Fixed, the antenna is occluded — move the robot.

### "Robot drifts in odom"

**Symptoms:** robot mows curved swaths instead of straight lines, or coverage misses small strips between adjacent runs.

**Diagnose:** Diagnostics → Heading sources panel. Compare Filter yaw, COG yaw, Magnetometer yaw. Large persistent disagreement (>5°) means yaw fusion is bad.

**Fix:**

1. Confirm IMU bias calibration is **Present** and `Implied pitch/roll` is < 1°. If larger, copy into `mowgli_robot.yaml` → `imu_pitch/imu_roll` and restart.
2. Confirm IMU yaw is calibrated (run from Diagnostics → IMU bias panel → **Run calibration**). After a fresh calibration, COG yaw and Filter yaw should agree once the robot is moving forward.
3. If you have a LiDAR, verify the ICP success rate on Diagnostics → Fusion Graph is > 95% — if it's lower, your LiDAR mount or the `lidar_yaw` setting is wrong.

### "Boundary violation on mow"

**Symptoms:** robot runs over the edge of a polygon, or mows outside a recorded area.

**Diagnose:** map view → check that the area's polygon actually covers what you intended. Coverage plans **inside** the recorded boundary: the outermost swath centreline sits `chassis_safety_inset` (0.20 m by default, Settings → Mowing) in from it, so a strip of that width along the edge is normally left uncut on purpose.

**Fix:** **Edit Map** → click the polygon → drag vertices. If the polygon is correct but the robot still wanders, the *yaw* is wrong (see "Robot drifts in odom" above) — straight-line FTCController error becomes yaw-driven.

### "Docking arrives misaligned"

**Symptoms:** robot stops 5–20 cm off-centre relative to the dock contacts; charger never engages.

**Diagnose:**

1. Diagnostics → Configuration cross-checks. Is `Dock pose` non-zero with Yaw value present? If Yaw is 0° but the dock physically faces a different direction, dock pose is wrong.
2. Diagnostics → Dock calibration card. Should show **Present** with the correct yaw.
3. Map view: does the `DOCK` marker arrow visually align with the physical dock?

**Fix:** the first thing to try is **Settings → Docking → Start dock calibration** with the robot on the dock and charging under RTK-Fixed: it reverses in a straight line, reads the GPS course heading, re-docks, and persists `dock_pose_x/y/yaw` — solving position *and* yaw. If you would rather pin it by hand, drive the robot onto the dock until it physically charges, then **Edit Map → environment pin → confirm**, which snaps the dock pose to the current robot pose. Either way, restart ROS2 so `hardware_bridge` and `map_server_node` reload the new params.

### "Emergency latched and won't clear"

**Symptoms:** Dashboard pill says "Emergency"; pressing the **Re-arm** button doesn't help.

**Diagnose:** the firmware is the safety authority — it will only clear the latch if the physical trigger is no longer asserted (`CLAUDE.md` § Safety). Likely causes: the tilt sensor still reads tilted, a wheel-lift sensor still reads off-ground, or a stop button is still held.

**Fix:** physically verify the robot is level, on the ground, and that neither stop button is pressed, then press **Re-arm** again. If the dock is detected as charging, the behaviour tree auto-resets on its own (`CLAUDE.md` invariant 9).

---

## Where to go next

- **Wiki — Architecture:** https://github.com/mowglinext/mowglinext/wiki/Architecture (deep dive into TF chain, fusion_graph, BT, coverage)
- **Wiki — Configuration:** https://github.com/mowglinext/mowglinext/wiki/Configuration (every YAML key)
- **Wiki — FAQ:** https://github.com/mowglinext/mowglinext/wiki/FAQ
- **First-boot checklist (incl. post-onboarding gaps):** [`docs/FIRST_BOOT.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/FIRST_BOOT.md)
