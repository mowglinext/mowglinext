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

> **Reference:** [`docs/FIRST_BOOT.md`](FIRST_BOOT.md) is the authoritative low-level checklist. This section is the operator-friendly version.

### Hardware prerequisites

- Robot built and wired (motor, blade, IMU, GPS receiver, optional LiDAR).
- STM32 firmware flashed with the Mowgli build (the Onboarding wizard can flash it for you — see [Step 4](#step-4--firmware) below).
- GPS antenna with a clear sky view. RTK-Fixed needs ≤ ~20° of obstruction, ideally none.
- A NTRIP correction source (free networks: Centipede in France, SAPOS in Germany, RTK2GO worldwide; or your own base station).
- Dock with charger.

### First power-on

1. Power the dock and place the robot on it. Wait for the green charging LED.
2. From any device on the same network, open `http://<robot-ip>:4006`. The IP is printed by the install script and is also visible from the dock's Wi-Fi access point label.
3. The GUI auto-redirects you to **Onboarding** the very first time (`gui/web/src/routes/root.tsx:73-79`). Follow the 5-step wizard — see [§2 Onboarding](#23-onboarding-page).
4. After onboarding, ROS2 restarts so the new `mowgli_robot.yaml` is reloaded. You land on the Dashboard.

### What "ready to mow" actually requires

Even after the wizard finishes, several calibration artefacts must exist before the robot will mow well. The wizard does **not** produce all of them today (this is the gap covered in [`ONBOARDING_IMPROVEMENTS.md`](ONBOARDING_IMPROVEMENTS.md)). Concretely you need:

| Artefact | Where to confirm | How to produce it today |
|---|---|---|
| `mowgli_robot.yaml` exists with `datum_lat/lon` set | Settings → GPS & Positioning, lat/lon non-zero | Onboarding step 2, **OR** Settings → GPS → "Use current GPS position" |
| `imu_calibration.txt` (gyro/accel bias) | Diagnostics → IMU bias calibration shows **Present** | Automatic — runs every time the robot returns to dock |
| `imu_yaw` solved + persisted to `mowgli_robot.yaml` | Sensors editor IMU Yaw value matches reality (typically 90° ± 90°) | Diagnostics → "Run calibration" on IMU panel, **OR** the compass icon next to IMU Yaw in the Sensors editor |
| `dock_pose_x/y/yaw` non-zero in `mowgli_robot.yaml` | Diagnostics → Dock calibration shows **Present** with a yaw value | Calibration drives 0.6 m + reverse during IMU yaw calibration; **OR** map editor → pin/environment icon "Set docking point" with the robot manually placed on the dock |
| At least one mowing area | Map page → Areas list shows ≥ 1 area | Map → "More" → "Area Recording", drive boundary, "Record finish" |
| RTK-Fixed (Fix type = `RTK FIX`) | Dashboard → GPS card says "RTK fix" / Diagnostics → GPS panel | Working NTRIP credentials + clear sky |

If any of these is missing, the robot may run but coverage will be drifty between sessions and docking will arrive misaligned. The Diagnostics page now exposes all three calibration artefacts (`gui/web/src/pages/DiagnosticsPage.tsx:917-1057`) — make it a habit to glance at the page after onboarding.

---

## 2. GUI tour

The desktop layout uses a left rail (`Root` in `gui/web/src/routes/root.tsx:268-435`); on mobile the left rail collapses into a slide-over and a bottom tab bar exposes Home / Map / Stats / Settings (`root.tsx:97-265`). Available routes (declared at `routes/root.tsx:21-30`):

`/mowglinext` Dashboard · `/map` Map · `/schedule` Schedule · `/onboarding` Onboarding · `/settings` Settings · `/logs` Logs · `/diagnostics` Diagnostics · `/statistics` Statistics

### 2.1 Dashboard

![Dashboard overview](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/dashboard/01-overview.png)

**What you can do here**

- See the robot's current high-level state (Idle, Autonomous, Recording, Manual Mowing) — the pill in the top-right reflects `HighLevelStatus.msg` values from `CLAUDE.md`.
- Read live battery %, GPS quality, blade state, motor temperature.
- Glance at "Today's work" (zone progress), "Next up" (pulled from the schedule), and "Health check" (RTK status, rain, emergency, motor temp).
- Tap **Start mowing** to begin. While mowing, the primary button becomes **Pause** — a true stop-in-place hold (blade off, halt where it stands, no dock trip) via `COMMAND_STOP` (8); tap **Continue** to resume. **Home** drives the robot back to the dock (`COMMAND_HOME`); the secondary ⚠️ button is also the emergency reset entry point.
- Expand **System Info** and **Sensors & Diagnostics** at the bottom to see the live IMU / GPS / wheel-tick stream.

![Dashboard with sensors expanded](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/dashboard/03-sensors-detail.png)

**Common tasks**

- *Start a mowing run:* tap **Start mowing**. Behavior tree clears the emergency latch (if needed), undocks via Nav2 BackUp, and iterates through every recorded area. See `CLAUDE.md` § "Cell-based multi-area strip coverage".
- *Reset emergency:* the small ⚠️ pill next to **Start mowing** sends `COMMAND_RESET_EMERGENCY` (254). The firmware is the safety authority; it will only un-latch if the physical trigger is no longer asserted.
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
| **Start** | Begin mowing (same as Dashboard's button) |
| **Emergency On / Off** | Toggle latched emergency |
| **Mow area** | Start mowing the currently-selected area only |
| **More** | Opens a dropdown — see screenshot below |

![More menu](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/map/02-more-menu.png)

The **More** menu contains everything that doesn't fit on the bottom bar:

- *Dark map* — toggle Mapbox satellite vs dark style.
- *Area Recording* — sends `COMMAND_RECORD_AREA` (3). Drive the robot along the boundary; finish via the same menu when you're done.
- *Mow Next Area* — advances the multi-area outer loop without restarting.
- *Pause / Stop* — sends `COMMAND_STOP` (8): a true **stop-in-place hold** (blade off, halt where it stands, stays put — it does **not** drive to the dock, that is *Home*/`COMMAND_HOME`). Resumable.
- *Continue* — resumes a paused run (re-issues `COMMAND_START`, picking up from the persisted mow-progress).
- *Manual Mowing* — `COMMAND_MANUAL_MOW` (7); blade managed from the GUI, motion via teleop. **Collision monitor stays active** (`CLAUDE.md` invariant 9).
- *Blade Forward / Backward / Off* — fire-and-forget commands; firmware decides whether to honour them.
- *Backup Map / Restore Map / Download GeoJSON* — full-map import/export.

**Edit mode**

![Map in edit mode](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/map/03-edit-mode.png)

Click **Edit Map** and the bottom bar is replaced by a vertical toolbar with: save / close / undo / redo / draw rectangle (border) / draw polygon (plus) / delete / merge cells / minus-square (subtract) / split-cells / form (open `EditAreaModal` for the selected area) / aim (centre on robot) / **environment pin (Set docking point)** / rotation degree input.

> **Important:** The "environment pin" button writes the robot's *current* `map`-frame pose into `mowgli_robot.yaml` as the new `dock_pose_x/y/yaw`. This calls `/map_server_node/set_docking_point` (see `gui/pkg/api/mowglinext.go:182`). Use it when the robot is physically sitting on the dock and properly aligned.

If you try to leave edit mode with unsaved changes you get a confirmation:

![Discard unsaved changes](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/map/04-save-confirmation-modal.png)

**Common tasks**

- *Record a mowing area:* dock the robot, **More → Area Recording**, drive it along the boundary at moderate speed (joystick appears in the Foxglove pane). Finish via **More → Area Recording** again — the trajectory is Douglas-Peucker-simplified and saved via `/map_server_node/add_area`. To cancel, send `COMMAND_RECORD_CANCEL` (6).
- *Set the dock pose precisely:* place the robot on the dock by hand (chargers contacting), wait for RTK-Fixed, **Edit Map → environment pin → confirm**.
- *Edit an existing polygon:* **Edit Map**, click an area, drag vertices; tap the form icon to rename / change type (work area / navigation / obstacle) / change mowing order.
- *Reorder mowing sequence:* in the right-hand Areas list, use the up/down arrows (visible in `EditAreaModal` mode).

### 2.3 Onboarding page

The 5-step wizard (`gui/web/src/pages/OnboardingPage.tsx:551-654`). The sequence is:

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

> **Critical:** the small **compass icon** next to **IMU Yaw** triggers `POST /api/calibration/imu-yaw` (`gui/pkg/api/calibration.go:63-122`). The robot drives ~0.6 m forward then back — do **not** click it indoors, on a slope, or near furniture. The service blocks for up to 150 s and writes `imu_yaw` (and pitch/roll if the stationary baseline is good enough) directly into `mowgli_robot.yaml`.

#### Step 4 — Firmware
![Firmware step](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/onboarding/05-firmware.png)

If you skipped this earlier, you can flash the STM32 from here. The flash UI (`FlashBoardComponent.tsx`) lets you pick the board variant, repository, branch, panel layout, and debug type:

![Flash board](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/onboarding/06-flash-firmware.png)

If your firmware is already up-to-date, click **Skip — Already Flashed**.

#### Step 5 — Complete
The CompleteStep (lines 457-528) marks `onboarding_completed=true` in the GUI's SQLite DB (`POST /api/settings/status`) and then triggers `restartRos2()` + `restartGui()` so the new YAML is reloaded.

> **Gap (not produced by the wizard):** dock pose, IMU yaw calibration result (unless the operator triggered it manually in step 3), magnetometer calibration, RTK-Fixed verification. See [`ONBOARDING_IMPROVEMENTS.md`](ONBOARDING_IMPROVEMENTS.md).

### 2.4 Settings page

The Settings page (`gui/web/src/pages/SettingsPage.tsx`) groups parameters into 11 tabs. Hardware is the default landing tab.

| Tab | Screenshot | Covers |
|---|---|---|
| Hardware | `settings/01-settings-overview.png` | Robot model picker + wheel/blade dimensions + chassis geometry |
| GPS & Positioning | `settings/02-gps-positioning.png` | Datum lat/lon, GPS protocol, NTRIP credentials |
| Sensors | `settings/03-sensors.png` | LiDAR enable, sensor placement (drag editor) |
| Localization | `settings/04-localization.png` | `use_fusion_graph`, scan matching, loop closure, magnetometer yaw |
| Mowing | `settings/05-mowing.png` | Speed, path spacing, mow angle, headland width, outline passes |
| Docking | `settings/06-docking.png` | Undock distance / speed, approach distance, max retries, charger detection |
| Battery | `settings/07-battery.png` | Full / empty / critical voltage, percentage thresholds for resume / dock |
| Safety | `settings/08-safety.png` | Tilt detection, lift detection, motor temperature stop/resume, max detour distance |
| Navigation | `settings/09-navigation.png` | Goal tolerances (transit XY, yaw, coverage XY), progress timeout |
| Rain | `settings/10-rain.png` | Behaviour (Ignore / Dock / Dock Until Dry / Pause Auto), resume delay, debounce |
| Advanced | `settings/11-advanced.png` | Raw key/value editor for `mowgli_robot.yaml` parameters not covered elsewhere |

Each tab persists changes to the GUI's settings store; the **Restart ROS2** button at the bottom-right (visible in every tab) reloads the ROS2 container so YAML-backed parameters are picked up.

**Overridden dot + reset to default.** The installed `mowgli_robot.yaml` is [sparse](https://github.com/mowglinext/mowglinext/wiki/Configuration#sparse-robot-config-model) — it stores only values that differ from the shipped defaults. Any field you have pinned to a non-default value shows a small **dot** before its label; a subtle **reset (undo) button** next to it reverts the field to its default. Reset deletes the key from the installed config so it falls back to the in-package template, and the backend prunes any saved value that already equals its default — so the installed file stays sparse and a later software update that changes a default automatically flows through to fields you never overrode.

The **Localization** tab is worth a closer look:

![Localization tab](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/settings/04-localization.png)

The four toggles (`gui/web/src/components/settings/LocalizationSection.tsx:23-63`):

- **Fusion Graph (iSAM2)** — replaces `ekf_map_node` with `fusion_graph_node` (GTSAM). Required to ride through long RTK-Float windows. Restart ROS2 after toggling.
- **LiDAR scan matching** — adds ICP between-factors. Greyed-out unless `use_fusion_graph` is on.
- **Loop closure** — adds loop-closure factors (5 m radius, 10 min minimum age). Greyed-out unless `use_fusion_graph` is on.
- **Magnetometer yaw** — fuses tilt-compensated mag yaw. **Off by default**, the in-app help text says: *"motor-induced bias makes the magnetometer unreliable on most chassis. Enable only after running mag calibration with motors-off and validating a stable |B|."*

### 2.5 Diagnostics page

This is the most information-dense page in the app. It is a near-superset of what the Wiki [Architecture](https://github.com/mowglinext/mowglinext/wiki/Architecture) describes, but rendered live.

![Diagnostics top](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/diagnostics/01-overview.png)

**Top of page:** Health pills (Containers OK, GPS: RTK FIX, Battery 88%, No Emergency, CPU 53.6 °C), Alerts panel, Containers table (mowgli-ros2 / mowgli-gui / mowgli-gps / mowgli-lidar / mowgli-mqtt with state + uptime), CPU temperature card.

![Pose + GPS + Fusion Graph](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/diagnostics/02-pose-gps.png)

**Filtered Pose / GPS / Fusion Graph (iSAM2) / Heading sources / BT State / Coverage**:

- *Fusion Graph (iSAM2)*: nodes in graph (13,092 in our session), loop closures (491), ICP success rate (100% / 15540 of 15573), **Pose σ** (0.0 cm — yaw ±1.15°). Two action buttons: **Save graph** (calls `~/save_graph` `Trigger`) and **Clear graph** (`~/clear_graph` `Trigger`). These wipe `<graph_save_prefix>.{graph,scans,meta}`.
- *Heading sources*: side-by-side comparison of the active filter yaw, GPS course-over-ground (`/imu/cog_heading`), and magnetometer yaw (`/imu/mag_yaw`). When mag is "Stale" the unary factor is not being fused.
- *Coverage*: per-area progress (cells mowed / total cells, obstacles, strips left).

![Calibration panels](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/diagnostics/03-bt-coverage-network.png)

**Configuration cross-checks + three calibration panels** (`DiagnosticsPage.tsx:917-1057`):

- *Configuration cross-checks*: the warning bar across the top — checks that the dock pose isn't all-zero, that the datum lat/lon is set, etc.
- *Dock calibration* — Present/Missing tag, `dock_pose_x/y/yaw_rad` from `mowgli_robot.yaml`. **Run calibration** kicks off the IMU yaw calibration service (which includes a dock pre-phase if the robot is charging).
- *IMU bias calibration* — read from `/ros2_ws/maps/imu_calibration.txt`. Shows calibrated-at timestamp, sample count (1000 in the live session), gyro bias vector, and implied pitch/roll. The hardware bridge auto-runs this every time the robot returns to the dock.
- *Magnetometer calibration* — read from `/ros2_ws/maps/mag_calibration.yaml`. Shows |B| mean / std / sample count. The **Enable & run** button currently only opens a notification telling you to flip the `do_mag_calibration` parameter on `calibrate_imu_yaw_node` — it is **not yet a one-click operation** (see `DiagnosticsPage.tsx:896-902` and `ONBOARDING_IMPROVEMENTS.md` gap #4).

![ROS Diagnostics](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/diagnostics/04-imu-wheel-bottom.png)

**Hardware Status + ROS Diagnostics**: the bottom is the canonical `diagnostic_msgs/DiagnosticArray` view. Click any row to expand the per-key/value detail. In our live session, `ekf_odom_node: odometry/filtered topic status = ERROR — No events recorded` is visible — this is expected when `use_fusion_graph=true` and `ekf_odom_node` is disabled, and the GUI doesn't yet know to suppress it.

### 2.6 Schedule

![Schedule](https://raw.githubusercontent.com/mowglinext/mowglinext/dev/docs/gui-walkthrough/screenshots/schedule/01-overview.png)

Weekly grid (Mon–Sun, 6:00–19:00). Bottom panels show:
- **This week** (count of active schedules + per-day chips).
- **Schedules** list + **+ New run** button.
- **Rules**: Rain-aware toggle, Auto-dock low (return at <20% battery).

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

**Two map-frame backends, mutually exclusive:**

1. **`use_fusion_graph = false`** (default) → `ekf_map_node` (robot_localization dual-EKF). Same inputs as fusion_graph but no LiDAR factors. Cheap, fast, but the map-frame estimate diverges fast during multi-minute RTK-Float windows.
2. **`use_fusion_graph = true`** (recommended once LiDAR is mounted) → `fusion_graph_node` (GTSAM iSAM2). Same inputs/outputs (`map → odom` TF, `/odometry/filtered_map`) but adds optional LiDAR scan-matching and loop closure factors.

**The four calibration artefacts the localizer needs:**

| Artefact | Stored in | Written by | When to retune |
|---|---|---|---|
| `datum_lat`, `datum_lon` | `mowgli_robot.yaml` | Onboarding step 2 (Use current GPS), or Settings → GPS → same button | Once at install. Re-run if you physically move the dock. |
| `imu_yaw` (mounting yaw) | `mowgli_robot.yaml` | Diagnostics → Run calibration / Sensors editor → compass icon | Once at install. Re-run if you re-mount the IMU. |
| `imu_pitch`, `imu_roll` (mounting tilt) | `mowgli_robot.yaml` | Same calibration service, only persisted when `stationary_samples_used ≥ 150` (`RobotComponentEditor.tsx:215-226`) | Once at install. Re-run if you re-mount the IMU or move it relative to the chassis. |
| `dock_pose_x/y/yaw` | `mowgli_robot.yaml` | (a) IMU yaw calibration when started while docked, **OR** (b) map editor → "Set docking point" with robot manually positioned on dock | Once at install. Re-run if you physically move the dock. |
| `imu_calibration.txt` (gyro/accel bias) | `/ros2_ws/maps/imu_calibration.txt` | Auto, every dock arrival, `hardware_bridge_node` | Continuous. Look at Diagnostics → IMU bias panel; if `Implied pitch/roll` is > ~1° you should bake those into `imu_pitch/roll` (see FIRST_BOOT.md §3). |
| `mag_calibration.yaml` (magnetometer) | `/ros2_ws/maps/mag_calibration.yaml` | `calibrate_imu_yaw_node` rotation phase, **only when `do_mag_calibration=true`** (off by default) | Optional — only needed if you toggle Magnetometer yaw on. |

> The `mowgli_robot.yaml` values above are exactly the per-robot calibration outputs that the **sparse** installed config is meant to carry (alongside the install-time datum/NTRIP/hardware choices). Everything else is a template default — see [Configuration → Sparse robot config model](https://github.com/mowglinext/mowglinext/wiki/Configuration#sparse-robot-config-model).

**What changes when you tune `use_fusion_graph`:**

- Off → robot tolerates RTK-Float windows of seconds, not minutes. Loses heading lock when GPS goes Float.
- On (with LiDAR) → robot rides through tens of minutes of RTK-Float on scan-matching alone, and loop closures pull drift back to mapped pose mid-session.
- On (without LiDAR) → equivalent to off. The cost is real (GTSAM is heavier than two EKFs) so leave off if no LiDAR.

### 3.2 Sensors

The drag-to-place editor in Settings → Sensors (or onboarding step 3) writes:

- `lidar_x/y/z/yaw` — LiDAR mount, in metres + radians, base_link frame.
- `imu_x/y/z/yaw` (and `imu_pitch/imu_roll` written by calibration) — IMU mount.
- `gps_antenna_x/y/z` — GPS antenna mount (no yaw — antenna is point-symmetric).

The robot rectangle uses dimensions from `/robot_description` (URDF). You can verify what the Nav2 stack thinks your robot looks like by comparing the rectangle to your physical chassis.

### 3.3 Navigation tuning

The Settings → Navigation panel exposes:

- **Transit XY Tolerance** (default 0.75 m) — FTC FollowPath "we arrived" radius (transit and dock approach).
- **Yaw Tolerance** (default 1.0 rad ≈ 57°) — final yaw at goal.
- **Coverage XY Tolerance** (default 0.05 m, hard-capped at 0.15 in `navigation.launch.py`) — `PathProgressGoalChecker` xy tolerance for the goal pose. MUST stay below `tool_width` (0.18 m). Earlier site configs carrying the legacy 0.50 m value were field-broken (`SimpleGoalChecker` fired on tick 1 because the strip end was within tolerance of the robot's current pose, FTC reported SUCCEEDED before publishing any cmd_vel, the BT loop spun forever). Coverage completion is now also gated on monotonic path-pose tracking >= 95 %, so loose tolerances no longer bypass the controller — but keeping it tight prevents the goal pose from being claimed prematurely on the final approach.
- **Progress Timeout** (default 30 s, operator-tunable via `progress_timeout_sec`) — Nav2 fails the action if `PoseProgressChecker` hasn't seen 0.15 m of translation OR 0.5 rad of rotation in this window. The angle gate keeps headland pivots from tripping "no progress".

Don't tighten transit XY below 0.20 m — FTC's PRE_ROTATE state can oscillate at the goal.

### 3.4 Schedule, blade, rain

These are operator-facing and self-explanatory in the UI:

- **Schedule** — add weekly recurrences with start time + duration. Rain-aware and auto-dock-low rules apply globally.
- **Mowing** (settings) — speed (0.50 m/s default for both transit and mowing), `tool_width` (0.18 m, single source for blade cut width and F2C swath spacing), `mow_angle_offset_deg` / `headland_width` exposed for F2C v2 coverage, perimeter (outline passes count and offset for the legacy strip planner).
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
3. If using fusion_graph, verify ICP success rate > 95% — if it's lower, your LiDAR mount or the `lidar_yaw` setting is wrong.

### "Boundary violation on mow"

**Symptoms:** robot runs over the edge of a polygon, or mows outside a recorded area.

**Diagnose:** map view → check that the area's polygon actually covers what you intended. The GUI applies a small inflation around obstacles only — the working polygon is the recorded boundary minus 0 m.

**Fix:** **Edit Map** → click the polygon → drag vertices. If the polygon is correct but the robot still wanders, the *yaw* is wrong (see "Robot drifts in odom" above) — straight-line FTCController error becomes yaw-driven.

### "Docking arrives misaligned"

**Symptoms:** robot stops 5–20 cm off-centre relative to the dock contacts; charger never engages.

**Diagnose:**

1. Diagnostics → Configuration cross-checks. Is `Dock pose` non-zero with Yaw value present? If Yaw is 0° but the dock physically faces a different direction, dock pose is wrong.
2. Diagnostics → Dock calibration card. Should show **Present** with the correct yaw.
3. Map view: does the `DOCK` marker arrow visually align with the physical dock?

**Fix:** drive the robot manually onto the dock until it physically charges, then **Edit Map → environment pin → confirm**. This snaps `dock_pose_x/y/yaw` to the current robot pose. Restart ROS2 so `hardware_bridge`, `map_server_node`, and `dock_yaw_to_set_pose` reload the new params.

### "Emergency latched and won't clear"

**Symptoms:** Dashboard pill says "Emergency"; pressing the ⚠️ pill reset doesn't help.

**Diagnose:** the firmware is the safety authority — it will only clear the latch if the physical trigger is no longer asserted (`CLAUDE.md` invariant 9). Likely causes: tilt sensor still reads tilted, lift sensor reads off-ground, blade temperature still over the threshold.

**Fix:** physically verify the robot is level, on the ground, and the blade has cooled. Then press the reset pill. If the dock is detected as charging, the BT will auto-reset on its own.

---

## Where to go next

- **Wiki — Architecture:** https://github.com/mowglinext/mowglinext/wiki/Architecture (deep dive into TF chain, fusion_graph, BT, coverage)
- **Wiki — Configuration:** https://github.com/mowglinext/mowglinext/wiki/Configuration (every YAML key)
- **Wiki — FAQ:** https://github.com/mowglinext/mowglinext/wiki/FAQ
- **First-boot checklist:** [`docs/FIRST_BOOT.md`](FIRST_BOOT.md)
- **Onboarding gaps + roadmap:** [`docs/ONBOARDING_IMPROVEMENTS.md`](ONBOARDING_IMPROVEMENTS.md)
