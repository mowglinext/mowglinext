# First Boot Checklist

After `mowglinext.sh` finishes and the containers come up, walk through this once per new install. Most of it is docked-only, but two steps do drive the mower a short, supervised distance — the IMU yaw calibration (§4) and the optional drive tuning (§6). Do those only when you are physically at the robot and ready to catch it.

> **After the onboarding wizard is not the same as "ready to mow."** Finishing the GUI wizard writes your install choices and (if it ran on the dock) the IMU-yaw + dock pose, then restarts ROS2. It does **not** by itself record a mowing area, run drive tuning, or guarantee an RTK-Fixed datum. Use this checklist to close those gaps before the first autonomous mow.

## 1. GUI & diagnostics come up

- Open `http://<mower-ip>:4006` in a browser.
- In the **Diagnostics** panel, you should see (within 30–60 s of boot):
  - `hardware_bridge` → OK, serial link open.
  - `gps` → publishing at 5 Hz. **Status: RTK Fixed** is the goal — keep reading if you are not there yet.
  - `lidar` (if enabled) → `/scan` publishing at ~10 Hz.
  - `fusion_graph` (the sole map+odom localizer; always enabled, optionally adds LiDAR scan-matching + loop-closure when LiDAR is present) → publishing on `/fusion_graph/diagnostics` at 1 Hz and owning both the `map→odom` and `odom→base_footprint` transforms.

> If `hardware_bridge`, `/imu/data`, and `/wheel_odom` all go silent at once right after flashing the STM32, it is almost always the post-flash USB re-enumeration failing — see [Troubleshooting](#no-imu--wheel--firmware-data-after-flashing-the-stm32) below to recover without a power cycle.

## 2. RTK Fixed

MowgliNext expects centimetre-accurate GPS. Without it, area recording is noisy and strip coverage drifts between sessions.

1. Check `/gps/fix` in Foxglove or via `ros2 topic echo --once /gps/fix | grep status`.
2. `status=2` means GBAS/RTK Fixed — you are done.
3. `status=1` or `0` means you are on SBAS or a basic fix. The usual fixes, in order:
   - Confirm antenna has a clear sky view (no tree canopy, no metal overhang).
   - Confirm the active YAML GNSS config is correct (`docker/config/mowgli/mowgli_robot.yaml` → `ntrip_host`, `ntrip_mountpoint`, `ntrip_user`, `ntrip_password`). `docker/.env` carries fallback-only first-boot defaults and does not override explicit YAML values.
   - `ros2 topic hz /rtcm` should print a steady, non-zero rate — the GNSS sidecar mirrors the caster's RTCM stream onto that topic. If nothing prints, the NTRIP client isn't getting RTCM.
   - If you move indoors or the sky view was bad at boot, the receiver may never converge — re-boot outdoors.

## 3. IMU calibration

- Whenever the robot is charging on the dock, `hardware_bridge_node` collects a short IMU calibration window (`imu_cal_samples`, default 200 samples ≈ 2 s at the firmware's 100 Hz IMU stream) and subtracts the mean bias from every subsequent reading. It runs at boot if the robot is already docked, again on every dock transition, and refreshes every `imu_cal_periodic_recal_sec` (default 600 s) while docked. Off the dock it self-calibrates once after `imu_cal_auto_rest_sec` (default 15 s) of standing still. The result is persisted to `/ros2_ws/maps/imu_calibration.txt`, so a container restart does not lose it.
- Look in the logs for:
  ```
  IMU calibration complete (200 samples) ...
  Implied mounting tilt: pitch=+X.XX°, roll=+X.XX° ...
  ```
- If `pitch` or `roll` is larger than ~1°, the IMU is physically mounted at an angle. Copy those values into `mowgli_robot.yaml` → `imu_pitch`, `imu_roll`, and recreate the container. Values under 1° are chip bias and are already removed by the calibration.

> **Editing `mowgli_robot.yaml` — sparse config model.** The installed `mowgli_robot.yaml` (`/ros2_ws/config/mowgli_robot.yaml`) is **sparse**: it should hold only per-robot overrides and calibration outputs. Every parameter's *default* lives in the in-package template (`mowgli_bringup/config/mowgli_robot.yaml`) and is deep-merged in at launch, so you only need to add a key when you want to override its default. In the GUI, most settings are editable directly; a small dot marks any value you have overridden and a **reset button** reverts it to the default (by deleting the key). Prefer the GUI over hand-editing — it keeps the file sparse for you.

## 4. IMU yaw calibration (requires motion)

The IMU's heading relative to forward is not auto-detected — it has to be solved by driving the robot a short distance.

- Only do this step once you are physically at the robot and ready to catch it if anything goes wrong.
- GUI → the onboarding wizard's **IMU yaw calibration** step, or later **Settings → Sensors** → the compass button on the robot diagram. The robot drives 0.6 m forward then back; apply the result and save, and the GUI writes the solved `imu_yaw` into `mowgli_robot.yaml`.
- Make sure the robot is on a level patch of open ground with roughly 1 m clear in front and behind.

## 5. Dock pose

- Dock position and yaw live in `mowgli_robot.yaml` (`dock_pose_x`, `dock_pose_y`, `dock_pose_yaw`) — single source of truth. The IMU/dock auto-calibration service and the "set dock pose" action in the GUI both write the measured values back to that file via in-place line edits that preserve comments. `hardware_bridge` and `map_server_node` read the values as ROS parameters at startup.
- Fastest path: GUI → **Settings → Docking** → **Start dock calibration** (one-click). With the robot on the dock, charging, and RTK-Fixed, it reverses 2 m in a straight line, fits the dock heading from the GPS course, re-docks, and persists the dock pose. The blade stays off throughout.
- Confirm the dock pose landed: Diagnostics → Dock calibration should read **Present** with a non-zero yaw. The IMU-yaw calibration (§4) also captures it as a side effect, but **only when it runs with the robot on the dock and charging**.
- Every dock-pose writer requires the robot to be physically on the dock (firmware reporting `is_charging`) with a fresh, accurate GPS fix — that includes the manual **Set dock pose** button in Settings → Sensors and dragging the dock marker in the map editor. `map_server_node` rejects an off-dock capture.
- The GPS datum and active GNSS operator settings live in `mowgli_robot.yaml`, consumed by the localizer's GPS pipeline and the GNSS sidecar at startup. Use the GUI/backend/Universal GNSS flow for receiver model/profile/signal configuration; `docker/.env` only provides fallback defaults when the YAML leaves a value unset (the installed YAML is sparse and deep-merged over the in-package template — see the sparse-config note above).

## 6. Drive tuning (recommended before the first mow)

The robot ships with generic wheel gains and a nominal odometry scale. For accurate straight-line odometry and clean swath tracking, run the drive tuning flow once — it drives the robot a few metres, measures wheel/ground speed against RTK, and writes the tuned feed-forward + PID gains to `mowgli_robot.yaml`.

- GUI → **Settings → Drive Motor** → *Calibration assistants*. It is **not** part of the onboarding wizard, so it is easy to miss.
- Run **Calibrate odometry / feed-forward** first (learns `ticks_per_meter` + the PWM-per-m/s feed-forward), then optionally **Auto-tune drive PID** (step-response gains). Each run needs a few metres of clear, level ground.
- The assistants refuse to start while an emergency is active or latched, and refuse to move while the robot is on the dock unless you explicitly allow undock. They offer a one-click **rollback** to the previous gains if a run looks worse. Note that they drive on a dedicated `/cmd_vel_tuning` lane at the highest twist_mux priority below emergency, so collision_monitor — which only filters the navigation lane — does **not** stop them. Stay with the robot.
- Skipping this is fine to try a first mow, but expect drifty coverage until it is done.

## 7. Record a mowing area

- Drive the mower manually (GUI → **Record Area**) along the boundary.
- Finish recording — the polygon is Douglas–Peucker simplified and saved via `/map_server_node/add_area`.
- Repeat for every area you want to mow.

## 8. First autonomous mow

Before you hit Start, confirm all of the following — these are the post-onboarding must-haves the wizard does not guarantee on its own:

- [ ] **RTK-Fixed** (§2) and a real **datum** set (Settings → GPS & Positioning shows non-zero lat/lon).
- [ ] **IMU bias** present (§3) and **IMU yaw** solved (§4).
- [ ] **Dock pose** shows **Present** in Diagnostics (§5).
- [ ] **Drive tuning** run (§6) — or accept drifty coverage for a first try.
- [ ] **At least one mowing area** recorded (§7).

Then:

- GUI → **Start**. The behavior tree will:
  1. Clear the emergency latch if still held.
  2. Undock via Nav2 BackUp (`undock_distance` 1.5 m at `undock_speed` 0.16 m/s).
  3. Iterate through each mowing area: plan the whole area once via `mowgli_coverage` (headland rings + serpentine swaths joined into continuous, hole-free sub-paths), transit blade-off to the start, then drive each sub-path end-to-end with FTCController. Then move to the next area.
  4. Dock when all areas are done, or when battery drops below the low-battery threshold.
- Progress is persisted (a per-area resume cursor into the planned path, plus the `mow_progress` grid layer the GUI draws) and survives restarts, so if you hit Emergency mid-mow you can resume later.

## Troubleshooting

### No IMU / wheel / firmware data after flashing the STM32

After flashing, the board resets and re-enumerates over USB. On this hardware the
re-enumeration intermittently **fails** (EMI), which looks like *all* firmware
topics going silent at once — `/imu/data`, `/wheel_odom`, `/hardware_bridge/status`
all dead — even though the ROS2 stack and GUI are fine. `hardware_bridge` opened
`/dev/mowgli` before the disconnect and is now holding a dead handle.

Confirm with `dmesg`: repeated `usb 5-1: device descriptor read/64, error -62`
ending in `unable to enumerate USB device`, and `/dev/mowgli` missing.

**Recover without a power cycle** — rebind the STM32's USB controller (it's on
`fc840000.usb`; the GPS is on the separate `fc8c0000.usb`, so GPS is undisturbed):

```bash
echo fc840000.usb | sudo tee /sys/bus/platform/drivers/ohci-platform/unbind
sleep 2
echo fc840000.usb | sudo tee /sys/bus/platform/drivers/ohci-platform/bind
```

The board re-enumerates cleanly (`dmesg` shows `Product: Mowgli`, no -62) and
`/dev/mowgli → ttyACM*` reappears. Then restart the ROS2 container so
`hardware_bridge` reopens the port: `docker restart mowgli-ros2`.

## Not yet supported

MowgliNext is in its first public beta, so expect gaps. Notably absent today: 3D slope-aware planning, per-schedule mowing time *windows* (a schedule fires at one start time on the days you pick, for one area), a live behavior-tree viewer in the GUI, fleet management across several mowers, and a mobile app. Headland passes ARE shipped — the coverage server plans concentric headland rings before the serpentine swaths (§8).

There is no maintained roadmap page; check the [issue tracker](https://github.com/mowglinext/mowglinext/issues) and [discussions](https://github.com/mowglinext/mowglinext/discussions) for what is being worked on.
