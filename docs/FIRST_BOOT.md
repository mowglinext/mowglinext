# First Boot Checklist

After `mowglinext.sh` finishes and the containers come up, walk through this once per new install. Everything here is docked-only — nothing asks you to drive the mower.

## 1. GUI & diagnostics come up

- Open `http://<mower-ip>:4006` in a browser.
- In the **Diagnostics** panel, you should see (within 30–60 s of boot):
  - `hardware_bridge` → OK, serial link open.
  - `fusion` → publishing at ~25–50 Hz.
  - `gps` → publishing at 5 Hz. **Status: RTK Fixed** is the goal — keep reading if you are not there yet.
  - `lidar` (if enabled) → `/scan` publishing at ~10 Hz.
  - `fusion_graph` (if `use_fusion_graph:=true` and LiDAR enabled) → publishing on `/fusion_graph/diagnostics` at 1 Hz.

## 2. RTK Fixed

MowgliNext expects centimetre-accurate GPS. Without it, area recording is noisy and strip coverage drifts between sessions.

1. Check `/gps/fix` in Foxglove or via `ros2 topic echo --once /gps/fix | grep status`.
2. `status=2` means GBAS/RTK Fixed — you are done.
3. `status=1` or `0` means you are on SBAS or a basic fix. The usual fixes, in order:
   - Confirm antenna has a clear sky view (no tree canopy, no metal overhang).
   - Confirm the active YAML GNSS config is correct (`docker/config/mowgli/mowgli_robot.yaml` → `ntrip_host`, `ntrip_mountpoint`, `ntrip_user`, `ntrip_password`). `docker/.env` carries fallback-only first-boot defaults and does not override explicit YAML values.
   - `ros2 topic hz /ntrip_client/rtcm` should print a rate around 50–60 Hz. If 0, the NTRIP client isn't getting RTCM.
   - If you move indoors or the sky view was bad at boot, the receiver may never converge — re-boot outdoors.

## 3. IMU calibration

- The first time the robot is charging on the dock, `hardware_bridge_node` runs a 20-second IMU calibration (1000 samples) and subtracts the mean bias from every subsequent reading.
- Look in the logs for:
  ```
  IMU calibration complete (1000 samples) ...
  Implied mounting tilt: pitch=+X.XX° roll=+X.XX° ...
  ```
- If `pitch` or `roll` is larger than ~1°, the IMU is physically mounted at an angle. Copy those values into `mowgli_robot.yaml` → `imu_pitch`, `imu_roll`, and recreate the container. Values under 1° are chip bias and are already removed by the calibration.

## 4. IMU yaw calibration (requires motion)

The IMU's heading relative to forward is not auto-detected — it has to be solved by driving the robot a short distance.

- Only do this step once you are physically at the robot and ready to catch it if anything goes wrong.
- GUI → **Diagnostics** → **Auto-calibrate IMU yaw** button. The robot drives 0.6 m forward then back and writes the solved `imu_yaw` into `mowgli_robot.yaml`.
- Make sure the robot is on a level patch of open ground with roughly 1 m clear in front and behind.

## 5. Dock pose

- Dock position and yaw live in `mowgli_robot.yaml` (`dock_pose_x`, `dock_pose_y`, `dock_pose_yaw`) — single source of truth. The IMU/dock auto-calibration service and the "set dock pose" action in the GUI both write the measured values back to that file via in-place line edits that preserve comments. `hardware_bridge`, `map_server_node`, and `dock_yaw_to_set_pose` read the values as ROS parameters at startup.
- The GPS datum and active GNSS operator settings live in `mowgli_robot.yaml`, consumed by `navsat_transform_node` and the GNSS sidecar at startup. Use the GUI/backend/Universal GNSS flow for receiver model/profile/signal configuration; `docker/.env` only provides fallback defaults when the YAML leaves a value unset.

## 6. Record a mowing area

- Drive the mower manually (GUI → **Record Area**) along the boundary.
- Finish recording — the polygon is Douglas–Peucker simplified and saved via `/map_server_node/add_mowing_area`.
- Repeat for every area you want to mow.

## 7. First autonomous mow

- GUI → **Start**. The behavior tree will:
  1. Clear the emergency latch if still held.
  2. Undock via Nav2 BackUp (1.5 m, 0.15 m/s).
  3. Iterate through each mowing area: plan the next strip, transit to its start, follow it with FTCController, repeat until the area is covered. Then move to the next area.
  4. Dock when all areas are done, or when battery drops below the low-battery threshold.
- Progress is persisted in the `mow_progress` grid layer and survives restarts, so if you hit Emergency mid-mow you can resume later.

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

## Not yet supported / coming soon

See the main [README](../README.md#not-planned--removed) for the authoritative list. Short version: headland passes, 3D slope-aware planning, multi-zone time-window scheduling, visual BT tree live viewer, fleet management, and the mobile app (PR #27) are all **coming soon**, not shipped today.
