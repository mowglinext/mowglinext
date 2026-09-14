# Diagnostic scripts

Single-shot tools for characterising the drivetrain + localization stack.
Built during the 2026-04-24 "Voie C" debug session that traced the
long-running "robot glisse sur la carte" / "trajectoires aberrantes"
symptoms down to a host-side PWM deadband in firmware.

All motion scripts enter the BT's `RECORDING` state (so the firmware accepts
`/cmd_vel_teleop` — in `IDLE`/`NULL` the STM32 zeroes the wheel targets), do
their thing, then `CANCEL` recording to return to `IDLE`. They all abort on
emergency; `rotate_at_rate.py` and `forward_5m.py` also abort when charging,
while `undock_and_rotate_90.py` deliberately starts on the dock and only warns
if it is still charging after the backup. User must be physically present near
the robot to intervene if anything goes wrong.

Run inside the `mowgli-ros2` container after `docker cp`ing the file:

```bash
docker cp ros2/scripts/diagnostics/<script>.py mowgli-ros2:/tmp/
docker exec mowgli-ros2 bash -c \
  'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
   python3 /tmp/<script>.py [args]'
```

## Motion patterns

- **`rotate_at_rate.py WZ DURATION`** — single controlled rotation.
  Reports wheel + gyro integrated yaw delta vs expected. Used to
  characterise the angular-velocity response curve — measured in 2026-04,
  when `wz` was still open-loop: wz=0.3 → 0.5×, wz=0.6 → 0.87×,
  wz=1.0 → 0.85× on YardForce 500. Since 2026-07-17 the STM32 closes a
  gyro-based yaw-rate loop (Option C), so a healthy robot should now
  report ratios near 1×.
- **`undock_and_rotate_90.py`** — backs up 1.5 m at 0.15 m/s then rotates
  90° at wz=0.30. Catches bugs that only surface when the robot is
  moving off-dock (gyro mid-motion bias, wheel slip during undock).
- **`forward_5m.py`** — 5 m straight-line drive with ramp up/down. Reports
  fusion displacement vs initial heading + yaw drift comparison
  between wheel, gyro and fusion. Surfaces grass-slip asymmetry.

All motion scripts assume the robot has a clear zone in the commanded
direction. The rotation script needs ~1.5 m radius; the straight-line
script needs 5-6 m in front.

## Session-log analyzers

Used on the JSONL files produced by `ros2/scripts/mow_session_monitor.py`
(mounted inside the container at `/ros2_ws/maps/<session>.jsonl` when
`--output-dir /ros2_ws/maps` is passed).

- **`analyze_session.py <file.jsonl>`** — one-shot summary: fusion position /
  yaw ranges, fusion σ_xy trend, GPS σ_xy + `gps_absolute_pose` sample count,
  wheel / gyro integrated yaw drift, RTK cov-check counters, gyro_z mean rate
  + integrated, wheel vx mean/max.
- **`session_timeline.py <file.jsonl>`** — every 100th sample (5 s at
  `--rate 20`, 10 s at the monitor's 10 Hz default): fusion pose, σ_xy,
  RTK-check arrivals/ok/violations. Use when `analyze_session.py` flags an
  anomaly and you want to see when.

What the monitor records, and how to read it back, is documented in
[`docs/claude/session-monitoring.md`](../../../docs/claude/session-monitoring.md).

## Typical workflow

```bash
# Start monitor in background (writes JSONL)
SESSION="my-test-$(date +%Y%m%d-%H%M)"
docker exec -d mowgli-ros2 bash -c \
  "source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
   timeout -s INT 120 python3 /ros2_ws/scripts/mow_session_monitor.py \
     --session $SESSION --output-dir /ros2_ws/maps --rate 20 > /tmp/mon.log 2>&1"
sleep 3  # let subscriptions settle

# Run the motion pattern
docker cp ros2/scripts/diagnostics/forward_5m.py mowgli-ros2:/tmp/
docker exec mowgli-ros2 bash -c \
  'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
   python3 /tmp/forward_5m.py'

# Wait for monitor to finish (auto-stops on timeout) then analyze
until docker exec mowgli-ros2 bash -c "tail -1 /ros2_ws/maps/$SESSION.jsonl | grep -q summary"; do sleep 2; done
docker cp ros2/scripts/diagnostics/analyze_session.py mowgli-ros2:/tmp/
docker exec mowgli-ros2 python3 /tmp/analyze_session.py /ros2_ws/maps/$SESSION.jsonl
```
