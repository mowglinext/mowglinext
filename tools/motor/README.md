# mowgli_tools

`mowgli_tools` packages controlled operator utilities that work with the live
ROS 2 stack. The package is kept as a sidecar under `tools/motor/` so the
motor-tuning workflow stays close to field tools instead of the core runtime
packages.

## `tune_drive_pid`

Drive-wheel PID assistant with:

- reference profiles for `yardforce_8w_1964` and `yardforce_12w_1600`
  (`yardforce_1600_12w` is accepted as an alias)
- automatic backup + rollback of the live `hardware_bridge` parameters
- guarded speed trials with stop commands between segments
- optional RTK-based validation from `/gps/status` + `/gps/absolute_pose`
- optional undock reverse when the robot starts on the charger
- YAML export of trial metrics and recommended parameters
- a dedicated tuning motion path that does not share the IMU / teleop lane:
  `/cmd_vel_tuning -> twist_mux -> /cmd_vel`
- live oscillation is treated as a calibration warning by default in
  feed-forward mode, with an opt-in strict abort flag when needed

Example, proposal only from the dock:

```bash
ros2 run mowgli_tools tune_drive_pid -- \
  --max-speed 0.3 \
  --duration 5 \
  --undock-distance 2.0 \
  --output /tmp/drive_pid_8w.yaml
```

Apply the recommended live parameters at the end of the session:

```bash
ros2 run mowgli_tools tune_drive_pid -- \
  --max-speed 0.3 \
  --duration 5 \
  --undock-distance 2.0 \
  --apply \
  --output /tmp/drive_pid_8w.yaml
```

Explicitly reset pass 1 to a preset before testing:

```bash
ros2 run mowgli_tools tune_drive_pid -- \
  --profile yardforce_8w_1964 \
  --reset-to-profile \
  --max-speed 0.3 \
  --duration 5 \
  --output /tmp/drive_pid_8w.yaml
```

Restore the last saved backup:

```bash
ros2 run mowgli_tools tune_drive_pid -- --rollback
```

Notes:

- `--apply` keeps the final values live in `hardware_bridge`, but does not edit
  persistent YAML config files.
- Without `--apply`, the tool restores the original live parameters after the
  trials.
- By default, pass 1 starts from the current live `hardware_bridge`
  parameters. `--profile` is kept as reference metadata unless
  `--reset-to-profile` or `--force-profile` is passed explicitly.
- In `ff` mode, suspected live oscillation is recorded in the report as a
  warning and the trial continues so first-bring-up calibration can still
  finish. Use `--abort-on-live-oscillation` to restore strict abort behavior.
- The `yardforce_8w_1964` preset starts from `1964 / 3 ~= 655 ticks/m` because
  the live bridge only sees one hall channel out of three. The motor being a
  4-pole inrunner is already reflected in the theoretical 1964 value, so it
  does not change that `/ 3` reduction.
- RTK checks are only used when `/gps/status` reports a valid RTK mode,
  corrections are active, and horizontal accuracy is within the configured
  threshold.
- `--cmd-topic` can still override the command topic manually, but the default
  uses `/cmd_vel_tuning`, which flows through `twist_mux` to `/cmd_vel`
  without sharing `/cmd_vel_teleop` with the WebSocket relay or IMU calibration.
