# mowgli_tools

`mowgli_tools` packages controlled operator utilities that work with the live
ROS 2 stack. The package is kept as a sidecar under `tools/motor/` so the
motor-tuning workflow stays close to field tools instead of the core runtime
packages.

## `tune_drive_pid`

Drive-wheel PID assistant with:

- profile presets for `yardforce_8w_1964` and `yardforce_12w_1600`
- automatic backup + rollback of the live `hardware_bridge` parameters
- guarded speed trials with stop commands between segments
- optional RTK-based validation from `/gps/status` + `/gps/absolute_pose`
- optional undock reverse when the robot starts on the charger
- YAML export of trial metrics and recommended parameters

Example, proposal only from the dock:

```bash
ros2 run mowgli_tools tune_drive_pid -- \
  --profile yardforce_8w_1964 \
  --max-speed 0.3 \
  --duration 5 \
  --undock-distance 2.0 \
  --output /tmp/drive_pid_8w.yaml
```

Apply the recommended live parameters at the end of the session:

```bash
ros2 run mowgli_tools tune_drive_pid -- \
  --profile yardforce_8w_1964 \
  --max-speed 0.3 \
  --duration 5 \
  --undock-distance 2.0 \
  --apply \
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
- The `yardforce_8w_1964` preset starts from `1964 / 3 ~= 655 ticks/m` because
  the live bridge only sees one hall channel out of three. The motor being a
  4-pole inrunner is already reflected in the theoretical 1964 value, so it
  does not change that `/ 3` reduction.
- RTK checks are only used when `/gps/status` reports a valid RTK mode,
  corrections are active, and horizontal accuracy is within the configured
  threshold.
