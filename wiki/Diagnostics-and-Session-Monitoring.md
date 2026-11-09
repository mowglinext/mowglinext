# Diagnostics and Session Monitoring

When the robot does something odd in the field (a localization jump, a stuck GNSS position, a transit that fails), a log line is rarely enough. This page describes the tools that record what the robot *was doing* while it happened, so a run can be analysed offline and attached to an issue.

All of them live in [`ros2/scripts/`](https://github.com/mowglinext/mowglinext/tree/dev/ros2/scripts). They only **subscribe**: the session monitor never publishes velocity commands or changes the behavior-tree state. The motion scripts are the exception, see [Motion test scripts](#motion-test-scripts).

| Tool | Runs | What it gives you |
|------|------|-------------------|
| `mow_session_monitor.py` | inside `mowgli-ros2` | A JSONL timeline (10 Hz by default) of pose, sensors, navigation and localization state |
| `diagnostics/analyze_session.py` | inside `mowgli-ros2` | A one-shot summary of a recorded session |
| `diagnostics/session_timeline.py` | inside `mowgli-ros2` | Every 100th sample, to find *when* something started |
| `diagnostics/capture_gnss_container_health.sh` | on the **host** | GNSS container restart count and raw container logs |
| `diagnostics/rotate_at_rate.py`, `forward_5m.py`, `undock_and_rotate_90.py` | inside `mowgli-ros2` | Controlled motion patterns for drivetrain characterisation |

## Record a session

Run the monitor **in parallel with every mowing test** (a Start, an undock, any autonomous motion or tuning run). Use a descriptive session name; it becomes the file name.

```bash
SESSION="2026-10-14-corner-pivot-test"
docker exec -d mowgli-ros2 bash -c "
  source /opt/ros/lyrical/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py \
    --session $SESSION --output-dir /ros2_ws/maps"
```

- `--output-dir /ros2_ws/maps` writes to the persistent `mowgli_maps` Docker volume, so the log survives a container restart. The script's built-in default is a *host* path that is not mounted inside the container, so always pass `--output-dir` (or bind-mount `docker/logs/mow_sessions/` yourself).
- `--rate` sets the sampling rate in Hz (default 10). A higher rate makes bigger files; 10 is enough for most questions, 20 for fast dynamics such as a turn.
- Stop it with Ctrl-C when it runs interactively (drop the `-d` and use `-it`). The monitor handles both SIGINT and SIGTERM; for a detached run you can bound it with `timeout -s INT 120 python3 ...`, as the diagnostics README does. On a clean stop it writes a **summary record** as the last line of the file.

### What is in the file

- **First line, metadata:** session name, UTC time, git branch, commit and dirty flag, the Docker image tags, and short hashes of `mowgli_robot.yaml`, `fusion_graph.yaml` and the Nav2 parameter files. That is what lets you tell runs from different builds or tunings apart.
- **One line per sample:** fused pose and covariance, the TF legs, wheel and IMU data, GPS fix and the bridged `/gps/status`, the localization mode, behavior-tree state, `cmd_vel` before and after the safety layer, the Nav2 plan and distance to goal, LiDAR scan health, and a set of cross-checks (fusion against GPS, wheel against gyro).
- **Last line, summary:** duration, samples, distance driven, peak errors, RTK health verdict, final battery voltage and BT state.

The full list of recorded fields is in [`docs/claude/session-monitoring.md`](https://github.com/mowglinext/mowglinext/blob/dev/docs/claude/session-monitoring.md).

## Read a session back

Copy the analysers into the container once, then point them at the JSONL file:

```bash
docker cp ros2/scripts/diagnostics/analyze_session.py mowgli-ros2:/tmp/
docker exec mowgli-ros2 python3 /tmp/analyze_session.py /ros2_ws/maps/$SESSION.jsonl
```

- `analyze_session.py` prints pose and yaw ranges, the σ trend of the fused position, GPS σ, wheel and gyro yaw drift, the RTK covariance-check counters and the mean gyro and wheel rates. Start here.
- `session_timeline.py` prints every 100th sample (5 s at `--rate 20`, 10 s at the default 10 Hz): pose, σ and RTK-check counters. Use it when the summary shows an anomaly and you want to see when it began.

The JSONL is plain JSON per line, so `jq` or a few lines of Python are enough for your own questions, for example plotting `session_elapsed_sec` against the fusion/GPS distance.

## GNSS problems: capture the container as well

A GNSS receiver that freezes or restarts is a *Docker-level* fact: the ROS monitor cannot see a container restart. For those investigations, run this on the **host**, next to the monitor and with the **same session name**:

```bash
bash ros2/scripts/diagnostics/capture_gnss_container_health.sh "$SESSION"
```

It samples the `gps` container's `RestartCount` once per second and follows its raw `docker logs` (the `receiver_node` and `ntrip_node` exit and restart lines). It writes `<session>.container-health.jsonl` and `<session>.receiver-logs.txt` in the current directory. Compare their timestamps with the monitor's `session_elapsed_sec` to see what the fused pose and the GPS were doing at the moment of a restart.

The monitor itself also records the raw pre-bridge receiver status and the receiver's own observation counter, so a freeze can be localised to the receiver, the NavSatFix adapter or the bridge.

## Motion test scripts

`rotate_at_rate.py`, `forward_5m.py` and `undock_and_rotate_90.py` **drive the robot**. They enter the behavior tree's `RECORDING` state (the firmware only accepts teleop commands there), run a pattern and then return to `IDLE`.

- Stay physically next to the robot and keep the emergency stop within reach.
- Make sure there is clear ground: about a 1.5 m radius for the rotation script, 5 to 6 m in front of the robot for the straight drive, and 1.5 m behind the dock for the undock script.
- They abort on an emergency; the rotation and straight-drive scripts also abort when the robot is charging. `undock_and_rotate_90.py` deliberately starts on the dock.

Run one together with the monitor, so the drive is recorded:

```bash
docker cp ros2/scripts/diagnostics/forward_5m.py mowgli-ros2:/tmp/
docker exec mowgli-ros2 bash -c \
  'source /opt/ros/lyrical/setup.bash && source /ros2_ws/install/setup.bash && \
   python3 /tmp/forward_5m.py'
```

The full monitor, drive and analyse workflow is in [`ros2/scripts/diagnostics/README.md`](https://github.com/mowglinext/mowglinext/blob/dev/ros2/scripts/diagnostics/README.md).

## Sharing a session in an issue

1. Record the session with a descriptive name and, for GNSS issues, the host-side container capture with the same name.
2. Run `analyze_session.py` and paste its output into the issue.
3. Attach the JSONL (compress it first, `gzip` shrinks it a lot) or the part around the event.
4. Say which build you ran. The metadata line already carries the commit and image tags, so keep that first line.

Notable sessions (a golden run, a clear failure) are worth committing under `docker/logs/mow_sessions/` so they survive in git history.

## Related

- [Contributing](Contributing): how to test and what CI checks.
- [Behavior Trees](Behavior-Trees) and [Architecture](Architecture): what the recorded states mean.
- [`docs/claude/session-monitoring.md`](https://github.com/mowglinext/mowglinext/blob/dev/docs/claude/session-monitoring.md): the complete field reference.
