# Mowing Session Monitoring

> How to record and read the JSONL session timeline. Loaded on demand from [`../../CLAUDE.md`](../../CLAUDE.md).

**Whenever a mowing test is run (COMMAND_START, undock, autonomous motion, or any tuning session that involves the robot moving), also run the session monitor in parallel.** Output is a JSONL timeline that can be diffed/plotted across sessions to see how tuning changes affect behavior.

```bash
# Detached background from the host (the default --output-dir is the HOST path
# /home/ubuntu/mowglinext/docker/logs/mow_sessions, which is NOT mounted inside
# the container — always redirect with --output-dir, or bind-mount docker/logs/):
docker exec -d mowgli-ros2 bash -c '
  source /opt/ros/lyrical/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py \
    --session 2026-04-29-fusion-graph-tuning-v1 \
    --output-dir /ros2_ws/maps'

# Interactively from inside the container (Ctrl-C to stop + write summary):
docker exec -it mowgli-ros2 bash -c '
  source /opt/ros/lyrical/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py --session <name> \
    --output-dir /ros2_ws/maps'
```

The `--output-dir /ros2_ws/maps` redirects to the named `install_mowgli_maps` Docker volume (`mowgli_maps:/ros2_ws/maps`, `install/compose/docker-compose.base.yml`) so logs persist outside the container. Or bind-mount `docker/logs/mow_sessions/` explicitly in compose for a host-visible path.

**What it records** (per-sample, 10 Hz default):
- Fused pose + twist from `/odometry/filtered_map` (x/y/z, yaw, vx/vy/wz) **+ position covariance (cov_xx, cov_yy, derived sigma_xy_m)**
- TF snapshots: `map→base_footprint` (composed through `map→odom→base_footprint`) plus each leg separately, `map→odom` and `odom→base_footprint` — **both legs come from `fusion_graph_node`** (the JSON keys are still the legacy `cartographer_map_base` / `map_to_odom` / `odom_to_base`)
- Wheel twist + covariance + integrated distance and yaw
- IMU gyro + accel + integrated gyro yaw
- GPS NavSatFix (lat/lon/alt/status/covariance/**header stamp**) + `/gps/absolute_pose` ENU
- **`gnss_status`**: the full bridged `/gps/status` (`mowgli_interfaces/GnssStatus`) — stamp, `position_observation_sequence`, fix/RTK type, `horizontal_accuracy_m`, satellite counts, correction-stream status. **`universal_gnss_status`**: the RAW pre-bridge `/universal_gnss_receiver/status` (`universal_gnss_msgs/GnssStatus`) — has its OWN `lat`/`lon`, independent of `/gps/fix`'s NavSatFix (a separate publish path inside `receiver_node`); comparing the two localises a stuck-position freeze to the receiver's solver vs the NavSatFix adapter specifically. **`receiver_snapshot`**: `/universal_gnss_receiver/get_snapshot`, polled ~1 Hz — the receiver's own `runtime_observations` counter plus `transport_healthy`/`parser_healthy`. All three added for the mowglinext#694 field-evidence set Pepeuch requested (issue comment, 2026-09-20); a bring-up without the `universal_gnss_msgs` schema overlay just logs a warning and leaves `universal_gnss_status`/`receiver_snapshot` null.
- Dock heading (`/gnss/heading` while charging)
- BT state (`state_name`, `current_area`, `current_path` — logged as `bt.strip`), hardware mode, emergency flags, battery
- `cmd_vel_nav` (Nav2 output) + `cmd_vel` (post-safety, what reaches motors)
- Nav2 `/plan` length, next pose, goal pose, distance-to-goal
- LiDAR scan health (valid point count, min range)
- **Yaw-source attribution** (`yaw_sources`): COG yaw (`/imu/cog_heading`), fusion_graph's own yaw (`/imu/fg_yaw`), and each one's delta against the fused yaw — `cog_minus_fusion_deg ≈ ±180°` is the 180°-flip / lever-arm signature. Plus five `/fusion_graph/diagnostics` keys: `cov_yawyaw`, `gyro_bias_z_rad_per_s`, `gps_rejects_wrongfix`, `gps_rejects_dead_reckoning`, `cog_flip_recoveries`. The node also publishes graph, tiled-map and LiDAR-anchor counters; inspect those live on `/fusion_graph/diagnostics` when diagnosing fallback CPU or factor activity.
- **`localization_mode_id`** (top-level, per sample): `localization_monitor_node`'s live verdict on `/mowgli/localization/mode_id` (mowglinext#694) — `0=DEAD_RECKONING, 1=GPS_ONLY, 2=RTK_FLOAT, 3=RTK_FIXED`. This is the same signal `fusion_graph`'s payload-staleness gate and the BT's `LocalizationGuard` pause on, captured directly instead of inferred from `rtk_cov_check`. The summary rolls it into `dead_reckoning_sec` (wall-clock time spent untrusted) and `final_gps_rejects_dead_reckoning`.
- **Cross-source consistency** (`cross_checks`): `fusion ↔ gps` distance and `wheel ↔ gyro` yaw drift
- **RTK covariance-drop health**: on every RTK-Fixed GPS arrival, confirm `/odometry/filtered_map` cov drops to σ≤~3 cm within 300 ms — surfaced as `cross_checks.rtk_cov_check.{arrivals,ok,violations}` per sample and rolled into a `rtk_cov_check.verdict` ("healthy" / "intermittent" / "gate_rejecting" / "no_rtk" / "insufficient_data") in the summary.

**Metadata header** (first line of the JSONL): session name, UTC timestamp, git branch + commit + dirty flag, docker image tags from `.env`, SHA-256 truncated hashes of `mowgli_robot.yaml`, `fusion_graph.yaml`, and the Nav2 params (`nav2_params_base.yaml` + `nav2_params_lidar.yaml` + `nav2_params_no_lidar.yaml`) — so sessions from different tunings are grouped/comparable.

**Summary record** (last line, written on Ctrl-C or clean shutdown): total duration, samples written, wheel-integrated distance, straight-line displacement, peak `fusion↔gps` error, peak `wheel↔gyro` yaw drift, peak `cog↔fusion` yaw gap, RTK cov-check totals + verdict, final battery voltage, final BT state.

**Reading a log back:** [`ros2/scripts/diagnostics/analyze_session.py <file.jsonl>`](../../ros2/scripts/diagnostics/analyze_session.py) prints a one-shot summary (pose/yaw ranges, σ_xy trend, gyro mean rate, wheel↔gyro drift, RTK cov-check counters); [`session_timeline.py <file.jsonl>`](../../ros2/scripts/diagnostics/session_timeline.py) prints every 100th sample (5 s at `--rate 20`, 10 s at the 10 Hz default) to locate *when* an anomaly started. Both take the JSONL path and are run inside the container after a `docker cp` — see [`ros2/scripts/diagnostics/README.md`](../../ros2/scripts/diagnostics/README.md) for the full monitor→drive→analyze workflow.

**Log directory:** `docker/logs/mow_sessions/<session_name>.jsonl`. Commit notable sessions (golden runs, failure cases) so they survive in git history.

**GNSS container-level facts** (restart count, `receiver_node`/`ntrip_node` exit/restart logs) are Docker-level, not ROS — the monitor has no visibility into them. Run [`ros2/scripts/diagnostics/capture_gnss_container_health.sh <session-name>`](../../ros2/scripts/diagnostics/capture_gnss_container_health.sh) on the HOST, alongside the monitor, with the SAME session name, when investigating mowglinext#694.
