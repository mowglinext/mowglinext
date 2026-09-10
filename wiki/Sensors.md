# Sensors

Sensor integration lives under `sensors/` and is consumed through the installer-generated compose stack.

## GNSS

MowgliNext exposes one GNSS contract to the rest of the system:

- `/gps/fix` — `sensor_msgs/msg/NavSatFix`
- `/gps/status` — typed GNSS status (`mowgli_interfaces/GnssStatus`) consumed by the GUI, the behavior tree, the hardware bridge and the LED ring. Dual-antenna heading arrives here as `baseline_azimuth_deg` / `baseline_solution_status`, not on a separate topic.
- `/rtcm` — public mirror of the NTRIP correction stream (`rtcm_msgs/Message`)
- `/diagnostics` — receiver + transport diagnostics

There is a single GNSS runtime: the **Universal GNSS sidecar** — the `mowgli-gps`
container built from `sensors/gps/` and started by `sensors/gps/start_gps.sh`.
`GNSS_STACK` accepts `universal` or `disabled` only; "no GNSS" is expressed by
not composing the container in at all, not by a fallback backend. The old
per-vendor runtimes (`sensors/unicore/`, `sensors/nmea/`, the standalone u-blox
driver) were removed — see [Retired GNSS paths](#retired-gnss-paths) below.

**RTK Fixed/Float flicker:** under motion an F9P's reported carrier solution
(`carrSoln`) can toggle Fixed↔Float every epoch even while position σ stays
~4 mm — a pure classification flicker, not a position problem. Two pieces of
the ROS2 stack absorb this so it doesn't propagate downstream:

- `localization_monitor_node` debounces the published localization mode
  (`mode_debounce_sec`, default 1.0 s) — see
  [Architecture › localization_monitor_node](Architecture#3c-localization_monitor_node).
- `corrections_active` follows the receiver's own differential-solution flag
  rather than a bursty RTCM freshness metric: the u-blox parser derives both
  `differential_corrections` and `corrections_active` from the solution flags
  (`SetCorrectionState()` in `gnss_protocols/src/ubx_parser.cpp`), and the
  Unicore parser maps the receiver's authoritative solution type. Both live in
  the Universal GNSS submodule; the sidecar bridge mirrors the result onto
  `/gps/status` unchanged.

### GNSS Flow

```text
Receiver
  -> Universal GNSS receiver_node
       |-> /gps/fix
       |-> /diagnostics
       `-> /_gps_internal/universal/status
             -> mowgli_gnss_bridge universal_gnss_topic_bridge
                  |-> /gps/status              (public mirror)
                  `-> /rtcm                    (public mirror)

/gps/fix
  |-> fusion_graph_node                        (GnssLeverArmFactor)
  `-> mowgli_localization/navsat_to_absolute_pose_node
        `-> /gps/absolute_pose + /gps/pose_cov (GUI / BT / map_server)

NTRIP corrections
  caster -> Universal GNSS ntrip_node
         -> /_gps_internal/universal/rtcm
         -> receiver_node
```

### Notes

- Universal GNSS is the only direct-GNSS stack. It does **not** run inside `mowgli-ros2`: `sensors/gps/start_gps.sh` launches `receiver_node`, the topic bridge and `ntrip_node` as three processes inside the `mowgli-gps` container. There is no `universal_gnss.launch.py` in `mowgli_bringup`.
- The installer writes the Universal GNSS env contract: `GNSS_STACK`, `GNSS_RECEIVER_FAMILY`, `GNSS_TRANSPORT`, `GNSS_SERIAL_DEVICE`, `GNSS_SERIAL_BAUD`, and `GNSS_NTRIP_*`. Empty values are deliberate ("not set") — `start_gps.sh` resolves every key **YAML → env → built-in default**, reading `/config/mowgli_robot.yaml` first.
- The persisted MowgliNext GNSS configurator seam now also accepts an optional `gnss_receiver_model` YAML field for Universal GNSS plan/apply tooling. When it is empty, MowgliNext calls Universal GNSS without `--model` and surfaces the tool's safe-fallback warnings instead of inferring anything from runtime state.
- Installer and GUI flows treat `GNSS_*` as the user-facing truth. `GNSS_BACKEND` and `GNSS_STATUS_SOURCE` survive as compatibility mirrors; the older `GPS_PORT` / `GPS_BAUD` / `GPS_PROTOCOL` / `UBLOX_*` / `UNICORE_*` keys are **actively deleted** from `docker/.env` on every install run (`remove_legacy_gnss_env_keys()` in `install/lib/env.sh`). `GPS_IMAGE` is unrelated — it is just the sidecar image name.
- For USB receivers, `GNSS_SERIAL_DEVICE` should normally be a stable `/dev/serial/by-id/...` path rather than a raw `ttyACM*` or `ttyUSB*` node.
- `921600` is the recommended validation baud for advanced u-blox and Unicore profiles.
- Corrected field validation on June 4, 2026 confirmed the Universal GNSS path on both a live u-blox F9P and a live Unicore UM982 with `/gps/fix`, `/gps/status`, `/diagnostics`, and `/rtcm` active in universal mode.
- The preferred stable receiver paths are `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00` for the F9P and `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` for the UM982.
- The corrected raw tty mappings from that validation were `/dev/ttyACM0` for the F9P and `/dev/ttyUSB0` for the UM982.
- The F9P stayed in RTK float through the sampled NTRIP window and did not reach RTK fixed.
- The UM982 mostly stayed in a plain fix state but did intermittently promote into RTK float while corrections were active.
- Raw UM982 output after the NTRIP session showed `PSRDIFF` position solutions and `RTKSTATUSA` records on the wire.
- `RTCMSTATUSA` was not observed in the sampled raw output windows, and Universal GNSS still treats it as semantic-only rather than a dedicated ROS status field.
- Universal GNSS owns `/gps/fix`, `/gps/status`, `/diagnostics`, and `/rtcm`. NMEA receivers route through it like any other family; there is no separate NMEA container and no Mowgli-local `/gps/status` reconstruction any more.
- The public Mowgli `/gps/status` schema mirrors the Universal GNSS
  projection directly in the GNSS sidecar bridge:
  - runtime-status fields keep canonical GNSS names such as
    `rtk_mode`, `baseline_azimuth_deg`, `baseline_pitch_deg`,
    `baseline_length_m`, and `baseline_solution_status`
  - diagnostics-derived correction-stream state is exposed separately through
    `correction_stream_status`
  - RTCM semantic MSM summary data is exposed separately through
    `msm_summary_*`
  - legacy `heading_deg`, `heading_accuracy_deg`, and
    `dual_antenna_heading` remain compatibility fields only during the current
    transition
- Runtime `/gps/status.receiver_model` remains observational/display-only in MowgliNext. It is not auto-copied into the persisted configurator `gnss_receiver_model`; an operator must choose that explicitly before MowgliNext forwards `--model` to Universal GNSS tools.
- The Universal GNSS submodule (`ros2/src/external/universal-gnss`) is pinned to
  the **mowglinext fork**, branch `main`, which tracks
  `Pepeuch/universal-gnss` main. Both fixes for issue #395 now live there:
  GLONASS-1230 treated as optional-for-RTK correction health, and a UM980
  `MODE ROVER UAV` default with a `--rover-dynamic-mode` override. The pin
  followed a stacked `fix/rover-dynamic-mode-uav` branch until 2026-09-04; that
  branch is gone and the fork's main is followed directly. The rationale lives
  in `.gitmodules`. Bumping it means re-pinning the gitlink, and a clone without
  `--recurse-submodules` silently misses the GNSS packages.
- Do not commit real `GNSS_NTRIP_PASSWORD` values or copy them into docs/logs.
- The devcontainer now mirrors the host `/dev` tree at `/host-dev` and re-links `/dev/serial/by-id` when the host provides it.
- Prefer `/dev/serial/by-id` when it is available. If it is missing in the runtime, confirm the live device through `/sys/class/tty/*/../manufacturer` and `/sys/class/tty/*/../product` only as a diagnostic fallback before wiring `GNSS_SERIAL_DEVICE`.
- Be cautious with raw `/dev/tty*` enumeration in containers: stale device nodes can persist even when the live sysfs mapping has changed.

### Retired GNSS paths

The per-vendor fallback runtimes are **gone** — there is nothing left to migrate
off, and nothing in the tree still references them. If you find one of these
names in an older document or a stale `docker/.env`, it is history, not a
supported option:

`sensors/unicore/` · `sensors/nmea/` · `sensors/gps/serial_ublox_driver.py` ·
`gnss_runtime_state_builder.cpp` · `gps_health_aggregator.py` ·
`rtcm_serial_bridge.py` · `ublox_gnss.launch.py` / `ublox_gnss.yaml` ·
`nmea_navsat_driver` · `ublox_dgnss_node` · `GNSS_STACK=legacy`

`sensors/gps/` itself is **not** legacy — it is the Universal GNSS sidecar
described above. `GNSS_STATUS_SOURCE` only ever resolves to `universal`, or to
`external` when GNSS is disabled or `HARDWARE_BACKEND=mavros` supplies the fix.
Compose generation asserts this: `install/lib/compose.sh` rejects any
`GNSS_STACK` other than `universal|disabled`, and the installer's compose and
env tests (`install/tests/test_compose_validity.sh`, `test_env_output.sh`) pin
the emitted values.

## LiDAR

LiDAR support is installer-selected through dedicated compose fragments. All
three variants run as the single `mowgli-lidar` container and publish `/scan`
(`sensor_msgs/LaserScan`, `frame_id: lidar_link`).

| `LIDAR_TYPE` | Image / driver | Port + baud |
|--------------|----------------|-------------|
| `ldlidar` (default) | `sensors/lidar-ldlidar/` — Myzhar `ldrobot-lidar-ros2`, LD06/LD14/LD19 | **Hardcoded** `/dev/lidar` @ 230400 in `ldlidar.yaml`; `LIDAR_PORT` / `LIDAR_BAUD` are ignored |
| `rplidar` | `sensors/lidar-rplidar/` — `rplidar_ros`, A2M8 launch file by default | `LIDAR_PORT` (`/dev/lidar`) + `LIDAR_BAUD` (115200) |
| `stl27l` | `sensors/lidar-stl27l/` — `ldlidar_stl_ros2` | `LIDAR_PORT` (`/dev/lidar`) + `LIDAR_BAUD` (921600) |
| `none` | no fragment emitted | — |

### Two separate LiDAR switches

`LIDAR_ENABLED` in `docker/.env` and `lidar_enabled` in `mowgli_robot.yaml` do
**different** jobs, and they can legitimately disagree:

- `LIDAR_ENABLED` decides only whether the **`mowgli-lidar` container** is
  composed in.
- `lidar_enabled` in `mowgli_robot.yaml` decides the **ROS2 side** — the
  `nav2_params_lidar` vs `nav2_params_no_lidar` overlay, the scan pipeline
  nodes, and the scan-based collision monitor. The `LIDAR_ENABLED` environment
  variable is **not read by the ROS2 stack at all** (removed 2026-08-31: as a
  "fallback when the yaml is silent" it let a stale `.env` silently override the
  operator's GUI toggle). An absent `lidar_enabled` key resolves to `false` with
  a loud startup warning naming the file and key.

See [Configuration › LiDAR](Configuration) for the full resolution rules.

### Scan pipeline

When `use_lidar` is true, the raw scan passes through two `mowgli_localization`
nodes before anything consumes it:

```text
/scan                            (driver container)
  -> scan_deskew_node            per-ray rotational deskew from the IMU buffer;
                                 passes through unchanged when the IMU is stale
  -> /scan_deskewed
  -> costmap_scan_filter_node
       |-> /scan_costmap         obstacle layers (ground-filtered)
       `-> /scan_collision       collision_monitor (deliberately NOT
                                 ground-filtered, so a slope-stripped
                                 obstacle still trips the near-field stop)
```

`scan_deskew_node` also warns after `scan_watchdog_period_s` (20 s) if no scan
ever arrives — that is the symptom of `lidar_enabled: true` with the container
not running.

`fusion_graph`'s scan-matching and loop-closure factors subscribe to
`/scan_deskewed`, and `use_scan_matching` / `use_loop_closure` are **ANDed with
`use_lidar`** in `navigation.launch.py` before they reach the node. Without that
AND a GPS-only stack ran the matcher against a topic nothing published, matching
nothing while reporting success-shaped diagnostics.

## Adding a New Sensor

1. Create a directory under `sensors/` with a Dockerfile. Pin the upstream
   driver by ref/SHA and apply any patches inside the Dockerfile — never vendor
   a patched upstream source tree into this repo.
2. Publish the standard contract: `/scan` (`LaserScan`, `frame_id: lidar_link`)
   for LiDAR, or `/gps/fix` + `/gps/status` + `/rtcm` for GNSS. Nothing in
   `sensors/` publishes TF or a pose.
3. Add a compose fragment in `install/compose/` and wire it into the
   `LIDAR_TYPE` / `GNSS_*` selection in `install/lib/compose.sh` and
   `install/lib/env.sh`.
4. Add a thin CI caller in `.github/workflows/` that reuses `_sensor-docker.yml`
   — images are built multi-arch (amd64 + arm64); the robot is arm64, so an
   arch-specific binary breaks only the real robot.

Contributor detail beyond this page lives in
[`sensors/CLAUDE.md`](https://github.com/mowglinext/mowglinext/blob/main/sensors/CLAUDE.md),
the deployment codemap
[`docs/claude/codemaps/deploy.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/codemaps/deploy.md),
and the topic/parameter indexes
[`docs/claude/ros-interfaces.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/ros-interfaces.md)
and
[`docs/claude/parameters.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/parameters.md).
