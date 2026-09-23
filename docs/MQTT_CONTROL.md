# MQTT control & monitoring

`mowgli_monitoring/mqtt_bridge_node` (`ros2/src/mowgli_monitoring/`) bridges a handful of ROS2
topics to a plain MQTT broker, so external tools — a Home Assistant integration, a mobile app, a
Node-RED flow — can watch the mower and issue high-level commands without talking ROS2 directly.
It is read-only monitoring, except for one inbound command topic that relays straight through to
the same `HighLevelControl` service the GUI's own buttons use.

This is the stable, versioned contract external integrations should build against — not the GUI's
internal REST/WebSocket API on `:4006` (unauthenticated, unversioned, an implementation detail of
the bundled frontend) and not the GUI's own separate embedded MQTT broker
(`gui/pkg/providers/mqtt.go`, a different prefix/payload shape used by the web UI).

## Enabling it

1. Set the broker connection in the GUI: **Settings → MQTT / Home Assistant**. Toggling it on
   writes `mqtt_enabled: true` (plus host/port/credentials/prefix/TLS) into the installed
   `mowgli_robot.yaml` — see root `CLAUDE.md` Invariant 15. There is no file to hand-edit, and this
   toggle alone is enough to start `mqtt_bridge_node` — no `.env` change needed.
2. Point `mqtt_host` at either the bundled broker (`localhost` — the `mowgli-mqtt` container from
   `install/compose/docker-compose.mqtt.yml`, composed in when `ENABLE_MQTT=true` in `.env`) or any
   external broker already running on your network (e.g. one on your Home Assistant server). These
   are independent switches on purpose: `ENABLE_MQTT` only decides whether the *bundled* broker
   container exists, `mqtt_enabled` only decides whether the bridge *node* runs — pointing the node
   at an external broker needs no bundled broker at all.
3. Restart the ROS2 stack for the new params to take effect (`mqtt_bridge_node`'s parameters are
   read once at startup, like every other node here).

### Home Assistant auto-discovery

Turn on **Home Assistant auto-discovery** in the same settings section to publish one retained
device discovery record at `homeassistant/device/<derived-id>/config`. Home Assistant then creates
a single MowgliNext device containing:

- lawn-mower state and the standard start, pause and dock controls;
- one explicit **Mow &lt;area name&gt;** button for every current mowable area;
- battery, coverage, GPS quality, RTK state and GPS location;
- charging, emergency and rain indicators;
- blade speed/current, battery voltage and charge current.

The bridge republishes the record whenever it reconnects and whenever Home Assistant announces
`online` on `homeassistant/status`. It also refreshes discovery when the polled mowable-area list
changes, so area buttons appear, rename and disappear with the map. A button press publishes the
area's current positional index to `<prefix>/start_area`; changing a map can reassign those indices,
so each button's discovery identity includes both its index and name. Home Assistant replaces the
button rather than silently leaving an existing automation aimed at a different physical area.

An area is an explicit button instead of a select entity because changing an MQTT select sends its
command immediately; choosing an item in a dropdown must not unexpectedly start a physical mower.
Turning discovery off publishes an empty retained record for the current topic prefix, which removes
the discovered device. Home Assistant's default discovery prefix (`homeassistant`) is used. Give
each mower connected to one broker a distinct `mqtt_topic_prefix`; that prefix also supplies its
stable Home Assistant device and entity IDs.

Changing a mower's topic prefix changes its discovery ID. Turn discovery off and restart once
before changing the prefix if the old retained device should be removed automatically.

## Security

The bundled broker (`install/config/mqtt/mosquitto.conf`) allows **anonymous connections on both
1883 (TCP) and 9001 (WebSocket), with no TLS**. `<mqtt_topic_prefix>/command` accepts any
high-level command from anyone who can reach the broker — treat it exactly like the GUI's own
unauthenticated `:4006` API (`gui/CLAUDE.md`): fine on a trusted LAN, never expose it to the
internet. Set `mqtt_username`/`mqtt_password`/`mqtt_use_ssl` if your own broker enforces auth/TLS.

## Topics

All topics are `<mqtt_topic_prefix>/<name>` (default prefix `mowgli`). Every payload is UTF-8 JSON
unless noted otherwise. QoS 1 throughout.

| Topic | Direction | Retained | Source | Rate |
|-------|-----------|----------|--------|------|
| `<prefix>/status` | out | yes | `/hardware_bridge/status` | latest value, at most `publish_rate` Hz |
| `<prefix>/power` | out | yes | `/hardware_bridge/power` | latest value, at most `publish_rate` Hz |
| `<prefix>/emergency` | out | yes | `/hardware_bridge/emergency` | every ROS message (not rate-limited) |
| `<prefix>/high_level_status` | out | yes | `/behavior_tree_node/high_level_status` | every ROS message (~1 Hz, not rate-limited) |
| `<prefix>/position` | out | no | `/wheel_odom` (**odom frame**, not GPS) | `publish_rate` Hz |
| `<prefix>/gps` | out | no | `/gps/fix` (raw `NavSatFix`) | `publish_rate` Hz |
| `<prefix>/pose` | out | no | `/odometry/filtered_map` (fused localizer pose, **map frame**) | latest value, at most `publish_rate` Hz |
| `<prefix>/rtk_status` | out | yes | `/gps/status` (`GnssStatus`) | latest value, at most `publish_rate` Hz |
| `<prefix>/area_boundary` | out | yes | `/map_server_node/get_mowing_area` (polled) | on change, polled every 10 s |
| `<prefix>/diagnostics` | out | no | `/diagnostics` | on change |
| `<prefix>/available` | out | yes | connection state (LWT) | on connect/disconnect |
| `<prefix>/areas` | out | yes | `/map_server_node/get_mowing_area` (polled) | ~every 10s |
| `<prefix>/command` | **in** | no (retained deliveries rejected) | → `/behavior_tree_node/high_level_control` | — |
| `<prefix>/start_area` | **in** | no (retained deliveries rejected) | → `/behavior_tree_node/start_in_area` | — |

### `<prefix>/high_level_status` — the primary "is it mowing?" topic

This is the one most integrations want first — it's the same state machine the GUI's dashboard
reads. Every field of `mowgli_interfaces/msg/HighLevelStatus.msg`:

```json
{
  "state": 2,
  "state_name": "AUTONOMOUS",
  "sub_state_name": "MOWING",
  "current_area": 2,
  "current_path": 3,
  "current_path_index": 0,
  "total_swaths": 40,
  "completed_swaths": 12,
  "skipped_swaths": 1,
  "coverage_percent": 42.5,
  "gps_quality_percent": 99.0,
  "battery_percent": 73.5,
  "is_charging": false,
  "emergency": false
}
```

`gps_quality_percent` is a genuine 0–100 percent on the wire — the bridge scales it up from the
underlying ROS field, which (despite its name) is actually a 0.0–1.0 fraction at the source
(`mowgli_behavior/src/status_snapshot.cpp` assigns the BT context's `gps_quality` — itself
`std::clamp(..., 0.0f, 1.0f)` — straight into `HighLevelStatus.gps_quality_percent` with no ×100).
If you're reading this field via any *other* path than `<prefix>/high_level_status` (e.g. straight
off the `/behavior_tree_node/high_level_status` ROS topic), remember it's 0.0–1.0 there, not 0–100.

**Field-observed staleness (mowglinext#644):** the bridge's subscription to the underlying ROS topic
has been seen to go stale for extended periods (30+ minutes) on a real deployment, continuing to
report old data on `<prefix>/high_level_status` while the ROS topic itself stayed fresh and this
node otherwise stayed connected. `mqtt_bridge_node` now watches for this — behavior_tree_node
republishes the ROS topic unconditionally at least once a second, so several seconds of silence on
that subscription makes the bridge recreate it automatically, with no restart needed. If you're
seeing this topic disagree with the mower's actual state for more than a few seconds, check the
bridge's own log for a "recreating the subscription" warning before assuming a code bug elsewhere.

`state` values (`mowgli_interfaces/msg/HighLevelStatus.msg`):

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `NULL` | Emergency / transitional |
| 1 | `IDLE` | Docked, charging, stop-hold, rain-wait, or mow complete — check `is_charging` and `sub_state_name` to tell these apart |
| 2 | `AUTONOMOUS` | Undocking, transit, mowing, recovering, **or returning to the dock** — driving home is state 2, not 1 |
| 3 | `RECORDING` | Recording an area boundary |
| 4 | `MANUAL_MOWING` | Manual/teleop mowing |

See [`docs/claude/high-level-api.md`](claude/high-level-api.md) for the full behaviour behind each
state and every `sub_state_name`.

### `<prefix>/status`

```json
{
  "mower_status": 0,
  "raspberry_pi_power": true,
  "is_charging": false,
  "esc_power": true,
  "rain_detected": false,
  "sound_module_available": true,
  "sound_module_busy": false,
  "ui_board_available": true,
  "mow_enabled": false,
  "mower_esc_status": 0,
  "mower_esc_temperature": 34.50,
  "mower_esc_current": 0.120,
  "mower_motor_temperature": 32.10,
  "mower_motor_rpm": 0.0
}
```

### `<prefix>/power`

```json
{
  "v_charge": 16.500,
  "v_battery": 15.800,
  "charge_current": 0.000,
  "charger_enabled": false,
  "charger_status": "idle",
  "battery_pct": 90.9
}
```

`battery_pct` is derived from `v_battery` over the 12.0–16.8 V 4S LiPo range and clamped to
[0, 100] — the same formula `diagnostics_node` uses. `<prefix>/high_level_status.battery_percent`
is the BT's own estimate and is the one the GUI dashboard shows; the two normally agree closely.

### `<prefix>/emergency`

```json
{"active_emergency": false, "latched_emergency": false, "reason": ""}
```

### `<prefix>/position` (odom frame — NOT for a map)

```json
{"x": 1.2345, "y": -6.7890, "theta": 0.0000}
```

Local `x`/`y` in metres from `/wheel_odom`'s origin, not georeferenced. Use `<prefix>/gps` instead
for anything that needs a real-world location (e.g. a Home Assistant `device_tracker`).

### `<prefix>/gps`

```json
{"latitude": 52.12345678, "longitude": -6.98765432, "altitude": 45.230, "status": 0, "service": 1}
```

Raw relay of `sensor_msgs/msg/NavSatFix` — `status` is `NavSatStatus.status`
(-1 `NO_FIX`, 0 `FIX`, 1 `SBAS_FIX`, 2 `GBAS_FIX`); it does **not** distinguish RTK Fixed from
Float, so don't read it as an RTK-quality signal — use `<prefix>/rtk_status` for that.

### `<prefix>/rtk_status`

```json
{"fix_type": 3, "fix_type_name": "RTK_FIXED", "rtk_mode": 3, "rtk_mode_name": "FIXED", "fix_valid": true, "quality_percent": 100}
```

Relay of `/gps/status` (`mowgli_interfaces/msg/GnssStatus`) — the **same** typed status and
`gnss_status_utils` helpers the robot's own LED ring and behavior tree read, so this can never
disagree with what the robot itself shows (e.g. the LED ring's amber "mowing without RTK fix"
pattern, or the GUI's own GPS % health-check card). `quality_percent` is
`gnss_status_utils::HardwareQualityPercent()` — a genuine 0–100 — not `GnssStatus.quality_percent`
directly, whose own population is backend-dependent and not guaranteed to be on that scale.

| Field | Values |
|-------|--------|
| `fix_type` / `fix_type_name` | 0 `NO_FIX`, 1 `GPS_FIX`, 2 `RTK_FLOAT`, 3 `RTK_FIXED`, 4 `DEAD_RECKONING` |
| `rtk_mode` / `rtk_mode_name` | 0 `UNKNOWN`, 1 `NONE`, 2 `FLOAT`, 3 `FIXED` |
| `fix_valid` | Overrides everything else — a stale/leftover `fix_type` with `fix_valid: false` means no usable fix, full stop |

### `<prefix>/area_boundary`

```json
{
  "datum_lat": 52.12345678,
  "datum_lon": 4.56789012,
  "areas": [
    {
      "index": 0,
      "name": "Front Lawn",
      "boundary": [[1.234, -0.567], [10.0, -0.567], [10.0, 8.0], [1.234, 8.0]],
      "obstacles": [[[3.0, 2.0], [4.0, 2.0], [4.0, 3.0], [3.0, 3.0]]]
    }
  ],
  "dock": {"x": 12.5, "y": -3.25, "yaw": 1.5708}
}
```

`dock` is the charging dock's pose in the same map frame (`dock_pose_x/y/yaw` from
`mowgli_robot.yaml`; `yaw` in radians, the heading the robot has when docked). It is **omitted**
when no dock is calibrated (all three values still at their `0/0/0` default) or any value is not
finite. The bridge reads it at startup, like the other consumers of these keys, so a dock
re-calibration shows up after a restart.

Polygon geometry for every recorded mowing area, so an external tool (e.g. a Home Assistant map
card) can render the boundary and obstacles the robot's own GUI shows. `datum_lat`/`datum_lon` are
the map-frame origin (`mowgli_robot.yaml`'s datum — the same one `map_server_node` and the
localizer use, see root `CLAUDE.md` Invariant 4): every `[x, y]` pair is a **map-frame offset in
metres** from that datum (east/north, equirectangular projection — the same math as
`wgs84_projection.hpp`), not a lat/lon pair itself. To place a point on a real map, project it back
through the datum with the same equirectangular formula. `boundary` is the area's outer polygon
(`MapArea.area`); `obstacles` is a list of polygons (`MapArea.obstacles`), one entry per obstacle,
empty when the area has none. Navigation-only areas (`MapArea.is_navigation_area`) are excluded —
they aren't mowed, so there's nothing useful to draw.

This topic is polled independently of the `<prefix>/areas` name-list topic above (each runs its own
`GetMowingArea` poll loop, on the same 10s cadence but not synchronised) — it exists purely to
describe geometry for drawing, not to identify areas for a `<prefix>/start_area` command. Indices
are **not guaranteed stable or contiguous** across a session (mowglinext#637) — match on `name`,
not `index`, if you need to correlate with `<prefix>/areas`. The bridge polls
`/map_server_node/get_mowing_area` every 10 seconds (index 0, 1, 2, … until the service reports
`success: false`, capped at 100 areas) and only republishes (retained) when the serialised geometry
actually changed, so a static map does not spam the broker.

### `<prefix>/pose`

```json
{"x": 12.5, "y": -3.25, "yaw": 1.5708}
```

The mower's fused pose in the **map frame** — the same frame as the polygons in
`<prefix>/area_boundary` and the `dock` there: `x` east, `y` north, in metres from the datum, `yaw`
in radians counter-clockwise from east (−π…π]. It is `/odometry/filtered_map`, the localizer's
canonical global pose, so it is smoother than the raw `<prefix>/gps` fix (which jitters, especially
at the dock) and it carries a heading, which `<prefix>/gps` does not. It needs no datum to be
placed on the `area_boundary` geometry. Not retained; a localizer that has not converged yet (non-
finite values) publishes nothing.

### `<prefix>/diagnostics`

```json
[
  {"name": "GPS", "level": 0, "message": "OK"},
  {"name": "LiDAR", "level": 1, "message": "No LiDAR scan received"}
]
```

`level`: 0 OK, 1 WARN, 2 ERROR (standard `diagnostic_msgs/DiagnosticStatus` levels).

### `<prefix>/available` (Last Will and Testament)

Retained `"online"` (published once connected, and again on every reconnect) or `"offline"`
(published by the broker on an ungraceful disconnect via LWT, or explicitly by the bridge on a
clean shutdown). Plain text, not JSON. Subscribe to this to distinguish "mower offline" from "mower
online but silently stuck" — the latter still updates `<prefix>/diagnostics`/`<prefix>/status`.

### `<prefix>/command` (inbound)

Payload is an **ASCII decimal integer string**, e.g. `"1"` — **not a raw byte**. The entire payload
must be digits only: whitespace, signs, and trailing characters are rejected. This is the single
most common mistake integrating against this topic: publish the string `"1"`, not the byte `0x01`.
Retained deliveries are rejected: an operator command must be a fresh publish, not broker state.

| Code | Constant | Effect |
|------|----------|--------|
| 1 | `COMMAND_START` | Start/resume mowing (or manual-resume from a charging hold above `battery_manual_resume_percent`) |
| 2 | `COMMAND_HOME` | Return to dock |
| 3 | `COMMAND_RECORD_AREA` / `COMMAND_S1` | Begin recording an area boundary |
| 5 | `COMMAND_RECORD_FINISH` | Finish recording, save the area |
| 6 | `COMMAND_RECORD_CANCEL` | Cancel recording, discard |
| 7 | `COMMAND_MANUAL_MOW` | Enter manual/teleop mowing |
| 8 | `COMMAND_STOP` | Pause in place (mower off, holds position — does **not** dock) |
| 254 | `COMMAND_RESET_EMERGENCY` | Declared but not wired on this channel — see below |
| 255 | `COMMAND_DELETE_MAPS` | Declared but not wired on this channel — see below |

Full semantics (e.g. what "start" does depending on current state): see
[`docs/claude/high-level-api.md`](claude/high-level-api.md). `4` (`COMMAND_S2`) is normalised
server-side to `COMMAND_START`.

The call is fire-and-forget: an unrecognised or out-of-range payload is logged and dropped with no
error published back to MQTT, and a valid command is dropped silently if
`/behavior_tree_node/high_level_control` isn't available (still starting up, or the BT node is
down). There is no ack/result topic — poll `<prefix>/high_level_status` after sending a command to
confirm it took effect.

`RESET_EMERGENCY` (254) and `DELETE_MAPS` (255) are declared constants in
`HighLevelControl.srv` but have **no BT guard on this channel** in practice — the GUI re-arms an
emergency via the separate `/hardware_bridge/emergency_stop` service and clears maps via
`/map_server_node/clear_map`, not through `HighLevelControl`. Don't rely on sending 254/255 over
MQTT to do either.

### `<prefix>/areas` (recorded mow areas)

```json
[
  {"index": 0, "name": "Front Lawn"},
  {"index": 2, "name": "Back Garden"}
]
```

Polled from `/map_server_node/get_mowing_area` roughly every 10 seconds (walking index 0, 1, 2, …
until the service reports `success=false` — the same pattern the GUI backend's own map polling
uses) and republished, retained, only when the resulting list actually changed. Navigation-only
areas (keepout/boundary zones that are never mowed) are excluded.

**⚠️ Interim, index-based contract — expect this to change.** `index` is the *raw*, purely
*positional* index `map_server_node` uses internally — recorded areas have **no stable ID** yet
([mowglinext#637](https://github.com/mowglinext/mowglinext/issues/637) tracks adding one). The
GUI's own area editor rebuilds its entire area list on any single-area add/edit/delete, which can
reassign *every* area's index in the process — so **do not cache an index across a session**.
Re-fetch `<prefix>/areas` and re-resolve the target by `name` before sending `<prefix>/start_area`
each time. Once #637 lands, this topic is expected to grow a stable `id` field and
`<prefix>/start_area` an id-based counterpart; this index-only shape is a stepping stone, not the
final contract — don't build a permanent integration against it without accounting for that.

### `<prefix>/start_area` (inbound — start mowing a specific area)

Payload is an **ASCII decimal integer string** matching the `index` field from `<prefix>/areas`
(same strict digits-only convention as `<prefix>/command` — publish `"2"`, not the byte `0x02`).
Retained deliveries are rejected, so this must be a fresh publish. Relays straight
through to `/behavior_tree_node/start_in_area`, which starts mowing that area now, **ahead of the
normal area-iteration order** — exactly as consequential as `<prefix>/command`'s `COMMAND_START`
(it raises that internally too). Same fire-and-forget contract: no ack/result topic, an
unrecognised/out-of-range payload is logged and dropped, and the command is dropped silently if
`/behavior_tree_node/start_in_area` isn't available. Poll `<prefix>/high_level_status` afterwards
to confirm it took effect. Subject to the same index-staleness caveat as `<prefix>/areas` above —
targeting a stale index can start the wrong area.

## Parameters

Read once at startup (`ros2/src/mowgli_monitoring/include/mowgli_monitoring/mqtt_bridge_node.hpp`):

| Param | Default | Set via |
|-------|---------|---------|
| `mqtt_host` | `localhost` | GUI Settings → MQTT (`mowgli_robot.yaml: mqtt_host`) |
| `mqtt_port` | `1883` | `mqtt_port` |
| `mqtt_username` / `mqtt_password` | `""` / `""` | `mqtt_username` / `mqtt_password` |
| `mqtt_topic_prefix` | `mowgli` | `mqtt_topic_prefix` |
| `use_ssl` | `false` | `mqtt_use_ssl` |
| `home_assistant_discovery_enabled` | `false` | `mqtt_home_assistant_discovery_enabled` |
| `mqtt_client_id` | `mowgli_ros2` | package-share `mqtt_bridge.yaml` only (not on the GUI) |
| `publish_rate` | `1.0` Hz | package-share `mqtt_bridge.yaml` only — also the rate limit for position/gps/status/power/rtk_status. The MQTT network loop runs on its own fixed 50 ms timer, independent of this |
