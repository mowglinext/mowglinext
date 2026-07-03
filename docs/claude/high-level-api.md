# High-Level Commands and States

> Reference for `mowgli_interfaces` `HighLevelControl.srv` / `HighLevelStatus.msg` and the BT flows they drive. Loaded on demand from [`../../CLAUDE.md`](../../CLAUDE.md).

## HighLevelControl.srv Commands
| Value | Constant | Description |
|-------|----------|-------------|
| 1 | `COMMAND_START` | Begin autonomous mowing |
| 2 | `COMMAND_HOME` | Return to dock |
| 3 | `COMMAND_RECORD_AREA` | Start area boundary recording |
| 4 | `COMMAND_S2` | Mow next area |
| 5 | `COMMAND_RECORD_FINISH` | Finish recording, save polygon |
| 6 | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| 7 | `COMMAND_MANUAL_MOW` | Enter manual mowing mode (teleop + blade) |
| 8 | `COMMAND_STOP` | Stop-in-place hold: mower off, halt, stay put (NOT dock — that is `COMMAND_HOME`). Routes to `StopHoldSequence`. Resumable via `COMMAND_START`. GUI Pause / Stop use this. |
| 254 | `COMMAND_RESET_EMERGENCY` | Reset latched emergency |
| 255 | `COMMAND_DELETE_MAPS` | Delete all maps |

## HighLevelStatus.msg States
| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `HIGH_LEVEL_STATE_NULL` | Emergency or transitional |
| 1 | `HIGH_LEVEL_STATE_IDLE` | Idle, docked, charging, returning home |
| 2 | `HIGH_LEVEL_STATE_AUTONOMOUS` | Autonomous mowing (undocking, transit, mowing, recovering) |
| 3 | `HIGH_LEVEL_STATE_RECORDING` | Area recording in progress |
| 4 | `HIGH_LEVEL_STATE_MANUAL_MOWING` | Manual mowing via teleop |

## Area Recording Flow
1. GUI sends `COMMAND_RECORD_AREA` (3) to start recording
2. BT enters `RecordArea` node — records position at 2 Hz, publishes live preview on `~/recording_trajectory`
3. User drives robot along boundary
4. GUI sends `COMMAND_RECORD_FINISH` (5) — trajectory is simplified (Douglas-Peucker) and saved via `/map_server_node/add_area`
5. Or GUI sends `COMMAND_RECORD_CANCEL` (6) — trajectory discarded

## Stop-in-place / Pause
- `COMMAND_STOP` (8) routes to the BT `StopHoldSequence`: mower off, halt where it stands, and hold — it does **not** drive to the dock (that is `COMMAND_HOME`). Resumable with `COMMAND_START` (picks up from the persisted `mow_progress`). The GUI's Pause / Stop buttons issue this; it is a true pause, distinct from Home/dock.

## Manual Mowing
- Dedicated BT state with `COMMAND_MANUAL_MOW` (7) — does not hijack recording mode
- Teleop via `/cmd_vel_teleop` (twist_mux priority)
- Blade managed by GUI (fire-and-forget to firmware)
- Collision_monitor, GPS, and the active map-frame localizer all remain active

> Protocol constants (`HL_MODE_*`) are manually mirrored in `firmware/stm32/ros_usbnode/include/mowgli_protocol.h` AND `ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` — keep both in sync with `HighLevelStatus.msg` (see [`commands.md`](commands.md) → Code Generation Workflow).
