# High-Level Commands and States

> Reference for `mowgli_interfaces` `HighLevelControl.srv` / `HighLevelStatus.msg` and the BT flows they drive. Loaded on demand from [`../../CLAUDE.md`](../../CLAUDE.md). Deeper per-file detail: [`codemaps/mowgli_behavior.md`](codemaps/mowgli_behavior.md), [`codemaps/mowgli_interfaces.md`](codemaps/mowgli_interfaces.md).

## HighLevelControl.srv Commands
| Value | Constant | Description |
|-------|----------|-------------|
| 1 | `COMMAND_START` | Begin autonomous mowing |
| 2 | `COMMAND_HOME` | Return to dock |
| 3 | `COMMAND_RECORD_AREA` (alias `COMMAND_S1`) | Start area boundary recording. Both constants are declared with value 3 |
| 4 | `COMMAND_S2` | Mow next area. **Normalised to `COMMAND_START` in the service handler** (`behavior_tree_node.cpp:561`) — there is no separate "next area" BT branch; mowing always resumes at the next un-mowed area via `GetNextUnmowedArea` |
| 5 | `COMMAND_RECORD_FINISH` | Finish recording, save polygon |
| 6 | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| 7 | `COMMAND_MANUAL_MOW` | Enter manual mowing mode (teleop + blade) |
| 8 | `COMMAND_STOP` | Stop-in-place hold: mower off, halt, stay put (NOT dock — that is `COMMAND_HOME`). Routes to `StopHoldSequence`. Resumable via `COMMAND_START`. GUI Pause / Stop use this. |
| 254 | `COMMAND_RESET_EMERGENCY` | Reset latched emergency. **Declared constant only — no BT branch guards on 254.** The GUI re-arms via the `/hardware_bridge/emergency_stop` service (`MowerStatus.tsx`), and the BT auto-resets on dock with its `ResetEmergency` node |
| 255 | `COMMAND_DELETE_MAPS` | Delete all maps. **Declared constant only — no BT branch guards on 255.** The GUI wipes areas via `/map_server_node/clear_map` |

## HighLevelStatus.msg States
| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `HIGH_LEVEL_STATE_NULL` | Emergency or transitional |
| 1 | `HIGH_LEVEL_STATE_IDLE` | Idle, docked, charging, stop-hold, waiting out rain, mowing/recording complete |
| 2 | `HIGH_LEVEL_STATE_AUTONOMOUS` | Autonomous mowing (undocking, transit, mowing, recovering **and `RETURNING_HOME`** — the drive to the dock is state 2, not 1) |
| 3 | `HIGH_LEVEL_STATE_RECORDING` | Area recording in progress |
| 4 | `HIGH_LEVEL_STATE_MANUAL_MOWING` | Manual mowing via teleop |

## Area Recording Flow
1. GUI sends `COMMAND_RECORD_AREA` (3) to start recording
2. BT enters `RecordArea` node — samples position at `area_record_rate_hz` (default **10 Hz**, from `mowgli_robot.yaml` via the blackboard; points closer than `kMinSampleSpacingM` = 0.05 m are dropped). Live preview is republished on `~/recording_trajectory` at `kPreviewPublishRateHz` = 2 Hz
3. User drives robot along boundary
4. GUI sends `COMMAND_RECORD_FINISH` (5) — trajectory is simplified (Douglas-Peucker, tolerance `area_simplification_tolerance`, default 0.05 m) and saved via `/map_server_node/add_area`
5. Or GUI sends `COMMAND_RECORD_CANCEL` (6) — trajectory discarded

## Stop-in-place / Pause
- `COMMAND_STOP` (8) routes to the BT `StopHoldSequence`: mower off, halt where it stands, and hold (state 1 / `IDLE`) — it does **not** drive to the dock (that is `COMMAND_HOME`), and unlike `IdleSequence` it does **not** PAUSE the Nav2 lifecycle, so a paused mission resumes promptly. Resumable with `COMMAND_START` (picks up from the persisted `mow_progress`); `~/clear_coverage_resume` is the operator's "start fresh" alternative. The GUI's Pause and "Stop Manual" buttons issue this; it is a true pause, distinct from Home/dock.

## GUI Scheduler and the IrriSense Soil Gate
The GUI's scheduler (`gui/pkg/providers/scheduler.go`) polls its `schedule:*` DB entries every minute and issues `COMMAND_START` (1) when one is due, after `safeToStart()` (no emergency, state not NULL/AUTONOMOUS/RECORDING). It then asks the soil provider (`types.ISoilProvider`, implemented by `gui/pkg/providers/irrisense.go`, the operator's **IrriSense Cloud** irrigation service polled every 10 min over its read-only `/api/ha/gardens/{id}` API with a read-only API token stored ONLY in the GUI key-value DB under `irrisense.*` — never in `mowgli_robot.yaml`). A zone is WET if enabled and (`deficit_mm ≤ wetDeficitMm` [2.0] OR `last_watered_at` within `dryAfterWateringHours` [3.0]); the garden is WET if ANY selected zone is (all enabled zones unless `irrisense.zoneIds` lists some) — pure rule in `irrisense_wetness.go`. The run is skipped ONLY when `Enabled && GateScheduler && Fresh && Wet` (`SoilStatus.BlocksScheduledMowing()`), and the skip is recorded on the schedule as `lastSkipReason` / `lastSkippedAt`. Everything else is **fail-open**: unreachable service, bad token, 429 (backed off per `Retry-After`), or a last good fetch older than `maxStaleMinutes` [90] reports `unknown` and the mow starts as before — a cloud outage must never keep the robot docked. Surface: `GET/PUT /api/irrisense/settings` (token write-only, masked on read), `GET /api/irrisense/status`, `GET /api/irrisense/gardens`; Settings → IrriSense section and the chip on the Schedule page.

## Manual Mowing
- Dedicated BT state with `COMMAND_MANUAL_MOW` (7) → `ManualMowingSequence` — does not hijack recording mode
- Teleop via `/cmd_vel_teleop` (twist_mux priority 20, above navigation's 10)
- Blade is driven by the **BT, not the GUI**: `ManualMowingSequence` ticks `SetMowerEnabled(true)` *after* `PublishHighLevelStatus(4)`, so the firmware has already mirrored `HL_MODE_MANUAL_MOWING` (it zeroes `target_blade_on_off` while its HL mode is IDLE/NULL). The GUI deliberately sends **no** `mow_enabled=1` — that call raced the firmware and killed the blade
- GPS and the map-frame localizer keep running, and collision_monitor stays up — but it filters the **Nav2 lane only** (`cmd_vel_nav` → `cmd_vel_monitored`), so teleop bypasses it. Firmware safety and the hardware_bridge dig detector (on the merged `~/cmd_vel`) are the backstops
- Exit with `COMMAND_STOP` (8) — the GUI's "Stop Manual" button; it halts in place and turns the blade off without driving to the dock

> Protocol constants (`HL_MODE_*`) are manually mirrored in `firmware/stm32/ros_usbnode/include/mowgli_protocol.h` AND `ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` — keep both in sync with `HighLevelStatus.msg` (see [`commands.md`](commands.md) → Code Generation Workflow).
