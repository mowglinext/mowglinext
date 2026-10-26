// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <chrono>
#include <cstdint>

namespace mowgli_behavior
{

struct BTContext;

/// Whether the mission has progress to resume. Empty per-area completion sets
/// and plan/orientation metadata alone do not count. Area identity and plan
/// compatibility are still validated when the area is probed and dispatched.
bool hasCoverageResumeState(const BTContext& ctx);

/// Disk persistence of the coverage RESUME state, so an interrupted mowing
/// session survives a full process/container restart (reboot, crash,
/// `docker restart`, a power-cycle after an emergency stop) — not just the
/// in-process BT halt/resume that BTContext already handles in RAM.
///
/// Persisted fields (the minimum needed to resume near where the robot
/// stopped): per-area resume cursor (`area_resume_pose_index`), the planned
/// path pose count per area (`area_path_pose_count`), the plan-geometry
/// fingerprint per area (`area_plan_fingerprint`, the resume-cursor staleness
/// key — a changed area/params re-plans to a different geometry, so the stale
/// cursor is discarded; a pose-count-only key was insufficient because the AUTO
/// mow-angle tie-break or sub-path split can change geometry at equal count),
/// the completed-swath sets (`area_completed_swaths`), the completed-area set
/// (`completed_areas`), the current area index, and — mowglinext#637 phase 2
/// — the stable area id last observed for each index (`area_ids`), so a
/// resume can detect that the GUI's area list changed since this file was
/// written and discard per-index state that would otherwise be misattributed
/// to whatever area now sits at that index.
///
/// The file is written atomically (temp + rename) and every reader tolerates a
/// missing / empty / corrupt file by returning false and leaving the context
/// untouched — a lost or unreadable resume file simply means "start fresh",
/// never a crash.

/// Write ctx's coverage resume state to ctx.coverage_resume_path.
/// No-op (returns false) when the path is empty. Returns true on a successful
/// write. Every snapshot carries a `saved_at` row (wall-clock seconds) — the
/// age the next boot judges restored progress by; `written` exists for tests.
bool saveCoverageResumeState(const BTContext& ctx);
bool saveCoverageResumeState(const BTContext& ctx, std::chrono::system_clock::time_point written);

bool beginCoverageOrientation(BTContext& ctx, uint32_t area);
void markCoverageStarted(BTContext& ctx, uint32_t area);

/// Load coverage resume state from ctx.coverage_resume_path INTO ctx.
/// Returns false (leaving ctx unchanged) when the path is empty, the file is
/// absent, or it cannot be parsed. Returns true when at least the header was
/// recognised and any state present was loaded.
bool loadCoverageResumeState(BTContext& ctx);

/// What a freshly loaded snapshot is still allowed to do at process start.
struct RestoredCommandPolicy
{
  bool auto_continue{false};  ///< START + resumable progress: re-enter MowingSequence.
  bool preserve_stop_hold{false};  ///< STOP latched during a critical charge hold.
  bool cleared_charge_pending{false};  ///< A saved battery_charge_pending was dropped.
  bool expired_progress{false};  ///< Progress older than kIncompleteResumeMaxAge was discarded.
};

/// Apply the boot policy to a context just filled by loadCoverageResumeState.
/// First the AGE: recorded progress whose snapshot (`saved_at`) is older than
/// kIncompleteResumeMaxAge is discarded (discardRestoredCoverageProgress) — it
/// no longer describes the lawn, and a day-old START must not move the mower
/// by itself when power returns. Field 2026-10-25: the age-out existed only in
/// RAM, so after three updater restarts Play resumed a 6-day-old "4 of 5
/// sub-paths done" snapshot, mowed the one ring unit left and declared the
/// lawn complete. Then the command: only an auto-continued mow keeps
/// current_command = START; a latched critical charge STOP keeps STOP;
/// everything else boots IDLE. Progress that boots IDLE is flagged as kept
/// (BTContext::resume_after_incomplete_end, aged from `saved_at`), so the
/// START handler applies the same age-out at Play as it does within one
/// process. battery_charge_pending survives ONLY together with an
/// auto-continued mow: on its own it would make an IDLE mower drive to the
/// dock right after power-on, without a command and without proving the
/// battery is still low (PR #1010 review). Battery recovery at IDLE stays
/// covered by the critical-battery entry, which re-reads the live battery.
/// When the flag is dropped the snapshot is re-saved after the first tick
/// (critical_dock_failure_persistence_requested). Not thread-safe; call before
/// subscribers and timers exist. The clocks exist for tests.
RestoredCommandPolicy applyRestoredCommandPolicy(
    BTContext& ctx,
    std::chrono::system_clock::time_point wall_now = std::chrono::system_clock::now(),
    std::chrono::steady_clock::time_point steady_now = std::chrono::steady_clock::now());

/// Drop every piece of restored coverage progress (completed swaths, cursors,
/// per-unit progress, finished areas, plan fingerprints, pose counts, the
/// single-area target) and the kept-progress flag. What describes the lawn's
/// phase (cross-hatch orientation) stays. Does not touch the file; the caller
/// re-saves.
void discardRestoredCoverageProgress(BTContext& ctx);

/// Consume the first live power sample after auto-restoring a valid mow.
/// A restart on the charger continues charging before any mission motion,
/// including snapshots predating battery_charge_pending. Caller serializes
/// access with the context mutex, as in the power subscriber.
void observeRestoredMowPower(BTContext& ctx, bool charging);

/// Clear commands/cursors at EndSession or an explicit progress reset. Retain
/// only cross-hatch orientation metadata when present; this cannot auto-start
/// the mower. EndSession must finish the phase before calling this function.
/// Remove the old resume file before writing phase metadata, so a failed write
/// cannot leave an old START command behind. Returns false if removal fails or
/// phase metadata cannot be saved (in the latter case resume state is already
/// gone). Callers must report failures; EndSession must still clear the live
/// command. Empty path / already absent file without metadata returns true.
bool clearCoverageResumeState(const BTContext& ctx);

}  // namespace mowgli_behavior
