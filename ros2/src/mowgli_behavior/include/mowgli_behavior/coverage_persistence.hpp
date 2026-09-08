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

#include <cstdint>

namespace mowgli_behavior
{

struct BTContext;

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
/// (`completed_areas`), and the current area index.
///
/// The file is written atomically (temp + rename) and every reader tolerates a
/// missing / empty / corrupt file by returning false and leaving the context
/// untouched — a lost or unreadable resume file simply means "start fresh",
/// never a crash.

/// Write ctx's coverage resume state to ctx.coverage_resume_path.
/// No-op (returns false) when the path is empty. Returns true on a successful
/// write.
bool saveCoverageResumeState(const BTContext& ctx);

bool beginCoverageOrientation(BTContext& ctx, uint32_t area);
void markCoverageStarted(BTContext& ctx, uint32_t area);

/// Load coverage resume state from ctx.coverage_resume_path INTO ctx.
/// Returns false (leaving ctx unchanged) when the path is empty, the file is
/// absent, or it cannot be parsed. Returns true when at least the header was
/// recognised and any state present was loaded.
bool loadCoverageResumeState(BTContext& ctx);

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
