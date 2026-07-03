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
/// path pose count per area (`area_path_pose_count`, used as a staleness key —
/// a changed area re-plans to a different pose count and the stale cursor is
/// discarded), the completed-swath sets (`area_completed_swaths`), the
/// completed-area set (`completed_areas`), and the current area index.
///
/// The file is written atomically (temp + rename) and every reader tolerates a
/// missing / empty / corrupt file by returning false and leaving the context
/// untouched — a lost or unreadable resume file simply means "start fresh",
/// never a crash.

/// Write ctx's coverage resume state to ctx.coverage_resume_path.
/// No-op (returns false) when the path is empty. Returns true on a successful
/// write.
bool saveCoverageResumeState(const BTContext& ctx);

/// Load coverage resume state from ctx.coverage_resume_path INTO ctx.
/// Returns false (leaving ctx unchanged) when the path is empty, the file is
/// absent, or it cannot be parsed. Returns true when at least the header was
/// recognised and any state present was loaded.
bool loadCoverageResumeState(BTContext& ctx);

/// Remove the persisted resume file (called by EndSession at a real session
/// boundary so the next COMMAND_START does not resume a finished session).
/// Returns true if the file was removed or was already absent.
bool clearCoverageResumeState(const BTContext& ctx);

}  // namespace mowgli_behavior
