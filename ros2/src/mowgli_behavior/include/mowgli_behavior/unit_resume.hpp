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
//
// What FollowStrip does with the REST of a coverage unit after a non-obstacle
// abort or cancel.
//
// It used to step the resume cursor past the abort and then advance() to the
// NEXT unit. The cursor is one scalar into the concatenation of all units, so
// completing any later unit moved it past everything that had been left behind.
// Field, 2026-09-17: the serpentine unit (30 127 poses, 84 % of the lawn) was
// cancelled at pose 280; units 7-9 were then driven, the cursor read
// 35794/35796 and the area was booked 100 % MOWED after 20 minutes.
//
// The remainder of the SAME unit is therefore re-dispatched from the stepped
// cursor. What bounds it is not a count per unit — a 1.1 km unit legitimately
// meets many path features — but the number of CONSECUTIVE resumes that made no
// real progress, which is the actual signature of a deterministic re-abort loop.

#pragma once

#include <cstddef>

namespace mowgli_behavior
{

struct UnitResumeCfg
{
  /// Consecutive no-progress resumes tolerated before the unit is given up.
  std::size_t max_consecutive{5};
  /// Poses driven since the last resume that count as real progress and clear
  /// the consecutive counter.
  std::size_t progress_reset_poses{135};
  /// Below this many poses left, the remainder is not worth a transit.
  std::size_t min_remaining_poses{27};
};

struct UnitResumeDecision
{
  bool resume{false};
  /// Counter value to store for the next abort of this unit.
  std::size_t consecutive{0};
};

/// @param unit_poses      poses in the (already trimmed) current unit
/// @param reached_idx     furthest pose reached in it before the abort
/// @param resume_idx      stepped cursor the remainder would restart from
/// @param consecutive     no-progress resumes already spent on this unit
inline UnitResumeDecision DecideUnitResume(std::size_t unit_poses,
                                           std::size_t reached_idx,
                                           std::size_t resume_idx,
                                           std::size_t consecutive,
                                           const UnitResumeCfg& cfg = {})
{
  const std::size_t spent = reached_idx >= cfg.progress_reset_poses ? 0 : consecutive;
  if (resume_idx >= unit_poses || unit_poses - resume_idx < cfg.min_remaining_poses)
  {
    return {false, spent};
  }
  if (spent >= cfg.max_consecutive)
  {
    return {false, spent};
  }
  return {true, spent + 1};
}

}  // namespace mowgli_behavior
