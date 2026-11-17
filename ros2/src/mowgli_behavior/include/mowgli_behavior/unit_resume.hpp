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
//
// Spending that budget must not cost the rest of the unit either. A hole-free
// area is one or two units, so "give up on the unit" meant giving up on the zone
// because of one bad spot; and a pass that gains less than 0.5 % of a large plan
// is not progress, so four such passes retired the area with the far side of the
// bad spot never tried. FollowStrip therefore leaps kUnitResumeLeapM ahead when
// the budget is spent and carries on: the cursor moves at least that far per
// budget, so the unit still ends, and what is lost is bounded to the bad stretch.

#pragma once

#include <cstddef>
#include <set>

namespace mowgli_behavior
{

/// First unit at or after `from` that is not recorded as mowed (`unit_count` when none is left).
///
/// FollowStrip drives an area's units in plan order and skips only those whose completion was
/// recorded: a unit left incomplete after a failed blade-off transit is driven again, and a
/// headland ring is driven before the interior. PlanCoverageArea uses the same rule to choose the
/// first transit goal, so the robot's first hop lands where the first goal is actually driven
/// (field 2026-10-12: it hopped to a resume point in unit 7 and FollowStrip then sent it back to
/// unit 1, 3.9 m away).
inline std::size_t nextUndoneUnit(std::size_t from,
                                  std::size_t unit_count,
                                  const std::set<std::size_t>& done)
{
  while (from < unit_count && done.count(from) > 0)
  {
    ++from;
  }
  return from;
}

/// Arc skipped when the no-progress budget is spent, instead of dropping the rest of the unit.
inline constexpr double kUnitResumeLeapM = 10.0;

struct UnitResumeCfg
{
  /// Consecutive no-progress resumes tolerated before the unit is given up.
  std::size_t max_consecutive{5};
  /// Poses driven since the last resume that count as real progress and clear
  /// the consecutive counter: about 1 m at the planner's 0.03 m spacing. A resume
  /// that actually drove is not the re-abort loop this budget exists to end, and
  /// the stepped cursor already keeps every resume moving forward along the unit.
  std::size_t progress_reset_poses{34};
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

/// True when DecideUnitResume refused only because the budget is spent, with enough of the unit
/// left to be worth a leap (kUnitResumeLeapM) rather than a drop.
inline bool UnitResumeBudgetSpent(const UnitResumeDecision& decision, const UnitResumeCfg& cfg = {})
{
  return !decision.resume && decision.consecutive >= cfg.max_consecutive;
}

}  // namespace mowgli_behavior
