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

/**
 * @file area_progress.hpp
 * @brief Did a pass add coverage execution? (GetNextUnmowedArea's no-progress budget.)
 *
 * The budget used to count only units booked FULLY mowed. A large area drawn with few points
 * plans into very few, very long units (rings + swaths, hundreds of metres each), so a pass
 * that mowed a long stretch and then stopped inside a unit — a transit refused at the next
 * boundary, an obstacle, a dig — counted as "nothing new": five of those retired an area that
 * was still being mowed. Coverage execution credit (BTContext::area_unit_execution)
 * measures newly observed progress, excluding skipped spans and replay. Recovery
 * offsets (area_unit_progress) also include selected targets and must not provide
 * this credit. Each credit consumes a fixed share of the finite plan.
 *
 * Pure and header-only so it is unit-testable without ROS.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>

namespace mowgli_behavior
{

/// Software coverage evidence used by the no-progress budget, not physical cut evidence.
struct AreaProgressMark
{
  /// Processed units whose ledger certifies no known gaps.
  std::size_t done_units = 0;
  /// Observed execution poses, including processed units; historical field name.
  std::size_t partial_poses = 0;
};

/// About 1 m at the coverage planner's 0.03 m pose spacing: the least an unfinished unit must
/// gain to count, on a small plan.
constexpr std::size_t kAreaProgressMinPartialPoses = 34;

/// Sum of logical resume offsets. Not coverage execution or retry credit.
inline std::size_t partialUnitPoses(const std::map<std::size_t, std::size_t>* unit_progress)
{
  std::size_t sum = 0;
  if (unit_progress == nullptr)
    return sum;
  for (const auto& unit_furthest : *unit_progress)
    sum += unit_furthest.second;
  return sum;
}

/// The least partial gain that counts for a plan of `total_poses`: `share_pct` of it, never
/// below kAreaProgressMinPartialPoses. Proportional so that a trickle of a few metres per pass
/// on a kilometre-long plan cannot keep the area alive for thousands of passes.
inline std::size_t minPartialProgressPoses(std::size_t total_poses, float share_pct)
{
  const double share = static_cast<double>(total_poses) * static_cast<double>(share_pct) / 100.0;
  const auto proportional = static_cast<std::size_t>(std::ceil(std::max(0.0, share)));
  return std::max(kAreaProgressMinPartialPoses, proportional);
}

/// Whether the pass that ended at `now` made progress since `last` (none = first dispatch).
/// A newly covered unit always counts. With the same covered units, observed execution
/// must have grown by `min_partial_gain`. Fewer covered units
/// than last time (the area's record was reset) does not count: the caller's high-water mark
/// stays where it was, exactly as before partial progress was considered.
inline bool areaPassMadeProgress(const std::optional<AreaProgressMark>& last,
                                 const AreaProgressMark& now,
                                 std::size_t min_partial_gain)
{
  if (!last)
    return true;
  if (now.done_units > last->done_units)
    return true;
  return now.done_units == last->done_units &&
         now.partial_poses >= last->partial_poses + min_partial_gain;
}

}  // namespace mowgli_behavior
