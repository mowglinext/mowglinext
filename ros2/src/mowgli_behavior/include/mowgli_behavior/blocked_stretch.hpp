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
// What FollowStrip does with the REST of a unit once an obstacle stretch has
// beaten every detour (issue #607).
//
// A detour's blade-off resume transit that fails, with the detour budget spent,
// used to skip the WHOLE remaining unit. Since #716 a unit is a continuous
// sub-path that can carry every headland ring of the lawn: field 2026-10-10, a
// 4 m hairpin of the outer ring against a hedge beat five detours and the skip
// threw away 382 m of a 1016 m plan (the four inner rings all the way round).
//
// Instead the unit is resumed PAST the blocked stretch: at the first pose that
// is at least `min_skip_m` of arc beyond the failed target AND clear of every
// position already recorded as a failed transit / stuck point this session.
// The same no-progress resume budget as any other same-unit resume
// (unit_resume.hpp) bounds the retries, so a stretch that keeps failing still
// ends the unit.
//
// Pure / ROS-message-only: unit-tested in test_blocked_stretch.cpp.

#pragma once

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/transit_avoidance.hpp"

namespace mowgli_behavior
{

/// Minimum arc skipped past a failed detour target. Long enough to leave a
/// hairpin / spike of the outer ring (the field case spanned ~4 m of path in
/// ~1.5 m of lawn) instead of landing in the same pinch one pose later.
inline constexpr double kBlockedStretchMinSkipM = 2.0;

/// A resume pose closer than this to a recorded failed target / stuck point is
/// in the blocked stretch. About a chassis length: the body of a robot parked
/// there would overlap the robot that already failed there.
inline constexpr double kBlockedStretchClearRadiusM = 0.60;

/// Arc driven since the last detour/resume that refills the detour budget: an
/// obstacle 10 m further along is a new obstacle, not the same one again.
inline constexpr double kDetourBudgetResetM = 10.0;

/// Index of the first pose in (`from`, end) whose arc distance from `from` is
/// at least `min_skip_m` and that lies outside `clear_radius_m` of every entry
/// of `blocked`. nullopt when no such pose is left.
inline std::optional<std::size_t> resumePastBlockedStretch(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    std::size_t from,
    const std::vector<FailedTransitTarget>& blocked,
    double min_skip_m = kBlockedStretchMinSkipM,
    double clear_radius_m = kBlockedStretchClearRadiusM)
{
  double arc = 0.0;
  for (std::size_t i = from + 1; i < poses.size(); ++i)
  {
    const auto& a = poses[i - 1].pose.position;
    const auto& b = poses[i].pose.position;
    arc += std::hypot(b.x - a.x, b.y - a.y);
    if (arc < min_skip_m)
    {
      continue;
    }
    if (!isKnownFailedTransit(b.x, b.y, blocked, clear_radius_m))
    {
      return i;
    }
  }
  return std::nullopt;
}

}  // namespace mowgli_behavior
