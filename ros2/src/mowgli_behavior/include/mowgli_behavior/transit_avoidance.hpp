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
// Session failed-transit-target avoidance (issue #732).
//
// FollowStrip's blade-off inter-unit NavigateToPose transit has no recovery
// beyond skip-and-retry-later: a transit blocked by a LiDAR-observed obstacle
// that is not drawn on the map (root CLAUDE.md Invariant 5 — coverage is
// never replanned around it) fails identically every time it is attempted.
// GetNextUnmowedArea's no-progress counter only resets on ANY progress
// elsewhere in the area, so a single permanently blocked transit was retried
// on literally every subsequent dispatch of that area — up to
// BTContext::kMaxAreaAttempts — each attempt burning the full
// transitDeadlineSec bound plus nav2's own internal retry cycle, before the
// area was finally given up on (field 2026-09-21: several minutes on one
// transit before the robot moved on).
//
// sendCurrentSwath already has a narrower version of this idea — it will not
// repeat the transit TransitToStrip has JUST failed (transit_to_strip_failed_
// at / sameTransitTarget) — but that guard is single-shot (consumed by the
// very next dispatch attempt, whatever its own target) and only wired for the
// area's FIRST unit. This header is the SESSION-scoped memory that guard
// lacked: once a transit to some target has failed (recorded by FollowStrip's
// transit_active_ abort handler), every LATER attempt at that same target
// this session — any unit, any dispatch — is short-circuited instead of
// repeating the failure.
//
// Pure / ROS-message-free: unit-tested in test_transit_avoidance.cpp. Mirrors
// dig_skip.hpp's recordDigPoint / insideDigZone shape.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace mowgli_behavior
{

/// One transit target that has already failed this session, map frame [m].
struct FailedTransitTarget
{
  double x{0.0};
  double y{0.0};
};

/// Two transit targets this close are the SAME destination — matches
/// sameTransitTarget's tolerance (coverage_nodes.hpp): FollowStrip's first
/// unit starts where TransitToStrip was sent, give or take the dig-zone front
/// trim, and the two mechanisms must agree on what counts as "the same
/// transit" or one could suppress a genuinely different target.
inline constexpr double kFailedTransitMergeDistM = 0.30;

/// Upper bound on remembered failed-transit targets per session. Far above
/// anything a real session produces; only keeps the per-dispatch scan bounded
/// if something goes badly wrong. Matches dig_skip.hpp's
/// kMaxSessionDigPoints rationale.
inline constexpr std::size_t kMaxSessionFailedTransits = 64;

/// New failed-transit list with `p` recorded. Returns the list unchanged when
/// `p` merges into an existing entry; drops the OLDEST entry when the bound
/// is reached.
inline std::vector<FailedTransitTarget> recordFailedTransit(
    const std::vector<FailedTransitTarget>& targets,
    const FailedTransitTarget& p,
    double merge_dist_m = kFailedTransitMergeDistM,
    std::size_t max_targets = kMaxSessionFailedTransits)
{
  for (const auto& t : targets)
  {
    if (std::hypot(t.x - p.x, t.y - p.y) <= merge_dist_m)
    {
      return targets;
    }
  }
  std::vector<FailedTransitTarget> out = targets;
  out.push_back(p);
  if (max_targets > 0 && out.size() > max_targets)
  {
    out.erase(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(out.size() - max_targets));
  }
  return out;
}

/// True when (x, y) is within `radius_m` of a target already recorded as
/// failed this session.
inline bool isKnownFailedTransit(double x,
                                 double y,
                                 const std::vector<FailedTransitTarget>& targets,
                                 double radius_m = kFailedTransitMergeDistM)
{
  if (radius_m <= 0.0)
  {
    return false;
  }
  const double r2 = radius_m * radius_m;
  return std::any_of(targets.begin(),
                     targets.end(),
                     [&](const FailedTransitTarget& t)
                     {
                       const double dx = t.x - x;
                       const double dy = t.y - y;
                       return dx * dx + dy * dy <= r2;
                     });
}

}  // namespace mowgli_behavior
