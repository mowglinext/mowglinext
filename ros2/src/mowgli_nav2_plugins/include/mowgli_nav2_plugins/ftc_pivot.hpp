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
// In-place PIVOTS at the coverage planner's explicit corners.
//
// When no forward turn-around arc fits between two adjacent passes, the planner
// keeps the row-end join inside the sub-path as a short straight whose
// zero-radius corners the robot must PIVOT at (field 2026-09-21: otherwise 128
// sub-paths, one blade-off transit per row end). FTC cannot drive such a corner
// while moving: the carrot crosses it, the heading error changes side and the
// angular command alternates at its clamp (2026-09-09: 29 sign flips in 3.3 s).
// So at a corner FTC:
//
//   FOLLOWING  the carrot is CAPPED at the corner's first pose, so the robot
//              drives straight to it and never steers at a carrot beyond it;
//   -> PIVOT   once base_link is within kPivotArrivalToleranceM of the corner
//              along its heading: stop, rotate in place to the corner's second
//              pose (the outgoing heading) with PRE_ROTATE's angular control and
//              limits, no lateral offset;
//   -> FOLLOWING from the second pose once aligned within tolerance.
//
// Corner contract (mowgli_interfaces/coverage_geometry.hpp): a corner is two
// consecutive poses at the same position, first with the incoming heading,
// second with the outgoing one; nothing else in a coverage plan has a
// zero-length step, so ordinary curvature (turn-around arcs, fillets, ring
// corners) can never trigger a pivot.
//
// Everything here is pure (no ROS), unit-tested in test_ftc_pivot.cpp.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "mowgli_interfaces/coverage_geometry.hpp"

namespace mowgli_nav2_plugins
{

/// A plan pose reduced to what corner detection needs.
struct PlanPose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Planner guarantee: a corner turns by more than kPivotCornerMinTurnRad. FTC
/// detects at half of it so float noise in the pose yaws cannot hide a corner.
inline constexpr double kPivotCornerDetectTurnRad =
    0.5 * mowgli_interfaces::coverage_geometry::kPivotCornerMinTurnRad;

/// How far ahead of base_link (along its heading) the corner may still be when
/// the pivot starts. The approach runs at min_speed_mps (0.15 m/s) with the
/// carrot pinned on the corner, i.e. 1.5 cm per 10 Hz tick: 2 cm fires on the
/// first tick within reach, so the pivot centre lands within about one tick of
/// the corner either side.
inline constexpr double kPivotArrivalToleranceM = 0.02;

inline double WrapPivotAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

/// Indices k of the pivot corners of a plan: pose k (incoming heading) and
/// pose k + 1 (outgoing heading) at the same position, turning by at least
/// kPivotCornerDetectTurnRad. The outgoing pose is never the last pose (there is
/// nothing to follow after it) and corners never chain.
inline std::vector<std::size_t> FindPivotCorners(const std::vector<PlanPose2D>& poses)
{
  namespace cg = mowgli_interfaces::coverage_geometry;
  std::vector<std::size_t> corners;
  for (std::size_t k = 0; k + 2 < poses.size(); ++k)
  {
    const double step = std::hypot(poses[k + 1].x - poses[k].x, poses[k + 1].y - poses[k].y);
    if (step >= cg::kPivotCornerMaxStepM)
    {
      continue;
    }
    if (std::abs(WrapPivotAngle(poses[k + 1].yaw - poses[k].yaw)) < kPivotCornerDetectTurnRad)
    {
      continue;
    }
    if (!corners.empty() && corners.back() + 1 == k)
    {
      continue;
    }
    corners.push_back(k);
  }
  return corners;
}

/// First corner at or after `index`, if any.
inline std::optional<std::size_t> NextPivotCorner(const std::vector<std::size_t>& corners,
                                                  std::size_t index)
{
  const auto it = std::lower_bound(corners.begin(), corners.end(), index);
  if (it == corners.end())
  {
    return std::nullopt;
  }
  return *it;
}

/// Where a freshly received plan starts tracking. A plan that OPENS on a corner
/// (FollowStrip resumed it exactly there) starts at the corner's outgoing pose:
/// the robot is already at that position and PRE_ROTATE turns it straight to
/// the outgoing heading instead of first to the incoming one and then pivoting.
inline std::size_t PivotAwareStartIndex(const std::vector<std::size_t>& corners, std::size_t start)
{
  return std::binary_search(corners.begin(), corners.end(), start) ? start + 1 : start;
}

/// True once the robot has reached the corner the carrot is capped on: the
/// corner lies no more than `tolerance_m` ahead along the robot's heading (or
/// already behind it).
inline bool PivotArrived(double corner_x_in_robot_frame_m, double tolerance_m)
{
  return corner_x_in_robot_frame_m <= tolerance_m;
}

/// True once the robot faces the outgoing heading within `tolerance_rad`.
inline bool PivotAligned(double heading_error_rad, double tolerance_rad)
{
  return std::abs(WrapPivotAngle(heading_error_rad)) < tolerance_rad;
}

/// [first, last] plan indices of the straight LEG containing `index`: from the
/// outgoing pose of the previous corner to the incoming pose of the next one.
/// Anything that reasons along the path (carrot resync, obstacle windows) must
/// stay inside it — across a corner the path heading, and so every lateral
/// offset, is discontinuous.
inline std::pair<std::size_t, std::size_t> PivotLeg(const std::vector<std::size_t>& corners,
                                                    std::size_t index,
                                                    std::size_t plan_size)
{
  std::size_t first = 0;
  std::size_t last = plan_size > 0 ? plan_size - 1 : 0;
  for (const std::size_t k : corners)
  {
    if (k + 1 <= index)
    {
      first = k + 1;
    }
    else if (k >= index)
    {
      last = k;
      break;
    }
  }
  return {first, std::max(first, last)};
}

/// Clip a window [start, end) of plan indices so it never extends past the
/// corner (the corner's incoming pose is the last one included).
inline std::size_t ClipWindowToCorner(std::size_t start,
                                      std::size_t end,
                                      std::optional<std::size_t> corner)
{
  if (!corner.has_value() || *corner < start)
  {
    return end;
  }
  return std::min(end, *corner + 1);
}

/// Headings at which to probe the footprint for the rotation from `from` to
/// `to` (radians, shortest direction), at most `max_step` apart, both ends
/// included. A non-positive step probes the two ends only.
inline std::vector<double> PivotSweepYaws(double from, double to, double max_step)
{
  const double delta = WrapPivotAngle(to - from);
  std::size_t n = 1;
  if (max_step > 0.0)
  {
    n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(std::abs(delta) / max_step)));
  }
  std::vector<double> yaws;
  yaws.reserve(n + 1);
  for (std::size_t i = 0; i <= n; ++i)
  {
    yaws.push_back(from + delta * static_cast<double>(i) / static_cast<double>(n));
  }
  return yaws;
}

}  // namespace mowgli_nav2_plugins
