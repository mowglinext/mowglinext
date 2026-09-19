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
// Session "dig skip zones" — the anti re-dig protection of issue #500, kept in
// the coverage-following layer so that it can NEVER block planning.
//
// A wheel-slip dig (hardware_bridge DigEvent) used to be stamped into the
// keepout mask as a PENDING keepout. FTC never read it (it follows the ACTIVE
// path and only looks at the local costmap), so it kept pushing through the
// hole, and once the strip was cancelled the robot stood INSIDE its own
// keepout: every transit was refused with START_OCCUPIED and the mission died
// mid-lawn (2026-09-10, 2026-09-17). A dig is now only a PROPOSAL in
// map_server (operator accept/reject); what keeps the robot out of the hole
// for the rest of the session is this header:
//
//   * every coverage pose within `radius_m` of a recorded dig point is SKIPPED:
//     FollowStrip dispatches the drivable run before the zone, then reaches the
//     first pose past it with the existing blade-off transit and continues;
//   * on a dig DURING a coverage goal, FollowStrip cancels the goal (FTC must
//     not keep pushing along the old path), waits for the bridge's bounded
//     reverse to finish (DigSettle*), and resumes the SAME unit past the zone.
//
// Nothing here marks a costmap cell, so "plan from the robot's own pose" is
// untouched by construction. The bridge's repeat-dig escalation
// (dig_escalation.hpp → DigObstructionGuard) stays the backstop.
//
// Pure / ROS-message-only: unit-tested in test_dig_skip.cpp.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mowgli_behavior
{

/// One recorded dig location, map frame [m].
struct DigPoint
{
  double x{0.0};
  double y{0.0};
};

/// Fallback skip radius when the launch file injects none (tests, ad-hoc
/// launches). The real value is DERIVED from the merged robot config:
/// robot_config_util.dig_skip_radius() = the chassis circumscribed radius.
inline constexpr double kDefaultDigSkipRadiusM = 0.60;

/// Two digs closer than this are the same hole: the second one is not stored.
inline constexpr double kDigPointMergeDistM = 0.10;

/// Upper bound on stored dig points per session. Far above anything a real
/// session produces (the bridge escalates after 3 same-spot latches); it only
/// keeps the per-dispatch scan bounded if something goes badly wrong.
inline constexpr std::size_t kMaxSessionDigPoints = 64;

/// A drivable run shorter than this (arc length) between two skip zones is not
/// worth a PRE_ROTATE + blade spin-up: it is skipped with the zones around it.
inline constexpr double kDigMinRunLengthM = 1.0;

/// New dig list with `p` recorded. Returns the list unchanged when `p` merges
/// into an existing point; drops the OLDEST point when the bound is reached.
inline std::vector<DigPoint> recordDigPoint(const std::vector<DigPoint>& digs,
                                            const DigPoint& p,
                                            double merge_dist_m = kDigPointMergeDistM,
                                            std::size_t max_points = kMaxSessionDigPoints)
{
  for (const auto& d : digs)
  {
    if (std::hypot(d.x - p.x, d.y - p.y) <= merge_dist_m)
    {
      return digs;
    }
  }
  std::vector<DigPoint> out = digs;
  out.push_back(p);
  if (max_points > 0 && out.size() > max_points)
  {
    out.erase(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(out.size() - max_points));
  }
  return out;
}

/// True when (x, y) lies within `radius_m` of any recorded dig point.
inline bool insideDigZone(double x, double y, const std::vector<DigPoint>& digs, double radius_m)
{
  if (radius_m <= 0.0)
  {
    return false;
  }
  const double r2 = radius_m * radius_m;
  return std::any_of(digs.begin(),
                     digs.end(),
                     [&](const DigPoint& d)
                     {
                       const double dx = d.x - x;
                       const double dy = d.y - y;
                       return dx * dx + dy * dy <= r2;
                     });
}

/// Half-open pose range [start, end) of a unit that may be driven blade-on.
struct DrivableRun
{
  std::size_t start{0};
  std::size_t end{0};
  bool empty() const
  {
    return start >= end;
  }
};

/// First run of consecutive poses at or after `from` that lie OUTSIDE every dig
/// zone and span at least `min_run_length_m` of arc length. Shorter runs —
/// including a sub-metre tail at the end of the unit — are skipped like the
/// zones around them: they are not worth a transit and a PRE_ROTATE.
///
/// Returns {poses.size(), poses.size()} when nothing drivable is left. With no
/// dig points (or radius <= 0) the whole remainder [from, size) is one run.
inline DrivableRun nextDrivableRun(const std::vector<geometry_msgs::msg::PoseStamped>& poses,
                                   std::size_t from,
                                   const std::vector<DigPoint>& digs,
                                   double radius_m,
                                   double min_run_length_m = kDigMinRunLengthM)
{
  const std::size_t n = poses.size();
  if (from >= n)
  {
    return {n, n};
  }
  if (digs.empty() || radius_m <= 0.0)
  {
    return {from, n};
  }

  const auto inside = [&](std::size_t i)
  {
    return insideDigZone(poses[i].pose.position.x, poses[i].pose.position.y, digs, radius_m);
  };

  std::size_t i = from;
  while (i < n)
  {
    while (i < n && inside(i))
    {
      ++i;
    }
    if (i >= n)
    {
      break;
    }
    const std::size_t start = i;
    double length = 0.0;
    ++i;
    while (i < n && !inside(i))
    {
      length += std::hypot(poses[i].pose.position.x - poses[i - 1].pose.position.x,
                           poses[i].pose.position.y - poses[i - 1].pose.position.y);
      ++i;
    }
    if (i - start >= 2 && length >= min_run_length_m)
    {
      return {start, i};
    }
  }
  return {n, n};
}

// ---------------------------------------------------------------------------
// Waiting for the bridge's bounded reverse.
//
// hardware_bridge owns the wire during its dig escape (dig_reverse_dist /
// dig_reverse_timeout_s) and publishes no "done" signal; anything Nav2 commands
// meanwhile is simply not executed. FollowStrip therefore waits until the robot
// has been STILL for a moment before it re-dispatches, bounded by max_wait_s so
// a robot that never settles (or no pose at all) cannot hang the pass.
// ---------------------------------------------------------------------------
struct DigSettleCfg
{
  /// Never re-dispatch sooner than this after the dig [s]: the reverse starts
  /// one bridge monitor tick after the hard stop, so an instant "still" reading
  /// is the stop, not the end of the escape.
  double min_wait_s{1.0};
  /// The pose must stay within still_dist_m for this long [s].
  double still_window_s{1.0};
  double still_dist_m{0.03};
  /// Hard bound [s]. Above the bridge's default 4 s reverse timeout + its 2 s
  /// detector re-arm, so by then the wire is ours again.
  double max_wait_s{8.0};
};

struct DigSettleState
{
  double elapsed_s{0.0};
  double still_s{0.0};
  bool have_anchor{false};
  double anchor_x{0.0};
  double anchor_y{0.0};
};

/// Advance the settle wait by `dt_s`. Returns true once the robot has settled
/// (or the hard bound expired). `have_pose` false = no TF this tick.
inline bool DigSettleStep(
    DigSettleState& st, const DigSettleCfg& cfg, double dt_s, bool have_pose, double x, double y)
{
  const double dt = dt_s > 0.0 ? dt_s : 0.0;
  st.elapsed_s += dt;
  if (have_pose)
  {
    if (!st.have_anchor || std::hypot(x - st.anchor_x, y - st.anchor_y) > cfg.still_dist_m)
    {
      st.have_anchor = true;
      st.anchor_x = x;
      st.anchor_y = y;
      st.still_s = 0.0;
    }
    else
    {
      st.still_s += dt;
    }
  }
  if (st.elapsed_s >= cfg.max_wait_s)
  {
    return true;
  }
  return st.elapsed_s >= cfg.min_wait_s && st.still_s >= cfg.still_window_s;
}

}  // namespace mowgli_behavior
