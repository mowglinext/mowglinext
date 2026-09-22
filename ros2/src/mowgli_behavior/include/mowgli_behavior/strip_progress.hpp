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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mowgli_behavior
{

/// Longest stretch of PATH (arc length) FollowStrip's progress cursor may
/// advance in one update [m]. The robot moves ~0.03 m per 10 Hz tick, so this
/// leaves a wide margin for a slow tick while keeping the search off the far
/// side of a full turn-around.
///
/// Field 2026-09-22: the cursor searched the nearest pose over the next 400 poses
/// (~12 m). Whenever the robot was more than half a swath spacing (6.5 cm) off its
/// line — every obstacle skirt — the nearest pose was on the next swath, and the
/// monotonic cursor never came back. Units were booked "reached 100 % → MOWED" at
/// ~80 % of FTC's own index, resumes after an abort started up to 37 m of path too
/// far ahead, and ~20 m² of a 152 m² lawn was planned but never driven.
///
/// This bound alone does NOT make a jump to the neighbouring serpentine swath
/// impossible, and an earlier version of this comment wrongly claimed it did
/// (issue #742). It assumed every row end carried a full turn-around arc. Since
/// #716 a short non-tangent join is a PIVOT JOIN whose connector is about one
/// swath spacing, so with 0.13 m spacing the abeam pose on the RETURN swath sits
/// at only 0.40 + 0.13 + 0.40 = 0.93 m of arc from a robot 0.40 m short of the
/// row end — inside this window, and nearer than its own line as soon as the
/// robot runs more than half a spacing wide. kMaxProgressHeadingDiffRad is what
/// actually closes that case; shrinking this bound cannot, because the offending
/// connector is shorter than any window the cursor can still keep up with.
constexpr double kMaxProgressAdvanceM = 1.0;

/// Widest disagreement between the robot's heading and a candidate pose's
/// orientation that still counts as the robot being ON that pose [rad].
///
/// A serpentine return swath is ANTIPARALLEL to the one being driven, so 90°
/// rejects it for as long as the robot is still heading up the current row —
/// which is exactly the window issue #742 exploits. It opens again once the
/// robot has genuinely rotated onto the next row, by which point it is at the
/// corner and advancing the cursor IS correct. A pivot corner (two poses at one
/// position, incoming then outgoing heading) therefore unblocks itself as the
/// robot turns through it, and a ring corner up to 107° behaves the same way.
///
/// Generous on purpose: it must never reject the pose the robot is actually on.
/// FTC holds 1.4 cm median / 3.3 cm p90 cross-track and its heading follows the
/// path through turn-around arcs and lateral skirts, so nothing legitimate comes
/// close to 90°.
constexpr double kMaxProgressHeadingDiffRad = 1.5707963267948966;  // pi/2

/// Yaw of a quaternion [rad]. Local so this header stays free of tf2 — it is
/// pure and unit-tested, and tf2::getYaw on a geometry_msgs quaternion drags in
/// a tf2_geometry_msgs link dependency for one atan2. Returns nullopt for a
/// quaternion that was never populated (all zero), which must not be read as
/// "heading 0".
inline std::optional<double> quaternionYaw(const geometry_msgs::msg::Quaternion& q)
{
  const double norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (norm2 < 1e-6)
  {
    return std::nullopt;
  }
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Yaw of a pose [rad]; nullopt when its orientation was never populated.
inline std::optional<double> poseYaw(const geometry_msgs::msg::Pose& pose)
{
  return quaternionYaw(pose.orientation);
}

/// Is `pose` oriented closely enough to `robot_yaw` for the robot to be on it?
/// An unoriented pose is admissible — we cannot judge it, and refusing every
/// such pose would freeze the cursor on a path that carries no orientations.
inline bool headingAdmissible(const geometry_msgs::msg::Pose& pose, double robot_yaw)
{
  const std::optional<double> yaw = poseYaw(pose);
  if (!yaw.has_value())
  {
    return true;
  }
  const double d = std::atan2(std::sin(*yaw - robot_yaw), std::cos(*yaw - robot_yaw));
  return std::abs(d) <= kMaxProgressHeadingDiffRad;
}

/// Monotonic progress cursor into a coverage unit: the pose nearest to the robot
/// (rx, ry) among those at most `max_advance_m` of arc length ahead of `cursor`
/// AND oriented within kMaxProgressHeadingDiffRad of the robot's heading
/// `robot_yaw`. Never moves backwards; an empty path leaves the cursor
/// unchanged. Proximity alone is not evidence of traversal — see issue #742.
inline std::size_t advanceProgressCursor(const std::vector<geometry_msgs::msg::PoseStamped>& poses,
                                         std::size_t cursor,
                                         double rx,
                                         double ry,
                                         double robot_yaw,
                                         double max_advance_m = kMaxProgressAdvanceM)
{
  if (poses.empty())
  {
    return cursor;
  }
  cursor = std::min(cursor, poses.size() - 1);
  double best_d2 = std::numeric_limits<double>::max();
  std::size_t best = cursor;
  double arc = 0.0;
  for (std::size_t i = cursor; i < poses.size(); ++i)
  {
    const auto& p = poses[i].pose.position;
    if (i > cursor)
    {
      const auto& q = poses[i - 1].pose.position;
      arc += std::hypot(p.x - q.x, p.y - q.y);
      if (arc > max_advance_m)
      {
        break;
      }
    }
    if (!headingAdmissible(poses[i].pose, robot_yaw))
    {
      continue;  // antiparallel: the return swath, not this one (issue #742)
    }
    const double d2 = (p.x - rx) * (p.x - rx) + (p.y - ry) * (p.y - ry);
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

/// How closely a pose FTC republished must match a unit pose to be taken for
/// it: the controller copies poses verbatim, so anything but float-exact means
/// a different pose.
constexpr double kControllerRejoinMatchTolerance = 1e-6;

/// Longest jump along the unit a controller rejoin may make the cursor take
/// [m]: FTC's turn fallback skips at most turn_fallback_max_rejoin_arc_m
/// (3.0 m shipped, parameter cap 10.0 m) past the robot, and the cursor may
/// trail the robot by up to kMaxProgressAdvanceM.
constexpr double kMaxControllerRejoinJumpM = 12.0;

/// FTC's turn fallback (mowgli_nav2_plugins/ftc_turn_fallback.hpp) improvises
/// a blocked turn and REJOINS the unit further on. It then republishes the rest
/// of the unit, from the rejoin pose, on FollowCoveragePath/global_plan — a
/// jump advanceProgressCursor can never follow: the skipped turn is often more
/// than kMaxProgressAdvanceM of path, and at a U-turn the cursor's own pose, on
/// the other swath 0.13 m away, stays nearer to the robot than anything in its
/// window for the rest of the unit (test_strip_progress.cpp pins both).
///
/// The republished front pose is an EXACT copy of one of the unit's poses: find
/// it ahead of `cursor` (within `max_arc_m` of path). Nothing but an exact
/// position AND orientation match counts, so a neighbouring swath or ring can
/// never be taken for it. nullopt when there is no such pose ahead.
inline std::optional<std::size_t> findControllerRejoin(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses,
    std::size_t cursor,
    const geometry_msgs::msg::Pose& rejoin,
    double max_arc_m = kMaxControllerRejoinJumpM)
{
  const auto same = [&rejoin](const geometry_msgs::msg::Pose& p)
  {
    constexpr double tol = kControllerRejoinMatchTolerance;
    return std::abs(p.position.x - rejoin.position.x) <= tol &&
           std::abs(p.position.y - rejoin.position.y) <= tol &&
           std::abs(p.orientation.z - rejoin.orientation.z) <= tol &&
           std::abs(p.orientation.w - rejoin.orientation.w) <= tol;
  };
  double arc = 0.0;
  for (std::size_t i = cursor + 1; i < poses.size(); ++i)
  {
    const auto& p = poses[i].pose.position;
    const auto& q = poses[i - 1].pose.position;
    arc += std::hypot(p.x - q.x, p.y - q.y);
    if (arc > max_arc_m)
    {
      break;
    }
    if (same(poses[i].pose))
    {
      return i;
    }
  }
  return std::nullopt;
}

}  // namespace mowgli_behavior
