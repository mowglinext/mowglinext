// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Retrace: undo the LAST piece the robot drove, instead of reversing blindly.
//
// Nav2's BackUp reverses along the robot's CURRENT heading and only checks the local costmap, which
// does not contain the boundary. Next to the recorded line (where the outer rings and most transits
// run) that reverse can take the robot out of the area: field 2026-10-26, a 0.30 m BackUp took a
// robot from 0.21 m to 0.57 m outside, which is a lethal boundary violation and an emergency stop.
//
// A retrace only ever covers ground the robot has JUST driven over, so it cannot leave the area it
// came from, and it refuses (the caller falls through to the next recovery) whenever it would have
// to turn or drive forward to do it. Pure geometry, no ROS: the node owns the TF buffer and the
// motion.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>

namespace mowgli_nav2_plugins::retrace
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Limits
{
  /// Spacing of the recorded breadcrumbs. Closer samples are dropped.
  double min_step_m{0.02};
  /// How much driven path is kept (older breadcrumbs are discarded).
  double history_m{3.0};
  /// Two consecutive poses further apart than this are a discontinuity, not driving.
  double max_jump_m{0.5};
  /// A retrace shorter than this is not worth a manoeuvre (and its direction is too noisy to
  /// trust).
  double min_distance_m{0.08};
  /// The straight line behind the robot must be within this angle of its rear axis.
  double max_heading_error_rad{0.35};  // 20 degrees
  /// Every breadcrumb of the retraced piece must lie this close to the straight line back.
  double max_lateral_dev_m{0.05};
};

enum class Refusal
{
  kNone,
  kNoHistory,  // fewer than min_distance_m of path recorded
  kWrongDirection,  // the last piece was driven in REVERSE: undoing it means driving forward
  kNotStraight,  // the last piece curved or the robot has turned since: undoing it needs a turn
};

struct Plan
{
  Refusal refusal{Refusal::kNone};
  Pose2D target;  // where the retraced piece started
  double distance_m{0.0};  // straight-line distance from the current pose to the target
  double ux{0.0};  // unit vector from the current pose towards the target
  double uy{0.0};
};

inline double normalizeAngle(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

/// The recent path the robot actually drove, oldest first.
class Breadcrumbs
{
public:
  explicit Breadcrumbs(Limits limits = {}) : limits_(limits)
  {
  }

  void add(const Pose2D& pose)
  {
    // A jump is not driving (the mower was carried, the pose was re-anchored): what came before it
    // says nothing about the ground behind the robot now.
    if (!crumbs_.empty() && dist(crumbs_.back(), pose) > limits_.max_jump_m)
      crumbs_.clear();
    if (!crumbs_.empty() && dist(crumbs_.back(), pose) < limits_.min_step_m)
      return;
    crumbs_.push_back(pose);
    trim();
  }

  void clear()
  {
    crumbs_.clear();
  }
  std::size_t size() const
  {
    return crumbs_.size();
  }

  /// Forget the newest `distance` metres of path. Called after a successful retrace, so the
  /// breadcrumbs do not still describe the ground that was just undone.
  void retreat(double distance)
  {
    double covered = 0.0;
    while (crumbs_.size() > 1 && covered < distance)
    {
      covered += dist(crumbs_[crumbs_.size() - 2], crumbs_.back());
      crumbs_.pop_back();
    }
  }

  /// Plan the undoing of the last `distance` metres of driven path, from the robot's `current`
  /// pose.
  Plan plan(const Pose2D& current, double distance) const
  {
    Plan result;
    if (crumbs_.empty() || distance < limits_.min_distance_m)
    {
      result.refusal = Refusal::kNoHistory;
      return result;
    }

    // Walk back from the robot along the breadcrumbs until `distance` of path is covered.
    std::deque<Pose2D> piece;  // newest first, starting at the robot
    piece.push_back(current);
    double covered = 0.0;
    Pose2D target = current;
    bool reached = false;
    for (auto it = crumbs_.rbegin(); it != crumbs_.rend(); ++it)
    {
      const Pose2D& previous = piece.back();
      const double step = dist(previous, *it);
      if (step < 1e-6)
        continue;
      if (covered + step >= distance)
      {
        const double t = (distance - covered) / step;
        target = {previous.x + t * (it->x - previous.x),
                  previous.y + t * (it->y - previous.y),
                  it->yaw};
        piece.push_back(target);
        reached = true;
        break;
      }
      covered += step;
      piece.push_back(*it);
    }
    if (!reached)
    {
      result.refusal = Refusal::kNoHistory;
      return result;
    }

    const double dx = target.x - current.x;
    const double dy = target.y - current.y;
    const double chord = std::hypot(dx, dy);
    if (chord < limits_.min_distance_m)
    {
      result.refusal = Refusal::kNotStraight;  // the path doubled back on itself
      return result;
    }
    result.target = target;
    result.distance_m = chord;
    result.ux = dx / chord;
    result.uy = dy / chord;

    // Reversing is only right when the target is behind the robot.
    const double forward_dot =
        std::cos(current.yaw) * result.ux + std::sin(current.yaw) * result.uy;
    const double cos_limit = std::cos(limits_.max_heading_error_rad);
    if (forward_dot >= cos_limit)
    {
      result.refusal = Refusal::kWrongDirection;
      return result;
    }
    if (-forward_dot < cos_limit)
    {
      result.refusal = Refusal::kNotStraight;
      return result;
    }

    // The piece itself must be straight: no breadcrumb may stray from the line back to the target.
    for (const Pose2D& p : piece)
    {
      const double lateral =
          std::abs((p.x - current.x) * result.uy - (p.y - current.y) * result.ux);
      if (lateral > limits_.max_lateral_dev_m)
      {
        result.refusal = Refusal::kNotStraight;
        return result;
      }
    }
    return result;
  }

private:
  static double dist(const Pose2D& a, const Pose2D& b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  void trim()
  {
    double total = 0.0;
    for (std::size_t i = crumbs_.size(); i > 1; --i)
    {
      total += dist(crumbs_[i - 1], crumbs_[i - 2]);
      if (total > limits_.history_m)
      {
        crumbs_.erase(crumbs_.begin(), crumbs_.begin() + static_cast<std::ptrdiff_t>(i - 2));
        return;
      }
    }
  }

  Limits limits_;
  std::deque<Pose2D> crumbs_;
};

/// How far the robot has moved along the unit vector (ux, uy) since `start`.
inline double progressAlong(const Pose2D& start, const Pose2D& now, double ux, double uy)
{
  return (now.x - start.x) * ux + (now.y - start.y) * uy;
}

}  // namespace mowgli_nav2_plugins::retrace
