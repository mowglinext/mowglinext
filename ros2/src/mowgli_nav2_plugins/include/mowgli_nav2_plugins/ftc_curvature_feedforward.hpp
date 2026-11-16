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
// Curvature regulation for FTC's FOLLOWING state: a curvature-feasible speed
// cap and an optional angular feed-forward, both evaluated where the ROBOT is
// on the plan.
//
// The problem (simulation 2026-11-07, multi_garden main lawn): FTC's carrot
// slows to speed_slow wherever the path bends, but speed_slow is derived from
// mowing_speed (0.8x, so 0.24-0.36 m/s), while the planner's turn-around arcs
// are R ~0.20 m and the yaw rate is capped at max_cmd_vel_ang 0.8 rad/s. Holding
// R 0.20 at 0.36 m/s needs 1.8 rad/s; the feasible speed is ~0.16 m/s. The
// robot runs wide, leaves the arc 70-80 mm off the next swath and converges
// over metres, so swath starts miss their overlap.
//
// The fix this header supports: find the robot on the plan (a bounded search
// back from the carrot, never across a pivot corner), look ahead over the
// braking distance, and cap the speed at what the tightest arc there allows
// with a yaw reserve left for feedback. The cap is evaluated from the robot,
// not the carrot, so it holds until the chassis itself has left the arc; the
// carrot reaching the outgoing straight does not release it.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

namespace mowgli_nav2_plugins
{

/// Shorter intervals carry no usable curvature (duplicate / pivot poses).
inline constexpr double kCurvatureMinIntervalM = 1e-6;
/// Tighter than the planner's smallest connector radius (0.10 m): a corner, not an arc.
inline constexpr double kCurvatureMaxRadPerM = 1.0 / 0.10;
/// A heading change this large over a zero-length / sharp interval is a corner.
inline constexpr double kCurvatureCornerRad = 0.05;
/// Robot farther than this from the searched plan window: no projection.
inline constexpr double kCurvatureMaxProjectionM = 0.40;
/// Search / preview bounds: a few metres of plan, never a scan of the plan.
inline constexpr std::size_t kCurvatureMaxIntervals = 96;
inline constexpr double kCurvatureMaxPreviewM = 2.0;

/// Accepted ranges of the controller's curvature parameters, at startup and
/// live (the lower bound of each is 0).
// Above 1 the gain would remove the carrot's heading lead more than once and
// steer a robot lagging the carrot against the turn.
inline constexpr double kCurvatureFeedforwardGainMax = 1.0;
inline constexpr double kCurvatureYawReserveMax = 0.9;
inline constexpr double kCurvatureResponseMaxS = 2.0;
inline constexpr double kCurvatureMinTurnMaxDeg = 180.0;

struct CurvaturePlanPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Signed curvature (rad/m) of one plan interval, or nullopt where it is not
/// an arc the chassis can drive through (pivot twin, sharp corner, connector).
inline std::optional<double> PlanIntervalCurvature(double length_m, double yaw_delta_rad)
{
  if (!std::isfinite(length_m) || !std::isfinite(yaw_delta_rad))
    return std::nullopt;
  const double wrapped = std::atan2(std::sin(yaw_delta_rad), std::cos(yaw_delta_rad));
  if (length_m < kCurvatureMinIntervalM)
  {
    // A zero-length step with a heading change is a pivot corner; one without
    // is a duplicate pose and carries nothing.
    if (std::abs(wrapped) > kCurvatureCornerRad)
      return std::nullopt;
    return 0.0;
  }
  const double curvature = wrapped / length_m;
  if (std::abs(curvature) > kCurvatureMaxRadPerM)
    return std::nullopt;
  return curvature;
}

/// Where the robot is on the plan: interval `index`, fraction `t` along it.
struct CurvatureRobotOnPlan
{
  std::size_t index{0};
  double t{0.0};
  double distance{0.0};  // robot to the projected point (m)
};

/// Project the robot (rx, ry) onto the plan, searching BACKWARD from the
/// carrot at (carrot_index, carrot_t) over at most `max_back_m` of plan. The
/// search never crosses a corner interval, so it stays within the current pivot
/// leg, and never moves forward of the carrot: it cannot jump to a neighbouring
/// swath or take the carrot's place. nullopt when the robot is not near the
/// window, or when the nearest point is the window's arc-length limit (the
/// robot is farther back than the search may look).
template <class PoseAt>
std::optional<CurvatureRobotOnPlan> ProjectRobotBehindCarrot(const PoseAt& pose_at,
                                                             std::size_t plan_size,
                                                             std::size_t carrot_index,
                                                             double carrot_t,
                                                             double rx,
                                                             double ry,
                                                             double max_back_m)
{
  if (plan_size < 2 || carrot_index + 1 >= plan_size || !std::isfinite(rx) || !std::isfinite(ry))
    return std::nullopt;
  std::optional<CurvatureRobotOnPlan> best;
  bool best_at_limit = false;
  double searched = 0.0;
  std::size_t i = carrot_index;
  std::size_t last_interval = i;
  bool bounded_exit = true;
  double t_hi = std::clamp(carrot_t, 0.0, 1.0);
  for (std::size_t n = 0; n < kCurvatureMaxIntervals; ++n)
  {
    const CurvaturePlanPose a = pose_at(i);
    const CurvaturePlanPose b = pose_at(i + 1);
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (!PlanIntervalCurvature(length, b.yaw - a.yaw))
    {
      bounded_exit = false;
      break;  // a corner: the robot is on this leg or the projection is ambiguous
    }
    last_interval = i;
    // Only the part of the interval inside the remaining arc budget.
    const double span = length * t_hi;
    double t_lo = 0.0;
    bool limited = false;
    if (span > max_back_m - searched && length > 0.0)
    {
      t_lo = t_hi - std::max(0.0, max_back_m - searched) / length;
      limited = true;
    }
    double t = t_hi;
    if (length > 0.0)
      t = std::clamp(((rx - a.x) * dx + (ry - a.y) * dy) / (length * length), t_lo, t_hi);
    const double d = std::hypot(a.x + t * dx - rx, a.y + t * dy - ry);
    if (!best || d < best->distance)
    {
      best = CurvatureRobotOnPlan{i, t, d};
      best_at_limit = limited && t <= t_lo;
    }
    searched += length * (t_hi - t_lo);
    if (limited || i == 0)
    {
      bounded_exit = false;
      break;
    }
    --i;
    t_hi = 1.0;
  }
  const bool best_at_interval_limit =
      bounded_exit && best && best->index == last_interval && best->t <= 0.0;
  if (!best || best->distance > kCurvatureMaxProjectionM || best_at_limit || best_at_interval_limit)
    return std::nullopt;
  return best;
}

/// Speed at which |curvature| uses (1 - yaw_reserve) of the yaw limit.
/// NaN means the required speed is below the crawl floor: unsupported, rather
/// than claiming the faster crawl can hold an infeasible curve.
inline double CurvatureFeasibleSpeed(double abs_curvature,
                                     double max_angular,
                                     double yaw_reserve,
                                     double min_speed)
{
  if (abs_curvature == 0.0)
    return std::numeric_limits<double>::infinity();
  if (!std::isfinite(abs_curvature) || abs_curvature < 0.0 || !std::isfinite(max_angular) ||
      !(max_angular > 0.0) || !std::isfinite(yaw_reserve) || !std::isfinite(min_speed) ||
      min_speed < 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  const double usable = max_angular * (1.0 - std::clamp(yaw_reserve, 0.0, 0.9));
  const double limit = usable / abs_curvature;
  return limit < min_speed ? std::numeric_limits<double>::quiet_NaN() : limit;
}

/// Speed allowed now so that a ramp decelerating at `deceleration` still
/// reaches `limit_speed` at `distance_m` ahead, after `response_s` of delay.
inline double BrakingAllowedSpeed(double limit_speed,
                                  double distance_m,
                                  double deceleration,
                                  double response_s)
{
  if (!std::isfinite(limit_speed))
    return limit_speed;
  if (!(deceleration > 0.0))
    return limit_speed;
  // Solve v^2 = limit^2 + 2 a (d - v * response) for the largest v >= limit.
  const double a = deceleration, tr = std::max(0.0, response_s);
  const double d = std::max(0.0, distance_m);
  const double p = a * tr;
  const double v = -p + std::sqrt(p * p + limit_speed * limit_speed + 2.0 * a * d);
  return std::max(limit_speed, v);
}

/// Curvature evidence for one control cycle. Everything is relative to the
/// robot's own projection on the plan; `supported` false means no correction
/// at all may be applied this cycle (no speed cap, no feed-forward).
struct CurvatureAtRobot
{
  bool supported{false};
  double curvature{0.0};  // signed plan curvature at the robot (rad/m)
  // Signed curvature speed x response_s ahead of the robot: what the chassis
  // will be driving by the time a command issued now has taken effect.
  double anticipated_curvature{0.0};
  double plan_yaw{0.0};  // plan heading at the robot (rad)
  double speed_cap{std::numeric_limits<double>::infinity()};
  double max_abs_curvature{0.0};  // tightest arc within the preview
};

/// Plan heading at fraction t of interval i, interpolated the way the carrot is.
template <class PoseAt>
double PlanYawAt(const PoseAt& pose_at, std::size_t i, double t)
{
  const double a = pose_at(i).yaw, b = pose_at(i + 1).yaw;
  return a + t * std::atan2(std::sin(b - a), std::cos(b - a));
}

/// Plan heading change from (from_i, from_t) forward to (to_i, to_t), summed
/// interval by interval so a half-turn of lead is pi, never an ambiguous wrap.
/// nullopt when `to` is behind `from` or beyond the interval bound.
template <class PoseAt>
std::optional<double> PlanHeadingChange(
    const PoseAt& pose_at, std::size_t from_i, double from_t, std::size_t to_i, double to_t)
{
  if (to_i < from_i || (to_i == from_i && to_t < from_t) || to_i - from_i > kCurvatureMaxIntervals)
    return std::nullopt;
  double sum = 0.0;
  for (std::size_t i = from_i; i <= to_i; ++i)
  {
    const double a = pose_at(i).yaw, b = pose_at(i + 1).yaw;
    const double length =
        std::hypot(pose_at(i + 1).x - pose_at(i).x, pose_at(i + 1).y - pose_at(i).y);
    if (!PlanIntervalCurvature(length, b - a))
      return std::nullopt;
    const double lo = i == from_i ? from_t : 0.0;
    const double hi = i == to_i ? to_t : 1.0;
    sum += (hi - lo) * std::atan2(std::sin(b - a), std::cos(b - a));
  }
  return sum;
}

/// Total heading change (rad, unsigned) of the curvature run that contains
/// interval `i`: the contiguous supported intervals around it that turn the
/// same way at least `min_abs_curvature` tightly. A U-turn arc is one run of
/// ~pi; a recorded-boundary vertex on a headland ring is a run of a few
/// degrees over a few centimetres. Bounded to kCurvatureMaxIntervals each way.
template <class PoseAt>
double CurvatureRunHeadingChange(const PoseAt& pose_at,
                                 std::size_t plan_size,
                                 std::size_t i,
                                 double min_abs_curvature)
{
  const auto interval = [&](std::size_t j) -> std::optional<double>
  {
    const CurvaturePlanPose a = pose_at(j);
    const CurvaturePlanPose b = pose_at(j + 1);
    const auto k = PlanIntervalCurvature(std::hypot(b.x - a.x, b.y - a.y), b.yaw - a.yaw);
    if (!k || std::abs(*k) < min_abs_curvature)
      return std::nullopt;
    return std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));
  };
  if (i + 1 >= plan_size)
    return 0.0;
  const auto own = interval(i);
  if (!own || *own == 0.0)
    return 0.0;
  const bool left = *own > 0.0;
  double sum = *own;
  for (std::size_t j = i + 1, n = 0; j + 1 < plan_size && n < kCurvatureMaxIntervals; ++j, ++n)
  {
    const auto d = interval(j);
    if (!d || (*d > 0.0) != left)
      break;
    sum += *d;
  }
  for (std::size_t j = i, n = 0; j > 0 && n < kCurvatureMaxIntervals; ++n)
  {
    const auto d = interval(--j);
    if (!d || (*d > 0.0) != left)
      break;
    sum += *d;
  }
  return std::abs(sum);
}

/// Curvature and speed cap from the robot's projection forward over the
/// braking distance at `speed`. The preview stops at a corner (a pivot halts
/// the chassis there anyway). An arc too tight to hold even at `min_speed`
/// (radius below min_speed / ((1 - reserve) x max_angular), ~0.12 m with the
/// shipped limits) is unsupported: the whole correction is dropped and FTC
/// drives its native law there.
///
/// `min_run_heading_rad` > 0 exempts short bends from the CAP (never from the
/// feasibility check): an interval caps the speed only when its run
/// (CurvatureRunHeadingChange over intervals at least `run_min_abs_curvature`
/// tight) turns that far in total. A few-degree kink costs a full brake and
/// re-acceleration but leaves almost no cross-track error; a turn-around arc
/// is the case the cap exists for.
template <class PoseAt>
CurvatureAtRobot CurvatureAhead(const PoseAt& pose_at,
                                std::size_t plan_size,
                                const CurvatureRobotOnPlan& at,
                                double speed,
                                double max_angular,
                                double yaw_reserve,
                                double min_speed,
                                double deceleration,
                                double response_s,
                                double min_run_heading_rad = 0.0,
                                double run_min_abs_curvature = 0.0)
{
  CurvatureAtRobot out;
  if (at.index + 1 >= plan_size)
    return out;
  const double v = std::max(0.0, speed);
  const double preview =
      std::min(kCurvatureMaxPreviewM,
               (deceleration > 0.0 ? v * v / (2.0 * deceleration) : 0.0) + v * response_s + 0.10);
  const double anticipation = v * std::max(0.0, response_s);
  double ahead = 0.0;  // plan distance from the robot to the interval's start
  std::size_t i = at.index;
  double t0 = at.t;
  for (std::size_t n = 0; n < kCurvatureMaxIntervals && i + 1 < plan_size; ++n, ++i, t0 = 0.0)
  {
    const CurvaturePlanPose a = pose_at(i);
    const CurvaturePlanPose b = pose_at(i + 1);
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const auto k = PlanIntervalCurvature(length, b.yaw - a.yaw);
    if (!k)
    {
      if (n == 0)
        return out;  // robot on a corner interval: unsupported
      break;
    }
    if (n == 0)
    {
      out.supported = true;
      out.curvature = *k;
      out.anticipated_curvature = *k;
      out.plan_yaw = PlanYawAt(pose_at, i, at.t);
    }
    if (ahead <= anticipation)
      out.anticipated_curvature = *k;
    const double abs_k = std::abs(*k);
    if (abs_k > 0.0)
    {
      out.max_abs_curvature = std::max(out.max_abs_curvature, abs_k);
      const double limit = CurvatureFeasibleSpeed(abs_k, max_angular, yaw_reserve, min_speed);
      if (!std::isfinite(limit))
        return CurvatureAtRobot{};  // no part of the correction on infeasible geometry
      if (min_run_heading_rad > 0.0 &&
          CurvatureRunHeadingChange(pose_at, plan_size, i, run_min_abs_curvature) <
              min_run_heading_rad)
      {
        ahead += length * (1.0 - t0);
        if (ahead > preview)
          break;
        continue;  // a short bend: driven at the native speed
      }
      out.speed_cap =
          std::min(out.speed_cap,
                   n == 0 ? limit : BrakingAllowedSpeed(limit, ahead, deceleration, response_s));
    }
    ahead += length * (1.0 - t0);
    if (ahead > preview)
      break;
  }
  return out;
}

/// Feed-forward angular rate for following `curvature` at `forward_speed`,
/// minus what the carrot's heading lead already supplies.
///
/// The native heading term steers at the CARROT's heading, so it already
/// carries kp_heading x (plan heading at the carrot - plan heading at the
/// robot); `lead_heading_rad` is exactly that change, accumulated along the
/// plan from the robot's projection (PlanHeadingChange, not a wrapped
/// difference). Only the deficit is supplied. Zero unless enabled and driving
/// forward; bounded by the angular limit.
inline double CurvatureFeedforward(double gain,
                                   double forward_speed,
                                   double curvature,
                                   double kp_heading,
                                   double lead_heading_rad,
                                   double max_angular)
{
  if (!std::isfinite(gain) || !(gain > 0.0) || !std::isfinite(forward_speed) ||
      !(forward_speed > 0.0) || !std::isfinite(curvature) || !std::isfinite(kp_heading) ||
      !std::isfinite(lead_heading_rad) || !std::isfinite(max_angular) || !(max_angular > 0.0))
  {
    return 0.0;
  }
  return std::clamp(gain * (forward_speed * curvature - kp_heading * lead_heading_rad),
                    -max_angular,
                    max_angular);
}

}  // namespace mowgli_nav2_plugins
