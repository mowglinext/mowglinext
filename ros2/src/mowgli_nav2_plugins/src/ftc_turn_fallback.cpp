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

#include "mowgli_nav2_plugins/ftc_turn_fallback.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include "mowgli_nav2_plugins/ftc_pivot.hpp"

namespace mowgli_nav2_plugins
{

namespace
{

/// Default rejoin test when the caller supplies no predicate: the body is clear
/// on the line for this much plan after the rejoin pose.
constexpr double kDefaultRejoinClearM = 0.5;

double YawOf(const geometry_msgs::msg::PoseStamped& p)
{
  const auto& q = p.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

FallbackPose ToFallbackPose(const geometry_msgs::msg::PoseStamped& p)
{
  return {p.pose.position.x, p.pose.position.y, YawOf(p)};
}

geometry_msgs::msg::PoseStamped ToPoseStamped(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

bool BodyBlocked(const nav2_costmap_2d::Costmap2D& costmap,
                 const ObstacleDeviation::Footprint& body,
                 double x,
                 double y,
                 double yaw)
{
  return ObstacleDeviation::footprintBlocked(costmap,
                                             ToPoseStamped(x, y, yaw),
                                             0.0,
                                             body,
                                             BoundaryGuard{},
                                             ObstacleDeviation::kLethalOnlyThreshold);
}

/// The lattice's off-line zone rule: the body AXIS (rear, base_link, front)
/// against the zone band.
bool AxisInZone(const BoundaryGuard& guard,
                const ObstacleDeviation::Footprint& footprint,
                double x,
                double y,
                double yaw)
{
  if (guard.costmap == nullptr)
  {
    return false;
  }
  double rear = 0.0;
  double front = 0.0;
  for (const auto& v : footprint)
  {
    rear = std::min(rear, v.x);
    front = std::max(front, v.x);
  }
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (const double along : {rear, 0.0, front})
  {
    if (guard.isLethalAt(x + along * c, y + along * s))
    {
      return true;
    }
  }
  return false;
}

double FootprintReach(const ObstacleDeviation::Footprint& footprint)
{
  double reach = 0.0;
  for (const auto& v : footprint)
  {
    reach = std::max(reach, std::hypot(v.x, v.y));
  }
  return reach;
}

std::vector<double> CumulativeArc(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  std::vector<double> arc(plan.size(), 0.0);
  for (std::size_t i = 1; i < plan.size(); ++i)
  {
    arc[i] = arc[i - 1] + std::hypot(plan[i].pose.position.x - plan[i - 1].pose.position.x,
                                     plan[i].pose.position.y - plan[i - 1].pose.position.y);
  }
  return arc;
}

double DistanceToSegment(double px, double py, double ax, double ay, double bx, double by)
{
  const double vx = bx - ax;
  const double vy = by - ay;
  const double l2 = vx * vx + vy * vy;
  const double u = l2 > 1e-12 ? std::clamp(((px - ax) * vx + (py - ay) * vy) / l2, 0.0, 1.0) : 0.0;
  return std::hypot(ax + u * vx - px, ay + u * vy - py);
}

bool InsidePolygon(const std::vector<std::pair<double, double>>& poly, double px, double py)
{
  bool inside = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
  {
    const double yi = poly[i].second;
    const double yj = poly[j].second;
    if ((yi > py) != (yj > py))
    {
      const double x_cross =
          (poly[j].first - poly[i].first) * (py - yi) / (yj - yi) + poly[i].first;
      if (px < x_cross)
      {
        inside = !inside;
      }
    }
  }
  return inside;
}

/// Probe headings of a rotation, spaced so the farthest footprint vertex moves
/// at most one costmap cell between probes (pivotSweepBlocked's spacing).
std::vector<double> RotationProbes(const nav2_costmap_2d::Costmap2D& costmap,
                                   const ObstacleDeviation::Footprint& footprint,
                                   double from_yaw,
                                   double to_yaw)
{
  const double reach = FootprintReach(footprint);
  const double step = reach > 0.0 ? costmap.getResolution() / reach : 0.0;
  return PivotSweepYaws(from_yaw, to_yaw, step);
}

/// Probe positions of a straight move, one cell apart, both ends included.
std::vector<std::pair<double, double>> StraightProbes(
    const nav2_costmap_2d::Costmap2D& costmap, double x0, double y0, double x1, double y1)
{
  const double len = std::hypot(x1 - x0, y1 - y0);
  const std::size_t n = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(len / std::max(1e-3, costmap.getResolution()))));
  std::vector<std::pair<double, double>> out;
  out.reserve(n + 1);
  for (std::size_t k = 0; k <= n; ++k)
  {
    const double f = static_cast<double>(k) / static_cast<double>(n);
    out.emplace_back(x0 + f * (x1 - x0), y0 + f * (y1 - y0));
  }
  return out;
}

/// Why one (rejoin, reverse) candidate fails, or nullopt when it is feasible.
struct Candidate
{
  FallbackPose start;
  double connector_m{0.0};
  double rotate_start{0.0};
  double rotate_rejoin{0.0};
  bool single_rotation{false};
};

/// The footprint and body the MOTIONS are planned with (cfg.plan_margin_m).
struct MotionShapes
{
  ObstacleDeviation::Footprint footprint;
  ObstacleDeviation::Footprint body;
};

/// `fp` grown by `margin` in every direction (vertices pushed away from the
/// centroid along both axes: exact for the rectangular chassis).
ObstacleDeviation::Footprint GrowFootprint(const ObstacleDeviation::Footprint& fp, double margin)
{
  if (margin <= 0.0 || fp.empty())
  {
    return fp;
  }
  double cx = 0.0;
  double cy = 0.0;
  for (const auto& v : fp)
  {
    cx += v.x;
    cy += v.y;
  }
  cx /= static_cast<double>(fp.size());
  cy /= static_cast<double>(fp.size());
  ObstacleDeviation::Footprint out = fp;
  for (auto& v : out)
  {
    v.x += (v.x >= cx) ? margin : -margin;
    v.y += (v.y >= cy) ? margin : -margin;
  }
  return out;
}

/// A planned rotation the executing PIVOT will accept whatever direction it
/// takes: up to unambiguous_rotation_rad the short way is certain and only that
/// sweep is checked; beyond it (up to 180 deg) the robot's real heading on
/// arrival decides the direction, so the FULL turn must be clear.
bool PlannedRotationClear(const TurnFallbackProblem& p,
                          const MotionShapes& shapes,
                          const TurnFallbackCfg& cfg,
                          double x,
                          double y,
                          double from_yaw,
                          double to_yaw)
{
  if (std::abs(WrapPivotAngle(to_yaw - from_yaw)) <= cfg.unambiguous_rotation_rad)
  {
    return RotationSweepClear(*p.costmap, p.guard, shapes.footprint, x, y, from_yaw, to_yaw);
  }
  return FullTurnClear(*p.costmap, p.guard, shapes.footprint, x, y, from_yaw);
}

std::optional<std::string> CheckCandidate(const TurnFallbackProblem& p,
                                          const MotionShapes& shapes,
                                          const TurnFallbackCfg& cfg,
                                          const FallbackPose& rejoin,
                                          Candidate& c)
{
  const auto& costmap = *p.costmap;
  const double dx = rejoin.x - c.start.x;
  const double dy = rejoin.y - c.start.y;
  c.connector_m = std::hypot(dx, dy);
  if (c.connector_m < cfg.min_connector_m)
  {
    c.single_rotation = true;
    c.rotate_start = WrapPivotAngle(rejoin.yaw - c.start.yaw);
    c.rotate_rejoin = 0.0;
    if (!PlannedRotationClear(p, shapes, cfg, c.start.x, c.start.y, c.start.yaw, rejoin.yaw))
    {
      return std::string("rotation at the start is blocked");
    }
    return std::nullopt;
  }
  const double heading = std::atan2(dy, dx);
  c.single_rotation = false;
  c.rotate_start = WrapPivotAngle(heading - c.start.yaw);
  c.rotate_rejoin = WrapPivotAngle(rejoin.yaw - heading);
  // Cheapest test first: the connector early-outs on the first blocked probe.
  if (!StraightSweepClear(
          costmap, p.guard, shapes.body, {c.start.x, c.start.y, heading}, rejoin.x, rejoin.y))
  {
    return std::string("connector is blocked");
  }
  if (!PlannedRotationClear(p, shapes, cfg, rejoin.x, rejoin.y, heading, rejoin.yaw))
  {
    return std::string("rotation at the rejoin is blocked");
  }
  if (!PlannedRotationClear(p, shapes, cfg, c.start.x, c.start.y, c.start.yaw, heading))
  {
    return std::string("rotation at the start is blocked");
  }
  return std::nullopt;
}

}  // namespace

const char* ToString(TurnFallbackVerdict verdict)
{
  switch (verdict)
  {
    case TurnFallbackVerdict::kPlanned:
      return "planned";
    case TurnFallbackVerdict::kNoBlockage:
      return "no lethal cell under the body on the plan ahead";
    case TurnFallbackVerdict::kNotATurn:
      return "blockage is not in a turn";
    case TurnFallbackVerdict::kNoRejoin:
      return "no safely reachable rejoin";
    case TurnFallbackVerdict::kBadInput:
      return "missing costmap, footprint or plan";
  }
  return "?";
}

FallbackPose BackedUp(const FallbackPose& pose, double distance_m)
{
  return {pose.x - distance_m * std::cos(pose.yaw),
          pose.y - distance_m * std::sin(pose.yaw),
          pose.yaw};
}

double PlanHeadingSweep(const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                        std::size_t center,
                        double behind_m,
                        double ahead_m)
{
  if (plan.empty() || center >= plan.size())
  {
    return 0.0;
  }
  const std::vector<double> arc = CumulativeArc(plan);
  std::size_t lo = center;
  while (lo > 0 && arc[center] - arc[lo - 1] <= behind_m)
  {
    --lo;
  }
  std::size_t hi = center;
  while (hi + 1 < plan.size() && arc[hi + 1] - arc[center] <= ahead_m)
  {
    ++hi;
  }
  double unwrapped = YawOf(plan[lo]);
  double lo_yaw = unwrapped;
  double hi_yaw = unwrapped;
  for (std::size_t i = lo + 1; i <= hi; ++i)
  {
    unwrapped += WrapPivotAngle(YawOf(plan[i]) - YawOf(plan[i - 1]));
    lo_yaw = std::min(lo_yaw, unwrapped);
    hi_yaw = std::max(hi_yaw, unwrapped);
  }
  return hi_yaw - lo_yaw;
}

bool RotationSweepClear(const nav2_costmap_2d::Costmap2D& costmap,
                        const BoundaryGuard& guard,
                        const ObstacleDeviation::Footprint& footprint,
                        double x,
                        double y,
                        double from_yaw,
                        double to_yaw)
{
  if (footprint.size() < 3)
  {
    return false;  // no polygon: cannot assert clear
  }
  for (const double yaw : RotationProbes(costmap, footprint, from_yaw, to_yaw))
  {
    if (BodyBlocked(costmap, footprint, x, y, yaw) || AxisInZone(guard, footprint, x, y, yaw))
    {
      return false;
    }
  }
  return true;
}

bool FullTurnClear(const nav2_costmap_2d::Costmap2D& costmap,
                   const BoundaryGuard& guard,
                   const ObstacleDeviation::Footprint& footprint,
                   double x,
                   double y,
                   double yaw)
{
  // Four quarter turns, each unambiguously counter-clockwise.
  for (int k = 0; k < 4; ++k)
  {
    const double from = yaw + k * (M_PI / 2.0);
    if (!RotationSweepClear(costmap, guard, footprint, x, y, from, from + M_PI / 2.0))
    {
      return false;
    }
  }
  return true;
}

bool StraightSweepClear(const nav2_costmap_2d::Costmap2D& costmap,
                        const BoundaryGuard& guard,
                        const ObstacleDeviation::Footprint& body,
                        const FallbackPose& from,
                        double to_x,
                        double to_y)
{
  if (body.size() < 3)
  {
    return false;
  }
  for (const auto& [x, y] : StraightProbes(costmap, from.x, from.y, to_x, to_y))
  {
    if (BodyBlocked(costmap, body, x, y, from.yaw) || AxisInZone(guard, body, x, y, from.yaw))
    {
      return false;
    }
  }
  return true;
}

ObstacleDeviation::Footprint RearStrip(const ObstacleDeviation::Footprint& footprint,
                                       double distance_m,
                                       double margin_m)
{
  if (footprint.empty())
  {
    return {};
  }
  double rear = std::numeric_limits<double>::max();
  double left = std::numeric_limits<double>::lowest();
  double right = std::numeric_limits<double>::max();
  for (const auto& v : footprint)
  {
    rear = std::min(rear, v.x);
    left = std::max(left, v.y);
    right = std::min(right, v.y);
  }
  const double back = rear - std::max(0.0, distance_m) - std::max(0.0, margin_m);
  const double m = std::max(0.0, margin_m);
  ObstacleDeviation::Footprint strip;
  for (const auto& [x, y] : std::vector<std::pair<double, double>>{
           {rear, left + m}, {rear, right - m}, {back, right - m}, {back, left + m}})
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    strip.push_back(p);
  }
  return strip;
}

bool ReverseSweepClear(const nav2_costmap_2d::Costmap2D& costmap,
                       const ObstacleDeviation::Footprint& footprint,
                       const FallbackPose& robot,
                       double distance_m,
                       double margin_m)
{
  if (footprint.size() < 3)
  {
    return false;
  }
  if (distance_m <= 0.0)
  {
    return true;
  }
  return !BodyBlocked(
      costmap, RearStrip(footprint, distance_m, margin_m), robot.x, robot.y, robot.yaw);
}

double FootprintClearance(const nav2_costmap_2d::Costmap2D& costmap,
                          const ObstacleDeviation::Footprint& footprint,
                          const FallbackPose& pose,
                          double max_m)
{
  if (footprint.size() < 3)
  {
    return 0.0;
  }
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  std::vector<std::pair<double, double>> poly;
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& v : footprint)
  {
    const double wx = pose.x + c * v.x - s * v.y;
    const double wy = pose.y + s * v.x + c * v.y;
    poly.emplace_back(wx, wy);
    min_x = std::min(min_x, wx);
    min_y = std::min(min_y, wy);
    max_x = std::max(max_x, wx);
    max_y = std::max(max_y, wy);
  }
  int mx0 = 0;
  int my0 = 0;
  int mx1 = 0;
  int my1 = 0;
  costmap.worldToMapEnforceBounds(min_x - max_m, min_y - max_m, mx0, my0);
  costmap.worldToMapEnforceBounds(max_x + max_m, max_y + max_m, mx1, my1);
  double best = max_m;
  for (int my = my0; my <= my1; ++my)
  {
    for (int mx = mx0; mx <= mx1; ++mx)
    {
      if (!ObstacleDeviation::isObstacleCell(
              costmap.getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my))))
      {
        continue;
      }
      double cx = 0.0;
      double cy = 0.0;
      costmap.mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), cx, cy);
      if (InsidePolygon(poly, cx, cy))
      {
        return 0.0;
      }
      for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
      {
        best = std::min(best,
                        DistanceToSegment(
                            cx, cy, poly[j].first, poly[j].second, poly[i].first, poly[i].second));
      }
    }
  }
  return best;
}

TurnFallbackClearances MeasureTurnFallbackClearances(const TurnFallbackProblem& problem,
                                                     const TurnFallbackPlan& plan,
                                                     double max_m)
{
  TurnFallbackClearances out;
  if (plan.verdict != TurnFallbackVerdict::kPlanned || problem.costmap == nullptr)
  {
    return out;
  }
  const auto& costmap = *problem.costmap;
  const auto measure = [&](const ObstacleDeviation::Footprint& fp, double x, double y, double yaw)
  {
    return FootprintClearance(costmap, fp, {x, y, yaw}, max_m);
  };
  // Reverse: the strip swept behind the body.
  if (plan.reverse_m > 0.0)
  {
    out.reverse_m = measure(RearStrip(problem.footprint, plan.reverse_m, 0.0),
                            problem.robot.x,
                            problem.robot.y,
                            problem.robot.yaw);
  }
  const double heading = plan.start.yaw + plan.rotate_start_rad;
  out.rotate_start_m = max_m;
  for (const double yaw : RotationProbes(costmap, problem.footprint, plan.start.yaw, heading))
  {
    out.rotate_start_m =
        std::min(out.rotate_start_m, measure(problem.footprint, plan.start.x, plan.start.y, yaw));
  }
  if (plan.connector_m > 0.0)
  {
    out.connector_m = max_m;
    for (const auto& [x, y] : StraightProbes(
             costmap, plan.start.x, plan.start.y, plan.rejoin_pose.x, plan.rejoin_pose.y))
    {
      out.connector_m = std::min(out.connector_m, measure(problem.body, x, y, heading));
    }
    out.rotate_rejoin_m = max_m;
    for (const double yaw :
         RotationProbes(costmap, problem.footprint, heading, plan.rejoin_pose.yaw))
    {
      out.rotate_rejoin_m =
          std::min(out.rotate_rejoin_m,
                   measure(problem.footprint, plan.rejoin_pose.x, plan.rejoin_pose.y, yaw));
    }
  }
  return out;
}

std::pair<std::size_t, std::size_t> FallbackWindow(
    const std::vector<geometry_msgs::msg::PoseStamped>& plan,
    std::size_t carrot_idx,
    double robot_x,
    double robot_y,
    double back_m,
    double ahead_m)
{
  if (plan.empty())
  {
    return {0, 0};
  }
  carrot_idx = std::min(carrot_idx, plan.size() - 1);
  std::size_t first = carrot_idx;
  double best = std::numeric_limits<double>::max();
  double back = 0.0;
  for (std::size_t i = carrot_idx;; --i)
  {
    const double d =
        std::hypot(plan[i].pose.position.x - robot_x, plan[i].pose.position.y - robot_y);
    if (d < best - 1e-9)
    {
      best = d;
      first = i;
    }
    if (i == 0)
    {
      break;
    }
    back += std::hypot(plan[i].pose.position.x - plan[i - 1].pose.position.x,
                       plan[i].pose.position.y - plan[i - 1].pose.position.y);
    if (back > back_m)
    {
      break;
    }
  }
  std::size_t last = first + 1;
  double ahead = 0.0;
  while (last < plan.size() && ahead <= ahead_m)
  {
    ahead += std::hypot(plan[last].pose.position.x - plan[last - 1].pose.position.x,
                        plan[last].pose.position.y - plan[last - 1].pose.position.y);
    ++last;
  }
  return {first, last};
}

TurnFallbackPlan PlanTurnFallback(const TurnFallbackProblem& p, const TurnFallbackCfg& cfg)
{
  TurnFallbackPlan out;
  if (p.costmap == nullptr || p.footprint.size() < 3 || p.body.size() < 3 || p.plan.size() < 3)
  {
    out.verdict = TurnFallbackVerdict::kBadInput;
    return out;
  }
  const auto& costmap = *p.costmap;
  const std::vector<double> arc = CumulativeArc(p.plan);
  const auto body_blocked_at = [&](std::size_t i)
  {
    const FallbackPose q = ToFallbackPose(p.plan[i]);
    return BodyBlocked(costmap, p.body, q.x, q.y, q.yaw);
  };

  // 1. The first plan pose ahead whose body, on the line, covers a lethal cell.
  std::optional<std::size_t> blocked;
  for (std::size_t i = 0; i < p.plan.size() && arc[i] <= cfg.blockage_scan_m; ++i)
  {
    if (body_blocked_at(i))
    {
      blocked = i;
      break;
    }
  }
  if (!blocked.has_value())
  {
    out.verdict = TurnFallbackVerdict::kNoBlockage;
    return out;
  }
  out.blocked = *blocked;
  out.blocked_arc_m = arc[*blocked];

  // 2. Only a blockage in a turn is this module's to handle.
  out.turn_rad = PlanHeadingSweep(p.plan, *blocked, cfg.turn_behind_m, cfg.turn_ahead_m);
  if (out.turn_rad < cfg.min_turn_rad)
  {
    out.verdict = TurnFallbackVerdict::kNotATurn;
    return out;
  }

  // 3. How far the robot may back straight up.
  const MotionShapes shapes{GrowFootprint(p.footprint, cfg.plan_margin_m),
                            GrowFootprint(p.body, cfg.plan_margin_m)};
  const double step = std::max(0.01, cfg.reverse_step_m);
  for (double r = step; r <= cfg.max_reverse_m + 1e-9; r += step)
  {
    if (!ReverseSweepClear(costmap, p.footprint, p.robot, r, cfg.plan_margin_m))
    {
      break;
    }
    out.reverse_limit_m = r;
  }

  // 4. The rejoin: least skipped path first, then least reverse.
  out.why = "no clear plan pose past the blockage within the search";
  for (std::size_t j = *blocked + 1; j + 2 < p.plan.size(); ++j)
  {
    if (arc[j] > cfg.max_rejoin_arc_m)
    {
      break;
    }
    if (body_blocked_at(j))
    {
      continue;
    }
    const FallbackPose rejoin = ToFallbackPose(p.plan[j]);
    std::optional<bool> followable;
    for (double r = 0.0; r <= out.reverse_limit_m + 1e-9; r += step)
    {
      if (out.evaluated >= cfg.max_evaluations)
      {
        out.why = "search budget exhausted";
        out.verdict = TurnFallbackVerdict::kNoRejoin;
        return out;
      }
      ++out.evaluated;
      Candidate c;
      c.start = BackedUp(p.robot, r);
      const std::optional<std::string> failure = CheckCandidate(p, shapes, cfg, rejoin, c);
      if (failure.has_value())
      {
        out.why = *failure;
        continue;
      }
      if (!followable.has_value())
      {
        if (p.followable)
        {
          followable = p.followable(j);
        }
        else
        {
          bool clear = true;
          for (std::size_t k = j; k < p.plan.size() && arc[k] - arc[j] <= kDefaultRejoinClearM; ++k)
          {
            if (body_blocked_at(k))
            {
              clear = false;
              break;
            }
          }
          followable = clear;
        }
      }
      if (!*followable)
      {
        out.why = "the plan is not followable from the rejoin";
        break;  // this rejoin is out whatever the reverse
      }
      out.verdict = TurnFallbackVerdict::kPlanned;
      out.reverse_m = r;
      out.rejoin = j;
      out.skipped_arc_m = arc[j];
      out.connector_m = c.single_rotation ? 0.0 : c.connector_m;
      out.rotate_start_rad = c.rotate_start;
      out.rotate_rejoin_rad = c.rotate_rejoin;
      out.start = c.start;
      out.rejoin_pose = rejoin;
      out.why.clear();
      return out;
    }
  }
  out.verdict = TurnFallbackVerdict::kNoRejoin;
  return out;
}

}  // namespace mowgli_nav2_plugins
