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

#include "mowgli_nav2_plugins/ftc_contour_bypass.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "mowgli_nav2_plugins/ftc_pivot.hpp"

namespace mowgli_nav2_plugins
{

namespace
{

constexpr double kClearanceCapM = 0.5;
/// Spacing of the plan poses the walk may leave the plan from (m).
constexpr double kStartBackStepM = 0.25;
constexpr double kPivotCountRad = 1.0 * M_PI / 180.0;
/// Rotations up to this are checked one way only (the fallback's default).
constexpr double kUnambiguousRotationRad = 160.0 * M_PI / 180.0;

double YawOf(const geometry_msgs::msg::PoseStamped& p)
{
  const auto& q = p.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

FallbackPose ToPose(const geometry_msgs::msg::PoseStamped& p)
{
  return {p.pose.position.x, p.pose.position.y, YawOf(p)};
}

geometry_msgs::msg::PoseStamped ToStamped(const FallbackPose& p)
{
  geometry_msgs::msg::PoseStamped s;
  s.pose.position.x = p.x;
  s.pose.position.y = p.y;
  s.pose.orientation.z = std::sin(p.yaw / 2.0);
  s.pose.orientation.w = std::cos(p.yaw / 2.0);
  return s;
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

/// Every geometric test of the bypass, with the shapes it is planned with and
/// a counter of footprint tests.
class Checker
{
public:
  Checker(const ContourBypassProblem& p, const ContourBypassCfg& cfg)
      : p_(p),
        footprint_(GrowFootprint(p.footprint, cfg.plan_margin_m)),
        body_(GrowFootprint(p.body, cfg.plan_margin_m))
  {
    for (const auto& v : footprint_)
    {
      rear_ = std::min(rear_, v.x);
      front_ = std::max(front_, v.x);
    }
  }

  /// The bare lattice body covers a TRUE-lethal cell (the hit test).
  bool BareBodyBlocked(const FallbackPose& q)
  {
    ++evaluations;
    return ObstacleDeviation::footprintBlocked(
        *p_.costmap, ToStamped(q), 0.0, p_.body, BoundaryGuard{});
  }

  /// The planning body is blocked, or (off the plan) its axis is in the zone.
  bool Blocked(const FallbackPose& q, bool off_plan)
  {
    ++evaluations;
    if (ObstacleDeviation::footprintBlocked(*p_.costmap, ToStamped(q), 0.0, body_, BoundaryGuard{}))
    {
      return true;
    }
    return off_plan && AxisInZone(q);
  }

  bool InGrid(double x, double y) const
  {
    unsigned int mx = 0;
    unsigned int my = 0;
    return p_.costmap->worldToMap(x, y, mx, my);
  }

  bool RotationClear(double x, double y, double from, double to)
  {
    const double turn = std::abs(WrapPivotAngle(to - from));
    if (turn < 1e-9)
    {
      return true;
    }
    evaluations += 1 + static_cast<std::size_t>(turn / 0.05);
    if (turn <= kUnambiguousRotationRad)
    {
      return RotationSweepClear(*p_.costmap, p_.guard, footprint_, x, y, from, to);
    }
    return FullTurnClear(*p_.costmap, p_.guard, footprint_, x, y, from);
  }

  bool StraightClear(const FallbackPose& from, double to_x, double to_y)
  {
    evaluations += 1 + static_cast<std::size_t>(std::hypot(to_x - from.x, to_y - from.y) /
                                                p_.costmap->getResolution());
    return StraightSweepClear(*p_.costmap, p_.guard, body_, from, to_x, to_y);
  }

  bool ReverseClear(double distance_m)
  {
    ++evaluations;
    return ReverseSweepClear(*p_.costmap, p_.footprint, p_.robot, distance_m, 0.0);
  }

  std::size_t evaluations{0};

private:
  bool AxisInZone(const FallbackPose& q) const
  {
    if (p_.guard.costmap == nullptr)
    {
      return false;
    }
    const double c = std::cos(q.yaw);
    const double s = std::sin(q.yaw);
    for (const double along : {rear_, 0.0, front_})
    {
      if (p_.guard.isLethalAt(q.x + along * c, q.y + along * s))
      {
        return true;
      }
    }
    return false;
  }

  const ContourBypassProblem& p_;
  ObstacleDeviation::Footprint footprint_;
  ObstacleDeviation::Footprint body_;
  double rear_{0.0};
  double front_{0.0};
};

/// Chamfer distance (m) from every cell to the nearest TRUE-lethal local cell
/// or zone-band cell, on the local costmap grid. Off the grid reads as 0.
class DistanceField
{
public:
  explicit DistanceField(const ContourBypassProblem& p)
      : cm_(*p.costmap),
        w_(cm_.getSizeInCellsX()),
        h_(cm_.getSizeInCellsY()),
        res_(cm_.getResolution()),
        d_(static_cast<std::size_t>(w_) * h_, std::numeric_limits<double>::infinity())
  {
    for (unsigned int y = 0; y < h_; ++y)
    {
      for (unsigned int x = 0; x < w_; ++x)
      {
        double wx = 0.0;
        double wy = 0.0;
        cm_.mapToWorld(x, y, wx, wy);
        if (ObstacleDeviation::isObstacleCell(cm_.getCost(x, y)) || p.guard.isLethalAt(wx, wy))
        {
          d_[Idx(x, y)] = 0.0;
        }
      }
    }
    Sweep();
  }

  /// Bilinear distance at (x, y).
  double At(double x, double y) const
  {
    const double fx = (x - cm_.getOriginX()) / res_ - 0.5;
    const double fy = (y - cm_.getOriginY()) / res_ - 0.5;
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= static_cast<int>(w_) || y0 + 1 >= static_cast<int>(h_))
    {
      return 0.0;
    }
    const double tx = fx - x0;
    const double ty = fy - y0;
    const auto v = [this](int cx, int cy)
    {
      return d_[Idx(cx, cy)];
    };
    return (1 - ty) * ((1 - tx) * v(x0, y0) + tx * v(x0 + 1, y0)) +
           ty * ((1 - tx) * v(x0, y0 + 1) + tx * v(x0 + 1, y0 + 1));
  }

  /// Unit gradient (away from the nearest obstacle), or (0, 0) when flat.
  std::pair<double, double> Gradient(double x, double y) const
  {
    const double gx = At(x + res_, y) - At(x - res_, y);
    const double gy = At(x, y + res_) - At(x, y - res_);
    const double n = std::hypot(gx, gy);
    return n > 1e-9 ? std::make_pair(gx / n, gy / n) : std::make_pair(0.0, 0.0);
  }

private:
  std::size_t Idx(unsigned int x, unsigned int y) const
  {
    return static_cast<std::size_t>(y) * w_ + x;
  }

  void Relax(int x, int y, int nx, int ny, double cost)
  {
    if (nx < 0 || ny < 0 || nx >= static_cast<int>(w_) || ny >= static_cast<int>(h_))
    {
      return;
    }
    double& here = d_[Idx(x, y)];
    here = std::min(here, d_[Idx(nx, ny)] + cost);
  }

  void Sweep()
  {
    const double diag = res_ * std::sqrt(2.0);
    for (int y = 0; y < static_cast<int>(h_); ++y)
    {
      for (int x = 0; x < static_cast<int>(w_); ++x)
      {
        Relax(x, y, x - 1, y, res_);
        Relax(x, y, x, y - 1, res_);
        Relax(x, y, x - 1, y - 1, diag);
        Relax(x, y, x + 1, y - 1, diag);
      }
    }
    for (int y = static_cast<int>(h_) - 1; y >= 0; --y)
    {
      for (int x = static_cast<int>(w_) - 1; x >= 0; --x)
      {
        Relax(x, y, x + 1, y, res_);
        Relax(x, y, x, y + 1, res_);
        Relax(x, y, x + 1, y + 1, diag);
        Relax(x, y, x - 1, y + 1, diag);
      }
    }
  }

  const nav2_costmap_2d::Costmap2D& cm_;
  unsigned int w_;
  unsigned int h_;
  double res_;
  std::vector<double> d_;
};

struct WalkContext
{
  const ContourBypassProblem& p;
  const ContourBypassCfg& cfg;
  const std::vector<double>& arc;
  std::size_t hit;
  Checker& check;
  const DistanceField& field;
  /// Distance at which a FRONTAL obstacle engages the contour (pivot room).
  double engage_front_m;
};

/// The rejoin plan pose the walk position `q` sits on, if any (Bug2's leave).
std::optional<std::size_t> LeaveAt(WalkContext& c, const FallbackPose& q)
{
  for (std::size_t j = c.hit + 1; j < c.p.plan.size(); ++j)
  {
    const double past = c.arc[j] - c.arc[c.hit];
    if (past < c.cfg.min_leave_past_hit_m)
    {
      continue;
    }
    if (past > c.cfg.max_rejoin_arc_m)
    {
      break;
    }
    const FallbackPose pj = ToPose(c.p.plan[j]);
    if (std::hypot(pj.x - q.x, pj.y - q.y) > c.cfg.leave_tol_m ||
        std::abs(WrapPivotAngle(q.yaw - pj.yaw)) > c.cfg.max_leave_heading_rad)
    {
      continue;
    }
    if (c.check.Blocked(pj, false) || !c.check.StraightClear(q, pj.x, pj.y) ||
        !c.check.RotationClear(pj.x, pj.y, q.yaw, pj.yaw))
    {
      continue;
    }
    if (c.p.followable && !c.p.followable(j))
    {
      continue;
    }
    return j;
  }
  return std::nullopt;
}

/// The heading the walk wants at `q`: straight on before contact, else the
/// contour tangent (obstacle kept on `side`) pulled onto follow_distance_m.
double DesiredHeading(const WalkContext& c, const FallbackPose& q, int side, bool& engaged)
{
  const double d = c.field.At(q.x, q.y);
  const auto [gx, gy] = c.field.Gradient(q.x, q.y);
  if (!engaged)
  {
    const bool ahead = gx * std::cos(q.yaw) + gy * std::sin(q.yaw) < -0.5;
    engaged =
        d <= c.cfg.follow_distance_m + c.cfg.engage_slack_m || (ahead && d <= c.engage_front_m);
    if (!engaged)
    {
      return q.yaw;
    }
  }
  if (gx == 0.0 && gy == 0.0)
  {
    return q.yaw;
  }
  // side +1 keeps the obstacle on the RIGHT: tangent = gradient turned -90 deg.
  const double s = static_cast<double>(side);
  const double tx = s * gy;
  const double ty = -s * gx;
  const double pull = std::clamp(c.cfg.follow_gain * (d - c.cfg.follow_distance_m), -1.0, 1.0);
  return std::atan2(ty - pull * gy, tx - pull * gx);
}

/// One step from `q`: the clear heading nearest the desired one, trying the
/// side AWAY from the obstacle first at equal deviation.
std::optional<FallbackPose> Step(WalkContext& c, const FallbackPose& q, int side, bool& engaged)
{
  const double want = DesiredHeading(c, q, side, engaged);
  const double away = static_cast<double>(side);  // +1: obstacle right, away = CCW
  const int n = static_cast<int>(std::floor(c.cfg.max_deviation_rad / c.cfg.heading_step_rad));
  for (int k = 0; k <= 2 * n; ++k)
  {
    const int m = (k + 1) / 2;
    const double dev = (k % 2 == 1 ? away : -away) * m * c.cfg.heading_step_rad;
    const double h = WrapPivotAngle(want + dev);
    const FallbackPose next{q.x + c.cfg.step_m * std::cos(h), q.y + c.cfg.step_m * std::sin(h), h};
    if (!c.check.InGrid(next.x, next.y) || c.check.Blocked(next, true) ||
        !c.check.RotationClear(q.x, q.y, q.yaw, h))
    {
      if (!engaged && k == 0)
      {
        engaged = true;  // straight on is blocked: contact
      }
      continue;
    }
    return next;
  }
  return std::nullopt;
}

/// Walk one side from `start`; fills side.walk / rejoin / why.
bool Walk(WalkContext& c, const FallbackPose& start, ContourSide& side)
{
  side.walk = {start};
  side.walk_m = 0.0;
  FallbackPose q = start;
  bool engaged = false;
  std::optional<FallbackPose> engage_at;
  double walked_since_engage = 0.0;
  while (side.walk_m < c.cfg.max_contour_m)
  {
    const auto next = Step(c, q, side.side, engaged);
    if (!next)
    {
      side.why = "no clear heading after " + std::to_string(side.walk_m).substr(0, 4) + " m";
      return false;
    }
    if (engaged && !engage_at)
    {
      engage_at = q;
    }
    q = *next;
    side.walk_m += c.cfg.step_m;
    walked_since_engage += engage_at ? c.cfg.step_m : 0.0;
    side.walk.push_back(q);
    if (const auto j = LeaveAt(c, q))
    {
      side.rejoin = *j;
      return true;
    }
    if (engage_at && walked_since_engage >= c.cfg.loop_min_walk_m &&
        std::hypot(q.x - engage_at->x, q.y - engage_at->y) <= c.cfg.loop_tol_m)
    {
      side.why = "contour closed on itself (plan ahead unreachable on this side)";
      return false;
    }
  }
  side.why = "contour longer than max_contour_m";
  return false;
}

/// Greedy farthest-reach simplification of the walk into pivot + straight legs.
bool Simplify(WalkContext& c, ContourSide& side)
{
  const FallbackPose rejoin = ToPose(c.p.plan[side.rejoin]);
  std::vector<FallbackPose> pts = side.walk;
  pts.push_back({rejoin.x, rejoin.y, rejoin.yaw});
  side.legs = {pts.front()};
  double yaw = pts.front().yaw;
  std::size_t i = 0;
  while (i + 1 < pts.size())
  {
    std::optional<std::size_t> reach;
    double reach_heading = yaw;
    for (std::size_t j = i + 1; j < pts.size(); ++j)
    {
      const double d = std::hypot(pts[j].x - pts[i].x, pts[j].y - pts[i].y);
      const double h = d < 1e-6 ? yaw : std::atan2(pts[j].y - pts[i].y, pts[j].x - pts[i].x);
      if (!c.check.StraightClear({pts[i].x, pts[i].y, h}, pts[j].x, pts[j].y) ||
          !c.check.RotationClear(pts[i].x, pts[i].y, yaw, h))
      {
        break;
      }
      // The vertex must leave pivot room for what follows it.
      const double next_heading = j + 1 < pts.size()
                                      ? std::atan2(pts[j + 1].y - pts[j].y, pts[j + 1].x - pts[j].x)
                                      : rejoin.yaw;
      if (!c.check.RotationClear(pts[j].x, pts[j].y, h, next_heading))
      {
        continue;
      }
      reach = j;
      reach_heading = h;
    }
    if (!reach)
    {
      side.why = "walk could not be simplified into clear legs";
      return false;
    }
    side.rotation_rad += std::abs(WrapPivotAngle(reach_heading - yaw));
    side.pivots += std::abs(WrapPivotAngle(reach_heading - yaw)) > kPivotCountRad ? 1 : 0;
    side.legs_m += std::hypot(pts[*reach].x - pts[i].x, pts[*reach].y - pts[i].y);
    yaw = reach_heading;
    side.legs.push_back({pts[*reach].x, pts[*reach].y, yaw});
    i = *reach;
  }
  // Final pivot onto the plan heading (checked by LeaveAt).
  side.rotation_rad += std::abs(WrapPivotAngle(rejoin.yaw - yaw));
  side.pivots += std::abs(WrapPivotAngle(rejoin.yaw - yaw)) > kPivotCountRad ? 1 : 0;
  side.legs.push_back(rejoin);
  return true;
}

double LegsClearance(const ContourBypassProblem& p, const ContourSide& side, double res)
{
  double best = kClearanceCapM;
  for (std::size_t k = 0; k + 1 < side.legs.size(); ++k)
  {
    const FallbackPose& a = side.legs[k];
    const FallbackPose& b = side.legs[k + 1];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const std::size_t n = std::max<std::size_t>(1, static_cast<std::size_t>(len / res));
    for (std::size_t m = 0; m <= n; ++m)
    {
      const double f = static_cast<double>(m) / static_cast<double>(n);
      const FallbackPose q{a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), b.yaw};
      best = std::min(best, FootprintClearance(*p.costmap, p.footprint, q, kClearanceCapM));
    }
  }
  return best;
}

struct StartPose
{
  FallbackPose pose;
  double reverse;
  std::size_t plan_idx;
};

/// The poses the walk may start from: the last clear plan pose before the hit,
/// then the robot backed straight up.
std::vector<StartPose> StartCandidates(WalkContext& c, std::optional<std::size_t> leave)
{
  std::vector<StartPose> starts;
  if (leave)
  {
    // The last clear plan pose, then every kStartBackStepM further back (FTC
    // drives the plan up to there): a frontal wall needs pivot room.
    double last_arc = std::numeric_limits<double>::infinity();
    for (std::size_t i = *leave + 1; i-- > 0;)
    {
      if (last_arc - c.arc[i] >= kStartBackStepM || i == *leave)
      {
        starts.push_back({ToPose(c.p.plan[i]), 0.0, i});
        last_arc = c.arc[i];
      }
    }
  }
  for (double r = c.cfg.reverse_step_m; r <= c.cfg.max_reverse_m + 1e-9; r += c.cfg.reverse_step_m)
  {
    if (!c.check.ReverseClear(r))
    {
      break;
    }
    starts.push_back({BackedUp(c.p.robot, r), r, 0});
  }
  return starts;
}

void PlanSide(WalkContext& c, const std::vector<StartPose>& starts, ContourSide& side)
{
  side.why = "no start pose (robot boxed in, nothing clear behind it)";
  for (const auto& [start, reverse, plan_idx] : starts)
  {
    ContourSide attempt;
    attempt.side = side.side;
    if (!Walk(c, start, attempt) || !Simplify(c, attempt))
    {
      side.why = attempt.why;
      side.walk_m = std::max(side.walk_m, attempt.walk_m);
      continue;
    }
    attempt.reached = true;
    attempt.reverse_m = reverse;
    attempt.start_idx = plan_idx;
    attempt.skipped_arc_m = c.arc[attempt.rejoin] - c.arc[plan_idx];
    attempt.min_clearance_m = LegsClearance(c.p, attempt, c.p.costmap->getResolution());
    attempt.est_time_s = (attempt.legs_m + 2.0 * reverse) / c.cfg.drive_speed_mps +
                         attempt.rotation_rad / c.cfg.turn_speed_rps;
    side = std::move(attempt);
    return;
  }
}

}  // namespace

const char* ToString(ContourBypassVerdict verdict)
{
  switch (verdict)
  {
    case ContourBypassVerdict::kPlanned:
      return "planned";
    case ContourBypassVerdict::kNoBlockage:
      return "no lethal cell under the body on the plan ahead";
    case ContourBypassVerdict::kNoLeave:
      return "no side reaches the plan again";
    case ContourBypassVerdict::kBadInput:
      return "missing costmap, footprint or plan";
  }
  return "?";
}

ContourBypassPlan PlanContourBypass(const ContourBypassProblem& p, const ContourBypassCfg& cfg)
{
  ContourBypassPlan out;
  out.left.side = 1;
  out.right.side = -1;
  if (p.costmap == nullptr || p.footprint.empty() || p.body.empty() || p.plan.size() < 2 ||
      cfg.step_m <= 0.0 || cfg.heading_step_rad <= 0.0)
  {
    return out;
  }
  const std::vector<double> arc = CumulativeArc(p.plan);
  Checker check(p, cfg);

  std::optional<std::size_t> hit;
  for (std::size_t i = 0; i < p.plan.size() && arc[i] <= cfg.blockage_scan_m; ++i)
  {
    if (check.BareBodyBlocked(ToPose(p.plan[i])))
    {
      hit = i;
      break;
    }
  }
  if (!hit)
  {
    out.verdict = ContourBypassVerdict::kNoBlockage;
    out.evaluations = check.evaluations;
    return out;
  }
  out.blocked = *hit;
  out.blocked_arc_m = arc[*hit];

  std::optional<std::size_t> leave;
  for (std::size_t i = *hit; i-- > 0;)
  {
    if (!check.Blocked(ToPose(p.plan[i]), false))
    {
      leave = i;
      break;
    }
  }
  out.leave_plan = leave.value_or(0);

  const DistanceField field(p);
  const double engage_front = [&]()
  {
    // The pivot sweep is tested with the GROWN footprint over every cell it
    // touches: its reach plus two cells (cell-centre distance + partial cells).
    double reach = 0.0;
    for (const auto& v : GrowFootprint(p.footprint, cfg.plan_margin_m))
    {
      reach = std::max(reach, std::hypot(v.x, v.y));
    }
    return reach + 2.0 * p.costmap->getResolution();
  }();
  WalkContext ctx{p, cfg, arc, *hit, check, field, engage_front};
  const auto starts = StartCandidates(ctx, leave);
  PlanSide(ctx, starts, out.left);
  PlanSide(ctx, starts, out.right);

  out.evaluations = check.evaluations;
  if (!out.left.reached && !out.right.reached)
  {
    out.verdict = ContourBypassVerdict::kNoLeave;
    return out;
  }
  out.verdict = ContourBypassVerdict::kPlanned;
  if (!out.right.reached || (out.left.reached && out.left.est_time_s <= out.right.est_time_s))
  {
    out.chosen = 1;
  }
  else
  {
    out.chosen = -1;
  }
  return out;
}

}  // namespace mowgli_nav2_plugins
