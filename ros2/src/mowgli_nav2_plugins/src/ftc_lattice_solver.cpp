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

#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mowgli_nav2_plugins
{

namespace
{

double StepLength(const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                  std::size_t a,
                  std::size_t b)
{
  return std::hypot(plan[a].pose.position.x - plan[b].pose.position.x,
                    plan[a].pose.position.y - plan[b].pose.position.y);
}

}  // namespace

LatticeWindow ResampleLatticeWindow(const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                    std::size_t carrot_idx,
                                    std::size_t leg_first,
                                    std::size_t leg_last,
                                    std::optional<std::size_t> corner,
                                    double ds,
                                    double lead_m,
                                    double horizon_m)
{
  LatticeWindow w;
  if (plan.empty() || carrot_idx >= plan.size())
  {
    return w;
  }
  std::vector<std::size_t> behind;
  double acc = 0.0;
  double total = 0.0;
  for (std::size_t i = carrot_idx; i > leg_first && total < lead_m; --i)
  {
    const double l = StepLength(plan, i, i - 1);
    acc += l;
    total += l;
    if (acc >= ds)
    {
      behind.push_back(i - 1);
      acc = 0.0;
    }
  }
  w.pose_idx.assign(behind.rbegin(), behind.rend());
  w.carrot_pos = w.pose_idx.size();
  w.pose_idx.push_back(carrot_idx);
  acc = 0.0;
  total = 0.0;
  std::size_t i = carrot_idx;
  for (; i + 1 < plan.size() && i + 1 <= leg_last && total < horizon_m; ++i)
  {
    const double l = StepLength(plan, i, i + 1);
    acc += l;
    total += l;
    if (acc >= ds)
    {
      w.pose_idx.push_back(i + 1);
      acc = 0.0;
    }
  }
  if (corner.has_value() && i == *corner && *corner > carrot_idx)
  {
    if (w.pose_idx.back() != *corner)
    {
      w.pose_idx.push_back(*corner);
    }
    w.corner_is_last_station = true;
  }
  return w;
}

LatticeSolver::LatticeSolver(const nav2_costmap_2d::Costmap2D& costmap,
                             const BoundaryGuard& guard,
                             const ObstacleDeviation::Footprint& body,
                             std::vector<geometry_msgs::msg::PoseStamped> poses,
                             std::size_t carrot_pos,
                             bool corner_is_last_station,
                             const LatticeSolverCfg& cfg)
    : costmap_(costmap),
      guard_(guard),
      body_(body),
      poses_(std::move(poses)),
      carrot_pos_(std::min(carrot_pos, poses_.empty() ? 0 : poses_.size() - 1)),
      cfg_(cfg)
{
  if (corner_is_last_station && !poses_.empty())
  {
    corner_station_ = poses_.size() - 1 - carrot_pos_;
  }
  stations_.reserve(poses_.size() - carrot_pos_);
  stations_.push_back(0.0);
  for (std::size_t i = carrot_pos_ + 1; i < poses_.size(); ++i)
  {
    stations_.push_back(stations_.back() +
                        std::hypot(poses_[i].pose.position.x - poses_[i - 1].pose.position.x,
                                   poses_[i].pose.position.y - poses_[i - 1].pose.position.y));
  }

  // Footprint tests are the expensive part and the DP asks for the same
  // (pose, offset) from several stations and levels: memoise per solver.
  half_ = static_cast<int>(
      std::floor(cfg_.lattice.max_offset / std::max(1e-9, cfg_.lattice.offset_step) + 1e-9));
  width_ = static_cast<std::size_t>(2 * half_ + 1);
  memo_.assign(poses_.size() * width_, -1);
  for (const auto& v : body_)
  {
    axis_rear_ = std::min(axis_rear_, v.x);
    axis_front_ = std::max(axis_front_, v.x);
  }
}

std::size_t LatticeSolver::StationsIn(double metres) const
{
  const double mean_ds = stations_.size() > 1
                             ? stations_.back() / static_cast<double>(stations_.size() - 1)
                             : cfg_.station_spacing_m;
  return static_cast<std::size_t>(std::ceil(std::max(0.0, metres) / std::max(1e-3, mean_ds)));
}

LatticeBlock LatticeSolver::PoseBlock(std::size_t pose, double offset)
{
  const std::size_t k = static_cast<std::size_t>(std::clamp(
      half_ + static_cast<int>(std::lround(offset / cfg_.lattice.offset_step)), 0, 2 * half_));
  signed char& cell = memo_[pose * width_ + k];
  if (cell < 0)
  {
    // Obstacles: the real chassis polygon against RAW lethal cells of the local
    // costmap, with NO zone guard — the guard samples every footprint cell
    // against a global band that already contains the body (the keepout band
    // is one chassis half-width, the boundary band one circumscribed radius),
    // which counts the body twice and made the planned line itself read
    // "blocked" beside every drawn obstacle.
    LatticeBlock block =
        ObstacleDeviation::footprintBlocked(costmap_,
                                            poses_[pose],
                                            offset,
                                            body_,
                                            BoundaryGuard{},
                                            ObstacleDeviation::kLethalOnlyThreshold)
            ? LatticeBlock::kObstacle
            : LatticeBlock::kFree;
    // Zone: only for a candidate that LEAVES the planned line (the plan is
    // authoritative — Invariant 5), and as a test of the body AXIS against the
    // band, which is exactly "the body, once".
    if (block == LatticeBlock::kFree && std::fabs(offset) > 1e-9 && guard_.costmap != nullptr)
    {
      const double yaw = tf2::getYaw(poses_[pose].pose.orientation);
      const double ox = poses_[pose].pose.position.x - offset * std::sin(yaw);
      const double oy = poses_[pose].pose.position.y + offset * std::cos(yaw);
      for (const double along : {axis_rear_, 0.0, axis_front_})
      {
        if (guard_.isLethalAt(ox + along * std::cos(yaw), oy + along * std::sin(yaw)))
        {
          block = LatticeBlock::kZone;
          break;
        }
      }
    }
    cell = static_cast<signed char>(block);
  }
  return static_cast<LatticeBlock>(cell);
}

// A node is tested over a SPAN of poses, not one:
//   behind — the robot trails the carrot by `lead`;
//   ahead  — `reaction`: the offset the profile asks for is only REACHED after
//            the blend, the lateral loop and the chassis have caught up, and
//            an obstacle grows as the LiDAR gets a closer look at it. Without
//            it the cheapest profile ramps at the last possible station with
//            zero slack (field 2026-09-17: a -0.30 m skirt planned for 13 s,
//            never started, then "no profile" 0.5 m from the obstacle).
LatticeBlock LatticeSolver::SpanBlock(std::size_t station,
                                      double offset,
                                      std::size_t ahead,
                                      std::size_t grace)
{
  if (station <= grace)
  {
    return LatticeBlock::kFree;
  }
  const std::size_t at = carrot_pos_ + station;
  const std::size_t from = at > carrot_pos_ ? at - carrot_pos_ : 0;
  const std::size_t to = std::min(poses_.size() - 1, at + ahead);
  for (std::size_t pose = from; pose <= to; ++pose)
  {
    const LatticeBlock b = PoseBlock(pose, offset);
    if (b != LatticeBlock::kFree)
    {
      return b;
    }
  }
  return LatticeBlock::kFree;
}

// Degrade in steps rather than give up: (1) with the reaction slack; (2) without
// it — we are already closer than we would like; (3) ignoring the stations the
// body already covers (the robot is where it is, exactly like station 0) so
// the profile still steers AWAY. Only when all three fail is the robot wedged.
//
// The HORIZON degrades too, and first: a column of the lattice that is blocked
// at every offset 2 m ahead (the hedge where the ring turns, a scan that paints
// a wall for one tick) makes the whole problem infeasible, but it is not a
// reason to stop NOW — field 2026-09-17: WEDGED + reverse-escape with the
// obstacle still 2.4 m away, flipping with a feasible plan every other tick.
// Only a blockage inside avoidance_min_horizon_m counts as wedged; beyond it
// the robot keeps driving on the longest prefix it can plan and looks again.
LatticeSolution LatticeSolver::Solve(double start_offset, int preferred_sign, int only_side)
{
  LatticeSolution out;
  out.planned_stations = stations_.size();
  if (stations_.empty())
  {
    return out;
  }
  const std::size_t reaction = StationsIn(cfg_.reaction_m);
  const std::size_t min_stations =
      std::min(stations_.size(), std::max<std::size_t>(2, StationsIn(cfg_.min_horizon_m) + 1));
  const std::size_t shrink = std::max<std::size_t>(1, StationsIn(0.25));
  for (std::size_t n = stations_.size(); !out.plan.feasible;
       n = (n > min_stations + shrink) ? n - shrink : min_stations)
  {
    const std::vector<double> prefix(stations_.begin(),
                                     stations_.begin() + static_cast<std::ptrdiff_t>(n));
    out.level = 0;
    for (const auto& [ahead, grace] :
         {std::pair<std::size_t, std::size_t>{reaction, 0},
          std::pair<std::size_t, std::size_t>{0, 0},
          std::pair<std::size_t, std::size_t>{0, StationsIn(cfg_.lead_m)}})
    {
      ++out.level;
      out.plan = PlanOffsetProfile(
          prefix,
          start_offset,
          preferred_sign,
          [&, ahead = ahead, grace = grace](std::size_t station, double offset)
          {
            return (only_side != 0 && offset * static_cast<double>(only_side) < -1e-9) ||
                   (station == corner_station_ && std::fabs(offset) > 1e-9) ||
                   SpanBlock(station, offset, ahead, grace) != LatticeBlock::kFree;
          },
          cfg_.lattice);
      if (out.plan.feasible)
      {
        break;
      }
    }
    out.planned_stations = n;
    if (n == min_stations)
    {
      break;
    }
  }
  return out;
}

bool LatticeFeasibleFrom(
    const nav2_costmap_2d::Costmap2D& costmap,
    const BoundaryGuard& guard,
    const ObstacleDeviation::Footprint& body,
    const std::vector<geometry_msgs::msg::PoseStamped>& plan,
    std::size_t carrot_idx,
    std::size_t leg_first,
    std::size_t leg_last,
    std::optional<std::size_t> corner,
    const std::function<geometry_msgs::msg::PoseStamped(const geometry_msgs::msg::PoseStamped&)>&
        to_costmap,
    const LatticeSolverCfg& cfg,
    double horizon_m)
{
  const LatticeWindow window = ResampleLatticeWindow(
      plan, carrot_idx, leg_first, leg_last, corner, cfg.station_spacing_m, cfg.lead_m, horizon_m);
  if (window.pose_idx.empty())
  {
    return false;
  }
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(window.pose_idx.size());
  for (const std::size_t i : window.pose_idx)
  {
    poses.push_back(to_costmap(plan[i]));
  }
  LatticeSolver solver(costmap,
                       guard,
                       body,
                       std::move(poses),
                       window.carrot_pos,
                       window.corner_is_last_station,
                       cfg);
  return solver.Solve(0.0, 0, 0).plan.feasible;
}

}  // namespace mowgli_nav2_plugins
