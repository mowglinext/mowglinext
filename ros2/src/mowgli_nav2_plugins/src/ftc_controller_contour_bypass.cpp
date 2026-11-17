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
// FTCController's side of the CONTOUR BYPASS (ftc_contour_bypass.hpp): the
// third stage where the lattice is WEDGED and the turn fallback found no way
// round (an obstacle on a straight, or a turn with no single-connector rejoin).
// It plans the Bug2-style contour walk on the same inputs as the turn fallback,
// then splices its legs into the working plan as runtime pivot corners +
// straight poses — the turn fallback's execution path: the PIVOT state with its
// per-tick sweep gate, FOLLOWING with the lattice and currentBodyInLethal on
// every leg, completion at the rejoin and the deadline in turnFallbackProgress,
// hold -> abort on anything new.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mowgli_nav2_plugins/ftc_contour_bypass.hpp"
#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/ftc_pivot.hpp"

namespace mowgli_nav2_plugins
{

namespace
{

const char* SideName(int side)
{
  return side > 0 ? "left" : (side < 0 ? "right" : "none");
}

std::string SideSummary(const ContourSide& s)
{
  if (!s.reached)
  {
    return std::string(SideName(s.side)) + ": " + s.why;
  }
  char buf[160];
  std::snprintf(buf,
                sizeof(buf),
                "%s: %.2f m in %zu legs, %zu pivots, ~%.0f s",
                SideName(s.side),
                s.legs_m,
                s.legs.size() > 1 ? s.legs.size() - 1 : 0,
                s.pivots,
                s.est_time_s);
  return buf;
}

}  // namespace

bool FTCController::tryContourBypass(std::size_t carrot_idx,
                                     const BoundaryGuard& guard,
                                     const ObstacleDeviation::Footprint& body)
{
  ContourBypassCfg cfg;
  // No reverse of its own: a robot too close to turn takes the WEDGED
  // reverse-escape (bounded, boundary-safe) and tries again from there.
  cfg.max_reverse_m = 0.0;
  cfg.max_contour_m = config_.contour_bypass_max_contour_m;
  cfg.max_rejoin_arc_m = config_.contour_bypass_max_rejoin_arc_m;
  cfg.follow_distance_m = config_.contour_bypass_follow_distance_m;
  cfg.blockage_scan_m = config_.avoidance_horizon_m;
  cfg.plan_margin_m = kContourBypassPlanMarginM;

  std::size_t first = 0;
  TurnFallbackProblem problem;
  std::string why;
  const auto t0 = std::chrono::steady_clock::now();
  if (!buildFallbackProblem(
          carrot_idx, guard, body, cfg.blockage_scan_m + cfg.max_rejoin_arc_m, first, problem, why))
  {
    RCLCPP_WARN(logger_,
                "FTCController: contour bypass not possible at idx=%u (%s) — WEDGED handling as "
                "before.",
                current_index_,
                why.c_str());
    return false;
  }
  double robot_to_plan = std::numeric_limits<double>::infinity();
  for (const auto& p : problem.plan)
  {
    robot_to_plan = std::min(robot_to_plan,
                             std::hypot(p.pose.position.x - problem.robot.x,
                                        p.pose.position.y - problem.robot.y));
  }
  cfg.start_on_plan = robot_to_plan <= kContourStartOnPlanM;
  const ContourBypassPlan plan = PlanContourBypass(problem, cfg);
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  const ContourSide* best = plan.Best();
  if (plan.verdict != ContourBypassVerdict::kPlanned || best == nullptr)
  {
    RCLCPP_WARN(logger_,
                "FTCController: contour bypass not possible at idx=%u (%s): blockage at idx %zu "
                "%.2f m ahead; %s; %s; %zu evaluations, %.1f ms — WEDGED handling as before.",
                current_index_,
                ToString(plan.verdict),
                first + plan.blocked,
                plan.blocked_arc_m,
                SideSummary(plan.left).c_str(),
                SideSummary(plan.right).c_str(),
                plan.evaluations,
                ms);
    return false;
  }
  RCLCPP_WARN(logger_,
              "FTCController: CONTOUR BYPASS at idx=%u — the lattice is WEDGED at idx %zu (%.2f m "
              "ahead) and the turn fallback found no way round. Walking the %s side: leave the "
              "plan at idx %zu, %.2f m contour -> %zu legs %.2f m, %zu pivots (%.0f deg), rejoin "
              "idx %zu, skipping %.2f m of path, min clearance %.2f m, ~%.0f s (other side: %s); "
              "%zu evaluations, %.1f ms.",
              current_index_,
              first + plan.blocked,
              plan.blocked_arc_m,
              SideName(best->side),
              first + best->start_idx,
              best->walk_m,
              best->legs.size() - 1,
              best->legs_m,
              best->pivots,
              best->rotation_rad * 180.0 / M_PI,
              first + best->rejoin,
              best->skipped_arc_m,
              best->min_clearance_m,
              best->est_time_s,
              SideSummary(best->side > 0 ? plan.right : plan.left).c_str(),
              plan.evaluations,
              ms);

  turn_fallback_.started = clock_->now();
  turn_fallback_.reverse_done_m = 0.0;
  turn_fallback_.reverse_time_s = 0.0;
  turn_fallback_.timeout_s = config_.contour_bypass_timeout_s;
  turn_fallback_.contour = true;
  ++turn_fallback_.count;
  if (!spliceContourBypass(*best, first))
  {
    RCLCPP_WARN(logger_,
                "FTCController: contour bypass could not be spliced — WEDGED handling as before.");
    turn_fallback_.phase = TurnFallbackPhase::kIdle;
    turn_fallback_.contour = false;
    return false;
  }
  turn_fallback_engaged_now_ = true;
  return true;
}

bool FTCController::spliceContourBypass(const ContourSide& side, std::size_t window_first)
{
  const std::size_t j = window_first + side.rejoin;
  const std::size_t s = window_first + side.start_idx;
  if (side.legs.size() < 3 || j + 2 >= global_plan_.size() || s >= j)
  {
    return false;
  }
  const std::string plan_frame = global_plan_[j].header.frame_id.empty()
                                     ? std::string("map")
                                     : global_plan_[j].header.frame_id;
  geometry_msgs::msg::TransformStamped costmap_to_plan;
  double rx = 0.0;
  double ry = 0.0;
  double ryaw = 0.0;
  try
  {
    const auto robot = tf_buffer_->lookupTransform(plan_frame, "base_link", tf2::TimePointZero);
    rx = robot.transform.translation.x;
    ry = robot.transform.translation.y;
    ryaw = tf2::getYaw(robot.transform.rotation);
    costmap_to_plan = tf_buffer_->lookupTransform(plan_frame,
                                                  costmap_ros_->getGlobalFrameID(),
                                                  tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(logger_, "FTCController: contour bypass splice: TF failed (%s).", ex.what());
    return false;
  }

  const auto& rejoin = global_plan_[j];
  const auto make = [&rejoin](double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header = rejoin.header;
    p.pose.position.x = x;
    p.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    p.pose.orientation = tf2::toMsg(q);
    return p;
  };

  std::vector<geometry_msgs::msg::PoseStamped> spliced;
  // Up to the leave-the-plan pose the robot keeps following the plan itself.
  double yaw_in = ryaw;
  std::vector<std::pair<double, double>> vertices;
  if (side.start_idx == 0)
  {
    vertices.emplace_back(rx, ry);
  }
  else
  {
    spliced.insert(spliced.end(),
                   global_plan_.begin() + static_cast<std::ptrdiff_t>(window_first + 1),
                   global_plan_.begin() + static_cast<std::ptrdiff_t>(s + 1));
    vertices.emplace_back(global_plan_[s].pose.position.x, global_plan_[s].pose.position.y);
    yaw_in = tf2::getYaw(global_plan_[s].pose.orientation);
  }
  // The intermediate vertices (legs[1 .. n-3]) were planned in the costmap
  // frame; legs[n-2] is the rejoin position, legs[n-1] the rejoin pose.
  for (std::size_t k = 1; k + 2 < side.legs.size(); ++k)
  {
    geometry_msgs::msg::PoseStamped in;
    in.pose.position.x = side.legs[k].x;
    in.pose.position.y = side.legs[k].y;
    in.pose.orientation.w = 1.0;
    geometry_msgs::msg::PoseStamped out;
    tf2::doTransform(in, out, costmap_to_plan);
    vertices.emplace_back(out.pose.position.x, out.pose.position.y);
  }
  const double jx = rejoin.pose.position.x;
  const double jy = rejoin.pose.position.y;
  vertices.emplace_back(jx, jy);

  bool first_vertex = true;
  for (std::size_t k = 0; k + 1 < vertices.size(); ++k)
  {
    const auto [ax, ay] = vertices[k];
    const auto [bx, by] = vertices[k + 1];
    const double len = std::hypot(bx - ax, by - ay);
    if (len < 1e-3)
    {
      continue;
    }
    const double heading = std::atan2(by - ay, bx - ax);
    // A corner is TWO poses at the same position: incoming then outgoing
    // heading (mowgli_interfaces/coverage_geometry.hpp). On the first vertex
    // of a plan start the incoming pose is the last prefix pose already.
    const bool turn = std::abs(WrapPivotAngle(heading - yaw_in)) >= kPivotCornerDetectTurnRad;
    if (turn && !(first_vertex && side.start_idx > 0))
    {
      spliced.push_back(make(ax, ay, yaw_in));
    }
    spliced.push_back(make(ax, ay, heading));
    const std::size_t n = std::max<std::size_t>(1,
                                                static_cast<std::size_t>(
                                                    std::ceil(len / kTurnFallbackConnectorStepM)));
    for (std::size_t m = 1; m < n; ++m)
    {
      const double f = static_cast<double>(m) / static_cast<double>(n);
      spliced.push_back(make(ax + f * (bx - ax), ay + f * (by - ay), heading));
    }
    yaw_in = heading;
    first_vertex = false;
  }
  const double jyaw = tf2::getYaw(rejoin.pose.orientation);
  if (std::abs(WrapPivotAngle(jyaw - yaw_in)) >= kPivotCornerDetectTurnRad)
  {
    spliced.push_back(make(jx, jy, yaw_in));  // corner: incoming heading
  }
  std::size_t rejoin_idx = spliced.size();
  spliced.push_back(rejoin);  // corner: outgoing heading = the plan pose itself
  spliced.insert(spliced.end(),
                 global_plan_.begin() + static_cast<std::ptrdiff_t>(j + 1),
                 global_plan_.end());
  if (!finalizeSplice(spliced, rejoin_idx, "contour bypass"))
  {
    return false;
  }

  const bool pivot_first = adoptFallbackPlan(std::move(spliced), rejoin_idx, side.skipped_arc_m);
  RCLCPP_INFO(logger_,
              "FTCController: contour bypass spliced from (%.2f, %.2f)%s: %zu vertices, rejoin at "
              "working idx %zu (%.2f, %.2f); %zu pivot corner(s) in the plan now.",
              rx,
              ry,
              pivot_first ? ", pivot first" : "",
              vertices.size(),
              rejoin_idx,
              jx,
              jy,
              pivot_corners_.size());
  return true;
}

}  // namespace mowgli_nav2_plugins
