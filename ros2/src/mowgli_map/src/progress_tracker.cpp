// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
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

// Point-in-polygon helper, boundary monitoring, recovery-point service,
// and user-promoted-obstacle application split out of map_server_node.cpp.
// Behaviour (recovery offset, replan trigger) is unchanged.

#include <algorithm>
#include <cmath>
#include <limits>

#include <std_msgs/msg/bool.hpp>

#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"

namespace mowgli_map
{

bool MapServerNode::point_in_polygon(const geometry_msgs::msg::Point32& pt,
                                     const geometry_msgs::msg::Polygon& polygon) noexcept
{
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 3)
  {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const float xi = pts[i].x, yi = pts[i].y;
    const float xj = pts[j].x, yj = pts[j].y;

    const bool intersect =
        ((yi > pt.y) != (yj > pt.y)) && (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi);

    if (intersect)
    {
      inside = !inside;
    }
  }
  return inside;
}
// ─────────────────────────────────────────────────────────────────────────────
// Boundary monitoring
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::check_boundary_violation(double x, double y)
{
  if (areas_.empty())
  {
    return;
  }

  geometry_msgs::msg::Point32 pt;
  pt.x = static_cast<float>(x);
  pt.y = static_cast<float>(y);
  pt.z = 0.0F;

  bool inside_any = false;
  double min_edge_dist = std::numeric_limits<double>::max();
  for (const auto& area : areas_)
  {
    if (point_in_polygon(pt, area.polygon))
    {
      inside_any = true;
      break;
    }
    // Only track distance-to-edge for areas we're outside of; used to
    // classify the violation as "soft" (still recoverable) vs "lethal"
    // (blade/motor hazard — stop immediately).
    const double d = point_to_polygon_distance(x, y, area.polygon);
    if (d < min_edge_dist)
    {
      min_edge_dist = d;
    }
  }

  // Sample debounce. on_odom fires at /odometry/filtered_map's rate
  // (~10 Hz), so the EKF can momentarily report the robot outside the
  // polygon for a single callback when an absolute-yaw sensor (COG,
  // mag) shifts the map→odom transform a few centimetres. Without
  // debouncing, that single tick asserts /boundary_violation, the BT
  // cancels FollowStrip, and a healthy mowing run aborts at <2 %
  // coverage. Require boundary_debounce_samples_ consecutive bad
  // samples before asserting; reset to 0 on the first inside-polygon
  // sample. The lethal escalation is intentionally NOT debounced —
  // if we're really 0.5 m outside, the blade has to stop *now*.
  const bool soft_outside = !inside_any && (min_edge_dist > soft_boundary_margin_m_);
  if (soft_outside)
  {
    if (consecutive_outside_samples_ < std::numeric_limits<int>::max())
    {
      ++consecutive_outside_samples_;
    }
  }
  else
  {
    consecutive_outside_samples_ = 0;
  }

  std_msgs::msg::Bool soft_msg;
  soft_msg.data = soft_outside &&
                  (consecutive_outside_samples_ >= boundary_debounce_samples_);
  boundary_violation_pub_->publish(soft_msg);

  std_msgs::msg::Bool lethal_msg;
  lethal_msg.data = !inside_any && (min_edge_dist > lethal_boundary_margin_m_);
  lethal_boundary_violation_pub_->publish(lethal_msg);

  // Only escalate logging when the blade is actively running. When the blade
  // is off the robot is either idle on the dock or transiting between areas —
  // both states legitimately place the robot outside any defined polygon, so
  // an ERROR-level log would just spam the rosout. The /boundary_violation
  // topics are still published unconditionally so the BT can react.
  if (!inside_any && mow_blade_enabled_)
  {
    if (lethal_msg.data)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(),
                            *get_clock(),
                            2000,
                            "LETHAL BOUNDARY VIOLATION: robot at (%.2f, %.2f) — %.2fm outside "
                            "nearest allowed area (margin=%.2fm)",
                            x,
                            y,
                            min_edge_dist,
                            lethal_boundary_margin_m_);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "BOUNDARY VIOLATION: robot at (%.2f, %.2f) — %.2fm outside nearest "
                           "allowed area (lethal at %.2fm)",
                           x,
                           y,
                           min_edge_dist,
                           lethal_boundary_margin_m_);
    }
  }
}

void MapServerNode::on_get_recovery_point(
    const mowgli_interfaces::srv::GetRecoveryPoint::Request::SharedPtr /*req*/,
    mowgli_interfaces::srv::GetRecoveryPoint::Response::SharedPtr res)
{
  res->success = false;
  res->distance_outside = 0.0;

  if (areas_.empty())
  {
    res->message = "no areas defined";
    return;
  }

  // Look up current robot pose in the map frame. Same path as
  // check_boundary_violation — the BT only invokes this service when a
  // violation is latched, so TF should be fresh.
  double rx = 0.0;
  double ry = 0.0;
  if (!tf_buffer_)
  {
    res->message = "tf buffer unavailable";
    return;
  }
  try
  {
    auto tf = tf_buffer_->lookupTransform(map_frame_, "base_footprint", tf2::TimePointZero);
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
  }
  catch (const tf2::TransformException& ex)
  {
    res->message = std::string("tf lookup failed: ") + ex.what();
    return;
  }

  // Already inside an area? No recovery needed.
  geometry_msgs::msg::Point32 robot_pt;
  robot_pt.x = static_cast<float>(rx);
  robot_pt.y = static_cast<float>(ry);
  robot_pt.z = 0.0F;
  for (const auto& area : areas_)
  {
    if (point_in_polygon(robot_pt, area.polygon))
    {
      res->message = "already inside a mowing area";
      // Still return the current pose as a safe recovery — callers can
      // ignore if success=false.
      res->recovery_pose.position.x = rx;
      res->recovery_pose.position.y = ry;
      res->recovery_pose.orientation.w = 1.0;
      return;
    }
  }

  // Find the globally-closest edge point across all area polygons.
  ClosestEdge best;
  for (const auto& area : areas_)
  {
    auto cand = closest_edge_point(rx, ry, area.polygon);
    if (cand.distance < best.distance)
    {
      best = cand;
    }
  }

  if (best.distance == std::numeric_limits<double>::max())
  {
    res->message = "no polygon edges found";
    return;
  }

  // Inward direction: from robot toward the closest edge, continuing past
  // the edge into the polygon interior.
  const double vx = best.x - rx;
  const double vy = best.y - ry;
  const double vlen = std::hypot(vx, vy);
  double nx = 0.0;
  double ny = 0.0;
  if (vlen > 1e-6)
  {
    nx = vx / vlen;
    ny = vy / vlen;
  }

  const double tx = best.x + boundary_recovery_offset_m_ * nx;
  const double ty = best.y + boundary_recovery_offset_m_ * ny;

  // Yaw facing inward — same direction as the offset.
  const double yaw = std::atan2(ny, nx);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  res->recovery_pose.position.x = tx;
  res->recovery_pose.position.y = ty;
  res->recovery_pose.position.z = 0.0;
  res->recovery_pose.orientation.x = 0.0;
  res->recovery_pose.orientation.y = 0.0;
  res->recovery_pose.orientation.z = sy;
  res->recovery_pose.orientation.w = cy;
  res->distance_outside = best.distance;
  res->success = true;
  res->message = "recovery pose computed";

  RCLCPP_INFO(get_logger(),
              "GetRecoveryPoint: robot=(%.2f, %.2f) outside by %.2fm → "
              "target=(%.2f, %.2f) yaw=%.2f",
              rx,
              ry,
              best.distance,
              tx,
              ty,
              yaw);
}
// ─────────────────────────────────────────────────────────────────────────────
// User-promoted obstacle application
// ─────────────────────────────────────────────────────────────────────────────

bool MapServerNode::apply_promoted_obstacle(size_t area_index,
                                            const geometry_msgs::msg::Polygon& polygon)
{
  // Validate + mutate areas_/obstacle_polygons_ under map_mutex_, then
  // release before calling apply_area_classifications (which locks
  // itself). This avoids both deadlock and a long-held mutex during
  // the polygon-iterator pass.
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (area_index >= areas_.size())
      return false;
    if (areas_[area_index].is_navigation_area)
      return false;
    if (polygon.points.size() < 3)
      return false;

    areas_[area_index].obstacles.push_back(polygon);
    obstacle_polygons_.push_back(polygon);
    masks_dirty_ = true;
  }
  apply_area_classifications();

  // Republish-trigger for any consumer of /map_server_node/replan_needed
  // (BT GetNextSegment requesters). The keepout-mask publisher will
  // pick up masks_dirty_ on its next tick.
  std_msgs::msg::Bool replan_msg;
  replan_msg.data = true;
  replan_needed_pub_->publish(replan_msg);
  return true;
}

}  // namespace mowgli_map
