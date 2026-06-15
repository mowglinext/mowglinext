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

#include "mowgli_behavior/recording_nodes.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/exceptions.h"

namespace mowgli_behavior
{

int RecordArea::area_counter_ = 0;

// ===========================================================================
// RecordArea — record trajectory and save as mowing area polygon
// ===========================================================================

BT::NodeStatus RecordArea::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Clear any previous recording
  trajectory_.clear();

  // Parse recording rate
  double rate_hz = 1.0;
  getInput<double>("record_rate_hz", rate_hz);
  if (rate_hz <= 0.0)
    rate_hz = 1.0;
  record_interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));

  // Create trajectory preview publisher (latched for GUI)
  if (!trajectory_pub_)
  {
    trajectory_pub_ =
        ctx->node->create_publisher<nav_msgs::msg::Path>("~/recording_trajectory",
                                                         rclcpp::QoS(1).transient_local());
  }

  // Record initial position
  record_position();
  last_record_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordArea: started recording (rate=%.1f Hz, interval=%ld ms)",
              rate_hz,
              record_interval_.count());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RecordArea::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Check for finish command (COMMAND_RECORD_FINISH=5)
  {
    std::unique_lock<std::mutex> lock(ctx->context_mutex);
    if (ctx->current_command == 5)
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "RecordArea: finish command received, %zu points recorded",
                  trajectory_.size());

      // Get parameters
      double tolerance = 0.2;
      uint32_t min_verts = 3;
      double min_area_val = 1.0;
      bool is_exclusion = false;
      getInput<double>("simplification_tolerance", tolerance);
      getInput<uint32_t>("min_vertices", min_verts);
      getInput<double>("min_area", min_area_val);
      getInput<bool>("is_exclusion_zone", is_exclusion);

      // Simplify trajectory
      auto simplified = douglas_peucker(trajectory_, tolerance);

      if (simplified.size() < min_verts)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "RecordArea: simplified polygon has only %zu vertices (min=%u), keeping raw",
                    simplified.size(),
                    min_verts);
        // Fall back to raw trajectory if simplification is too aggressive
        if (trajectory_.size() >= min_verts)
        {
          simplified = trajectory_;
        }
        else
        {
          RCLCPP_ERROR(ctx->node->get_logger(),
                       "RecordArea: not enough points recorded (%zu, min=%u)",
                       trajectory_.size(),
                       min_verts);
          ctx->current_command = 0;
          trajectory_.clear();
          return BT::NodeStatus::FAILURE;
        }
      }

      // Validate area
      double area = polygon_area(simplified);
      if (area < min_area_val)
      {
        RCLCPP_ERROR(ctx->node->get_logger(),
                     "RecordArea: polygon area %.2f m^2 is below minimum %.2f m^2",
                     area,
                     min_area_val);
        ctx->current_command = 0;
        trajectory_.clear();
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_INFO(ctx->node->get_logger(),
                  "RecordArea: simplified %zu -> %zu vertices, area=%.2f m^2",
                  trajectory_.size(),
                  simplified.size(),
                  area);

      // Reset the command under the lock, then RELEASE context_mutex before
      // the (up to ~15 s: 5 s wait_for_service + 10 s future poll) blocking
      // add_area service call. Holding the mutex across that call stalled the
      // status/command callbacks for the whole save. trajectory_ is only
      // touched on this tick thread, so it needs no lock.
      ctx->current_command = 0;
      lock.unlock();

      bool saved = save_area(simplified, is_exclusion);
      trajectory_.clear();

      // Publish empty trajectory to clear preview
      nav_msgs::msg::Path empty_path;
      empty_path.header.frame_id = "map";
      empty_path.header.stamp = ctx->node->get_clock()->now();
      trajectory_pub_->publish(empty_path);

      return saved ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    // Check for cancel command (COMMAND_RECORD_CANCEL=6)
    if (ctx->current_command == 6)
    {
      RCLCPP_INFO(ctx->node->get_logger(), "RecordArea: recording cancelled");
      ctx->current_command = 0;
      trajectory_.clear();

      // Publish empty trajectory to clear preview
      nav_msgs::msg::Path empty_path;
      empty_path.header.frame_id = "map";
      empty_path.header.stamp = ctx->node->get_clock()->now();
      trajectory_pub_->publish(empty_path);

      return BT::NodeStatus::FAILURE;
    }

    // Check for home command (COMMAND_HOME=2) — treat as an implicit cancel.
    // Do NOT clear current_command here so that HomeSequence can pick it up
    // on the next tick and navigate back to the dock.
    if (ctx->current_command == 2)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "RecordArea: Home command received during recording — "
                  "discarding %zu recorded points and aborting",
                  trajectory_.size());
      trajectory_.clear();

      // Publish empty trajectory to clear preview
      nav_msgs::msg::Path empty_path;
      empty_path.header.frame_id = "map";
      empty_path.header.stamp = ctx->node->get_clock()->now();
      trajectory_pub_->publish(empty_path);

      // Return FAILURE so the Fallback in MainLogic falls through to HomeSequence.
      return BT::NodeStatus::FAILURE;
    }
  }

  // Record position at the configured rate
  auto now = std::chrono::steady_clock::now();
  if (now - last_record_time_ >= record_interval_)
  {
    record_position();
    last_record_time_ = now;

    // Publish trajectory preview
    nav_msgs::msg::Path preview;
    preview.header.frame_id = "map";
    preview.header.stamp = ctx->node->get_clock()->now();
    for (const auto& pt : trajectory_)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = preview.header;
      pose.pose.position.x = static_cast<double>(pt.x);
      pose.pose.position.y = static_cast<double>(pt.y);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      preview.poses.push_back(pose);
    }
    trajectory_pub_->publish(preview);

    RCLCPP_DEBUG(ctx->node->get_logger(), "RecordArea: %zu points recorded", trajectory_.size());
  }

  return BT::NodeStatus::RUNNING;
}

void RecordArea::onHalted()
{
  trajectory_.clear();
  // Publish empty trajectory to clear preview
  if (trajectory_pub_)
  {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = "map";
    trajectory_pub_->publish(empty_path);
  }
}

void RecordArea::record_position()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  try
  {
    auto tf = ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(tf.transform.translation.x);
    pt.y = static_cast<float>(tf.transform.translation.y);
    pt.z = 0.0f;

    // Skip if too close to last point (less than 5cm) to avoid duplicates
    if (!trajectory_.empty())
    {
      const auto& last = trajectory_.back();
      double dx = static_cast<double>(pt.x - last.x);
      double dy = static_cast<double>(pt.y - last.y);
      if (dx * dx + dy * dy < 0.0025)  // 5cm squared
      {
        return;
      }
    }

    trajectory_.push_back(pt);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(ctx->node->get_logger(),
                         *ctx->node->get_clock(),
                         5000,
                         "RecordArea: TF lookup failed: %s",
                         ex.what());
  }
}

// ---------------------------------------------------------------------------
// Douglas-Peucker line simplification
// ---------------------------------------------------------------------------

std::vector<geometry_msgs::msg::Point32> RecordArea::douglas_peucker(
    const std::vector<geometry_msgs::msg::Point32>& points, double tolerance)
{
  if (points.size() < 3)
  {
    return points;
  }

  std::vector<bool> keep(points.size(), false);
  keep[0] = true;
  keep[points.size() - 1] = true;

  dp_recursive(points, tolerance, 0, points.size() - 1, keep);

  std::vector<geometry_msgs::msg::Point32> result;
  result.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i)
  {
    if (keep[i])
    {
      result.push_back(points[i]);
    }
  }

  return result;
}

void RecordArea::dp_recursive(const std::vector<geometry_msgs::msg::Point32>& points,
                              double tolerance,
                              size_t start,
                              size_t end,
                              std::vector<bool>& keep)
{
  if (end <= start + 1)
  {
    return;
  }

  double max_dist = 0.0;
  size_t max_idx = start;

  for (size_t i = start + 1; i < end; ++i)
  {
    double dist = perpendicular_distance(points[i], points[start], points[end]);
    if (dist > max_dist)
    {
      max_dist = dist;
      max_idx = i;
    }
  }

  if (max_dist > tolerance)
  {
    keep[max_idx] = true;
    dp_recursive(points, tolerance, start, max_idx, keep);
    dp_recursive(points, tolerance, max_idx, end, keep);
  }
}

double RecordArea::perpendicular_distance(const geometry_msgs::msg::Point32& pt,
                                          const geometry_msgs::msg::Point32& line_start,
                                          const geometry_msgs::msg::Point32& line_end)
{
  double dx = static_cast<double>(line_end.x - line_start.x);
  double dy = static_cast<double>(line_end.y - line_start.y);

  double line_len_sq = dx * dx + dy * dy;
  if (line_len_sq < 1e-12)
  {
    // Degenerate line segment — return distance to start point
    double px = static_cast<double>(pt.x - line_start.x);
    double py = static_cast<double>(pt.y - line_start.y);
    return std::sqrt(px * px + py * py);
  }

  // Perpendicular distance = |cross product| / |line length|
  double cross = std::abs(dx * static_cast<double>(line_start.y - pt.y) -
                          static_cast<double>(line_start.x - pt.x) * dy);

  return cross / std::sqrt(line_len_sq);
}

double RecordArea::polygon_area(const std::vector<geometry_msgs::msg::Point32>& points)
{
  if (points.size() < 3)
  {
    return 0.0;
  }

  // Shoelace formula
  double area = 0.0;
  size_t n = points.size();
  for (size_t i = 0; i < n; ++i)
  {
    size_t j = (i + 1) % n;
    area += static_cast<double>(points[i].x) * static_cast<double>(points[j].y);
    area -= static_cast<double>(points[j].x) * static_cast<double>(points[i].y);
  }

  return std::abs(area) / 2.0;
}

bool RecordArea::save_area(const std::vector<geometry_msgs::msg::Point32>& points,
                           bool is_exclusion_zone)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!add_area_client_)
  {
    add_area_client_ =
        helper->create_client<mowgli_interfaces::srv::AddMowingArea>("/map_server_node/add_area");
  }

  if (!add_area_client_->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "RecordArea: add_area service not available");
    return false;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();

  // Build polygon
  geometry_msgs::msg::Polygon polygon;
  polygon.points = points;
  request->area.area = polygon;

  // Auto-generate area name
  ++area_counter_;
  if (is_exclusion_zone)
  {
    request->area.name = "exclusion_" + std::to_string(area_counter_);
  }
  else
  {
    request->area.name = "recorded_area_" + std::to_string(area_counter_);
  }

  request->is_navigation_area = false;

  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordArea: saving area '%s' with %zu vertices (exclusion=%s)",
              request->area.name.c_str(),
              points.size(),
              is_exclusion_zone ? "true" : "false");

  auto future = add_area_client_->async_send_request(request);
  // Poll future without spinning (avoids executor deadlock)
  {
    auto timeout = std::chrono::seconds(10);
    auto start = std::chrono::steady_clock::now();
    bool completed = false;
    while (rclcpp::ok())
    {
      if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        completed = true;
        break;
      }
      if (std::chrono::steady_clock::now() - start > timeout)
      {
        break;
      }
    }
    if (!completed)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "RecordArea: add_area service call timed out");
      return false;
    }
  }

  auto response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "RecordArea: add_area service returned failure");
    return false;
  }

  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordArea: area '%s' saved successfully",
              request->area.name.c_str());

  return true;
}

}  // namespace mowgli_behavior
