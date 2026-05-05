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

#include "mowgli_map/map_server_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace mowgli_map
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

MapServerNode::MapServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("map_server_node", options)
{
  // ── Declare and read parameters ──────────────────────────────────────────
  resolution_ = declare_parameter<double>("resolution", 0.05);
  map_size_x_ = declare_parameter<double>("map_size_x", 20.0);
  map_size_y_ = declare_parameter<double>("map_size_y", 20.0);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  decay_rate_per_hour_ = declare_parameter<double>("decay_rate_per_hour", 0.1);
  mower_width_ = declare_parameter<double>("mower_width", 0.18);
  // Path C — fail-count tuning. See header for semantics.
  dead_promote_threshold_ = declare_parameter<double>("dead_promote_threshold", 3.0);
  dead_decay_rate_per_hour_ = declare_parameter<double>("dead_decay_rate_per_hour", 0.5);
  dead_unblock_threshold_ = declare_parameter<double>("dead_unblock_threshold", 1.0);
  map_file_path_ = declare_parameter<std::string>("map_file_path", "");
  areas_file_path_ = declare_parameter<std::string>("areas_file_path", "");
  publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
  keepout_nav_margin_ = declare_parameter<double>("keepout_nav_margin", 1.5);
  // Two-tier boundary: if the robot is outside every defined area, we
  // publish /boundary_violation (BT attempts a recovery back inside). If
  // the robot is further than lethal_boundary_margin beyond any area
  // edge, we also publish /lethal_boundary_violation — BT must
  // emergency-stop because blade/motors outside the authorised zone
  // can do real damage.
  lethal_boundary_margin_m_ = declare_parameter<double>("lethal_boundary_margin_m", 0.5);
  // Soft boundary deadband: distance the robot must be outside ANY area
  // before /boundary_violation fires. Without this, RTK noise (~3 mm)
  // and FTC tracking error around strip endpoints (which sit
  // strip_boundary_margin_m_ inside the polygon) triggers recovery the
  // moment the robot grazes the edge, producing endless transit/abort
  // recovery loops with 30-60 s gaps between strips.
  soft_boundary_margin_m_ = declare_parameter<double>("soft_boundary_margin_m", 0.10);
  boundary_recovery_offset_m_ = declare_parameter<double>("boundary_recovery_offset_m", 0.8);
  boundary_inner_margin_m_ = declare_parameter<double>("boundary_inner_margin_m", 0.3);
  strip_boundary_margin_m_ = declare_parameter<double>("strip_boundary_margin_m", 0.5);
  mow_angle_override_deg_ =
      declare_parameter<double>("mow_angle_deg", std::numeric_limits<double>::quiet_NaN());
  mow_angle_offset_deg_ = declare_parameter<double>("mow_angle_offset_deg", 0.0);
  mow_angle_increment_deg_ = declare_parameter<double>("mow_angle_increment_deg", 0.0);

  // Dock approach corridor — extends the no-mow zone in front of the dock
  // so coverage strips stop before the 1.5 m straight-line alignment that
  // opennav_docking needs for the final approach. Length is measured from
  // dock_pose in the -X direction (dock local frame, same direction as
  // staging_x_offset). Width is symmetric around the approach axis.
  dock_approach_corridor_length_m_ =
      declare_parameter<double>("dock_approach_corridor_length_m", 1.5);
  dock_approach_corridor_half_width_m_ =
      declare_parameter<double>("dock_approach_corridor_half_width_m", 0.40);

  RCLCPP_INFO(get_logger(),
              "MapServerNode: resolution=%.3f m, size=%.1f×%.1f m, frame='%s'",
              resolution_,
              map_size_x_,
              map_size_y_,
              map_frame_.c_str());

  // ── TF buffer for map-frame robot position lookup ────────────────────────
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Initialise map ───────────────────────────────────────────────────────
  init_map();
  last_decay_time_ = now();

  // ── Publishers ───────────────────────────────────────────────────────────
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("~/grid_map", rclcpp::QoS(1));

  mow_progress_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("~/mow_progress", rclcpp::QoS(1));

  coverage_cells_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("~/coverage_cells", rclcpp::QoS(1));

  // Costmap filter publishers: transient_local durability so that Nav2 costmap
  // filter nodes that start after this node still receive the latched message.
  auto transient_qos = rclcpp::QoS(1).transient_local();

  keepout_filter_info_pub_ =
      create_publisher<nav2_msgs::msg::CostmapFilterInfo>("/costmap_filter_info", transient_qos);

  keepout_mask_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("/keepout_mask", transient_qos);

  speed_filter_info_pub_ =
      create_publisher<nav2_msgs::msg::CostmapFilterInfo>("/speed_filter_info", transient_qos);

  speed_mask_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/speed_mask", transient_qos);

  // ── Subscribers ──────────────────────────────────────────────────────────
  occupancy_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(1),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
      {
        on_occupancy_grid(std::move(msg));
      });

  status_sub_ = create_subscription<mowgli_interfaces::msg::Status>(
      "/hardware_bridge/status",
      rclcpp::QoS(1),
      [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg)
      {
        on_mower_status(std::move(msg));
      });

  auto odom_topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered_map");
  odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(odom_topic,
                                                   rclcpp::QoS(1),
                                                   [this](
                                                       nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                   {
                                                     on_odom(std::move(msg));
                                                   });

  // Live obstacle source for the cell-segment walker. The global costmap
  // already merges /scan markings via Nav2's obstacle_layer + inflation,
  // and shares the map frame, so we can sample it directly without TF.
  // obstacle_tracker remains the path for user-validated persistent
  // obstacles (separate concern: review/approve + survive restarts).
  const auto costmap_topic =
      declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
  costmap_obstacle_threshold_ = declare_parameter<int>("costmap_obstacle_threshold", 80);
  costmap_max_age_s_ = declare_parameter<double>("costmap_max_age_s", 2.0);
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic,
      rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
      {
        on_costmap(std::move(msg));
      });

  // ── Services ─────────────────────────────────────────────────────────────
  save_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_save_map(req, res);
      });

  load_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/load_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_load_map(req, res);
      });

  clear_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_clear_map(req, res);
      });

  add_area_srv_ = create_service<mowgli_interfaces::srv::AddMowingArea>(
      "~/add_area",
      [this](const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
             mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res)
      {
        on_add_area(req, res);
      });

  get_mowing_area_srv_ = create_service<mowgli_interfaces::srv::GetMowingArea>(
      "~/get_mowing_area",
      [this](const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
             mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res)
      {
        on_get_mowing_area(req, res);
      });

  set_docking_point_srv_ = create_service<mowgli_interfaces::srv::SetDockingPoint>(
      "~/set_docking_point",
      [this](const mowgli_interfaces::srv::SetDockingPoint::Request::SharedPtr req,
             mowgli_interfaces::srv::SetDockingPoint::Response::SharedPtr res)
      {
        on_set_docking_point(req, res);
      });

  save_areas_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_areas",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_save_areas(req, res);
      });

  load_areas_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/load_areas",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_load_areas(req, res);
      });

  // ── Strip planner services ──────────────────────────────────────────────
  get_next_strip_srv_ = create_service<mowgli_interfaces::srv::GetNextStrip>(
      "~/get_next_strip",
      [this](const mowgli_interfaces::srv::GetNextStrip::Request::SharedPtr req,
             mowgli_interfaces::srv::GetNextStrip::Response::SharedPtr res)
      {
        on_get_next_strip(req, res);
      });

  // Path C cell-based coverage. New service kept side-by-side with
  // get_next_strip during the migration.
  get_next_segment_srv_ = create_service<mowgli_interfaces::srv::GetNextSegment>(
      "~/get_next_segment",
      [this](const mowgli_interfaces::srv::GetNextSegment::Request::SharedPtr req,
             mowgli_interfaces::srv::GetNextSegment::Response::SharedPtr res)
      {
        on_get_next_segment(req, res);
      });

  mark_segment_blocked_srv_ = create_service<mowgli_interfaces::srv::MarkSegmentBlocked>(
      "~/mark_segment_blocked",
      [this](const mowgli_interfaces::srv::MarkSegmentBlocked::Request::SharedPtr req,
             mowgli_interfaces::srv::MarkSegmentBlocked::Response::SharedPtr res)
      {
        on_mark_segment_blocked(req, res);
      });

  clear_dead_cells_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_dead_cells",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_clear_dead_cells(req, res);
      });

  get_coverage_status_srv_ = create_service<mowgli_interfaces::srv::GetCoverageStatus>(
      "~/get_coverage_status",
      [this](const mowgli_interfaces::srv::GetCoverageStatus::Request::SharedPtr req,
             mowgli_interfaces::srv::GetCoverageStatus::Response::SharedPtr res)
      {
        on_get_coverage_status(req, res);
      });

  get_recovery_point_srv_ = create_service<mowgli_interfaces::srv::GetRecoveryPoint>(
      "~/get_recovery_point",
      [this](const mowgli_interfaces::srv::GetRecoveryPoint::Request::SharedPtr req,
             mowgli_interfaces::srv::GetRecoveryPoint::Response::SharedPtr res)
      {
        on_get_recovery_point(req, res);
      });

  // ── Replanning parameters ────────────────────────────────────────────────
  replan_cooldown_sec_ = declare_parameter<double>("replan_cooldown_sec", 30.0);
  last_replan_time_ = now();

  // ── Replan / boundary publishers ────────────────────────────────────────
  replan_needed_pub_ = create_publisher<std_msgs::msg::Bool>("~/replan_needed", rclcpp::QoS(1));
  boundary_violation_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/boundary_violation", rclcpp::QoS(1));
  lethal_boundary_violation_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/lethal_boundary_violation", rclcpp::QoS(1));

  docking_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/docking_pose",
                                                        rclcpp::QoS(1).transient_local());

  // ── Obstacle subscription ─────────────────────────────────────────────
  obstacle_sub_ = create_subscription<mowgli_interfaces::msg::ObstacleArray>(
      "/obstacle_tracker/obstacles",
      rclcpp::QoS(1),
      [this](mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
      {
        on_obstacles(std::move(msg));
      });

  // ── Load pre-defined areas from parameters ────────────────────────────
  load_areas_from_params();

  // ── Auto-load persisted areas from file (overrides parameter areas) ───
  if (!areas_file_path_.empty())
  {
    try
    {
      load_areas_from_file(areas_file_path_);
      RCLCPP_INFO(get_logger(), "Loaded persisted areas from %s", areas_file_path_.c_str());
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(), "No persisted areas to load: %s", ex.what());
    }
  }

  // Resize map to fit loaded areas (if any).
  resize_map_to_areas();

  // Dock pose: single source of truth is mowgli_robot.yaml. Calibration
  // (calibrate_imu_yaw_node) and manual GUI placement (~/set_docking_point
  // below) write back to that file, so the parameters declared here are
  // always the latest persisted values.
  double dock_x = declare_parameter<double>("dock_pose_x", 0.0);
  double dock_y = declare_parameter<double>("dock_pose_y", 0.0);
  double dock_yaw = declare_parameter<double>("dock_pose_yaw", 0.0);

  if (dock_x != 0.0 || dock_y != 0.0 || dock_yaw != 0.0)
  {
    docking_pose_.position.x = dock_x;
    docking_pose_.position.y = dock_y;
    docking_pose_.position.z = 0.0;
    docking_pose_.orientation.w = std::cos(dock_yaw / 2.0);
    docking_pose_.orientation.z = std::sin(dock_yaw / 2.0);
    docking_pose_.orientation.x = 0.0;
    docking_pose_.orientation.y = 0.0;
    docking_pose_set_ = true;
    RCLCPP_INFO(get_logger(),
                "Dock pose from mowgli_robot.yaml: (%.3f, %.3f) yaw=%.3f",
                dock_x,
                dock_y,
                dock_yaw);
  }

  // Publish docking pose if available (transient_local ensures late subscribers get it).
  if (docking_pose_set_)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = docking_pose_;
    docking_pose_pub_->publish(pose_msg);

    // Store dock exclusion polygon — used to mark dock cells as NO_GO_ZONE
    // in the classification layer so strips are not planned through the dock
    // NOR through the straight-line approach corridor that opennav_docking
    // needs for the final 1.5 m alignment. Rectangle in dock local frame:
    //   +X (into dock structure): dock_forward (covers robot when docked)
    //   -X (approach corridor)  : dock_approach_corridor_length_m_
    //   ±Y                      : dock_approach_corridor_half_width_m_
    // This is asymmetric — the robot must stay out of the approach lane so
    // it always reaches staging pose with the correct heading, but we still
    // cover the dock structure itself.
    const double dock_forward = 0.45;  // +X extent into dock (was symmetric)
    const double approach_back = dock_approach_corridor_length_m_;
    const double half_width = dock_approach_corridor_half_width_m_;
    const double d_x = docking_pose_.position.x;
    const double d_y = docking_pose_.position.y;
    const double d_yaw = 2.0 * std::atan2(docking_pose_.orientation.z, docking_pose_.orientation.w);
    const double cy = std::cos(d_yaw);
    const double sy = std::sin(d_yaw);
    const double corners[][2] = {
        {dock_forward, half_width},
        {dock_forward, -half_width},
        {-approach_back, -half_width},
        {-approach_back, half_width},
    };
    for (const auto& c : corners)
    {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(d_x + cy * c[0] - sy * c[1]);
      pt.y = static_cast<float>(d_y + sy * c[0] + cy * c[1]);
      pt.z = 0.0f;
      dock_exclusion_polygon_.points.push_back(pt);
    }
    dock_exclusion_polygon_.points.push_back(dock_exclusion_polygon_.points.front());
    has_dock_exclusion_ = true;
    RCLCPP_INFO(get_logger(),
                "Dock exclusion zone (with approach corridor): pose=(%.2f, %.2f) "
                "yaw=%.2f, forward=%.2fm, approach=%.2fm, half_width=%.2fm",
                d_x,
                d_y,
                d_yaw,
                dock_forward,
                approach_back,
                half_width);
  }

  // ── Publish timer ────────────────────────────────────────────────────────
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));

  publish_timer_ = create_wall_timer(period_ns,
                                     [this]()
                                     {
                                       on_publish_timer();
                                     });

  RCLCPP_INFO(get_logger(), "MapServerNode ready (%zu areas loaded).", areas_.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// Subscription callbacks
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_occupancy_grid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  grid_map::GridMap incoming;
  if (!grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "occupancy_in", incoming))
  {
    RCLCPP_WARN(get_logger(), "on_occupancy_grid: failed to convert OccupancyGrid");
    return;
  }

  const auto& info = msg->info;
  const float res = static_cast<float>(info.resolution);
  const float ox = static_cast<float>(info.origin.position.x) + res * 0.5F;
  const float oy = static_cast<float>(info.origin.position.y) + res * 0.5F;

  for (uint32_t row = 0; row < info.height; ++row)
  {
    for (uint32_t col = 0; col < info.width; ++col)
    {
      const int8_t cell_val = msg->data[static_cast<std::size_t>(row * info.width + col)];
      if (cell_val < 0)
      {
        continue;
      }

      const grid_map::Position pos(static_cast<double>(ox + static_cast<float>(col) * res),
                                   static_cast<double>(oy + static_cast<float>(row) * res));

      grid_map::Index idx;
      if (!map_.getIndex(pos, idx))
      {
        continue;
      }

      map_.at(std::string(layers::OCCUPANCY), idx) = (cell_val > 50) ? 1.0F : 0.0F;
    }
  }
}

void MapServerNode::on_mower_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  mow_blade_enabled_ = msg->mow_enabled;
}

void MapServerNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/)
{
  // Use TF for the definitive map-frame robot position.
  // The odom message position may be in odom frame, not map frame.
  double x = 0.0, y = 0.0;
  if (tf_buffer_)
  {
    try
    {
      auto tf = tf_buffer_->lookupTransform(map_frame_, "base_footprint", tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
    }
    catch (const tf2::TransformException&)
    {
      return;  // No TF yet, skip
    }
  }
  else
  {
    return;
  }

  check_boundary_violation(x, y);

  if (!mow_blade_enabled_)
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    mark_cells_mowed(x, y);
  }
}

void MapServerNode::on_obstacles(mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  diff_and_update_obstacles(msg->obstacles);
}

void MapServerNode::on_costmap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = std::move(msg);
}

bool MapServerNode::is_costmap_blocked(double x, double y) const
{
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr cm;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    cm = latest_costmap_;
  }
  if (!cm)
    return false;  // no costmap yet — fall back to classification only

  // Reject stale costmaps so we don't act on data from a dead producer.
  const auto now_t = now();
  const rclcpp::Time stamp(cm->header.stamp);
  if (now_t.nanoseconds() > 0 && stamp.nanoseconds() > 0)
  {
    const double age_s = (now_t - stamp).seconds();
    if (age_s > costmap_max_age_s_)
      return false;
  }

  const auto& info = cm->info;
  if (info.resolution <= 0.0F || info.width == 0U || info.height == 0U)
    return false;

  const double dx = x - info.origin.position.x;
  const double dy = y - info.origin.position.y;
  if (dx < 0.0 || dy < 0.0)
    return false;
  const auto col = static_cast<uint32_t>(dx / info.resolution);
  const auto row = static_cast<uint32_t>(dy / info.resolution);
  if (col >= info.width || row >= info.height)
    return false;

  const int8_t v = cm->data[static_cast<std::size_t>(row) * info.width + col];
  if (v < 0)
    return false;  // unknown
  return static_cast<int>(v) >= costmap_obstacle_threshold_;
}

// ─────────────────────────────────────────────────────────────────────────────
// Timer callback
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_publish_timer()
{
  const rclcpp::Time now_time = now();
  const double elapsed = (now_time - last_decay_time_).seconds();
  last_decay_time_ = now_time;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    apply_decay(elapsed);

    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(map_);
    grid_map_pub_->publish(std::move(grid_map_msg));

    mow_progress_pub_->publish(mow_progress_to_occupancy_grid());
    coverage_cells_pub_->publish(coverage_cells_to_occupancy_grid());

    // Only publish masks when something changed. The publishers use
    // transient_local QoS so late subscribers (e.g. costmap_filter)
    // automatically receive the most recent mask. Republishing a
    // stale cached mask each tick was triggering the global_costmap
    // KeepoutFilter to reload its filter every second ("New filter
    // mask arrived" log), invalidating active plans and causing
    // docking nav-to-staging to never settle.
    if (masks_dirty_)
    {
      publish_keepout_mask();
      publish_speed_mask();
      masks_dirty_ = false;
    }
  }
}

}  // namespace mowgli_map
