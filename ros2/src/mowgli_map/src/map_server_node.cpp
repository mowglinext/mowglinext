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
  tool_width_ = declare_parameter<double>("tool_width", 0.18);
  yaw_convergence_threshold_rad_ =
      declare_parameter<double>("yaw_convergence_threshold_rad", 0.00873);  // 0.5°
  yaw_convergence_window_s_ = declare_parameter<double>("yaw_convergence_window_s", 5.0);
  yaw_convergence_min_samples_ =
      static_cast<size_t>(declare_parameter<int>("yaw_convergence_min_samples", 20));
  map_file_path_ = declare_parameter<std::string>("map_file_path", "");
  areas_file_path_ = declare_parameter<std::string>("areas_file_path", "");
  publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
  keepout_nav_margin_ = declare_parameter<double>("keepout_nav_margin", 1.5);
  // Hard area-boundary enforcement: when true (operator default), the keepout
  // mask marks every cell OUTSIDE the union of all areas (mowing + navigation)
  // as LETHAL — the planner cannot route there and MPPI cannot steer out of
  // the zone. enforce_boundary_margin_m_ is the small free slack left outside
  // each edge so RTK drift at the boundary does not self-reject. When false,
  // the legacy keepout_nav_margin_ free band governs. See the field note on the
  // 0.32 m concave-boundary excursion (project_coverage_boundary_excursion).
  lethal_outside_areas_ = declare_parameter<bool>("lethal_outside_areas", true);
  enforce_boundary_margin_m_ = declare_parameter<double>("enforce_boundary_margin_m", 0.25);
  // Two-tier boundary: if the robot is outside every defined area, we
  // publish /boundary_violation (BT attempts a recovery back inside). If
  // the robot is further than lethal_boundary_margin beyond any area
  // edge, we also publish /lethal_boundary_violation — BT must
  // emergency-stop because blade/motors outside the authorised zone
  // can do real damage.
  lethal_boundary_margin_m_ = declare_parameter<double>("lethal_boundary_margin_m", 0.5);
  // Soft boundary deadband: distance the robot's BASE_LINK pose must
  // be outside the operator polygon before /boundary_violation fires.
  // 0.30 m covers FTC tracking error under the wheel-PI motor model
  // (steady-state ~0.05 m, post-pivot transient up to ~0.20 m) plus
  // a small slack so the chassis can briefly graze the polygon edge
  // during corner traversals without ping-ponging into recovery.
  // The blade extends tool_width / 2 = 0.09 m from base_link, so a
  // 0.30 m base_link excursion still keeps the BLADE TIP within
  // 0.39 m of the inside — well clear of the 0.50 m lethal margin,
  // which is the hard safety net and bypasses this deadband.
  //
  // 0.20 m → 0.30 m (2026-05-19) after the sim run hit 5×
  // NavigateInsideBoundary recoveries in a single coverage attempt
  // — every recovery triggered on transient corner-pivot drift of
  // 0.20-0.30 m, not actual chassis-off-polygon excursions.
  soft_boundary_margin_m_ = declare_parameter<double>("soft_boundary_margin_m", 0.30);
  // How many consecutive on_odom samples must report the robot outside
  // before /boundary_violation asserts true (filters single-tick EKF
  // jumps). At ~10 Hz odometry, 3 samples ≈ 300 ms — long enough to
  // ignore a map→odom burp from a momentary absolute-yaw correction,
  // short enough that an actual excursion still aborts mowing well
  // before the lethal margin (0.5 m) is reached at the mowing speed
  // of 0.20 m/s. Lethal violations bypass this debounce.
  boundary_debounce_samples_ =
      static_cast<int>(declare_parameter<int>("boundary_debounce_samples", 3));
  boundary_recovery_offset_m_ = declare_parameter<double>("boundary_recovery_offset_m", 0.8);
  boundary_inner_margin_m_ = declare_parameter<double>("boundary_inner_margin_m", 0.3);
  strip_boundary_margin_m_ = declare_parameter<double>("strip_boundary_margin_m", 0.5);
  mow_angle_override_deg_ =
      declare_parameter<double>("mow_angle_deg", std::numeric_limits<double>::quiet_NaN());

  // Bypass-arc tuning (cleaning-robot detour around discrete obstacles).
  // chassis_width sets the lateral-offset basis; bypass_safety_margin
  // adds clearance on top; bypass_max_length is the give-up threshold
  // along the row (anything longer is treated as a wall, not an obstacle).
  // The give-up threshold reads max_obstacle_avoidance_distance from
  // mowgli_robot.yaml so it lives alongside other physical params.
  chassis_width_m_ = declare_parameter<double>("chassis_width", 0.40);
  bypass_safety_margin_m_ = declare_parameter<double>("bypass_safety_margin_m", 0.05);
  bypass_max_length_m_ = declare_parameter<double>("max_obstacle_avoidance_distance", 2.0);

  // Dock body (physical structure the robot cannot drive into). Cells
  // inside are marked OBSTACLE_PERMANENT — F2C strips stop at the body
  // edge and Smac treats them as lethal. Defaults match the YardForce500
  // dock; override per-site in mowgli_robot.yaml.
  dock_body_length_m_ = declare_parameter<double>("dock_body_length_m", 0.80);
  dock_body_width_m_ = declare_parameter<double>("dock_body_width_m", 0.55);

  // Dock approach corridor (behind the dock along -X). Cells here are
  // classified DOCKING_AREA (mowable — corridor lawn still gets cut) and
  // explicitly carved out of the keepout mask so Smac can plan post-undock
  // transit. Length is measured from dock_pose in the -X direction (dock
  // local frame, same direction as staging_x_offset). Width is symmetric.
  dock_approach_corridor_length_m_ =
      declare_parameter<double>("dock_approach_corridor_length_m", 1.5);
  dock_approach_corridor_half_width_m_ =
      declare_parameter<double>("dock_approach_corridor_half_width_m", 0.40);

  // When false (default), tracker observations stay TENTATIVE/CONFIRMED/
  // PERSISTENT but are NEVER stamped into a mowing area's classification
  // layer without operator action. The GUI's "Promote obstacle" button
  // (which calls ~/promote_obstacle) is the only path to a permanent
  // keepout. This avoids the failure mode where a parked car, a person
  // standing still, or a misclassified sensor blob persistently shrinks
  // the mowable area without anyone noticing — the previous default ate
  // the polygon over time.
  auto_promote_persistent_obstacles_ =
      declare_parameter<bool>("auto_promote_persistent_obstacles", false);

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

  // ── Publishers ───────────────────────────────────────────────────────────
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("~/grid_map", rclcpp::QoS(1));

  // Mowed-area progress overlay. transient_local so the GUI receives the latest
  // accumulated coverage immediately on (re)connect.
  mow_progress_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("~/mow_progress",
                                                     rclcpp::QoS(1).transient_local());

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

  // GPS pose-with-covariance feed (from navsat_to_absolute_pose_node), used
  // only by on_set_docking_point to gate the service on RTK quality. The
  // service must not pin a dock pose while in RTK-Float (σ ~10-50 cm) or
  // when the GPS feed is stale.
  dock_set_gps_accuracy_max_m_ =
      declare_parameter<double>("dock_set_gps_accuracy_max_m", dock_set_gps_accuracy_max_m_);
  dock_set_gps_max_age_s_ =
      declare_parameter<double>("dock_set_gps_max_age_s", dock_set_gps_max_age_s_);
  dock_set_status_max_age_s_ =
      declare_parameter<double>("dock_set_status_max_age_s", dock_set_status_max_age_s_);
  gps_pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/gps/pose_cov",
      rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
      {
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const rclcpp::Time t = now();
        std::lock_guard<std::mutex> lk(last_gps_pose_cov_mutex_);
        last_gps_pose_cov_ = std::move(msg);
        last_gps_pose_cov_time_ = t;
        // Maintain a rolling window for on_set_docking_point's averaged
        // dock-pose capture (see recent_gps_xy_ in the header).
        recent_gps_xy_.emplace_back(t, x, y);
        while (!recent_gps_xy_.empty() &&
               (t - std::get<0>(recent_gps_xy_.front())).seconds() > dock_set_gps_avg_window_s_)
        {
          recent_gps_xy_.pop_front();
        }
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

  get_recovery_point_srv_ = create_service<mowgli_interfaces::srv::GetRecoveryPoint>(
      "~/get_recovery_point",
      [this](const mowgli_interfaces::srv::GetRecoveryPoint::Request::SharedPtr req,
             mowgli_interfaces::srv::GetRecoveryPoint::Response::SharedPtr res)
      {
        on_get_recovery_point(req, res);
      });

  // ── Replan / boundary publishers ────────────────────────────────────────
  replan_needed_pub_ = create_publisher<std_msgs::msg::Bool>("~/replan_needed", rclcpp::QoS(1));
  boundary_violation_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/boundary_violation", rclcpp::QoS(1));
  lethal_boundary_violation_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/lethal_boundary_violation", rclcpp::QoS(1));

  docking_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/docking_pose",
                                                        rclcpp::QoS(1).transient_local());

  // ── Obstacle-tracker snapshot (monitoring only) ───────────────────────
  // The tracker output is no longer auto-mirrored into the classification
  // layer or obstacle_polygons_. We only cache the most recent message so
  // ~/promote_obstacle can resolve a tracker id → polygon. Real-time
  // avoidance now flows through the costmap obstacle_layer + the
  // gravity-aware ground filter, and permanent keepouts come from the
  // user via promote_obstacle.
  obstacle_sub_ = create_subscription<mowgli_interfaces::msg::ObstacleArray>(
      "/obstacle_tracker/obstacles",
      rclcpp::QoS(1),
      [this](mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
      {
        on_obstacles(std::move(msg));
      });

  promote_obstacle_srv_ = create_service<mowgli_interfaces::srv::PromoteObstacle>(
      "~/promote_obstacle",
      [this](const mowgli_interfaces::srv::PromoteObstacle::Request::SharedPtr req,
             mowgli_interfaces::srv::PromoteObstacle::Response::SharedPtr res)
      {
        on_promote_obstacle(req, res);
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

    // Build the three coupled dock polygons. All three rectangles live in
    // the dock's local frame (origin at docking_pose_, +X forward) and are
    // rotated into the map frame using d_yaw.
    //
    //   * body     — physical dock structure: from -dock_body_length_m
    //                (rear of body, behind dock origin) to 0 along +X,
    //                ±dock_body_width_m/2 along Y. Marks OBSTACLE_PERMANENT.
    //   * corridor — approach lane: from -approach_back to
    //                -dock_body_length_m along +X (i.e. immediately behind
    //                the body), ±half_width along Y. Marks DOCKING_AREA,
    //                carved out of keepout mask.
    //   * exclusion — union (-approach_back to 0, ±max half-widths). Kept
    //                 for visualization / GUI overlay only.
    //
    // Convention: dock_pose_ is the *docked-robot* pose (front of robot
    // touching the dock). +X in dock-local frame points AWAY from the dock
    // (the staging direction); the body sits in -X, behind the docked
    // robot's reference point. The corridor extends further in -X.
    const double body_len = dock_body_length_m_;
    const double body_half_width = 0.5 * dock_body_width_m_;
    const double corridor_back = dock_approach_corridor_length_m_;
    const double corridor_half_width = dock_approach_corridor_half_width_m_;
    const double d_x = docking_pose_.position.x;
    const double d_y = docking_pose_.position.y;
    const double d_yaw = 2.0 * std::atan2(docking_pose_.orientation.z, docking_pose_.orientation.w);
    const double cy = std::cos(d_yaw);
    const double sy = std::sin(d_yaw);

    auto append_rect =
        [&](geometry_msgs::msg::Polygon& poly, double x_min, double x_max, double y_half)
    {
      const double corners[][2] = {
          {x_max, y_half},
          {x_max, -y_half},
          {x_min, -y_half},
          {x_min, y_half},
      };
      for (const auto& c : corners)
      {
        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(d_x + cy * c[0] - sy * c[1]);
        pt.y = static_cast<float>(d_y + sy * c[0] + cy * c[1]);
        pt.z = 0.0F;
        poly.points.push_back(pt);
      }
      poly.points.push_back(poly.points.front());
    };

    append_rect(dock_body_polygon_, -body_len, 0.0, body_half_width);
    append_rect(dock_corridor_polygon_, -corridor_back, -body_len, corridor_half_width);
    append_rect(dock_exclusion_polygon_,
                -corridor_back,
                0.0,
                std::max(body_half_width, corridor_half_width));
    has_dock_exclusion_ = true;
    RCLCPP_INFO(get_logger(),
                "Dock polygons: pose=(%.2f, %.2f) yaw=%.2f rad — "
                "body %.2fm × %.2fm (OBSTACLE_PERMANENT), "
                "corridor %.2fm × %.2fm (DOCKING_AREA, keepout carve-out)",
                d_x,
                d_y,
                d_yaw,
                body_len,
                2.0 * body_half_width,
                corridor_back - body_len,
                2.0 * corridor_half_width);
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
  last_is_charging_ = msg->is_charging;
  last_status_time_ = now();
}

void MapServerNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/)
{
  // Use TF for the definitive map-frame robot position.
  // The odom message position may be in odom frame, not map frame.
  double x = 0.0, y = 0.0;
  double yaw = 0.0;
  if (tf_buffer_)
  {
    try
    {
      auto tf = tf_buffer_->lookupTransform(map_frame_, "base_footprint", tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      // Yaw extraction valid because the EKF runs in two_d_mode (roll/pitch ≈ 0).
      const auto& q = tf.transform.rotation;
      yaw = 2.0 * std::atan2(q.z, q.w);
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

  // Maintain a rolling window of recent yaws for the docking-set gate.
  // Read the window length live each tick so `ros2 param set` works.
  {
    std::lock_guard<std::mutex> lk(recent_yaws_mutex_);
    const rclcpp::Time now_t = now();
    recent_yaws_.emplace_back(now_t, yaw);
    const rclcpp::Duration window =
        rclcpp::Duration::from_seconds(get_parameter("yaw_convergence_window_s").as_double());
    while (!recent_yaws_.empty() && (now_t - recent_yaws_.front().first) > window)
    {
      recent_yaws_.pop_front();
    }
  }

  // Latch most recent map-frame position so reachability BFS can use
  // the robot's actual location as a seed (preferred over the area
  // centroid when the robot is inside the area).
  last_robot_x_ = x;
  last_robot_y_ = y;

  // Accumulate the mowed footprint while the blade is running.
  if (mow_blade_enabled_)
  {
    stamp_mow_progress(x, y);
  }

  check_boundary_violation(x, y);
}

void MapServerNode::on_obstacles(mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
{
  // 1. Always cache the latest snapshot so ~/promote_obstacle can
  //    resolve a tracker id → polygon without a round-trip through the
  //    GUI. (Pre-existing behavior; the GUI flow stays unchanged.)
  // 2. Auto-promote PERSISTENT obstacles that the tracker has reached
  //    high confidence about into the classification layer of whichever
  //    mowing (non-navigation) area contains the obstacle's centroid.
  //    This makes the strip planner skip cells that overlap real,
  //    confirmed obstacles instead of generating strips through them
  //    and relying on FollowStrip to abort + IncrementSkippedSwaths.
  //    Each tracker id is auto-promoted at most once per node lifetime.
  //    User-driven ~/promote_obstacle still owns persistence to areas.dat.
  std::vector<std::pair<size_t, geometry_msgs::msg::Polygon>> to_promote;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    last_tracker_snapshot_ = msg->obstacles;

    // Auto-promotion is opt-in (default off). Snapshot still updates so
    // the GUI / ~/promote_obstacle service can resolve a tracker id; we
    // just skip the automatic stamp into the classification layer.
    if (!auto_promote_persistent_obstacles_)
    {
      return;
    }

    for (const auto& obs : msg->obstacles)
    {
      if (obs.status != mowgli_interfaces::msg::TrackedObstacle::PERSISTENT)
        continue;
      if (auto_promoted_obstacle_ids_.count(obs.id))
        continue;
      if (obs.polygon.points.size() < 3)
        continue;

      geometry_msgs::msg::Point32 centroid32;
      centroid32.x = static_cast<float>(obs.centroid.x);
      centroid32.y = static_cast<float>(obs.centroid.y);

      for (size_t i = 0; i < areas_.size(); ++i)
      {
        if (areas_[i].is_navigation_area)
          continue;
        if (point_in_polygon(centroid32, areas_[i].polygon))
        {
          to_promote.emplace_back(i, obs.polygon);
          auto_promoted_obstacle_ids_.insert(obs.id);
          RCLCPP_INFO(get_logger(),
                      "auto-promoting tracker obstacle #%u (centroid %.2f,%.2f) "
                      "into area %zu — strip planner will skip overlapping cells",
                      obs.id,
                      obs.centroid.x,
                      obs.centroid.y,
                      i);
          break;
        }
      }
    }
  }

  // apply_promoted_obstacle takes map_mutex_ itself, so call outside the
  // lock above. Failures are logged but non-fatal: the tracker id stays
  // in auto_promoted_obstacle_ids_ either way to prevent retry storms.
  for (const auto& [area_idx, poly] : to_promote)
  {
    if (!apply_promoted_obstacle(area_idx, poly))
    {
      RCLCPP_WARN(get_logger(),
                  "auto-promotion to area %zu rejected (polygon validation failed)",
                  area_idx);
    }
  }
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
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(map_);
    grid_map_pub_->publish(std::move(grid_map_msg));

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

    // Republish the mowed overlay only when it grew this period (transient_local
    // keeps late subscribers up to date without re-sending an unchanged grid).
    if (mow_progress_dirty_)
    {
      publish_mow_progress();
      mow_progress_dirty_ = false;
    }
  }
}

void MapServerNode::stamp_mow_progress(double x, double y)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  const std::string layer = "mowed";
  // Lazily (re)create the progress grid whenever the main map's geometry
  // changes (startup default → resize-to-areas, or a map delete). Reusing the
  // main map's geometry keeps the overlay aligned with everything else.
  if (!mow_progress_map_.exists(layer) || mow_progress_map_.getSize()(0) != map_.getSize()(0) ||
      mow_progress_map_.getSize()(1) != map_.getSize()(1) ||
      mow_progress_map_.getPosition() != map_.getPosition())
  {
    mow_progress_map_ = grid_map::GridMap({layer});
    mow_progress_map_.setFrameId(map_frame_);
    mow_progress_map_.setGeometry(map_.getLength(), map_.getResolution(), map_.getPosition());
    mow_progress_map_[layer].setConstant(0.0F);
  }

  const grid_map::Position center(x, y);
  const double radius = std::max(tool_width_ * 0.5, mow_progress_map_.getResolution());
  for (grid_map::CircleIterator it(mow_progress_map_, center, radius); !it.isPastEnd(); ++it)
  {
    if (mow_progress_map_.at(layer, *it) < 100.0F)
    {
      mow_progress_map_.at(layer, *it) = 100.0F;
      mow_progress_dirty_ = true;
    }
  }
}

void MapServerNode::publish_mow_progress()
{
  const std::string layer = "mowed";
  if (!mow_progress_map_.exists(layer))
  {
    return;
  }
  nav_msgs::msg::OccupancyGrid grid;
  // 0 → unmowed (rendered transparent by the GUI), 100 → mowed. The converter
  // handles the grid_map↔OccupancyGrid index convention correctly.
  grid_map::GridMapRosConverter::toOccupancyGrid(mow_progress_map_, layer, 0.0F, 100.0F, grid);
  mow_progress_pub_->publish(grid);
}

}  // namespace mowgli_map
