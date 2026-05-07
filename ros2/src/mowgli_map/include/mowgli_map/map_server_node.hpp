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

#ifndef MOWGLI_MAP__MAP_SERVER_NODE_HPP_
#define MOWGLI_MAP__MAP_SERVER_NODE_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mowgli_map/map_types.hpp"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <mowgli_interfaces/msg/obstacle_array.hpp>
#include <mowgli_interfaces/msg/status.hpp>
#include <mowgli_interfaces/srv/add_mowing_area.hpp>
#include <mowgli_interfaces/srv/get_coverage_status.hpp>
#include <mowgli_interfaces/srv/get_mowing_area.hpp>
#include <mowgli_interfaces/srv/get_recovery_point.hpp>
#include <mowgli_interfaces/srv/paint_swath.hpp>
#include <mowgli_interfaces/srv/set_docking_point.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mowgli_map
{

/// @brief Multi-layer map service node for the Mowgli robot mower.
///
/// Maintains a grid_map::GridMap with four semantic layers:
///   - occupancy       : binary free/occupied for Nav2 costmap
///   - classification  : CellType enum stored as float
///   - mow_progress    : [0,1] freshness of mowing, decays over time
///   - confidence      : cumulative sensor observation count
///
/// The node subscribes to SLAM occupancy grids, odometry, and mower status,
/// and publishes the full multi-layer map plus a visualisation OccupancyGrid
/// for mow_progress. Persistence and zone management are offered as services.
class MapServerNode : public rclcpp::Node
{
public:
  /// @brief Construct the node, declare parameters, create map, wire up all
  ///        publishers, subscribers, services, and timers.
  explicit MapServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  ~MapServerNode() override = default;

  // Non-copyable, non-movable (ROS nodes are singletons in practice)
  MapServerNode(const MapServerNode&) = delete;
  MapServerNode& operator=(const MapServerNode&) = delete;
  MapServerNode(MapServerNode&&) = delete;
  MapServerNode& operator=(MapServerNode&&) = delete;

  // ── Accessors used by unit tests ────────────────────────────────────────

  /// Direct access to the underlying map (test-only, guarded by map_mutex_).
  grid_map::GridMap& map()
  {
    return map_;
  }
  const grid_map::GridMap& map() const
  {
    return map_;
  }

  /// Mutex guarding the map (test-only).
  std::mutex& map_mutex()
  {
    return map_mutex_;
  }

  /// Expose decay rate for unit tests.
  double decay_rate_per_hour() const
  {
    return decay_rate_per_hour_;
  }

  /// Expose mower width for unit tests.
  double mower_width() const
  {
    return mower_width_;
  }

  /// Run the publish/decay timer callback once (test-only).
  void tick_once(double elapsed_seconds);

  /// Mark cells mowed around a given position (test-only).
  void mark_mowed(double x, double y);

  /// Clear all layers to their default values.
  void clear_map_layers();

  /// Build coverage cells OccupancyGrid (test-only accessor).
  nav_msgs::msg::OccupancyGrid coverage_cells_to_occupancy_grid() const;

  /// Test-only: directly invoke the add_area service handler.
  void add_area_for_test(const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
                         mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res);

  /// Test-only: directly invoke get_mowing_area service handler.
  void get_mowing_area_for_test(const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
                                mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res);

  /// Test-only: round-trip persistence through save/load_areas_to_file.
  void save_areas_for_test(const std::string& path);
  void load_areas_for_test(const std::string& path);

private:
  // ── ROS callbacks ────────────────────────────────────────────────────────

  /// Convert incoming nav_msgs/OccupancyGrid to the occupancy layer.
  void on_occupancy_grid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /// Cache the latest Nav2 costmap (used by the cell-segment walker as a
  /// live obstacle source — independent of the slower obstacle_tracker
  /// pipeline, which is reserved for user-validated persistent obstacles).
  void on_costmap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /// True when the cached Nav2 costmap reports the world point (x, y) as
  /// occupied (cell value ≥ costmap_obstacle_threshold_, or inflated
  /// LETHAL after Nav2 conversion). Returns false if no costmap has been
  /// received yet, so callers fall back to the classification layer alone.
  bool is_costmap_blocked(double x, double y) const;

  /// Update mow blade state from mower status.
  void on_mower_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg);

  /// Update mow_progress and confidence layers based on robot position.
  /// Also checks boundary violation.
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// Receive persistent obstacle updates from ObstacleTracker.
  void on_obstacles(mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg);

  // ── Timer callback ───────────────────────────────────────────────────────

  /// Apply decay to mow_progress, publish grid_map and progress OccupancyGrid.
  void on_publish_timer();

  // ── Services ─────────────────────────────────────────────────────────────

  void on_save_map(const std_srvs::srv::Trigger::Request::SharedPtr req,
                   std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_load_map(const std_srvs::srv::Trigger::Request::SharedPtr req,
                   std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_clear_map(const std_srvs::srv::Trigger::Request::SharedPtr req,
                    std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_add_area(const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
                   mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res);

  void on_get_mowing_area(const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
                          mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res);

  void on_set_docking_point(const mowgli_interfaces::srv::SetDockingPoint::Request::SharedPtr req,
                            mowgli_interfaces::srv::SetDockingPoint::Response::SharedPtr res);

  void on_save_areas(const std_srvs::srv::Trigger::Request::SharedPtr req,
                     std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_load_areas(const std_srvs::srv::Trigger::Request::SharedPtr req,
                     std_srvs::srv::Trigger::Response::SharedPtr res);

  // ── Coverage services ────────────────────────────────────────────────────

  /// Mark mow_progress along a path (called by FollowCoveragePath after a
  /// successful FollowPath run — opennav_coverage doesn't track which
  /// cells were actually driven, so we paint them here for resume +
  /// obstacle-disappear handling).
  void on_paint_swath(const mowgli_interfaces::srv::PaintSwath::Request::SharedPtr req,
                      mowgli_interfaces::srv::PaintSwath::Response::SharedPtr res);

  /// Manual reset: revert every LAWN_DEAD cell back to LAWN
  /// and zeros fail_count. Useful at session start or when the
  /// operator removes the obstacle that caused the DEAD promotion.
  void on_clear_dead_cells(const std_srvs::srv::Trigger::Request::SharedPtr req,
                           std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_get_coverage_status(
      const mowgli_interfaces::srv::GetCoverageStatus::Request::SharedPtr req,
      mowgli_interfaces::srv::GetCoverageStatus::Response::SharedPtr res);

  /// Compute a recovery pose inside the nearest mowing area.
  ///
  /// Called by the BT SoftBoundaryHandler when the robot has drifted past a
  /// polygon edge but is still inside the lethal margin. Finds the closest
  /// point on the nearest polygon edge, offsets `boundary_recovery_offset_m_`
  /// further along the inward direction (robot → edge), and returns a Pose
  /// facing into the area.
  void on_get_recovery_point(const mowgli_interfaces::srv::GetRecoveryPoint::Request::SharedPtr req,
                             mowgli_interfaces::srv::GetRecoveryPoint::Response::SharedPtr res);

  // ── Helpers ───────────────────────────────────────────────────────────────

  /// Initialise the grid_map with all four layers and correct geometry.
  void init_map();

  /// Resize the map to fit all loaded areas (with margin), re-initialising layers.
  void resize_map_to_areas();

  /// Convert mow_progress layer to a nav_msgs/OccupancyGrid (0–100 scale).
  nav_msgs::msg::OccupancyGrid mow_progress_to_occupancy_grid() const;

  /// Apply time-based decay to the mow_progress layer.
  /// @param elapsed_seconds Time since last decay application.
  void apply_decay(double elapsed_seconds);

  /// Mark all cells within mower_width_ / 2 of (x, y) as freshly mowed.
  void mark_cells_mowed(double x, double y);

  /// Check whether a point is inside a polygon (ray-casting algorithm).
  static bool point_in_polygon(const geometry_msgs::msg::Point32& pt,
                               const geometry_msgs::msg::Polygon& polygon) noexcept;

  /// Build and publish the keepout OccupancyGrid mask and CostmapFilterInfo.
  /// Outside the mowing boundary → 100 (lethal).  No-go zones → 100.
  /// Inside the mowing boundary → 0 (free).
  /// Does nothing if mowing_area_polygon_ has fewer than 3 points.
  /// Caller must hold map_mutex_.
  void publish_keepout_mask();

  /// Check if the robot is outside all allowed polygons and publish violation.
  void check_boundary_violation(double x, double y);

  /// Compare incoming obstacles to current set and trigger replan if needed.
  void diff_and_update_obstacles(
      const std::vector<mowgli_interfaces::msg::TrackedObstacle>& incoming);

  /// Build and publish the speed OccupancyGrid mask and CostmapFilterInfo.
  /// Cells within one tool_width of the mowing boundary → 50 (50 % speed).
  /// All other interior cells → 0 (full speed).
  /// Does nothing if areas_ is empty.
  /// Caller must hold map_mutex_.
  void publish_speed_mask();

  /// Load pre-defined mowing/navigation areas from ROS parameters.
  void load_areas_from_params();

  /// Parse a polygon from "x1,y1;x2,y2;..." string format.
  static geometry_msgs::msg::Polygon parse_polygon_string(const std::string& s);

  /// Serialize a polygon to "x1,y1;x2,y2;..." string format.
  static std::string polygon_to_string(const geometry_msgs::msg::Polygon& poly);

  /// Save areas and docking point to a YAML file.
  void save_areas_to_file(const std::string& path);

  /// Load areas and docking point from a YAML file.
  void load_areas_from_file(const std::string& path);

  /// Reapply area classifications to the map grid (called after loading areas).
  void apply_area_classifications();

  // ── Coverage statistics ──────────────────────────────────────────────────
  // Path planning has moved to opennav_coverage; we only keep the cell-level
  // bookkeeping here. compute_coverage_stats counts mowed / total / obstacle
  // cells in an area so GetCoverageStatus and GetNextUnmowedArea can decide
  // whether work remains. PaintSwath (above) writes the mowed cells.

  void compute_coverage_stats(size_t area_index,
                              uint32_t& total,
                              uint32_t& mowed,
                              uint32_t& obstacle_cells) const;

  // ── Area entry ────────────────────────────────────────────────────────────

  /// A named area (mowing or navigation) with optional interior obstacles.
  struct AreaEntry
  {
    std::string name;
    geometry_msgs::msg::Polygon polygon;
    std::vector<geometry_msgs::msg::Polygon> obstacles;
    bool is_navigation_area{false};
  };

  // ── Parameters ────────────────────────────────────────────────────────────
  double resolution_;
  double map_size_x_;
  double map_size_y_;
  std::string map_frame_;
  double decay_rate_per_hour_;
  double mower_width_;
  // Path C — fail-count + DEAD promotion tuning. Defaults are
  // declared in the constructor.
  /// Number of consecutive segment failures before a cell is
  /// promoted to LAWN_DEAD.
  double dead_promote_threshold_{3.0};
  /// fail_count decay rate. Cells unblock when fail_count drops
  /// below dead_unblock_threshold_. Default 0.5 / hour means a cell
  /// at 3 failures auto-recovers in ~6h, at 2 failures in ~4h. Set
  /// to 0 to disable decay entirely (DEAD is forever).
  double dead_decay_rate_per_hour_{0.5};
  /// fail_count threshold below which LAWN_DEAD reverts to LAWN.
  double dead_unblock_threshold_{1.0};
  std::string map_file_path_;
  std::string areas_file_path_;
  double publish_rate_;
  double keepout_nav_margin_;
  /// Distance past the nearest allowed-area edge at which a boundary
  /// violation is classified as "lethal" (emergency stop) rather than
  /// just "soft" (attempt recovery back inside).
  double lethal_boundary_margin_m_{0.5};

  /// Deadband for the soft boundary violation flag — RTK noise + FTC
  /// tracking error must push the robot more than this many metres
  /// outside any polygon before /boundary_violation fires.
  double soft_boundary_margin_m_{0.10};

  /// How far inside the polygon the soft-recovery pose should sit, measured
  /// along the robot → edge direction. Large enough that subsequent controller
  /// jitter doesn't immediately cross the boundary again.
  double boundary_recovery_offset_m_{0.8};

  /// Cells inside a mowing area but within this distance of the polygon edge
  /// are marked LETHAL in the keepout mask, so the Smac planner keeps the
  /// transit/coverage path that much away from the real boundary. This gives
  /// the FTC controller room to track without overshooting past the edge.
  /// Default 0.3 m — pairs with inflation_radius 0.4 m for a total soft-wall
  /// of ~0.7 m inside the polygon.
  double boundary_inner_margin_m_{0.3};


  /// Dock physical body dimensions in dock-local frame (origin = robot's
  /// body when docked, +X = direction robot faces). Defines the rectangle
  /// used for both the dock_exclusion_polygon (classification + keepout
  /// carve-out) and the dock_planning_polygon (F2C polygon hole). No
  /// approach corridor — that was creating phantom obstacles 1.5 m around
  /// the dock that blocked all Nav2 motion after undock.
  double dock_body_forward_m_{0.45};
  double dock_body_back_m_{0.35};
  double dock_body_half_width_m_{0.275};

  // ── State ─────────────────────────────────────────────────────────────────
  grid_map::GridMap map_;
  mutable std::mutex map_mutex_;

  bool mow_blade_enabled_{false};
  rclcpp::Time last_decay_time_;

  // ── Replanning state ──────────────────────────────────────────────────────
  /// Number of persistent obstacles at last replan (for change detection).
  std::size_t last_obstacle_count_{0};
  /// Persistent obstacle IDs already included in a replan.
  std::set<uint32_t> planned_obstacle_ids_;
  /// Timestamp of last replan trigger (for cooldown).
  rclcpp::Time last_replan_time_;
  /// Minimum seconds between replan triggers.
  double replan_cooldown_sec_{30.0};
  /// Whether a replan is pending (deferred due to cooldown).
  bool replan_pending_{false};

  /// Pre-defined areas (mowing zones + navigation corridors).
  /// Any cell inside ANY area polygon is free in the keepout mask;
  /// everything outside is lethal.
  std::vector<AreaEntry> areas_;

  /// Obstacle polygons: regions within the allowed areas that are off-limits
  /// (trees, flower beds, etc.).  Marked as lethal in the keepout mask.
  std::vector<geometry_msgs::msg::Polygon> obstacle_polygons_;

  /// Docking point in map frame.
  geometry_msgs::msg::Pose docking_pose_;
  bool docking_pose_set_{false};

  /// Dock exclusion polygon — full rectangle (dock structure + approach
  /// corridor). Used to mark cells DOCKING_AREA in the classification
  /// layer so mow_progress doesn't accumulate there, and to keep the
  /// keepout mask honest around the corridor.
  geometry_msgs::msg::Polygon dock_exclusion_polygon_;
  bool has_dock_exclusion_{false};

  /// Dock *planning* polygon — small rectangle covering only the
  /// physical dock structure, NOT the approach corridor. Sent to F2C
  /// as a polygon hole on GetMowingArea so the coverage planner cuts
  /// strips around the dock without the corridor inflating the cut.
  /// Without this split the corridor (1.5 m back × 0.8 m wide) cuts
  /// 5+ strips into disjoint segments and the boustrophedon
  /// transitions between segments confuse MPPI's local controller —
  /// see the orbital-trap analysis in the F2C swath capture.
  geometry_msgs::msg::Polygon dock_planning_polygon_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mow_progress_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr coverage_cells_pub_;

  // Costmap filter mask publishers (transient_local so late subscribers receive
  // the last message immediately — required by Nav2 costmap filter design).
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr keepout_filter_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_mask_pub_;
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr speed_filter_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr speed_mask_pub_;
  bool keepout_filter_info_sent_{false};
  bool speed_filter_info_sent_{false};

  /// Cached masks — recomputed only when areas/obstacles change.
  nav_msgs::msg::OccupancyGrid cached_keepout_mask_;
  nav_msgs::msg::OccupancyGrid cached_speed_mask_;
  bool masks_dirty_{true};

  // Replan and boundary violation publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_needed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr boundary_violation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lethal_boundary_violation_pub_;

  // Docking pose publisher (transient_local so late subscribers get the last value)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr docking_pose_pub_;

  // ── Subscribers ───────────────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  /// Latest Nav2 costmap (global by default — same frame as map_), guarded
  /// by `costmap_mutex_`. Read on every cell-walker step via
  /// `is_costmap_blocked`. Independent from `map_` so the costmap callback
  /// doesn't contend with the publish timer / segment service.
  mutable std::mutex costmap_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_costmap_;

  /// OccupancyGrid value (0–100) at which a costmap cell is considered an
  /// obstacle by the cell walker. 80 maps to Nav2 inflated/lethal cost
  /// (raw cost ≥ 200 after the standard OccupancyGrid conversion).
  int costmap_obstacle_threshold_{80};

  /// Maximum age of the cached costmap before `is_costmap_blocked` falls
  /// back to "unknown" (returns false). Guards against acting on a stale
  /// costmap if the producer dies.
  double costmap_max_age_s_{2.0};

  // ── Services ──────────────────────────────────────────────────────────────
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_srv_;
  rclcpp::Service<mowgli_interfaces::srv::AddMowingArea>::SharedPtr add_area_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetMowingArea>::SharedPtr get_mowing_area_srv_;
  rclcpp::Service<mowgli_interfaces::srv::SetDockingPoint>::SharedPtr set_docking_point_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_areas_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_areas_srv_;
  rclcpp::Service<mowgli_interfaces::srv::PaintSwath>::SharedPtr paint_swath_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_dead_cells_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetCoverageStatus>::SharedPtr get_coverage_status_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetRecoveryPoint>::SharedPtr get_recovery_point_srv_;

  // ── TF ────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Timers ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__MAP_SERVER_NODE_HPP_
