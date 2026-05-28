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
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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
#include <mowgli_interfaces/srv/get_next_segment.hpp>
#include <mowgli_interfaces/srv/get_next_strip.hpp>
#include <mowgli_interfaces/srv/get_remaining_area_polygon.hpp>
#include <mowgli_interfaces/srv/get_recovery_point.hpp>
#include <mowgli_interfaces/srv/mark_segment_blocked.hpp>
#include <mowgli_interfaces/srv/promote_obstacle.hpp>
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
  double tool_width() const
  {
    return tool_width_;
  }

  /// Run the publish/decay timer callback once (test-only).
  void tick_once(double elapsed_seconds);

  /// Mark cells mowed around a given position (test-only).
  void mark_mowed(double x, double y);

  /// Clear all layers to their default values.
  void clear_map_layers();

  /// Build coverage cells OccupancyGrid (test-only accessor).
  nav_msgs::msg::OccupancyGrid coverage_cells_to_occupancy_grid() const;

  /// Compute convex hull of 2D points (Andrew's monotone chain).
  static std::vector<std::pair<double, double>> convex_hull(
      std::vector<std::pair<double, double>> pts);

  /// Compute optimal mow angle from polygon via Minimum Bounding Rectangle.
  /// Returns angle in radians: the direction strips should run parallel to.
  static double compute_optimal_mow_angle(const geometry_msgs::msg::Polygon& poly);

  /// Compute or retrieve cached strip layout for an area (test-only).
  void ensure_strip_layout(size_t area_index);

  /// Path C — public test handle for the cell-based segment selector.
  /// Caller must hold map_mutex_ for thread safety. See the private
  /// declaration below for parameter semantics.
  bool find_next_segment_public(size_t area_index,
                                double robot_x,
                                double robot_y,
                                double robot_yaw,
                                double prefer_dir_yaw,
                                bool boustrophedon,
                                double max_segment_length_m,
                                double& out_start_x,
                                double& out_start_y,
                                double& out_end_x,
                                double& out_end_y,
                                int& out_cell_count,
                                std::string& out_termination_reason,
                                bool& out_is_long_transit,
                                bool& out_coverage_complete,
                                std::vector<std::pair<double, double>>* out_via_points = nullptr) const;

  /// Test-only: forward to the private apply_promoted_obstacle.
  /// Lets `test_map_server` exercise obstacle promotion without going
  /// through the ROS service plumbing.
  bool apply_promoted_obstacle_for_test(size_t area_index,
                                        const geometry_msgs::msg::Polygon& polygon)
  {
    return apply_promoted_obstacle(area_index, polygon);
  }

  /// Test-only: directly invoke the add_area service handler.
  void add_area_for_test(const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
                         mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res);

  /// Test-only: directly invoke get_mowing_area service handler.
  void get_mowing_area_for_test(const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
                                mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res);

  /// Test-only: round-trip persistence through save/load_areas_to_file.
  void save_areas_for_test(const std::string& path);
  void load_areas_for_test(const std::string& path);

  /// Test-only: directly invoke the get_remaining_area_polygon handler.
  void get_remaining_area_polygon_for_test(
      const mowgli_interfaces::srv::GetRemainingAreaPolygon::Request::SharedPtr req,
      mowgli_interfaces::srv::GetRemainingAreaPolygon::Response::SharedPtr res)
  {
    on_get_remaining_area_polygon(req, res);
  }

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

  /// Cache the latest /obstacle_tracker/obstacles message so the
  /// promote_obstacle service can look up an observation by id. The
  /// tracker subscription is for snapshot lookup ONLY — it no longer
  /// mutates the classification layer or obstacle_polygons_. User
  /// validation (via promote_obstacle) is the single source of truth
  /// for permanent keepouts now.
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

  /// User-promotion of a tracker observation (or raw polygon) to a
  /// permanent keepout. See PromoteObstacle.srv for the contract.
  void on_promote_obstacle(
      const mowgli_interfaces::srv::PromoteObstacle::Request::SharedPtr req,
      mowgli_interfaces::srv::PromoteObstacle::Response::SharedPtr res);

  // ── Strip planner services ───────────────────────────────────────────────

  void on_get_next_strip(const mowgli_interfaces::srv::GetNextStrip::Request::SharedPtr req,
                         mowgli_interfaces::srv::GetNextStrip::Response::SharedPtr res);

  /// Path C — cell-based coverage. Returns the next short segment to mow
  /// from the robot's current pose, ending at the first obstacle / dead
  /// cell / boundary or at max_segment_length, whichever comes first.
  /// See GetNextSegment.srv for the full contract.
  void on_get_next_segment(const mowgli_interfaces::srv::GetNextSegment::Request::SharedPtr req,
                           mowgli_interfaces::srv::GetNextSegment::Response::SharedPtr res);

  /// Path C — fail-count + DEAD promotion. Increments fail_count for
  /// each cell along a failed segment path; cells exceeding
  /// dead_promote_threshold are reclassified as LAWN_DEAD. See
  /// MarkSegmentBlocked.srv.
  void on_mark_segment_blocked(
      const mowgli_interfaces::srv::MarkSegmentBlocked::Request::SharedPtr req,
      mowgli_interfaces::srv::MarkSegmentBlocked::Response::SharedPtr res);

  /// Path C — manual reset. Reverts every LAWN_DEAD cell back to LAWN
  /// and zeros fail_count. Useful at session start or when the
  /// operator removes the obstacle that caused the DEAD promotion.
  void on_clear_dead_cells(const std_srvs::srv::Trigger::Request::SharedPtr req,
                           std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_get_coverage_status(
      const mowgli_interfaces::srv::GetCoverageStatus::Request::SharedPtr req,
      mowgli_interfaces::srv::GetCoverageStatus::Response::SharedPtr res);

  /// Returns the area polygon minus the union of mowed cells (mow_progress
  /// >= 0.3) as a list of disjoint MapArea pieces. Each piece carries the
  /// original area's obstacles plus the mowed region as additional holes.
  /// Used by the BT's PlanCoverageArea node to feed opennav_coverage with
  /// only the remaining work on each (re)start. See
  /// GetRemainingAreaPolygon.srv for the full contract.
  void on_get_remaining_area_polygon(
      const mowgli_interfaces::srv::GetRemainingAreaPolygon::Request::SharedPtr req,
      mowgli_interfaces::srv::GetRemainingAreaPolygon::Response::SharedPtr res);

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

  /// Mark all cells within tool_width_ / 2 of (x, y) as freshly mowed.
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

  /// Append a user-validated polygon as a permanent keepout for an area.
  /// Called by the ~/promote_obstacle service. Updates obstacle_polygons_,
  /// re-runs apply_area_classifications so cells become NO_GO_ZONE, marks
  /// masks_dirty_, and triggers a replan. Manages map_mutex_ internally
  /// — caller must NOT hold it.
  /// @return false if the polygon has fewer than 3 points or area_index
  ///         is out of range / a navigation area.
  bool apply_promoted_obstacle(size_t area_index, const geometry_msgs::msg::Polygon& polygon);

  /// Topological reachability analysis (DEAD redesign 2026-05-07).
  /// BFS over the area's grid cells from a seed (robot pose if inside
  /// area, else area centroid + spiral fallback), treating any cell
  /// outside the polygon, in OBSTACLE_*/NO_GO_ZONE, or costmap-blocked
  /// as a wall. Cells inside the polygon but unreachable get flipped
  /// to LAWN_DEAD; previously-DEAD cells that are now reachable flip
  /// back to LAWN. Manages map_mutex_ internally — caller must NOT hold
  /// it. Cheap (<10 ms on a 30×30 m area at 0.05 m resolution); safe to
  /// call from a 0.5 Hz timer.
  void recompute_reachability_for_area(size_t area_index);

  // try_emit_perimeter_ring is declared further down, alongside the
  // SegmentResult struct it returns into.

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

  // ── Strip planner helpers ─────────────────────────────────────────────────

public:
  /// A single mowing strip (one column in boustrophedon order).
  struct Strip
  {
    geometry_msgs::msg::Point start;  // Map frame
    geometry_msgs::msg::Point end;  // Map frame
    int column_index{0};
  };

  /// Cached strip layout for an area.
  struct StripLayout
  {
    std::vector<Strip> strips;
    double mow_angle{0.0};
    bool valid{false};
  };

  /// Pure helper: among the candidate strips marked eligible[i]==true, pick the
  /// one whose nearest endpoint (start or end) is closest to (robot_x, robot_y),
  /// and orient the returned strip so that `start` is that nearest endpoint.
  /// Returns -1 in `out_index` and leaves `out_strip` untouched if no eligible
  /// strip exists. Free-standing & state-free so it can be unit-tested without
  /// spinning a Node.
  static void select_nearest_endpoint_strip(const std::vector<Strip>& strips,
                                            const std::vector<bool>& eligible,
                                            double robot_x,
                                            double robot_y,
                                            int& out_index,
                                            Strip& out_strip);

private:
  /// Find next unmowed strip. Returns false if coverage is complete.
  bool find_next_unmowed_strip(
      size_t area_index, double robot_x, double robot_y, Strip& out_strip, bool prefer_headland);

  /// Convert a strip to a nav_msgs::Path, splitting at obstacle cells.
  nav_msgs::msg::Path strip_to_path(const Strip& strip, size_t area_index) const;

  /// Check if a strip is sufficiently mowed (>threshold of cells done).
  bool is_strip_mowed(const Strip& strip, double threshold_pct = 0.2) const;

  /// Check if a strip is blocked by obstacles (>threshold of obstacle cells).
  /// Blocked strips are treated as "frontier" and skipped during planning.
  bool is_strip_blocked(const Strip& strip, double blocked_threshold = 0.5) const;

  /// Compute coverage statistics for an area.
  void compute_coverage_stats(size_t area_index,
                              uint32_t& total,
                              uint32_t& mowed,
                              uint32_t& obstacle_cells) const;

  // ── Path C cell-based coverage ────────────────────────────────────────────

  /// Result of a single segment selection.
  struct SegmentResult
  {
    /// Start position of the segment in map frame. Equals the robot
    /// position for in-place segments, or a row entry point when the
    /// next unmowed cell is on a different row.
    double start_x{0.0};
    double start_y{0.0};
    /// Final cell of the segment (last cell that will be mowed by
    /// FollowSegment). Together with start (and via_points if any),
    /// defines the path the controller will follow.
    double end_x{0.0};
    double end_y{0.0};
    /// Optional intermediate waypoints between start_* and end_*. Empty
    /// for straight in-row segments. Populated when the planner
    /// generated a bypass arc around an obstacle: the via points are
    /// the corners of the arc (lateral offset out, offset-row span,
    /// lateral return). Path = start → via[0] → via[1] → ... → end.
    std::vector<std::pair<double, double>> via_points{};
    /// Number of cells traversed by this segment (along path_spacing
    /// granularity). For diagnostics, used as segments_remaining
    /// estimate scaling.
    int cell_count{0};
    /// Why the segment ended at end_*. Mirrors srv termination_reason.
    std::string termination_reason{};
    /// True when the segment requires a transit (>~0.5 m gap or large
    /// turn) — the BT must disengage the blade for the move.
    bool is_long_transit{false};
    /// True when the area is fully covered (no unmowed reachable cell
    /// remains). The other fields are unset in this case.
    bool coverage_complete{false};
  };

  /// Cell-based segment selector. Searches `mow_progress` for the
  /// nearest unmowed reachable LAWN cell to the robot, then walks
  /// along prefer_dir_yaw (or its inverse for boustrophedon alternate
  /// rows) until the row ends, an obstacle/dead cell appears, or
  /// max_segment_length is reached. Caller must hold map_mutex_.
  bool find_next_segment(size_t area_index,
                         double robot_x,
                         double robot_y,
                         double robot_yaw,
                         double prefer_dir_yaw,
                         bool boustrophedon,
                         double max_segment_length_m,
                         SegmentResult& out_segment) const;

  /// Generate a perimeter-ring segment around the next user-promoted
  /// obstacle whose annulus (the unmowed strip the bypass arcs leave
  /// behind) is still mostly unmowed. Used as the last fallback in
  /// find_next_segment before declaring coverage_complete: in-row
  /// strips bypass each obstacle, leaving a one-tool-width annulus
  /// of unmowed grass; this pass closes it. Caller must hold
  /// map_mutex_.
  /// @return true if a ring was emitted (out_seg populated), false if
  ///         every obstacle's annulus is already covered.
  bool try_emit_perimeter_ring(size_t area_index, SegmentResult& out_seg) const;

  // ── Area entry ────────────────────────────────────────────────────────────

  /// A named area (mowing or navigation) with optional interior obstacles.
  struct AreaEntry
  {
    std::string name;
    geometry_msgs::msg::Polygon polygon;
    std::vector<geometry_msgs::msg::Polygon> obstacles;
    bool is_navigation_area{false};
  };

  /// One-shot per-area flag (mutable so the const find_next_segment
  /// can record it): false means find_next_segment will emit a
  /// perimeter (headland) pass on its next call for that area before
  /// falling back to the boustrophedon row planner. The headland
  /// path mows ~one mower-width inside the polygon edge so the robot
  /// has a free lane to turn into at every subsequent strip end —
  /// without this, the boustrophedon strips run all the way to the
  /// inset boundary and headland turns happen right at the polygon
  /// edge where the robot can't safely pivot. Reset by
  /// reset_mow_progress / area edits.
  mutable std::set<size_t> headland_emitted_areas_;

  // ── Parameters ────────────────────────────────────────────────────────────
  double resolution_;
  double map_size_x_;
  double map_size_y_;
  std::string map_frame_;
  double decay_rate_per_hour_;
  double tool_width_;
  std::string map_file_path_;
  std::string areas_file_path_;
  double publish_rate_;
  double keepout_nav_margin_;
  /// Distance past the nearest allowed-area edge at which a boundary
  /// violation is classified as "lethal" (emergency stop) rather than
  /// just "soft" (attempt recovery back inside).
  double lethal_boundary_margin_m_{0.5};

  /// Deadband for the soft boundary violation flag — the robot's
  /// chassis must be MORE than this distance outside the operator
  /// polygon before /boundary_violation fires. Defaults to
  /// chassis_width / 2 = 0.20 m so the chassis can briefly graze
  /// outside the polygon during corner traversals (FTC tracking
  /// error ~0.15 m) without triggering recovery. The blade itself
  /// only extends tool_width / 2 = 0.09 m from base_link, so even
  /// at the worst-case 0.20 m chassis excursion the blade tip is
  /// still inside-polygon — no unauthorised cutting. The lethal
  /// boundary at 0.50 m remains the hard safety net.
  double soft_boundary_margin_m_{0.20};

  /// Number of consecutive on_odom samples that must report the robot
  /// outside (beyond soft_boundary_margin_m_) before /boundary_violation
  /// asserts true. Filters out single-tick EKF jumps caused by absolute
  /// yaw corrections during PRE_ROTATE — without it, a 100 ms map→odom
  /// burp is enough to abort an entire mowing run.
  int boundary_debounce_samples_{3};

  /// Live counter of consecutive samples reporting the robot outside.
  /// Reset to 0 the first time the robot is back inside the polygon.
  int consecutive_outside_samples_{0};

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

  /// How far inside the polygon strip endpoints must sit. Applied when the
  /// coverage planner generates strips: the axis-aligned bounding-box
  /// y-intersections are shrunk by this value on both ends. Must cover the
  /// controller's worst-case lateral tracking error — field test showed
  /// ~0.5 m overshoot at 0.3 m/s transit, so default 0.5 m is the minimum
  /// safe margin. Was previously hard-coded to tool_width_ (~0.18 m)
  /// which let coverage paths land well past the polygon edge during
  /// tracker overshoot.
  double strip_boundary_margin_m_{0.5};

  /// Mowing strip angle override (degrees). NaN = auto-compute from polygon
  /// shape via Minimum Bounding Rectangle. 0 = north-south, 90 = east-west.
  double mow_angle_override_deg_{std::numeric_limits<double>::quiet_NaN()};

  /// Dock body extent in dock local frame (m). The body is the physical
  /// dock structure the robot cannot drive through. Cells inside the body
  /// rectangle are marked OBSTACLE_PERMANENT — strips stop here, and Smac
  /// treats it as lethal. Defaults match the YardForce500 dock.
  double dock_body_length_m_{0.80};
  double dock_body_width_m_{0.55};

  /// Dock approach corridor in dock local frame (m). Rectangle behind the
  /// dock body along -X used by opennav_docking for final alignment. Cells
  /// here are classified DOCKING_AREA (mowable — corridor lawn still gets
  /// cut) and explicitly carved out of the keepout mask so Smac can plan
  /// transit through them post-undock.
  double dock_approach_corridor_length_m_{1.5};
  double dock_approach_corridor_half_width_m_{0.40};

  /// Robot chassis width (m). Read from mowgli_robot.yaml so the bypass
  /// arc planner uses the actual robot footprint when sizing the lateral
  /// offset around discrete obstacles.
  double chassis_width_m_{0.40};

  /// Bypass-arc tuning knobs.
  ///   bypass_safety_margin_m_  — extra clearance added to chassis_width/2
  ///                              when offsetting around an obstacle.
  ///                              0.05 m is enough to absorb FTC tracking
  ///                              error without hitting collision_monitor.
  ///   bypass_max_length_m_     — give-up threshold along the row. If the
  ///                              obstacle's u-extent exceeds this, the
  ///                              segment ends at the obstacle entry as
  ///                              before — at that scale it's a wall, not
  ///                              a discrete obstacle, and the next-row
  ///                              scan will pick up the cells past it.
  ///                              Reads max_obstacle_avoidance_distance
  ///                              from mowgli_robot.yaml (default 2.0).
  double bypass_safety_margin_m_{0.05};
  double bypass_max_length_m_{2.0};

  // ── State ─────────────────────────────────────────────────────────────────
  grid_map::GridMap map_;
  mutable std::mutex map_mutex_;

  bool mow_blade_enabled_{false};
  rclcpp::Time last_decay_time_;

  /// Most recent map-frame robot position (latched in on_odom). Used as
  /// the preferred seed for reachability_for_area when the robot is
  /// inside the area being analysed.
  double last_robot_x_{0.0};
  double last_robot_y_{0.0};
  /// Throttle the reachability BFS — recomputing every publish_timer
  /// tick is wasted work when neither the costmap nor the polygons
  /// changed. Recompute when masks_dirty_ flips OR every
  /// reachability_period_s seconds.
  rclcpp::Time last_reachability_time_{0, 0, RCL_ROS_TIME};
  double reachability_period_s_{2.0};

  /// Pre-defined areas (mowing zones + navigation corridors).
  /// Any cell inside ANY area polygon is free in the keepout mask;
  /// everything outside is lethal.
  std::vector<AreaEntry> areas_;

  /// Obstacle polygons: regions within the allowed areas that are off-limits
  /// (trees, flower beds, etc.). Marked as lethal in the keepout mask.
  /// Single source of truth: area YAML on disk + ~/promote_obstacle. Not
  /// auto-mirrored from /obstacle_tracker/obstacles anymore (that path
  /// was always-on and clobbered any user-validated keepouts on every
  /// tracker tick).
  std::vector<geometry_msgs::msg::Polygon> obstacle_polygons_;

  /// Most recent /obstacle_tracker/obstacles snapshot, kept ONLY so that
  /// the ~/promote_obstacle service can resolve a tracker id → polygon
  /// without a round-trip through the GUI. Has no effect on costmap or
  /// classification — promote_obstacle is the only path that mutates
  /// permanent keepouts.
  std::vector<mowgli_interfaces::msg::TrackedObstacle> last_tracker_snapshot_;

  /// Tracker ids whose polygons we have already pushed into the
  /// classification layer via the auto-promotion path (on_obstacles).
  /// Bounded growth: each PERSISTENT obstacle id is auto-promoted at
  /// most once per node lifetime. Cleared on `~/clear_obstacles`. Only
  /// populated when auto_promote_persistent_obstacles_ is true.
  std::set<uint32_t> auto_promoted_obstacle_ids_;

  /// When false (default), tracker observations never become permanent
  /// keepouts on their own — only the operator-driven ~/promote_obstacle
  /// service mutates the classification layer. When true, restores the
  /// pre-2026-05-13 behavior where any PERSISTENT TrackedObstacle inside
  /// a mowing area is auto-stamped as OBSTACLE_PERMANENT.
  bool auto_promote_persistent_obstacles_{false};

  /// Docking point in map frame.
  geometry_msgs::msg::Pose docking_pose_;
  bool docking_pose_set_{false};

  /// Rolling window of recent map→base_footprint yaw samples (radians).
  /// Pushed by on_odom; consumed by on_set_docking_point to gate the
  /// service on EKF yaw convergence. After a mowgli-ros2 restart the EKF
  /// boots at yaw=0 and only converges to the true heading via gyro+wheel
  /// integration / COG / mag; on a stationary robot with no COG signal
  /// the convergence can take 30 s+, during which /gps/absolute_pose
  /// swings by lever_arm·sin(Δyaw) — i.e. hundreds of mm when yaw drifts
  /// tens of degrees. Persisting a dock pose during that window pins it
  /// to a wildly wrong location. The gate rejects set_docking_point when
  /// the recent yaw std exceeds yaw_convergence_threshold_rad_.
  std::deque<std::pair<rclcpp::Time, double>> recent_yaws_;
  mutable std::mutex recent_yaws_mutex_;
  double yaw_convergence_threshold_rad_{0.00873};  ///< 0.5°
  double yaw_convergence_window_s_{5.0};
  size_t yaw_convergence_min_samples_{20};

  /// Latest /hardware_bridge/status snapshot. on_set_docking_point requires
  /// last_is_charging_=true so the operator can't pin a dock pose while the
  /// robot is parked elsewhere. last_status_time_ guards against stale
  /// snapshots (e.g. firmware bridge crashed) — the gate rejects when the
  /// last status is older than dock_set_status_max_age_s_.
  bool last_is_charging_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};

  /// Latest /gps/pose_cov snapshot. on_set_docking_point requires the
  /// max(σ_xx, σ_yy) below dock_set_gps_accuracy_max_m_ AND a recent sample
  /// (< dock_set_gps_max_age_s_). RTK-Fixed reports σ ≈ 3 mm here; Float is
  /// 10-50 cm.
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr last_gps_pose_cov_;
  rclcpp::Time last_gps_pose_cov_time_{0, 0, RCL_ROS_TIME};
  mutable std::mutex last_gps_pose_cov_mutex_;

  /// Rolling window of recent /gps/pose_cov (x, y) map-frame positions, used
  /// by on_set_docking_point to AVERAGE the docked position. The dock pose
  /// MUST be captured from the independent GPS-vs-datum projection, NOT the
  /// fused /odometry/filtered_map: when the robot is charging, fusion_graph
  /// gauge-resets the fused pose onto the *existing* dock_pose, so capturing
  /// the fused pose just re-stores the old (possibly wrong) value — a
  /// calibration that can never correct itself. /gps/pose_cov is the raw
  /// lever-arm-corrected GPS position and is free of that circularity.
  /// Averaging over a few seconds beats the ~1-3 cm single-sample RTK jitter
  /// (the systematic dock_pose error we are fixing was ~5 cm, so an unaveraged
  /// sample would trade one error for another).
  std::deque<std::tuple<rclcpp::Time, double, double>> recent_gps_xy_;
  double dock_set_gps_avg_window_s_{3.0};
  size_t dock_set_gps_avg_min_samples_{10};

  /// Thresholds for the on_set_docking_point gates beyond yaw convergence.
  double dock_set_gps_accuracy_max_m_{0.04};   ///< 4 cm
  double dock_set_gps_max_age_s_{2.0};
  double dock_set_status_max_age_s_{3.0};

  /// Three coupled dock polygons in map frame, all derived from
  /// docking_pose_ + dock_body/corridor parameters. Built once at startup.
  ///   * dock_body_polygon_     — physical dock body (0.80×0.55 m default).
  ///                              Marks OBSTACLE_PERMANENT in classification;
  ///                              strips stop here, Smac treats as lethal.
  ///   * dock_corridor_polygon_ — approach lane behind dock (1.5×0.80 m).
  ///                              Marks DOCKING_AREA in classification
  ///                              (mowable); explicitly carved out of the
  ///                              keepout mask so Smac can plan post-undock.
  ///   * dock_exclusion_polygon_ — union of the two above (kept for backward
  ///                              compat / visualization). Not consumed by
  ///                              the planner directly; body and corridor
  ///                              polygons drive all real behavior.
  geometry_msgs::msg::Polygon dock_body_polygon_;
  geometry_msgs::msg::Polygon dock_corridor_polygon_;
  geometry_msgs::msg::Polygon dock_exclusion_polygon_;
  bool has_dock_exclusion_{false};

  /// Cached strip layouts per area (recomputed when area changes).
  std::vector<StripLayout> strip_layouts_;

  /// Track current strip index per area for boustrophedon ordering.
  std::vector<int> current_strip_idx_;

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
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_pose_cov_sub_;

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
  rclcpp::Service<mowgli_interfaces::srv::GetNextStrip>::SharedPtr get_next_strip_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetNextSegment>::SharedPtr get_next_segment_srv_;
  rclcpp::Service<mowgli_interfaces::srv::MarkSegmentBlocked>::SharedPtr mark_segment_blocked_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_dead_cells_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetCoverageStatus>::SharedPtr get_coverage_status_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetRemainingAreaPolygon>::SharedPtr
      get_remaining_area_polygon_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetRecoveryPoint>::SharedPtr get_recovery_point_srv_;
  rclcpp::Service<mowgli_interfaces::srv::PromoteObstacle>::SharedPtr promote_obstacle_srv_;

  // ── TF ────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Timers ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__MAP_SERVER_NODE_HPP_
