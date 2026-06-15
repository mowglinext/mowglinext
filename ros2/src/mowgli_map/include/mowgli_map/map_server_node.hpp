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
#include <mowgli_interfaces/srv/get_mowing_area.hpp>
#include <mowgli_interfaces/srv/get_recovery_point.hpp>
#include <mowgli_interfaces/srv/promote_obstacle.hpp>
#include <mowgli_interfaces/srv/set_docking_point.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mowgli_map
{

/// @brief Multi-layer map service node for the Mowgli robot mower.
///
/// Maintains a grid_map::GridMap with two semantic layers:
///   - occupancy       : binary free/occupied for Nav2 costmap
///   - classification  : CellType enum stored as float (drives the keepout
///                       and speed costmap filter masks)
///
/// The node subscribes to SLAM occupancy grids, odometry, and mower status,
/// and publishes the full multi-layer map. Persistence and zone management
/// are offered as services.
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

  /// Expose mower width for unit tests.
  double tool_width() const
  {
    return tool_width_;
  }

  /// Clear all layers to their default values.
  void clear_map_layers();

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

  /// Test-only: build the keepout mask and return a copy. Exercises
  /// publish_keepout_mask() (which caches into cached_keepout_mask_) without
  /// a live ROS subscriber. Takes map_mutex_ internally — caller must NOT
  /// hold it. Lets tests assert the grid_map→OccupancyGrid index convention
  /// (CLAUDE.md #14) and the lethal-outside-areas boundary policy.
  nav_msgs::msg::OccupancyGrid build_keepout_mask_for_test()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    publish_keepout_mask();
    return cached_keepout_mask_;
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

  /// Latch the robot's map-frame position and check boundary violation.
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// Cache the latest /obstacle_tracker/obstacles message so the
  /// promote_obstacle service can look up an observation by id. The
  /// tracker subscription is for snapshot lookup ONLY — it no longer
  /// mutates the classification layer or obstacle_polygons_. User
  /// validation (via promote_obstacle) is the single source of truth
  /// for permanent keepouts now.
  void on_obstacles(mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg);

  // ── Timer callback ───────────────────────────────────────────────────────

  /// Publish the grid_map and (when dirty) the keepout/speed costmap masks.
  void on_publish_timer();

  /// Stamp a tool-width disc of "mowed" into mow_progress_map_ at the given
  /// map-frame robot position. Lazily (re)creates the grid to mirror map_'s
  /// geometry. Takes map_mutex_ internally — caller must NOT hold it.
  void stamp_mow_progress(double x, double y);

  /// Publish mow_progress_map_ as a nav_msgs/OccupancyGrid on ~/mow_progress.
  /// Caller MUST hold map_mutex_.
  void publish_mow_progress();

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
  void on_promote_obstacle(const mowgli_interfaces::srv::PromoteObstacle::Request::SharedPtr req,
                           mowgli_interfaces::srv::PromoteObstacle::Response::SharedPtr res);

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
  double tool_width_;
  std::string map_file_path_;
  std::string areas_file_path_;
  double publish_rate_;
  double keepout_nav_margin_;
  /// When true (default — operator intent "lethal area where there is no
  /// navigation or mowing area"), the keepout mask marks EVERY cell outside
  /// the union of all area polygons (mowing + navigation, minus obstacle
  /// holes) as LETHAL (100), so the Smac planner never routes there and MPPI
  /// never steers the robot out of the authorised zone. The free band that
  /// keepout_nav_margin_ would otherwise leave outside each edge is reduced
  /// to enforce_boundary_margin_m_ (a small robot-radius slack so RTK drift
  /// at the edge does not self-reject as "Start occupied"), and the dock
  /// corridor carve-out still keeps a non-lethal lane for transit/docking.
  /// When false, the legacy keepout_nav_margin_ behaviour is restored.
  /// Disable per-site only if the dock/transit corridor is not covered by a
  /// navigation area and the hard boundary would strand docking.
  bool lethal_outside_areas_{true};
  /// Slack (m) added OUTSIDE each area edge that stays FREE even when
  /// lethal_outside_areas_ is on. Absorbs RTK/pose drift at the boundary so
  /// the planner does not refuse a start pose that sits a few cm past the
  /// recorded line (the recorded outline IS the robot's CENTRE path, so the
  /// footprint legitimately overhangs it). Kept small (~robot radius) so the
  /// planner cannot draft transit detours far outside the polygon — that is
  /// the regression keepout_nav_margin_ at 0.45 m reopened. Inflation of the
  /// lethal boundary is also bounded by listing keepout_filter BEFORE
  /// inflation_layer in the costmap plugins so the wall is not inflated
  /// inward (see nav2_params_*.yaml).
  double enforce_boundary_margin_m_{0.25};
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

  /// Dedicated grid accumulating the mowed area, published as an OccupancyGrid
  /// (~/mow_progress). It carries a single "mowed" layer and is lazily resized
  /// to mirror map_'s geometry, so it stays independent of the main map's
  /// occupancy/classification lifecycle. Guarded by map_mutex_.
  grid_map::GridMap mow_progress_map_;
  bool mow_progress_dirty_{false};

  bool mow_blade_enabled_{false};

  /// Most recent map-frame robot position (latched in on_odom).
  double last_robot_x_{0.0};
  double last_robot_y_{0.0};

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
  double dock_set_gps_accuracy_max_m_{0.04};  ///< 4 cm
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

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mow_progress_pub_;

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
