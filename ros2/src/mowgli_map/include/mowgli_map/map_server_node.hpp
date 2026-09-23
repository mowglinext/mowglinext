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

#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "mowgli_map/dock_antenna_capture.hpp"
#include "mowgli_map/map_types.hpp"
#include "mowgli_map/mow_progress.hpp"
#include "mowgli_map/polygon_raster.hpp"
#include "mowgli_map/safe_transform_listener.hpp"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <mowgli_interfaces/msg/dig_event.hpp>
#include <mowgli_interfaces/msg/map_obstacle_info.hpp>
#include <mowgli_interfaces/msg/obstacle_array.hpp>
#include <mowgli_interfaces/msg/status.hpp>
#include <mowgli_interfaces/srv/add_mowing_area.hpp>
#include <mowgli_interfaces/srv/clear_obstacle.hpp>
#include <mowgli_interfaces/srv/get_mowing_area.hpp>
#include <mowgli_interfaces/srv/get_recovery_point.hpp>
#include <mowgli_interfaces/srv/promote_obstacle.hpp>
#include <mowgli_interfaces/srv/set_docking_point.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mowgli_map
{

/// Default path to the runtime mowgli_robot.yaml — bind-mounted into the
/// container so writes survive across redeploys. mowgli_robot.yaml is the
/// single source of truth for dock_pose_x/y/yaw. Overridable via the
/// `robot_yaml_path` parameter (tests point it at a temp file).
inline constexpr const char* kRuntimeRobotYaml = "/ros2_ws/config/mowgli_robot.yaml";

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

  /// Test-only: add a verified tool pose to the accumulated footprint.
  void stamp_mow_progress_for_test(double x, double y)
  {
    stamp_mow_progress(x, y);
  }

  /// Test-only: return the mowed-layer value at a map-frame position.
  float mow_progress_value_for_test(double x, double y) const;

  /// Test-only: run the regular publish path without waiting for the timer.
  void publish_mow_progress_for_test()
  {
    on_publish_timer();
  }

  /// Test-only: report whether a current mow-progress grid is cached.
  bool mow_progress_cache_valid_for_test() const;

  /// Clear all layers to their default values.
  void clear_map_layers();

  /// Test-only: forward to the private apply_promoted_obstacle.
  /// Lets `test_map_server` exercise obstacle promotion without going
  /// through the ROS service plumbing.
  bool apply_promoted_obstacle_for_test(size_t area_index,
                                        const geometry_msgs::msg::Polygon& polygon,
                                        const std::string& name = {})
  {
    return apply_promoted_obstacle(area_index,
                                   polygon,
                                   name,
                                   mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER);
  }

  /// Test-only: directly invoke the promote / discard service handlers.
  void promote_obstacle_for_test(
      const mowgli_interfaces::srv::PromoteObstacle::Request::SharedPtr req,
      mowgli_interfaces::srv::PromoteObstacle::Response::SharedPtr res)
  {
    on_promote_obstacle(req, res);
  }
  void discard_obstacle_for_test(
      const mowgli_interfaces::srv::ClearObstacle::Request::SharedPtr req,
      mowgli_interfaces::srv::ClearObstacle::Response::SharedPtr res)
  {
    on_discard_obstacle(req, res);
  }

  /// Test-only: identity of one obstacle of an area (name / source /
  /// pending / id), so persistence and proposal tests can assert on it.
  [[nodiscard]] mowgli_interfaces::msg::MapObstacleInfo obstacle_info_for_test(
      size_t area_index, size_t obstacle_index) const
  {
    mowgli_interfaces::msg::MapObstacleInfo info;
    if (area_index < areas_.size() && obstacle_index < areas_[area_index].obstacles.size())
    {
      const auto& obs = areas_[area_index].obstacles[obstacle_index];
      info.name = obs.name;
      info.source = obs.source;
      info.pending = obs.pending;
      info.id = obs.id;
    }
    return info;
  }

  /// Test-only: feed a dig report through the real handler.
  void on_dig_event_for_test(mowgli_interfaces::msg::DigEvent::ConstSharedPtr msg)
  {
    on_dig_event(std::move(msg));
  }
  /// Test-only: stand in for on_odom's TF-derived position latch (tests have
  /// no TF tree), so the "robot stands on the proposal" accept guard can be
  /// exercised.
  void set_robot_position_for_test(double x, double y)
  {
    last_robot_x_ = x;
    last_robot_y_ = y;
    have_robot_pose_ = true;
  }

  /// Test-only: forward to the private mowing_area_containing.
  [[nodiscard]] std::optional<size_t> mowing_area_containing_for_test(double x, double y) const
  {
    return mowing_area_containing(x, y);
  }

  /// Test-only: obstacle-store sizes, to assert promotion / load is
  /// idempotent (one keepout → one entry in each store). Single-threaded test
  /// use only — no locking.
  size_t obstacle_polygon_count_for_test() const
  {
    return obstacle_polygons_.size();
  }
  size_t area_obstacle_count_for_test(size_t area_index) const
  {
    return area_index < areas_.size() ? areas_[area_index].obstacles.size() : 0;
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

  /// Test-only: inspect the in-memory dock pose after a datum migration.
  const geometry_msgs::msg::Pose& docking_pose_for_test() const
  {
    return docking_pose_;
  }
  bool docking_pose_set_for_test() const
  {
    return docking_pose_set_;
  }

  /// Test-only: directly invoke the set_docking_point service handler.
  void set_docking_point_for_test(
      const mowgli_interfaces::srv::SetDockingPoint::Request::SharedPtr req,
      mowgli_interfaces::srv::SetDockingPoint::Response::SharedPtr res)
  {
    on_set_docking_point(req, res);
  }

  /// Test-only: satisfy on_set_docking_point's gate (1) (is_charging). Goes
  /// through the same update as the real /hardware_bridge/status callback, so
  /// leaving the dock drops the on-dock antenna samples here too.
  void set_charging_status_for_test(bool charging)
  {
    update_charging_status(charging);
  }

  /// Test-only: directly invoke the ~/capture_dock_antenna handler.
  void capture_dock_antenna_for_test(std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    on_capture_dock_antenna(std::make_shared<std_srvs::srv::Trigger::Request>(), res);
  }

  /// Test-only: inspect / age the pending on-dock antenna capture.
  const PendingAntennaCapture& pending_antenna_for_test() const
  {
    return pending_antenna_;
  }
  void age_pending_antenna_for_test(double seconds)
  {
    pending_antenna_.stamp_s -= seconds;
  }

  /// Test-only: what the real /gps/fix callback does with one RTK-Fixed
  /// sample — it is kept ONLY while the robot is charging.
  void on_fixed_antenna_sample_for_test(double east, double north)
  {
    push_dock_antenna_sample(east, north);
  }

  /// Test-only: satisfy gate (2) (freshness/accuracy) with one fresh
  /// /gps/pose_cov-shaped sample. Since the #446 fix this no longer feeds
  /// the position-averaging window itself — that now draws from the raw
  /// antenna samples pushed via push_gps_antenna_for_test() below, which are
  /// immune to the fused-yaw bias /gps/pose_cov's lever-arm correction can
  /// carry (see on_set_docking_point's use_gps_position block).
  void push_gps_pose_cov_for_test(double x, double y, double sigma_m)
  {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.covariance[0] = sigma_m * sigma_m;
    msg->pose.covariance[7] = sigma_m * sigma_m;
    const rclcpp::Time t = now();
    std::lock_guard<std::mutex> lk(last_gps_pose_cov_mutex_);
    last_gps_pose_cov_ = msg;
    last_gps_pose_cov_time_ = t;
  }

  /// Test-only: satisfy gate (3) (yaw convergence) with `count` identical,
  /// tightly-converged yaw samples at `yaw_rad`.
  void push_converged_yaw_for_test(double yaw_rad, size_t count)
  {
    std::lock_guard<std::mutex> lk(recent_yaws_mutex_);
    const rclcpp::Time t = now();
    for (size_t i = 0; i < count; ++i)
    {
      recent_yaws_.emplace_back(t, yaw_rad);
    }
  }

  /// Test-only: push one raw (yaw-independent) antenna ENU sample into the
  /// window on_set_docking_point's use_gps_position path averages — bypasses
  /// the real /gps/fix subscription + RTK-status gate + wgs84 projection, the
  /// same way push_gps_pose_cov_for_test() bypasses /gps/pose_cov. Call
  /// repeatedly to build up dock_set_gps_avg_min_samples_ samples.
  void push_gps_antenna_for_test(double east, double north)
  {
    std::lock_guard<std::mutex> lk(recent_gps_antenna_mutex_);
    recent_gps_antenna_enu_.emplace_back(now(), east, north);
  }

  /// Test-only: inject a known GPS lever arm (base_footprint→gps_link, body
  /// frame) so on_set_docking_point's antenna re-projection has something to
  /// apply without a live TF tree — mirrors how the real node resolves it
  /// from URDF-published TF (see lever_arm_known_ below).
  void set_gps_lever_arm_for_test(double lever_arm_x, double lever_arm_y)
  {
    lever_arm_known_ = true;
    lever_arm_x_ = lever_arm_x;
    lever_arm_y_ = lever_arm_y;
  }

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
  // ── Area entry ────────────────────────────────────────────────────────────

  /// One interior keepout of an area, with the identity that makes a
  /// machine-generated obstacle auditable. Mirrors
  /// mowgli_interfaces/msg/MapObstacleInfo (same SOURCE_* values).
  struct ObstacleEntry
  {
    geometry_msgs::msg::Polygon polygon;
    /// Operator-facing label. Empty for legacy / unnamed keepouts.
    std::string name;
    /// MapObstacleInfo::SOURCE_USER / _TRACKER / _DIG.
    uint8_t source{mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER};
    /// True while this is only a PROPOSAL: INERT — skipped by the keepout
    /// mask, the classification layer, get_mowing_area's `obstacles` (coverage
    /// holes) and areas.dat; listed only in MapArea.proposed_obstacles.
    /// Cleared (and the polygon applied) by ~/promote_obstacle{pending_id};
    /// dropped by ~/discard_obstacle. Every consumer of AreaEntry::obstacles
    /// MUST skip pending entries.
    bool pending{false};
    /// Session-scoped handle for those two services. 0 = loaded from disk.
    uint32_t id{0};
  };

  /// A named area (mowing or navigation) with optional interior obstacles.
  struct AreaEntry
  {
    std::string name;
    geometry_msgs::msg::Polygon polygon;
    std::vector<ObstacleEntry> obstacles;
    bool is_navigation_area{false};
    /// Stable, persistent identifier (mowglinext#637) — see MapArea.msg's
    /// `id` field doc comment for the full contract. 0 only transiently,
    /// in-memory, before this entry is first saved; NEVER the same thing
    /// as this entry's position in `areas_` (that shifts on any rebuild,
    /// this does not). Unlike ObstacleEntry::id above, this one is
    /// PERSISTED and must stay stable across a restart — see
    /// next_area_id_'s doc comment for how it survives one.
    uint32_t id{0};
  };

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

  /// Latch the robot's map-frame position, update verified cutting progress,
  /// and check boundary violation.
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

  /// Stamp a tool-width swept footprint into mow_progress_map_ ending at the
  /// given map-frame tool position. Lazily (re)creates the grid to mirror
  /// map_'s geometry. Takes map_mutex_ internally — caller must NOT hold it.
  void stamp_mow_progress(double x, double y);

  /// Rebuild the cached OccupancyGrid from mow_progress_map_.
  /// Caller MUST hold map_mutex_.
  void rebuild_mow_progress_cache();

  /// Publish the cached mow-progress OccupancyGrid on ~/mow_progress.
  /// Caller MUST hold map_mutex_.
  void publish_cached_mow_progress();

  /// Clear the progress map and its cached OccupancyGrid after a map reset.
  /// Caller MUST hold map_mutex_.
  void reset_mow_progress();

  /// Create an empty progress map matching map_'s current geometry and mark it
  /// for publication so transient_local history cannot retain stale geometry.
  /// Caller MUST hold map_mutex_.
  void initialize_mow_progress_map();

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

  /// ~/capture_dock_antenna: average the RAW antenna position while seated on
  /// the dock (charging + RTK gates) and hold it, unpersisted, for the
  /// set_docking_point use_pending_antenna write. See dock_antenna_capture.hpp.
  void on_capture_dock_antenna(const std_srvs::srv::Trigger::Request::SharedPtr req,
                               std_srvs::srv::Trigger::Response::SharedPtr res);

  /// Dock-pose gates shared by ~/set_docking_point and ~/capture_dock_antenna.
  /// Each returns the rejection text, or nullopt when the gate passes.
  std::optional<std::string> dock_charging_gate_rejection();
  std::optional<std::string> dock_gps_accuracy_gate_rejection();
  std::optional<std::string> average_recent_dock_antenna(Enu& mean, size_t& sample_count);
  std::optional<std::string> resolve_gps_lever_arm();

  /// Single place the charging state changes. Leaving the dock CLEARS the
  /// antenna window: it must only ever hold samples taken ON the dock.
  void update_charging_status(bool charging);

  /// Keep one RTK-Fixed raw antenna sample — only while charging.
  void push_dock_antenna_sample(double east, double north);

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

  /// Reject a pending proposal (currently: wheel-slip dig reports) by its
  /// MapObstacleInfo.id. A proposal was never applied nor persisted, so this
  /// only drops it from the list the GUI shows.
  void on_discard_obstacle(const mowgli_interfaces::srv::ClearObstacle::Request::SharedPtr req,
                           mowgli_interfaces::srv::ClearObstacle::Response::SharedPtr res);

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

  /// Append a polygon as an APPLIED keepout for an area. Called by the
  /// ~/promote_obstacle service (operator action) and the tracker
  /// auto-promotion opt-in — never by the dig-report path. Updates
  /// obstacle_polygons_, re-runs apply_area_classifications so cells become
  /// NO_GO_ZONE, marks masks_dirty_, and triggers a replan. Manages
  /// map_mutex_ internally — caller must NOT hold it.
  ///
  /// @return false if the polygon has fewer than 3 points or area_index
  ///         is out of range / a navigation area.
  bool apply_promoted_obstacle(
      size_t area_index,
      const geometry_msgs::msg::Polygon& polygon,
      const std::string& name = {},
      uint8_t source = mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER);

  /// Record an INERT proposal for an area: listed for the operator
  /// (MapArea.proposed_obstacles) and nothing else — no keepout mask, no
  /// NO_GO cells, no coverage hole, no replan, no areas.dat. Manages
  /// map_mutex_ internally.
  /// @return the proposal's session id, or nullopt when the area is invalid /
  ///         a navigation area, the polygon is degenerate, or an obstacle or
  ///         proposal already sits at that spot.
  [[nodiscard]] std::optional<uint32_t> add_obstacle_proposal(
      size_t area_index,
      const geometry_msgs::msg::Polygon& polygon,
      const std::string& name,
      uint8_t source);

  /// Accept the pending obstacle carrying `pending_id`: clear its pending
  /// flag (optionally renaming it) and APPLY it — keepout mask, NO_GO cells,
  /// replan — so the next save also writes it to areas.dat.
  /// @return the area index it belongs to, or nullopt when no pending
  ///         obstacle has that id.
  [[nodiscard]] std::optional<size_t> accept_pending_obstacle(uint32_t pending_id,
                                                              const std::string& name);

  /// Distance the robot CENTRE must keep from a proposal's polygon for an
  /// accept to be safe: the mask band that becomes lethal around it + one cell.
  [[nodiscard]] double accept_clearance_m() const;

  /// Accept guard. Returns the robot's distance to the pending polygon (0 when
  /// inside it) when accepting `pending_id` NOW would leave the robot inside
  /// the resulting lethal region — i.e. unable to plan from its own pose
  /// (START_OCCUPIED). nullopt = safe to accept (or unknown id / no pose yet).
  /// Manages map_mutex_ internally.
  [[nodiscard]] std::optional<double> robot_inside_accepted_band(uint32_t pending_id);

  /// Drop the pending obstacle carrying `pending_id` from its area's list.
  /// @return false when no PENDING obstacle has that id.
  bool discard_pending_obstacle(uint32_t pending_id);

  /// Save areas.dat if a path is configured, logging (not throwing) on
  /// failure: the live state is already updated, so the next save retries.
  void persist_areas_best_effort(const char* context);

  /// Build an ObstacleEntry, handing it the next session-scoped id so the
  /// operator can address it through ~/promote_obstacle / ~/discard_obstacle.
  [[nodiscard]] ObstacleEntry make_obstacle_entry(const geometry_msgs::msg::Polygon& polygon,
                                                  const std::string& name,
                                                  uint8_t source,
                                                  bool pending);

  /// has_duplicate_obstacle() (internal_helpers.hpp) over an area's
  /// ObstacleEntry list — same centroid-epsilon rule, different element type.
  /// `include_pending=false` ignores inert proposals: an APPLIED keepout is
  /// never a duplicate of something that was not applied.
  [[nodiscard]] static bool has_duplicate_obstacle_entry(
      const std::vector<ObstacleEntry>& existing,
      const geometry_msgs::msg::Polygon& candidate,
      double eps,
      bool include_pending = true);

  /// Handle a wheel-slip dig report from hardware_bridge_node.
  ///
  /// The bridge has already hard-stopped and reversed out. Resolves which
  /// mowing area contains the dig point, builds a compact regular polygon the
  /// size of the PHYSICAL dig (the wheel ruts — dig_proposal_polygon, never a
  /// chassis-sized box), and records it as an inert PROPOSAL
  /// (add_obstacle_proposal). It must never apply anything: the robot stands
  /// ~0.2-0.3 m from the point, and a keepout there refused every plan from
  /// its own pose (START_OCCUPIED, 2026-09-10 and 2026-09-17). Issue #500's
  /// re-dig loop is prevented by FollowStrip's dig skip zone
  /// (mowgli_behavior/dig_skip.hpp); only the operator turns a proposal into
  /// a keepout (~/promote_obstacle{pending_id}).
  void on_dig_event(mowgli_interfaces::msg::DigEvent::ConstSharedPtr msg);

  /// Index of the first mowing (non-navigation) area whose polygon contains
  /// the point, or std::nullopt if the point is outside every mowing area.
  [[nodiscard]] std::optional<size_t> mowing_area_containing(double x, double y) const;

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

  /// fsync `tmp_path`, rename it over `path`, fsync the directory. Throws (and
  /// removes the temp file) if the data cannot be made durable; `path` is then
  /// untouched.
  static void commit_file_atomically(const std::string& tmp_path, const std::string& path);

  /// Load areas and docking point from a YAML file.
  void load_areas_from_file(const std::string& path);

  /// Datum-change migration (issue #216). areas.dat is stamped with the
  /// datum its metre coordinates were recorded against. When the stamp
  /// differs from the datum this node was launched with (operator moved the
  /// base / re-centred the datum), every persisted polygon and the dock pose
  /// are re-projected into the new datum frame (old-ENU → WGS84 → new-ENU,
  /// the exact inverse/forward of the localizer's projection), the dock pose
  /// is spliced back into mowgli_robot.yaml, and areas.dat is re-saved with
  /// the new stamp — so the map stays glued to the physical garden instead
  /// of shifting with the datum.
  ///
  /// @param file_datum_lat/lon  Stamp parsed from areas.dat (NaN if absent).
  /// @param path                areas.dat path, for the re-stamping save.
  /// Caller must NOT hold map_mutex_.
  void migrate_areas_datum(double file_datum_lat, double file_datum_lon, const std::string& path);

  /// Reapply area classifications to the map grid (called after loading areas).
  /// Takes map_mutex_; clears classification_dirty_.
  void apply_area_classifications();

  /// Same, for a caller that already holds map_mutex_.
  void apply_area_classifications_locked();

  /// Re-stamp the CLASSIFICATION layer if an edit left it stale (see
  /// classification_dirty_). Every READER of that layer calls this first.
  /// Caller must hold map_mutex_.
  void ensure_classification_current_locked();

  /// Cell-centre coordinates of `map`, per row / column index. With
  /// `through_float` each coordinate is rounded through float, which is what
  /// the keepout mask's per-cell definition tests (Point32).
  static raster::CellAxes make_cell_axes(const grid_map::GridMap& map, bool through_float);

  /// Push the next keepout-mask rebuild kMapEditSettle into the future. Called
  /// by the two services a map REPLACE is made of (clear_map, add_area): the
  /// GUI sends clear_map → add_area × N → save_areas back to back, and one
  /// rebuild after the burst replaces N rebuilds of N half-built maps.
  /// Caller must NOT hold map_mutex_.
  void defer_mask_rebuild();

  /// (Re)build the three coupled dock polygons (body / corridor / exclusion)
  /// from the current docking_pose_ and set has_dock_exclusion_. Clears the
  /// polygons first so it is safe to call repeatedly. Called from the
  /// constructor AND on_set_docking_point so the lethal dock body + corridor
  /// carve-out follow a live dock re-placement without a node restart.
  void rebuild_dock_polygons();

  // ── Parameters ────────────────────────────────────────────────────────────
  double resolution_;
  double map_size_x_;
  double map_size_y_;
  std::string map_frame_;
  double tool_width_;
  /// Record wheel-slip dig locations as operator-reviewable proposals.
  bool dig_obstacle_enabled_{true};
  /// Radius of a dig proposal before the per-event slip growth [m]: the disc
  /// covering both drive-wheel contact patches (internal_helpers.hpp).
  double dig_proposal_radius_m_{0.0};
  std::string map_file_path_;
  std::string areas_file_path_;

  /// WGS84 datum this session's map frame is anchored to (from
  /// mowgli_robot.yaml, injected at launch — same values
  /// navsat_to_absolute_pose_node projects GPS fixes with). Used to stamp
  /// areas.dat on save and to detect+migrate a datum change on load
  /// (see migrate_areas_datum). 0/0 = unset (no GPS site configured yet).
  double datum_lat_{0.0};
  double datum_lon_{0.0};

  /// Runtime mowgli_robot.yaml path for dock-pose splice-back writes.
  /// Defaults to kRuntimeRobotYaml; tests override it to a temp file.
  std::string robot_yaml_path_{kRuntimeRobotYaml};
  double publish_rate_;
  double keepout_nav_margin_;
  /// When true (default — operator intent "lethal area where there is no
  /// navigation or mowing area"), the keepout mask marks EVERY cell outside
  /// the union of all area polygons (mowing + navigation, minus obstacle
  /// holes) as LETHAL (100), so the Smac planner never routes there and MPPI
  /// never steers the robot out of the authorised zone. The free band that
  /// keepout_nav_margin_ would otherwise leave outside each edge is reduced
  /// to enforce_boundary_margin_m_ (a small slack so RTK drift
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
  /// footprint legitimately overhangs it). The 0.40 m literal below is only
  /// the standalone-run fallback; full_system.launch.py FLOORS the injected
  /// value at the live chassis CIRCUMSCRIBED RADIUS
  /// (robot_config_util.chassis_circumscribed_radius = 0.597 m shipped),
  /// because chassis_safety_inset is 0: the outermost coverage pass puts the
  /// CENTRE on the recorded line and the footprint then reaches up to that
  /// radius outside it in any orientation — 0.275 m sideways but 0.53 m
  /// forward at a row end. 0.40 itself replaced a 0.25 (~robot radius) value
  /// under which a robot riding the outer headland ring had only ~5 cm between
  /// its centre cell and the inscribed-cost band — under one 0.08 m cell — so
  /// Smac transits sporadically failed "Start occupied" and FollowStrip
  /// skipped whole sub-paths (headland rings) on no-LiDAR/GPS-only installs.
  /// TRADE-OFF: the injected floor is ABOVE lethal_boundary_margin_m_ (0.5),
  /// so the planner's lethal wall no longer engages strictly inside that
  /// e-stop tripwire, and the traversable band is wider than the 0.45 m
  /// keepout_nav_margin_ regression that let transit detours drift outside.
  /// Only kSoftPenaltyMaskCost (mid-cost, never free) keeps the planner off
  /// it; watch /boundary_violation in the field.
  /// The lethal boundary is NOT inflated: the global costmap lists
  /// inflation_layer BEFORE keepout_filter (see nav2_params_*.yaml), because
  /// this band already is the body's room.
  double enforce_boundary_margin_m_{0.40};
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

  /// Cells inside a mowing/navigation area but within this distance of the
  /// polygon edge get a SOFT mid-cost penalty in the keepout mask (the same
  /// kSoftPenaltyMaskCost the outside-slack band uses, costmap_filters.cpp)
  /// — NEVER lethal. This nudges the global planner (Smac, used for
  /// point-to-point TRANSIT) to prefer a route that stays that far inside
  /// the recorded edge when one exists, without ever refusing to start,
  /// end, or pass through the band. Coverage/mowing itself is unaffected —
  /// FTC tracks the F2C path against the LOCAL costmap, which never carries
  /// this mask. Read declare_parameters(), not this initialiser — the
  /// actual default lives in the declare_parameter<double> call plus the
  /// template (mowgli_robot.yaml.boundary_inner_margin_m), per the usual
  /// gotcha.
  ///
  /// This was a LETHAL band in an earlier version of this change and was
  /// reworked to mid-cost after review: lethal here collides with
  /// chassis_safety_inset (both default to 0.20 m — the outermost coverage
  /// ring is planned exactly chassis_safety_inset inside the line, so a
  /// lethal band there plus inflation_radius would swallow the ring itself
  /// and reopen the START_OCCUPIED skip cascade, issue #487) and would also
  /// wall off any area-to-area seam narrower than 2x the inflated margin.
  /// A lethal version of this was ALSO tried even earlier and reverted
  /// (2026-04-23, commit 7f4b43d5) because dock poses commonly sit close to
  /// the polygon edge and a few cm of GNSS drift landed the robot's OWN
  /// position in a lethal cell the planner could not route out of — the
  /// mid-cost design means that failure mode cannot recur even without the
  /// dock exemption below, since a soft-cost start/goal pose never fails
  /// "Start occupied".
  double boundary_inner_margin_m_{0.3};

  /// Cells within this distance of docking_pose_ are exempt from the
  /// boundary_inner_margin_m_ penalty above, regardless of direction — kept
  /// even though the mid-cost design no longer strictly needs it for
  /// safety, so the dock approach carries no bias at all rather than merely
  /// "never blocked". Unlike dock_corridor_polygon_ (which only carves out
  /// the corridor BEHIND the dock body), this also covers the
  /// staging/approach side the robot actually occupies right after
  /// undocking, where GNSS is often still settling. 0 disables the
  /// exemption. Only applied while has_dock_exclusion_ is true (a dock pose
  /// has been set).
  double dock_inner_margin_exempt_radius_m_{2.5};

  /// LETHAL band grown around drawn / dig / promoted obstacle polygons in the
  /// keepout mask (parameter keepout_obstacle_margin). It is the WHOLE body
  /// model of the mask's consumer: SmacPlanner2D is a point check, and the
  /// global costmap lists inflation_layer BEFORE keepout_filter so the mask is
  /// not inflated on top. full_system.launch.py injects
  /// robot_config_util.keepout_obstacle_margin = the footprint half-width,
  /// raised to follow an operator-raised obstacle_margin.
  ///
  /// NOT coverage_server.obstacle_margin: that one offsets a CENTRELINE and
  /// also carries FTC's clearance + tracking slack, so it is deliberately
  /// larger — a robot on its coverage line must stay outside this band or
  /// every transit from there is START_OCCUPIED. 0 = polygon edge only.
  double keepout_obstacle_margin_m_{0.0};

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
  /// Serialized only when the progress map changes; published unchanged between
  /// updates so reconnecting WebSocket clients receive the current overlay.
  nav_msgs::msg::OccupancyGrid mow_progress_cache_;
  bool mow_progress_cache_valid_{false};
  /// Throttle for cached mowed-overlay publication. Conversion remains O(cells)
  /// only when the progress map is dirty; unchanged cached grids are cheap to
  /// republish at this interval for GUI reconnect reliability.
  double mow_progress_publish_period_s_{2.0};
  rclcpp::Time last_mow_progress_pub_time_{0, 0, RCL_ROS_TIME};

  std::string mow_progress_tool_frame_{"blade_link"};
  double mow_progress_min_blade_rpm_{1000.0};
  double mow_progress_blade_telemetry_max_age_s_{1.0};
  bool mow_blade_requested_{false};
  bool mow_blade_active_{false};
  double mow_blade_rpm_{0.0};
  rclcpp::Time mow_blade_telemetry_time_{0, 0, RCL_ROS_TIME};
  MowProgressInhibitReason mow_progress_reason_{MowProgressInhibitReason::kBladeNotRequested};
  bool have_last_mow_tool_position_{false};
  grid_map::Position last_mow_tool_position_;

  /// Most recent map-frame robot position (latched in on_odom).
  double last_robot_x_{0.0};
  double last_robot_y_{0.0};
  /// False until the first TF lookup in on_odom succeeds.
  bool have_robot_pose_{false};

  /// Pre-defined areas (mowing zones + navigation corridors).
  /// Any cell inside ANY area polygon is free in the keepout mask;
  /// everything outside is lethal.
  std::vector<AreaEntry> areas_;

  /// Next id to mint for a new area (mowglinext#637). UNLIKE
  /// next_obstacle_id_ below, this one is PERSISTED (areas.dat's
  /// `next_area_id:` line) and must survive a restart — recovered on load
  /// as max(loaded area ids) + 1, mirroring obstacle_tracker_node's own
  /// next_id_ recovery pattern for its (also-persisted) tracked-obstacle
  /// ids. Never reset by ~/clear_map: an id must never be reused for a
  /// different area, even across a clear, in case something external
  /// still holds a reference to the old one.
  uint32_t next_area_id_{1};

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

  /// Monotonic session-scoped handle handed out to every obstacle entry, so
  /// ~/promote_obstacle{pending_id} and ~/discard_obstacle can address one
  /// proposal unambiguously. Starts at 1 — 0 means "no handle".
  uint32_t next_obstacle_id_{1};

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

  /// Rolling window of recent RAW (yaw-independent) GPS antenna positions —
  /// /gps/fix projected straight through wgs84_projection.hpp, RTK-Fixed
  /// samples only — used by on_set_docking_point to AVERAGE the antenna
  /// position, then lever-arm-correct it ONCE with whatever yaw this call is
  /// about to persist (see the use_gps_position block). The dock pose MUST
  /// be captured independently of the fused /odometry/filtered_map: while
  /// charging, fusion_graph gauge-resets the fused pose onto the *existing*
  /// dock_pose, so capturing it would just re-store the old (possibly wrong)
  /// value — a calibration that can never correct itself.
  ///
  /// This averages the RAW antenna position rather than /gps/pose_cov's
  /// lever-arm-corrected one on purpose (issue #446): /gps/pose_cov applies
  /// the correction with whatever the FUSED yaw is AT EACH SAMPLE, so a
  /// stable-but-biased fused yaw (e.g. a settled-wrong magnetometer lock)
  /// silently corrupted the averaged position by
  /// lever_arm_length * sin(yaw_bias) with no way to detect it after the
  /// fact — issue #446's reported ~10 cm lateral dock-position error is
  /// consistent with exactly this (~19 degrees of bias at the default 0.3 m
  /// forward GPS offset). Averaging the yaw-independent raw antenna position
  /// and correcting with the call's OWN final yaw (which for yaw_source ==
  /// MOTION is a fresh, independently-measured heading, not the possibly-
  /// stale one used to correct earlier /gps/pose_cov samples) makes the
  /// result correct regardless of what the fused yaw happened to be during
  /// capture — an earlier revision instead cross-checked the fused yaw
  /// against /imu/cog_heading and REJECTED on disagreement, but on the dock
  /// a fresh COG sample is essentially never available (cog_to_imu_node's
  /// stationary latch inflates σ well past any usable threshold within
  /// seconds of the last forward motion), so that gate rejected the MOTION
  /// call that is the only non-circular way to fix a stale yaw. Do not
  /// reintroduce it.
  std::deque<std::tuple<rclcpp::Time, double, double>> recent_gps_antenna_enu_;
  mutable std::mutex recent_gps_antenna_mutex_;
  /// 12 s comfortably fills min_samples_ at a sustained ~1 Hz RTK-Fixed
  /// stream (the lowest `gnss_profile_rate_hz` the GUI offers) with margin
  /// for the occasional non-Fixed epoch; see the declare_parameter call site
  /// for why this needed widening from the 3 s it inherited pre-#446.
  double dock_set_gps_avg_window_s_{12.0};
  size_t dock_set_gps_avg_min_samples_{10};

  /// Raw antenna mean taken on the dock by ~/capture_dock_antenna, waiting for
  /// the motion yaw (set_docking_point use_pending_antenna). Memory only,
  /// single-use, expires after dock_antenna_capture_ttl_s_ (the robot may have
  /// been moved since). Services run on the node's single-threaded executor.
  PendingAntennaCapture pending_antenna_{};
  double dock_antenna_capture_ttl_s_{300.0};

  /// GPS lever arm (base_footprint→gps_link, body frame), resolved lazily
  /// from TF the same way navsat_to_absolute_pose_node resolves its own copy
  /// — this node runs as a separate process and cannot read that one's
  /// cached value. Retried on each use_gps_position capture until available;
  /// see the resolve call in on_set_docking_point.
  bool lever_arm_known_{false};
  double lever_arm_x_{0.0};
  double lever_arm_y_{0.0};

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

  /// How long the map must have been left alone before the publish timer
  /// rebuilds the keepout mask after a clear_map / add_area. Long enough to
  /// span the gap between two calls of a GUI map replace (a websocket + DDS
  /// round trip each), short enough that a single recorded area reaches Nav2
  /// within a couple of timer ticks.
  static constexpr std::chrono::milliseconds kMapEditSettle{1500};

  /// The publish timer leaves a dirty mask alone until this instant. Steady
  /// clock: a settle window must not depend on /clock or sim time.
  std::chrono::steady_clock::time_point mask_rebuild_not_before_{};

  /// True when areas_ changed but the CLASSIFICATION layer was not re-stamped
  /// yet. add_area used to rasterise the WHOLE area list inside the service
  /// callback (twice for the new area), so a replace of N areas cost N²
  /// polygon fills while the caller waited; the layer has exactly two readers
  /// (the keepout mask and the save_map dump), so it is rebuilt lazily, once,
  /// by whichever of them runs first. Guarded by map_mutex_.
  bool classification_dirty_{false};

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
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::DigEvent>::SharedPtr dig_event_sub_;

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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_dock_antenna_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_areas_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_areas_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetRecoveryPoint>::SharedPtr get_recovery_point_srv_;
  rclcpp::Service<mowgli_interfaces::srv::PromoteObstacle>::SharedPtr promote_obstacle_srv_;
  rclcpp::Service<mowgli_interfaces::srv::ClearObstacle>::SharedPtr discard_obstacle_srv_;

  // ── TF ────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // Destruction-safe listener (safe_transform_listener.hpp): tf2_ros's own
  // dedicated-thread listener can hang its destructor.
  std::unique_ptr<SafeTransformListener> tf_listener_;

  // ── Timers ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__MAP_SERVER_NODE_HPP_
