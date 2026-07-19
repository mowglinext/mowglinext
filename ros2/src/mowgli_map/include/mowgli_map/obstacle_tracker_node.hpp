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

#ifndef MOWGLI_MAP__OBSTACLE_TRACKER_NODE_HPP_
#define MOWGLI_MAP__OBSTACLE_TRACKER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <mowgli_interfaces/msg/obstacle_array.hpp>
#include <mowgli_interfaces/srv/clear_obstacle.hpp>
#include <mowgli_interfaces/srv/get_mowing_area.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ObstacleTrackerAlgorithmTest;

namespace mowgli_map
{

/// @brief LiDAR-based obstacle tracker with transient/persistent classification.
///
/// Subscribes to /scan (fast early detection) and /map (OccupancyGrid
/// published by our map_server_node from the mowing-area boundary).  For the
/// scan path each valid range is transformed to the map frame via TF.  For
/// the map path occupied cells are extracted directly in map frame, filtered
/// to keep only cells that are at least map_obstacle_min_dist_from_boundary
/// metres inside the map's occupied boundary region (to avoid treating the
/// boundary edges themselves as obstacles), then clustered with DBSCAN.
/// Both paths feed the same associate_clusters() pipeline.
///
/// Outputs:
///   - obstacle_tracker/obstacles   (mowgli_interfaces/msg/ObstacleArray)
///   - obstacle_tracker/markers     (visualization_msgs/msg/MarkerArray)
///
/// Services:
///   - obstacle_tracker/clear_obstacle  (mowgli_interfaces/srv/ClearObstacle)
///   - obstacle_tracker/clear_all       (std_srvs/srv/Trigger)
///   - obstacle_tracker/save            (std_srvs/srv/Trigger)
///   - obstacle_tracker/load            (std_srvs/srv/Trigger)
class ObstacleTrackerNode : public rclcpp::Node
{
  friend class ::ObstacleTrackerAlgorithmTest;

public:
  /// @brief Construct the node, declare parameters, wire publishers/subscribers/services.
  explicit ObstacleTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  ~ObstacleTrackerNode() override = default;

  // Non-copyable, non-movable (ROS nodes are singletons in practice)
  ObstacleTrackerNode(const ObstacleTrackerNode&) = delete;
  ObstacleTrackerNode& operator=(const ObstacleTrackerNode&) = delete;
  ObstacleTrackerNode(ObstacleTrackerNode&&) = delete;
  ObstacleTrackerNode& operator=(ObstacleTrackerNode&&) = delete;

private:
  // ── Internal state ────────────────────────────────────────────────────────

  /// Runtime representation of a tracked cluster.  Hull and centroid are
  /// maintained in the map frame.  Once promoted, persistent=true.
  struct TrackedObstacle
  {
    uint32_t id;
    std::vector<std::pair<double, double>> hull_points;  ///< Boundary hull vertices (map frame)
    double cx{0.0};  ///< Centroid X (map frame)
    double cy{0.0};  ///< Centroid Y (map frame)
    double radius{0.0};  ///< Bounding-circle radius
    rclcpp::Time first_seen;
    rclcpp::Time last_seen;
    uint32_t observation_count{0};
    bool persistent{false};  ///< Promoted after persistence_threshold seconds
  };

  // ── ROS callbacks ─────────────────────────────────────────────────────────

  /// Seed the cached full costmap from a full snapshot, then re-cluster.
  /// The global costmap runs with always_send_full_costmap:false, so this
  /// full grid on /global_costmap/costmap only arrives on first connect (and
  /// on a costmap resize) — subsequent obstacle changes come via
  /// on_costmap_update.
  void on_costmap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /// Apply an incremental costmap update (delta mode) to the cached full grid,
  /// then re-cluster. Without this the tracker would see the costmap exactly
  /// once at startup and no obstacle would ever accumulate the ~observations
  /// needed to promote to PERSISTENT (the promotion coverage metric assumes a
  /// ~publish_rate cadence of observations).
  void on_costmap_update(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr msg);

  /// Flood-fill the cached costmap into clusters and feed associate_clusters().
  /// Shared by the full-snapshot (on_costmap) and delta (on_costmap_update)
  /// paths. Copies the cached grid under costmap_mutex_ so the flood-fill runs
  /// lock-free.
  void process_costmap();

  /// Extract occupied cells from the OccupancyGrid (published by
  /// map_server_node), filter by boundary distance, cluster with DBSCAN,
  /// and associate clusters.  Primary source.
  void on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /// Promote/expire obstacles, then publish ObstacleArray and MarkerArray.
  void on_publish_timer();

  // ── Services ──────────────────────────────────────────────────────────────

  void on_clear_obstacle(mowgli_interfaces::srv::ClearObstacle::Request::SharedPtr req,
                         mowgli_interfaces::srv::ClearObstacle::Response::SharedPtr res);

  void on_clear_all(std_srvs::srv::Trigger::Request::SharedPtr req,
                    std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_save(std_srvs::srv::Trigger::Request::SharedPtr req,
               std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_load(std_srvs::srv::Trigger::Request::SharedPtr req,
               std_srvs::srv::Trigger::Response::SharedPtr res);

  // ── Algorithms ────────────────────────────────────────────────────────────

  /// DBSCAN clustering on a flat point list.
  /// @param points  Input 2-D points.
  /// @param eps     Neighbourhood radius (metres).
  /// @param min_pts Minimum cluster size.
  /// @return        Vector of clusters; each cluster is a vector of points.
  std::vector<std::vector<std::pair<double, double>>> dbscan(
      const std::vector<std::pair<double, double>>& points, double eps, int min_pts) const;

  /// Andrew's monotone-chain convex hull (returns hull in CCW order).
  /// Degenerate inputs (< 3 points) are returned as-is.
  std::vector<std::pair<double, double>> convex_hull(
      const std::vector<std::pair<double, double>>& points) const;

  /// Boundary-preserving hull via angular sweep from centroid.
  /// Keeps the farthest point per angular bin so concave shapes (L, U)
  /// are not inflated to their convex hull.  Falls back to convex_hull
  /// for very small clusters (< 4 points).
  std::vector<std::pair<double, double>> boundary_hull(
      const std::vector<std::pair<double, double>>& pts) const;

  /// Match each new cluster to an existing obstacle or create a new entry.
  /// Caller must hold mutex_.
  void associate_clusters(const std::vector<std::vector<std::pair<double, double>>>& clusters,
                          const rclcpp::Time& stamp);

  /// Merge overlapping tracked obstacles into single shapes.
  /// Two obstacles merge when their centroids are closer than the sum of their radii.
  /// Caller must hold mutex_.
  void merge_overlapping();

  /// Promote obstacles meeting persistence_threshold; remove stale transients.
  /// Caller must hold mutex_.
  void promote_persistent(const rclcpp::Time& now);

  /// Inflate a convex hull outward by inflation_radius_ metres.
  std::vector<std::pair<double, double>> inflate_hull(
      const std::vector<std::pair<double, double>>& hull, double radius) const;

  /// Point-in-polygon test (ray casting algorithm).
  /// Returns true if point (px, py) is inside the polygon.
  bool point_in_polygon(double px,
                        double py,
                        const std::vector<std::pair<double, double>>& polygon) const;

  /// Fetch mowing area boundary from map_server_node via GetMowingArea service.
  void fetch_boundary();

  // ── Persistence helpers ───────────────────────────────────────────────────

  void save_to_file() const;
  void load_from_file();

  // ── Parameters ────────────────────────────────────────────────────────────
  double cluster_tolerance_{0.15};  ///< DBSCAN epsilon (m)
  int min_cluster_points_{3};  ///< DBSCAN min_pts
  double persistence_threshold_{30.0};  ///< Seconds until promoted to PERSISTENT
  double transient_timeout_{5.0};  ///< Seconds until stale transient removed
  double min_obstacle_radius_{0.05};  ///< Minimum cluster bounding radius (m)
  double max_obstacle_radius_{5.0};  ///< Maximum cluster bounding radius (m)
  double inflation_radius_{0.15};  ///< Polygon inflation for output (m)
  double publish_rate_{1.0};  ///< Timer frequency (Hz)
  std::string persistence_file_;  ///< Path for YAML save/load
  std::string map_frame_{"map"};
  std::string map_topic_{"/map"};  ///< OccupancyGrid topic (from map_server_node)
  int occupied_threshold_{65};  ///< Cells >= this value are treated as occupied
  double map_obstacle_min_dist_from_boundary_{0.2};  ///< Min distance from boundary edge (m)
  double boundary_margin_{0.3};  ///< Reject clusters within this margin of boundary edge (m)

  // ── State ─────────────────────────────────────────────────────────────────
  std::vector<TrackedObstacle> tracked_;  ///< All currently tracked obstacles
  uint32_t next_id_{1};  ///< Monotonically increasing ID counter
  mutable std::mutex mutex_;  ///< Guards tracked_ and next_id_

  /// Mowing area boundary polygon (map frame). Empty until fetched.
  std::vector<std::pair<double, double>> boundary_polygon_;
  /// Inset boundary polygon (shrunk by boundary_margin_) for filtering.
  std::vector<std::pair<double, double>> boundary_inset_;
  bool boundary_loaded_{false};

  // ── TF ────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<mowgli_interfaces::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ── Subscribers ───────────────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  /// Cached full global costmap. Seeded by on_costmap (full snapshot) and
  /// patched in place by on_costmap_update (delta). Guarded by costmap_mutex_.
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool have_costmap_{false};
  mutable std::mutex costmap_mutex_;

  // ── Services ──────────────────────────────────────────────────────────────
  rclcpp::Service<mowgli_interfaces::srv::ClearObstacle>::SharedPtr clear_obstacle_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_all_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_srv_;

  // ── Boundary service client ─────────────────────────────────────────────
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr boundary_client_;
  rclcpp::TimerBase::SharedPtr boundary_fetch_timer_;

  // ── Timers ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__OBSTACLE_TRACKER_NODE_HPP_
