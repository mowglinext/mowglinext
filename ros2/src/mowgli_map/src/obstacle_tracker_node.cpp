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

#include "mowgli_map/obstacle_tracker_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mowgli_interfaces/srv/get_mowing_area.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace mowgli_map
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

ObstacleTrackerNode::ObstacleTrackerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("obstacle_tracker", options)
{
  // ── Declare and read parameters ──────────────────────────────────────────
  cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.15);
  min_cluster_points_ = declare_parameter<int>("min_cluster_points", 3);
  persistence_threshold_ = declare_parameter<double>("persistence_threshold", 30.0);
  transient_timeout_ = declare_parameter<double>("transient_timeout", 5.0);
  min_obstacle_radius_ = declare_parameter<double>("min_obstacle_radius", 0.05);
  max_obstacle_radius_ = declare_parameter<double>("max_obstacle_radius", 5.0);
  inflation_radius_ = declare_parameter<double>("inflation_radius", 0.15);
  persistence_file_ = declare_parameter<std::string>("persistence_file", "");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
  map_obstacle_min_dist_from_boundary_ =
      declare_parameter<double>("map_obstacle_min_dist_from_boundary", 0.5);
  boundary_margin_ = declare_parameter<double>("boundary_margin", 0.3);

  RCLCPP_INFO(get_logger(),
              "ObstacleTrackerNode: cluster_tolerance=%.3f m, min_pts=%d, "
              "persistence=%.1f s, transient_timeout=%.1f s, frame='%s', "
              "map_topic='%s', occupied_threshold=%d, boundary_margin=%.2f m",
              cluster_tolerance_,
              min_cluster_points_,
              persistence_threshold_,
              transient_timeout_,
              map_frame_.c_str(),
              map_topic_.c_str(),
              occupied_threshold_,
              map_obstacle_min_dist_from_boundary_);

  // ── TF ───────────────────────────────────────────────────────────────────
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Publishers ───────────────────────────────────────────────────────────
  obstacle_pub_ =
      create_publisher<mowgli_interfaces::msg::ObstacleArray>("obstacle_tracker/obstacles",
                                                              rclcpp::QoS(1));

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_tracker/markers",
                                                                       rclcpp::QoS(1));

  // ── Subscribers ──────────────────────────────────────────────────────────
  // Use global costmap obstacle layer instead of raw scan — much cheaper
  // than DBSCAN on every scan, and gives cleaner obstacle shapes.
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap",
      rclcpp::QoS(1),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
      {
        on_costmap(std::move(msg));
      });

  // The global costmap runs in delta mode (always_send_full_costmap:false),
  // so the full grid above is latched once at connect and every subsequent
  // change arrives here as an OccupancyGridUpdate. Subscribing to both keeps
  // the tracker's observation cadence at ~publish_frequency — required for
  // promotion to ever fire.
  costmap_update_sub_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      "/global_costmap/costmap_updates",
      rclcpp::QoS(1),
      [this](map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr msg)
      {
        on_costmap_update(std::move(msg));
      });

  // map_server_node publishes with a transient-local QoS so late
  // subscribers receive the last map immediately on connect.
  {
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_,
        map_qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
        {
          on_map(std::move(msg));
        });
  }

  // ── Services ─────────────────────────────────────────────────────────────
  clear_obstacle_srv_ = create_service<mowgli_interfaces::srv::ClearObstacle>(
      "obstacle_tracker/clear_obstacle",
      [this](mowgli_interfaces::srv::ClearObstacle::Request::SharedPtr req,
             mowgli_interfaces::srv::ClearObstacle::Response::SharedPtr res)
      {
        on_clear_obstacle(std::move(req), std::move(res));
      });

  clear_all_srv_ =
      create_service<std_srvs::srv::Trigger>("obstacle_tracker/clear_all",
                                             [this](std_srvs::srv::Trigger::Request::SharedPtr req,
                                                    std_srvs::srv::Trigger::Response::SharedPtr res)
                                             {
                                               on_clear_all(std::move(req), std::move(res));
                                             });

  save_srv_ =
      create_service<std_srvs::srv::Trigger>("obstacle_tracker/save",
                                             [this](std_srvs::srv::Trigger::Request::SharedPtr req,
                                                    std_srvs::srv::Trigger::Response::SharedPtr res)
                                             {
                                               on_save(std::move(req), std::move(res));
                                             });

  load_srv_ =
      create_service<std_srvs::srv::Trigger>("obstacle_tracker/load",
                                             [this](std_srvs::srv::Trigger::Request::SharedPtr req,
                                                    std_srvs::srv::Trigger::Response::SharedPtr res)
                                             {
                                               on_load(std::move(req), std::move(res));
                                             });

  // ── Publish timer ────────────────────────────────────────────────────────
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));

  publish_timer_ = create_wall_timer(period_ns,
                                     [this]()
                                     {
                                       on_publish_timer();
                                     });

  // ── Boundary service client ─────────────────────────────────────────────
  boundary_client_ =
      create_client<mowgli_interfaces::srv::GetMowingArea>("/map_server_node/get_mowing_area");

  // Retry fetching the boundary every 5 seconds until map_server is ready.
  boundary_fetch_timer_ = create_wall_timer(std::chrono::seconds(5),
                                            [this]()
                                            {
                                              fetch_boundary();
                                            });

  // ── Load persisted obstacles on startup ──────────────────────────────────
  if (!persistence_file_.empty())
  {
    load_from_file();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Scan callback
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::on_costmap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    latest_costmap_ = *msg;  // Full snapshot replaces the cache.
    have_costmap_ = true;
  }
  process_costmap();
}

void ObstacleTrackerNode::on_costmap_update(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!have_costmap_)
      return;  // No base grid yet — wait for the first full snapshot.

    const int gw = static_cast<int>(latest_costmap_.info.width);
    const int gh = static_cast<int>(latest_costmap_.info.height);
    const int ux = msg->x;
    const int uy = msg->y;
    const int uw = static_cast<int>(msg->width);
    const int uh = static_cast<int>(msg->height);

    // Reject an update that doesn't fit the cached grid. A costmap resize
    // always republishes the full grid on /global_costmap/costmap, so it is
    // safe to drop the delta and wait for that next full snapshot.
    if (ux < 0 || uy < 0 || uw <= 0 || uh <= 0 || ux + uw > gw || uy + uh > gh ||
        static_cast<int>(msg->data.size()) != uw * uh)
      return;

    for (int row = 0; row < uh; ++row)
    {
      const int dst_off = (uy + row) * gw + ux;
      const int src_off = row * uw;
      for (int col = 0; col < uw; ++col)
        latest_costmap_.data[dst_off + col] = msg->data[src_off + col];
    }
    // Keep the cached grid's stamp fresh so obstacle aging/promotion in
    // associate_clusters()/promote_persistent() advances with each delta.
    latest_costmap_.header.stamp = msg->header.stamp;
  }
  process_costmap();
}

void ObstacleTrackerNode::process_costmap()
{
  nav_msgs::msg::OccupancyGrid msg;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!have_costmap_)
      return;
    msg = latest_costmap_;  // Copy so the flood-fill runs without the lock.
  }

  const int w = static_cast<int>(msg.info.width);
  const int h = static_cast<int>(msg.info.height);
  const double res = msg.info.resolution;
  const double ox = msg.info.origin.position.x;
  const double oy = msg.info.origin.position.y;

  if (w == 0 || h == 0)
    return;

  // Mark cells that are unsafe for the robot (cost >= 50 captures the
  // inflation zone, not just lethal cells). This gives obstacle polygons
  // that match the full no-go area including safety margin.
  constexpr int8_t OBSTACLE_COST = 50;
  std::vector<bool> visited(w * h, false);
  std::vector<std::vector<std::pair<double, double>>> clusters;

  // Flood-fill to find connected components of obstacle cells
  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      const int idx = y * w + x;
      if (visited[idx] || msg.data[idx] < OBSTACLE_COST)
        continue;

      // BFS flood-fill
      std::vector<std::pair<double, double>> cluster;
      std::vector<std::pair<int, int>> queue;
      queue.emplace_back(x, y);
      visited[idx] = true;

      while (!queue.empty())
      {
        auto [cx, cy] = queue.back();
        queue.pop_back();

        // Convert cell to map frame
        double mx = ox + (cx + 0.5) * res;
        double my = oy + (cy + 0.5) * res;
        cluster.emplace_back(mx, my);

        // 4-connected neighbors
        for (auto [dx, dy] : std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
        {
          int nx = cx + dx, ny = cy + dy;
          if (nx >= 0 && nx < w && ny >= 0 && ny < h)
          {
            int nidx = ny * w + nx;
            if (!visited[nidx] && msg.data[nidx] >= OBSTACLE_COST)
            {
              visited[nidx] = true;
              queue.emplace_back(nx, ny);
            }
          }
        }
      }

      if (static_cast<int>(cluster.size()) >= min_cluster_points_)
      {
        clusters.push_back(std::move(cluster));
      }
    }
  }

  const rclcpp::Time stamp(msg.header.stamp);
  std::lock_guard<std::mutex> lock(mutex_);
  associate_clusters(clusters, stamp);
}

// ─────────────────────────────────────────────────────────────────────────────
// Map callback (primary source — full SLAM occupancy grid)
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  const uint32_t width = msg->info.width;
  const uint32_t height = msg->info.height;
  const float res = msg->info.resolution;  // metres per cell

  if (width == 0 || height == 0 || res <= 0.0F)
  {
    return;
  }

  // Minimum number of cells that correspond to the boundary margin.
  // A cell is accepted only when every neighbour within this radius is also
  // occupied, which means the cell is deep enough inside a solid region to
  // not be a boundary edge cell.
  const int margin_cells =
      static_cast<int>(std::ceil(map_obstacle_min_dist_from_boundary_ / static_cast<double>(res)));

  // Origin of the grid in the map frame.
  const double origin_x = msg->info.origin.position.x;
  const double origin_y = msg->info.origin.position.y;

  // Build a fast lookup: is cell (row, col) occupied?
  // We use a flat boolean shadow to avoid recomputing the threshold each time.
  const int n_cells = static_cast<int>(width * height);
  std::vector<bool> occupied(n_cells, false);
  for (int i = 0; i < n_cells; ++i)
  {
    // Promote to int before comparison: int8_t values are -1 (unknown), 0 (free),
    // and 1–100 (occupied probability).  occupied_threshold_ is declared int so
    // comparing directly avoids any sign-extension surprises.
    const int val = static_cast<int>(msg->data[static_cast<size_t>(i)]);
    if (val >= occupied_threshold_)
    {
      occupied[static_cast<size_t>(i)] = true;
    }
  }

  // Collect map-frame points for cells that are:
  //   1. Occupied (value >= occupied_threshold_)
  //   2. At least margin_cells away from any free/unknown cell in a square
  //      neighbourhood (approximation of distance-from-boundary).
  std::vector<std::pair<double, double>> map_points;
  // Reserve conservatively — occupied cells are typically a small fraction.
  map_points.reserve(512);

  for (uint32_t row = 0; row < height; ++row)
  {
    for (uint32_t col = 0; col < width; ++col)
    {
      const size_t idx = static_cast<size_t>(row * width + col);
      if (!occupied[idx])
      {
        continue;
      }

      // Check that all cells within margin_cells form a solid occupied block.
      // We scan the bounding square; any non-occupied cell disqualifies this
      // cell as a boundary-interior point.
      if (margin_cells > 0)
      {
        bool interior = true;
        const int r_min = static_cast<int>(row) - margin_cells;
        const int r_max = static_cast<int>(row) + margin_cells;
        const int c_min = static_cast<int>(col) - margin_cells;
        const int c_max = static_cast<int>(col) + margin_cells;

        for (int nr = r_min; nr <= r_max && interior; ++nr)
        {
          if (nr < 0 || nr >= static_cast<int>(height))
          {
            interior = false;
            break;
          }
          for (int nc = c_min; nc <= c_max && interior; ++nc)
          {
            if (nc < 0 || nc >= static_cast<int>(width))
            {
              interior = false;
              break;
            }
            if (!occupied[static_cast<size_t>(nr * static_cast<int>(width) + nc)])
            {
              interior = false;
            }
          }
        }

        if (!interior)
        {
          continue;
        }
      }

      // Convert cell centre to map frame.
      const double mx = origin_x + (static_cast<double>(col) + 0.5) * static_cast<double>(res);
      const double my = origin_y + (static_cast<double>(row) + 0.5) * static_cast<double>(res);
      map_points.emplace_back(mx, my);
    }
  }

  if (map_points.empty())
  {
    return;
  }

  RCLCPP_DEBUG(get_logger(),
               "on_map: %zu interior occupied cells from %ux%u grid (margin=%d cells)",
               map_points.size(),
               width,
               height,
               margin_cells);

  const rclcpp::Time map_stamp(msg->header.stamp);

  // Use a larger cluster tolerance for map data because cells are discrete
  // and the resolution may leave gaps inside solid objects.  Two adjacent
  // cells at typical 5 cm resolution are sqrt(2)*0.05 ≈ 0.07 m apart; one
  // cell of separation is 0.10 m.  Use the configured cluster_tolerance
  // directly — the user can tune it.
  const auto clusters = dbscan(map_points, cluster_tolerance_, min_cluster_points_);

  std::lock_guard<std::mutex> lock(mutex_);
  associate_clusters(clusters, map_stamp);
}

// ─────────────────────────────────────────────────────────────────────────────
// Publish timer
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::on_publish_timer()
{
  const rclcpp::Time now_stamp = now();

  std::lock_guard<std::mutex> lock(mutex_);
  promote_persistent(now_stamp);
  merge_overlapping();

  // ── Build ObstacleArray (persistent obstacles only) ───────────────────────
  mowgli_interfaces::msg::ObstacleArray obstacle_array;
  obstacle_array.header.stamp = now_stamp;
  obstacle_array.header.frame_id = map_frame_;

  // ── Build MarkerArray (all obstacles for debugging) ───────────────────────
  visualization_msgs::msg::MarkerArray marker_array;

  // Delete-all marker to clear stale entries.
  {
    visualization_msgs::msg::Marker del;
    del.header.stamp = now_stamp;
    del.header.frame_id = map_frame_;
    del.ns = "obstacles";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(del);
  }

  int32_t marker_id = 0;

  for (const auto& obs : tracked_)
  {
    // Publish all obstacles in the ObstacleArray regardless of persistence.
    mowgli_interfaces::msg::TrackedObstacle out_obs;
    out_obs.id = obs.id;
    out_obs.centroid.x = obs.cx;
    out_obs.centroid.y = obs.cy;
    out_obs.centroid.z = 0.0;
    out_obs.radius = obs.radius;
    out_obs.first_seen = rclcpp::Time(obs.first_seen).operator builtin_interfaces::msg::Time();
    out_obs.observation_count = obs.observation_count;
    out_obs.status = obs.persistent ? mowgli_interfaces::msg::TrackedObstacle::PERSISTENT
                                    : mowgli_interfaces::msg::TrackedObstacle::TRANSIENT;

    // Inflate the hull for the output polygon.
    const auto inflated = inflate_hull(obs.hull_points, inflation_radius_);
    for (const auto& [hx, hy] : inflated)
    {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(hx);
      p.y = static_cast<float>(hy);
      p.z = 0.0F;
      out_obs.polygon.points.push_back(p);
    }

    obstacle_array.obstacles.push_back(out_obs);

    // ── Marker: LINE_STRIP for the hull ──────────────────────────────────
    visualization_msgs::msg::Marker hull_marker;
    hull_marker.header.stamp = now_stamp;
    hull_marker.header.frame_id = map_frame_;
    hull_marker.ns = "obstacles";
    hull_marker.id = marker_id++;
    hull_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    hull_marker.action = visualization_msgs::msg::Marker::ADD;
    hull_marker.scale.x = 0.03;
    hull_marker.pose.orientation.w = 1.0;

    // Green = transient, red = persistent.
    if (obs.persistent)
    {
      hull_marker.color.r = 1.0F;
      hull_marker.color.g = 0.0F;
    }
    else
    {
      hull_marker.color.r = 0.0F;
      hull_marker.color.g = 1.0F;
    }
    hull_marker.color.b = 0.0F;
    hull_marker.color.a = 0.85F;

    for (const auto& [hx, hy] : inflated)
    {
      geometry_msgs::msg::Point p;
      p.x = hx;
      p.y = hy;
      p.z = 0.05;
      hull_marker.points.push_back(p);
    }
    // Close the loop.
    if (!inflated.empty())
    {
      geometry_msgs::msg::Point p;
      p.x = inflated.front().first;
      p.y = inflated.front().second;
      p.z = 0.05;
      hull_marker.points.push_back(p);
    }
    marker_array.markers.push_back(hull_marker);

    // ── Marker: TEXT showing ID and observation count ─────────────────────
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = hull_marker.header;
    text_marker.ns = "obstacle_labels";
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = obs.cx;
    text_marker.pose.position.y = obs.cy;
    text_marker.pose.position.z = 0.3;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.15;
    text_marker.color.r = 1.0F;
    text_marker.color.g = 1.0F;
    text_marker.color.b = 1.0F;
    text_marker.color.a = 1.0F;
    text_marker.text = std::string(obs.persistent ? "P" : "T") + "#" + std::to_string(obs.id) +
                       " n=" + std::to_string(obs.observation_count);
    marker_array.markers.push_back(text_marker);
  }

  obstacle_pub_->publish(obstacle_array);
  marker_pub_->publish(marker_array);

  RCLCPP_DEBUG(get_logger(),
               "Published %zu obstacles (%zu markers)",
               obstacle_array.obstacles.size(),
               marker_array.markers.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// Services
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::on_clear_obstacle(
    mowgli_interfaces::srv::ClearObstacle::Request::SharedPtr req,
    mowgli_interfaces::srv::ClearObstacle::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto before = tracked_.size();
  tracked_.erase(std::remove_if(tracked_.begin(),
                                tracked_.end(),
                                [&](const TrackedObstacle& o)
                                {
                                  return o.id == req->obstacle_id;
                                }),
                 tracked_.end());
  const bool removed = tracked_.size() < before;
  res->success = removed;
  res->message = removed ? "Obstacle " + std::to_string(req->obstacle_id) + " removed."
                         : "Obstacle " + std::to_string(req->obstacle_id) + " not found.";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void ObstacleTrackerNode::on_clear_all(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                       std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto count = tracked_.size();
  tracked_.clear();
  next_id_ = 1;
  res->success = true;
  res->message = "Cleared " + std::to_string(count) + " obstacles.";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void ObstacleTrackerNode::on_save(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (persistence_file_.empty())
  {
    res->success = false;
    res->message = "persistence_file parameter is not set.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }
  try
  {
    save_to_file();
    res->success = true;
    res->message = "Saved obstacles to " + persistence_file_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Save failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void ObstacleTrackerNode::on_load(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (persistence_file_.empty())
  {
    res->success = false;
    res->message = "persistence_file parameter is not set.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }
  try
  {
    load_from_file();
    res->success = true;
    res->message = "Loaded obstacles from " + persistence_file_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Load failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Boundary fetching
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::fetch_boundary()
{
  if (boundary_loaded_)
  {
    boundary_fetch_timer_->cancel();
    return;
  }

  if (!boundary_client_->service_is_ready())
  {
    RCLCPP_INFO_THROTTLE(get_logger(),
                         *get_clock(),
                         10000,
                         "Waiting for /map_server_node/get_mowing_area service...");
    return;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  request->index = 0;

  auto future = boundary_client_->async_send_request(
      request,
      [this](rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedFuture result)
      {
        auto response = result.get();
        if (!response->success || response->area.area.points.empty())
        {
          RCLCPP_WARN(get_logger(), "GetMowingArea returned no boundary — will retry.");
          return;
        }

        // Convert polygon to vector of (x, y) pairs.
        boundary_polygon_.clear();
        for (const auto& pt : response->area.area.points)
        {
          boundary_polygon_.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y));
        }

        // Compute inset polygon (shrink by boundary_margin_).
        // Simple approach: move each vertex toward the centroid by boundary_margin_.
        double cx = 0.0, cy = 0.0;
        for (const auto& [x, y] : boundary_polygon_)
        {
          cx += x;
          cy += y;
        }
        cx /= static_cast<double>(boundary_polygon_.size());
        cy /= static_cast<double>(boundary_polygon_.size());

        boundary_inset_.clear();
        for (const auto& [x, y] : boundary_polygon_)
        {
          const double dx = x - cx;
          const double dy = y - cy;
          const double dist = std::hypot(dx, dy);
          if (dist > 1e-6)
          {
            const double shrink = std::min(boundary_margin_ / dist, 0.5);
            boundary_inset_.emplace_back(x - dx * shrink, y - dy * shrink);
          }
          else
          {
            boundary_inset_.emplace_back(x, y);
          }
        }

        boundary_loaded_ = true;
        boundary_fetch_timer_->cancel();

        RCLCPP_INFO(get_logger(),
                    "Loaded mowing area boundary: %zu vertices, margin=%.2f m. "
                    "Obstacles outside boundary will be rejected.",
                    boundary_polygon_.size(),
                    boundary_margin_);

        // Clear any existing obstacles that are outside the boundary
        // (accumulated before boundary was loaded).
        std::lock_guard<std::mutex> lock(mutex_);
        const auto before = tracked_.size();
        tracked_.erase(std::remove_if(tracked_.begin(),
                                      tracked_.end(),
                                      [this](const TrackedObstacle& obs)
                                      {
                                        return !point_in_polygon(obs.cx, obs.cy, boundary_inset_);
                                      }),
                       tracked_.end());
        if (tracked_.size() < before)
        {
          RCLCPP_INFO(get_logger(),
                      "Purged %zu obstacles outside mowing area boundary.",
                      before - tracked_.size());
        }
      });
}

// ─────────────────────────────────────────────────────────────────────────────
// Point-in-polygon (ray casting)
// ─────────────────────────────────────────────────────────────────────────────

bool ObstacleTrackerNode::point_in_polygon(
    double px, double py, const std::vector<std::pair<double, double>>& polygon) const
{
  if (polygon.size() < 3)
  {
    return false;
  }

  bool inside = false;
  const size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double xi = polygon[i].first, yi = polygon[i].second;
    const double xj = polygon[j].first, yj = polygon[j].second;

    if (((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

// ─────────────────────────────────────────────────────────────────────────────
// DBSCAN
// ─────────────────────────────────────────────────────────────────────────────

std::vector<std::vector<std::pair<double, double>>> ObstacleTrackerNode::dbscan(
    const std::vector<std::pair<double, double>>& points, double eps, int min_pts) const
{
  const int n = static_cast<int>(points.size());
  constexpr int UNVISITED = -1;
  constexpr int NOISE = -2;

  std::vector<int> labels(n, UNVISITED);
  int cluster_id = 0;

  const double eps2 = eps * eps;

  auto neighbours_of = [&](int idx)
  {
    std::vector<int> nb;
    nb.reserve(16);
    const double px = points[idx].first;
    const double py = points[idx].second;
    for (int j = 0; j < n; ++j)
    {
      const double dx = points[j].first - px;
      const double dy = points[j].second - py;
      if (dx * dx + dy * dy <= eps2)
      {
        nb.push_back(j);
      }
    }
    return nb;
  };

  for (int i = 0; i < n; ++i)
  {
    if (labels[i] != UNVISITED)
    {
      continue;
    }

    auto nb = neighbours_of(i);
    if (static_cast<int>(nb.size()) < min_pts)
    {
      labels[i] = NOISE;
      continue;
    }

    labels[i] = cluster_id;

    // Expand cluster — iterate by index to allow growth during expansion.
    for (size_t k = 0; k < nb.size(); ++k)
    {
      const int q = nb[k];
      if (labels[q] == NOISE)
      {
        labels[q] = cluster_id;
        continue;
      }
      if (labels[q] != UNVISITED)
      {
        continue;
      }
      labels[q] = cluster_id;

      auto q_nb = neighbours_of(q);
      if (static_cast<int>(q_nb.size()) >= min_pts)
      {
        for (int qn : q_nb)
        {
          nb.push_back(qn);
        }
      }
    }

    ++cluster_id;
  }

  // Collect clusters.
  std::vector<std::vector<std::pair<double, double>>> result(cluster_id);
  for (int i = 0; i < n; ++i)
  {
    if (labels[i] >= 0)
    {
      result[labels[i]].push_back(points[i]);
    }
  }
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Convex hull (Andrew's monotone chain)
// ─────────────────────────────────────────────────────────────────────────────

std::vector<std::pair<double, double>> ObstacleTrackerNode::convex_hull(
    const std::vector<std::pair<double, double>>& pts) const
{
  if (pts.size() < 3)
  {
    return pts;
  }

  auto sorted = pts;
  std::sort(sorted.begin(), sorted.end());
  sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());

  if (sorted.size() < 3)
  {
    return sorted;
  }

  const int n = static_cast<int>(sorted.size());

  // Cross product of vectors OA and OB.
  auto cross = [](const std::pair<double, double>& O,
                  const std::pair<double, double>& A,
                  const std::pair<double, double>& B) -> double
  {
    return (A.first - O.first) * (B.second - O.second) -
           (A.second - O.second) * (B.first - O.first);
  };

  std::vector<std::pair<double, double>> hull;
  hull.reserve(2 * n);

  // Lower hull.
  for (int i = 0; i < n; ++i)
  {
    while (hull.size() >= 2 &&
           cross(hull[hull.size() - 2], hull[hull.size() - 1], sorted[i]) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(sorted[i]);
  }

  // Upper hull.
  const int lower_size = static_cast<int>(hull.size()) + 1;
  for (int i = n - 2; i >= 0; --i)
  {
    while (static_cast<int>(hull.size()) >= lower_size &&
           cross(hull[hull.size() - 2], hull[hull.size() - 1], sorted[i]) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(sorted[i]);
  }

  hull.pop_back();  // Last point equals first.
  return hull;
}

// ─────────────────────────────────────────────────────────────────────────────
// Boundary-preserving hull (angular sweep from centroid)
// ─────────────────────────────────────────────────────────────────────────────

std::vector<std::pair<double, double>> ObstacleTrackerNode::boundary_hull(
    const std::vector<std::pair<double, double>>& pts) const
{
  if (pts.size() < 4)
  {
    return convex_hull(pts);
  }

  // Compute centroid.
  double cx = 0.0;
  double cy = 0.0;
  for (const auto& [x, y] : pts)
  {
    cx += x;
    cy += y;
  }
  cx /= static_cast<double>(pts.size());
  cy /= static_cast<double>(pts.size());

  // Sort by angle from centroid.
  auto sorted = pts;
  std::sort(sorted.begin(),
            sorted.end(),
            [cx, cy](const auto& a, const auto& b)
            {
              return std::atan2(a.second - cy, a.first - cx) <
                     std::atan2(b.second - cy, b.first - cx);
            });

  // Remove interior points: keep only the farthest point per angular bin.
  std::vector<std::pair<double, double>> boundary;
  constexpr double angular_resolution = 0.1;  // ~6 degrees
  double last_angle = -999.0;

  for (const auto& [x, y] : sorted)
  {
    double angle = std::atan2(y - cy, x - cx);
    if (angle - last_angle < angular_resolution && !boundary.empty())
    {
      // Same angular bin — keep the one farther from centroid.
      auto& prev = boundary.back();
      double d_prev = std::hypot(prev.first - cx, prev.second - cy);
      double d_curr = std::hypot(x - cx, y - cy);
      if (d_curr > d_prev)
      {
        prev = {x, y};
      }
    }
    else
    {
      boundary.push_back({x, y});
      last_angle = angle;
    }
  }

  if (boundary.size() < 3)
  {
    return convex_hull(pts);
  }

  return boundary;
}

// ─────────────────────────────────────────────────────────────────────────────
// Hull inflation (push each vertex outward from centroid)
// ─────────────────────────────────────────────────────────────────────────────

std::vector<std::pair<double, double>> ObstacleTrackerNode::inflate_hull(
    const std::vector<std::pair<double, double>>& hull, double radius) const
{
  if (hull.empty())
  {
    return hull;
  }

  // Compute centroid of hull.
  double cx = 0.0;
  double cy = 0.0;
  for (const auto& [x, y] : hull)
  {
    cx += x;
    cy += y;
  }
  cx /= static_cast<double>(hull.size());
  cy /= static_cast<double>(hull.size());

  std::vector<std::pair<double, double>> inflated;
  inflated.reserve(hull.size());

  for (const auto& [x, y] : hull)
  {
    const double dx = x - cx;
    const double dy = y - cy;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-9)
    {
      inflated.emplace_back(x + radius, y);
    }
    else
    {
      inflated.emplace_back(x + dx / dist * radius, y + dy / dist * radius);
    }
  }
  return inflated;
}

// ─────────────────────────────────────────────────────────────────────────────
// Cluster association
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::associate_clusters(
    const std::vector<std::vector<std::pair<double, double>>>& clusters, const rclcpp::Time& stamp)
{
  constexpr double association_dist = 0.5;  // metres

  for (const auto& cluster : clusters)
  {
    if (cluster.empty())
    {
      continue;
    }

    // Compute cluster centroid and bounding radius.
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& [x, y] : cluster)
    {
      cx += x;
      cy += y;
    }
    cx /= static_cast<double>(cluster.size());
    cy /= static_cast<double>(cluster.size());

    double max_r = 0.0;
    for (const auto& [x, y] : cluster)
    {
      max_r = std::max(max_r, std::hypot(x - cx, y - cy));
    }

    // Filter by radius bounds.
    if (max_r < min_obstacle_radius_ || max_r > max_obstacle_radius_)
    {
      continue;
    }

    // Filter by mowing area boundary: reject clusters outside or near the edge.
    if (boundary_loaded_ && !point_in_polygon(cx, cy, boundary_inset_))
    {
      continue;
    }

    // Find nearest existing obstacle centroid.
    double nearest_dist = std::numeric_limits<double>::max();
    TrackedObstacle* nearest = nullptr;

    for (auto& obs : tracked_)
    {
      const double d = std::hypot(obs.cx - cx, obs.cy - cy);
      if (d < nearest_dist)
      {
        nearest_dist = d;
        nearest = &obs;
      }
    }

    if (nearest != nullptr && nearest_dist <= association_dist)
    {
      // Update existing obstacle with running-average centroid.
      const double alpha = 1.0 / (nearest->observation_count + 1.0);
      nearest->cx = nearest->cx * (1.0 - alpha) + cx * alpha;
      nearest->cy = nearest->cy * (1.0 - alpha) + cy * alpha;
      nearest->radius = std::max(nearest->radius, max_r);
      nearest->hull_points = boundary_hull(cluster);
      nearest->last_seen = stamp;
      nearest->observation_count++;
    }
    else
    {
      // Create new TrackedObstacle.
      TrackedObstacle obs;
      obs.id = next_id_++;
      obs.cx = cx;
      obs.cy = cy;
      obs.radius = max_r;
      obs.hull_points = boundary_hull(cluster);
      obs.first_seen = stamp;
      obs.last_seen = stamp;
      obs.observation_count = 1;
      obs.persistent = false;
      tracked_.push_back(std::move(obs));
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Merge overlapping obstacles
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::merge_overlapping()
{
  // Merge obstacles whose centroids are closer than the sum of their radii
  // (plus a small margin). The merged obstacle gets a new convex hull
  // encompassing both point sets.
  constexpr double merge_margin = 0.10;  // extra margin for merge decision
  bool merged_any = true;

  while (merged_any)
  {
    merged_any = false;
    for (size_t i = 0; i < tracked_.size(); ++i)
    {
      for (size_t j = i + 1; j < tracked_.size(); ++j)
      {
        const double dist =
            std::hypot(tracked_[i].cx - tracked_[j].cx, tracked_[i].cy - tracked_[j].cy);
        const double merge_dist = tracked_[i].radius + tracked_[j].radius + merge_margin;

        if (dist < merge_dist)
        {
          // Merge j into i: combine hull points, recompute centroid and hull.
          auto& a = tracked_[i];
          auto& b = tracked_[j];

          std::vector<std::pair<double, double>> combined = a.hull_points;
          combined.insert(combined.end(), b.hull_points.begin(), b.hull_points.end());
          a.hull_points = boundary_hull(combined);

          // Recompute centroid and radius from new hull.
          double cx = 0.0;
          double cy = 0.0;
          for (const auto& [x, y] : a.hull_points)
          {
            cx += x;
            cy += y;
          }
          cx /= static_cast<double>(a.hull_points.size());
          cy /= static_cast<double>(a.hull_points.size());
          a.cx = cx;
          a.cy = cy;

          double max_r = 0.0;
          for (const auto& [x, y] : a.hull_points)
          {
            max_r = std::max(max_r, std::hypot(x - cx, y - cy));
          }
          a.radius = max_r;

          // Keep the more mature obstacle's metadata.
          a.observation_count += b.observation_count;
          if (b.first_seen < a.first_seen)
          {
            a.first_seen = b.first_seen;
          }
          if (b.last_seen > a.last_seen)
          {
            a.last_seen = b.last_seen;
          }
          a.persistent = a.persistent || b.persistent;

          RCLCPP_DEBUG(get_logger(),
                       "Merged obstacle #%u into #%u (dist=%.2f < %.2f)",
                       b.id,
                       a.id,
                       dist,
                       merge_dist);

          // Remove j
          tracked_.erase(tracked_.begin() + static_cast<long>(j));
          merged_any = true;
          break;  // restart inner loop
        }
      }
      if (merged_any)
        break;  // restart outer loop
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistence promotion / stale eviction
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::promote_persistent(const rclcpp::Time& now)
{
  const auto now_sec = now.seconds();

  tracked_.erase(
      std::remove_if(tracked_.begin(),
                     tracked_.end(),
                     [&](TrackedObstacle& obs)
                     {
                       const double age = now_sec - obs.first_seen.seconds();
                       const double since_seen = now_sec - obs.last_seen.seconds();

                       // Remove stale transients.
                       if (!obs.persistent && since_seen > transient_timeout_)
                       {
                         RCLCPP_DEBUG(get_logger(),
                                      "Removing stale transient obstacle #%u (unseen %.1f s)",
                                      obs.id,
                                      since_seen);
                         return true;
                       }

                       // Promote to persistent when age threshold is met and at least 50 %
                       // of expected observations are present (minimum 3).
                       if (!obs.persistent && age >= persistence_threshold_)
                       {
                         const double expected = age * publish_rate_;
                         const double coverage =
                             static_cast<double>(obs.observation_count) / std::max(expected, 1.0);
                         if (coverage >= 0.5 && obs.observation_count >= 3)
                         {
                           obs.persistent = true;
                           RCLCPP_INFO(get_logger(),
                                       "Obstacle #%u promoted to PERSISTENT (age=%.1f s, "
                                       "observations=%u, coverage=%.2f)",
                                       obs.id,
                                       age,
                                       obs.observation_count,
                                       coverage);
                         }
                       }

                       return false;
                     }),
      tracked_.end());
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistence: save
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::save_to_file() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::ofstream out(persistence_file_);
  if (!out.is_open())
  {
    throw std::runtime_error("Cannot open file for writing: " + persistence_file_);
  }

  out << "# mowgli obstacle tracker — persistent obstacles\n";
  out << "obstacles:\n";

  for (const auto& obs : tracked_)
  {
    if (!obs.persistent)
    {
      continue;  // Only persist promoted obstacles.
    }
    out << "  - id: " << obs.id << "\n";
    out << "    cx: " << obs.cx << "\n";
    out << "    cy: " << obs.cy << "\n";
    out << "    radius: " << obs.radius << "\n";
    out << "    first_seen_sec: " << std::fixed << obs.first_seen.seconds() << "\n";
    out << "    observation_count: " << obs.observation_count << "\n";
    out << "    hull_points:\n";
    for (const auto& [hx, hy] : obs.hull_points)
    {
      out << "      - [" << hx << ", " << hy << "]\n";
    }
  }

  RCLCPP_INFO(get_logger(), "Saved obstacles to '%s'", persistence_file_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistence: load
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleTrackerNode::load_from_file()
{
  std::ifstream in(persistence_file_);
  if (!in.is_open())
  {
    RCLCPP_WARN(get_logger(),
                "Persistence file '%s' not found — starting with empty obstacle list.",
                persistence_file_.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  tracked_.clear();

  TrackedObstacle current;
  bool in_obstacle = false;
  bool in_hull = false;
  uint32_t max_id = 0;

  std::string line;
  while (std::getline(in, line))
  {
    // Trim leading whitespace to determine indent level.
    const size_t indent = line.find_first_not_of(' ');
    if (indent == std::string::npos || line[indent] == '#')
    {
      continue;
    }
    const std::string trimmed = line.substr(indent);

    // New obstacle entry.
    if (trimmed.rfind("- id:", 0) == 0)
    {
      if (in_obstacle)
      {
        current.persistent = true;
        tracked_.push_back(current);
        max_id = std::max(max_id, current.id);
      }
      current = TrackedObstacle();
      in_obstacle = true;
      in_hull = false;
      current.id = static_cast<uint32_t>(std::stoul(trimmed.substr(5)));
      continue;
    }

    if (!in_obstacle)
    {
      continue;
    }

    // Hull point list marker.
    if (trimmed.rfind("hull_points:", 0) == 0)
    {
      in_hull = true;
      continue;
    }

    // Hull point entry: "- [x, y]"
    if (in_hull && trimmed.rfind("- [", 0) == 0)
    {
      const size_t comma = trimmed.find(',', 3);
      const size_t close = trimmed.find(']', comma);
      if (comma != std::string::npos && close != std::string::npos)
      {
        const double hx = std::stod(trimmed.substr(3, comma - 3));
        const double hy = std::stod(trimmed.substr(comma + 1, close - comma - 1));
        current.hull_points.emplace_back(hx, hy);
      }
      continue;
    }

    // Scalar fields — stop hull mode when we encounter a non-hull key.
    if (trimmed.find(':') != std::string::npos && trimmed.rfind("- [", 0) != 0)
    {
      in_hull = false;
    }

    auto parse_double = [&](const std::string& key) -> double
    {
      if (trimmed.rfind(key + ": ", 0) == 0)
      {
        return std::stod(trimmed.substr(key.size() + 2));
      }
      return std::numeric_limits<double>::quiet_NaN();
    };

    auto parse_uint = [&](const std::string& key) -> uint32_t
    {
      if (trimmed.rfind(key + ": ", 0) == 0)
      {
        return static_cast<uint32_t>(std::stoul(trimmed.substr(key.size() + 2)));
      }
      return 0;
    };

    const double cx = parse_double("cx");
    if (std::isfinite(cx))
    {
      current.cx = cx;
      continue;
    }

    const double cy = parse_double("cy");
    if (std::isfinite(cy))
    {
      current.cy = cy;
      continue;
    }

    const double rad = parse_double("radius");
    if (std::isfinite(rad))
    {
      current.radius = rad;
      continue;
    }

    const double fss = parse_double("first_seen_sec");
    if (std::isfinite(fss))
    {
      current.first_seen = rclcpp::Time(static_cast<int32_t>(fss),
                                        static_cast<uint32_t>((fss - std::floor(fss)) * 1e9));
      current.last_seen = now();  // Treat as just-seen so it doesn't expire immediately.
      continue;
    }

    const uint32_t obs_count = parse_uint("observation_count");
    if (obs_count > 0)
    {
      current.observation_count = obs_count;
      continue;
    }
  }

  // Push the last obstacle.
  if (in_obstacle)
  {
    current.persistent = true;
    tracked_.push_back(current);
    max_id = std::max(max_id, current.id);
  }

  // Advance ID counter past all loaded IDs.
  next_id_ = max_id + 1;

  RCLCPP_INFO(get_logger(),
              "Loaded %zu persistent obstacles from '%s'",
              tracked_.size(),
              persistence_file_.c_str());
}

}  // namespace mowgli_map
