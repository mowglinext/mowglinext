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

// Area CRUD, map I/O service handlers (save/load/clear_map), area
// persistence (areas.dat), dock-pose setter (with mowgli_robot.yaml
// line-splice update), classification re-application, and unit-test
// entry points — all split out of map_server_node.cpp without
// changing the on-disk formats or service interfaces.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mowgli_map/map_server_node.hpp"
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace mowgli_map
{

// Path to the runtime mowgli_robot.yaml — bind-mounted into the container
// so writes survive across redeploys. mowgli_robot.yaml is the single
// source of truth for dock_pose_x/y/yaw; this helper rewrites the three
// scalar values in place via per-line substring splicing so comments
// and surrounding structure are preserved (yaml-cpp would round-trip
// and strip them).
constexpr const char* kRuntimeRobotYaml = "/ros2_ws/config/mowgli_robot.yaml";

// Splice a new numeric value into a "<indent><key>:<spaces><number><rest>"
// line, anchored on the indent so a key whose name happens to contain ours
// (e.g. dock_pose_x_offset) is not matched.
inline void splice_yaml_scalar(std::string& content,
                               const std::string& key,
                               const std::string& new_value)
{
  size_t scan = 0;
  while (scan < content.size())
  {
    const size_t line_start = scan;
    size_t cursor = line_start;
    while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t'))
      ++cursor;
    const size_t indent_end = cursor;
    if (indent_end > line_start && cursor + key.size() < content.size() &&
        content.compare(cursor, key.size(), key) == 0 && content[cursor + key.size()] == ':')
    {
      cursor += key.size() + 1;
      while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t'))
        ++cursor;
      const size_t val_start = cursor;
      while (cursor < content.size())
      {
        const char c = content[cursor];
        const bool is_num =
            (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
        if (!is_num)
          break;
        ++cursor;
      }
      if (cursor > val_start)
      {
        content.replace(val_start, cursor - val_start, new_value);
        return;
      }
    }
    const size_t nl = content.find('\n', line_start);
    if (nl == std::string::npos)
      break;
    scan = nl + 1;
  }
}

inline bool update_dock_pose_in_robot_yaml(const std::string& path,
                                           double x,
                                           double y,
                                           double yaw_rad)
{
  std::ifstream in(path);
  if (!in.good())
    return false;
  std::stringstream buf;
  buf << in.rdbuf();
  std::string content = buf.str();
  in.close();

  auto fmt = [](double v)
  {
    std::ostringstream s;
    s << std::fixed << std::setprecision(6) << v;
    return s.str();
  };
  splice_yaml_scalar(content, "dock_pose_x", fmt(x));
  splice_yaml_scalar(content, "dock_pose_y", fmt(y));
  splice_yaml_scalar(content, "dock_pose_yaw", fmt(yaw_rad));

  const std::string tmp_path = path + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out.good())
      return false;
    out << content;
    if (!out.good())
      return false;
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, path, ec);
  return !ec;
}

geometry_msgs::msg::Polygon MapServerNode::parse_polygon_string(const std::string& s)
{
  geometry_msgs::msg::Polygon poly;
  if (s.empty())
  {
    return poly;
  }

  std::istringstream pts_stream(s);
  std::string point_str;
  while (std::getline(pts_stream, point_str, ';'))
  {
    std::istringstream coord_stream(point_str);
    std::string x_str, y_str;
    if (std::getline(coord_stream, x_str, ',') && std::getline(coord_stream, y_str, ','))
    {
      geometry_msgs::msg::Point32 p;
      p.x = std::stof(x_str);
      p.y = std::stof(y_str);
      p.z = 0.0f;
      poly.points.push_back(p);
    }
  }
  return poly;
}

void MapServerNode::load_areas_from_params()
{
  // Declare area parameter arrays with empty defaults.
  const auto area_names =
      declare_parameter<std::vector<std::string>>("area_names", std::vector<std::string>{});
  const auto area_polygons =
      declare_parameter<std::vector<std::string>>("area_polygons", std::vector<std::string>{});
  const auto area_is_navigation =
      declare_parameter<std::vector<bool>>("area_is_navigation", std::vector<bool>{});
  const auto area_obstacles =
      declare_parameter<std::vector<std::string>>("area_obstacles", std::vector<std::string>{});

  if (area_names.empty())
  {
    RCLCPP_WARN(get_logger(),
                "No areas configured (area_names is empty). "
                "Keepout mask will not be published until areas are added via service.");
    return;
  }

  if (area_names.size() != area_polygons.size())
  {
    RCLCPP_ERROR(get_logger(),
                 "area_names (%zu) and area_polygons (%zu) must have the same length!",
                 area_names.size(),
                 area_polygons.size());
    return;
  }

  for (std::size_t i = 0; i < area_names.size(); ++i)
  {
    AreaEntry entry;
    entry.name = area_names[i];
    entry.polygon = parse_polygon_string(area_polygons[i]);
    entry.is_navigation_area = (i < area_is_navigation.size()) && area_is_navigation[i];

    if (entry.polygon.points.size() < 3)
    {
      RCLCPP_WARN(get_logger(),
                  "Skipping area '%s': polygon has %zu vertices (need >= 3)",
                  entry.name.c_str(),
                  entry.polygon.points.size());
      continue;
    }

    // Parse obstacle polygons (semicolon-separated polygon strings, pipe-separated).
    // Format: "x1,y1;x2,y2;x3,y3|x4,y4;x5,y5;x6,y6" for multiple obstacles.
    if (i < area_obstacles.size() && !area_obstacles[i].empty())
    {
      std::istringstream obs_stream(area_obstacles[i]);
      std::string obs_str;
      while (std::getline(obs_stream, obs_str, '|'))
      {
        auto obs_poly = parse_polygon_string(obs_str);
        if (obs_poly.points.size() >= 3)
        {
          entry.obstacles.push_back(obs_poly);
          obstacle_polygons_.push_back(obs_poly);
        }
      }
    }

    RCLCPP_INFO(get_logger(),
                "Loaded area '%s': %zu vertices, %s, %zu obstacles",
                entry.name.c_str(),
                entry.polygon.points.size(),
                entry.is_navigation_area ? "navigation" : "mowing",
                entry.obstacles.size());

    areas_.push_back(std::move(entry));
  }
}
void MapServerNode::init_map()
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  map_ = grid_map::GridMap({std::string(layers::OCCUPANCY),
                            std::string(layers::CLASSIFICATION),
                            std::string(layers::MOW_PROGRESS),
                            std::string(layers::CONFIDENCE),
                            std::string(layers::FAIL_COUNT)});

  map_.setFrameId(map_frame_);
  map_.setGeometry(grid_map::Length(map_size_x_, map_size_y_),
                   resolution_,
                   grid_map::Position(0.0, 0.0));

  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);
  map_[std::string(layers::FAIL_COUNT)].setConstant(defaults::FAIL_COUNT);

  RCLCPP_DEBUG(get_logger(),
               "Grid map created: %zu×%zu cells",
               static_cast<std::size_t>(map_.getSize()(0)),
               static_cast<std::size_t>(map_.getSize()(1)));
}

void MapServerNode::resize_map_to_areas()
{
  if (areas_.empty())
  {
    return;
  }

  // Compute bounding box of all area polygons.
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& area : areas_)
  {
    for (const auto& pt : area.polygon.points)
    {
      min_x = std::min(min_x, static_cast<double>(pt.x));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      max_y = std::max(max_y, static_cast<double>(pt.y));
    }
  }

  // Add 5m margin on each side for navigation around the areas.
  constexpr double margin = 5.0;
  const double new_size_x = (max_x - min_x) + 2.0 * margin;
  const double new_size_y = (max_y - min_y) + 2.0 * margin;
  const double center_x = (min_x + max_x) * 0.5;
  const double center_y = (min_y + max_y) * 0.5;

  // Only resize if the new size differs meaningfully from the current one.
  if (std::abs(new_size_x - map_size_x_) < resolution_ &&
      std::abs(new_size_y - map_size_y_) < resolution_)
  {
    return;
  }

  map_size_x_ = new_size_x;
  map_size_y_ = new_size_y;

  std::lock_guard<std::mutex> lock(map_mutex_);
  map_.setGeometry(grid_map::Length(map_size_x_, map_size_y_),
                   resolution_,
                   grid_map::Position(center_x, center_y));

  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);
  map_[std::string(layers::FAIL_COUNT)].setConstant(defaults::FAIL_COUNT);

  masks_dirty_ = true;

  RCLCPP_INFO(get_logger(),
              "Map resized to %.1f×%.1f m (center: %.1f, %.1f) to fit %zu areas",
              map_size_x_,
              map_size_y_,
              center_x,
              center_y,
              areas_.size());
}
void MapServerNode::on_save_map(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (map_file_path_.empty())
  {
    res->success = false;
    res->message = "map_file_path parameter is empty; cannot save.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    const std::string yaml_path = map_file_path_ + ".yaml";
    const std::string data_path = map_file_path_ + ".dat";

    std::ofstream yaml(yaml_path);
    if (!yaml.is_open())
    {
      throw std::runtime_error("Cannot open " + yaml_path + " for writing");
    }
    yaml << "resolution: " << resolution_ << "\n"
         << "map_size_x: " << map_size_x_ << "\n"
         << "map_size_y: " << map_size_y_ << "\n"
         << "map_frame: " << map_frame_ << "\n"
         << "rows: " << map_.getSize()(0) << "\n"
         << "cols: " << map_.getSize()(1) << "\n"
         << "pos_x: " << map_.getPosition().x() << "\n"
         << "pos_y: " << map_.getPosition().y() << "\n";
    yaml.close();

    std::ofstream dat(data_path, std::ios::binary);
    if (!dat.is_open())
    {
      throw std::runtime_error("Cannot open " + data_path + " for writing");
    }

    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);

    const auto& occ = map_[std::string(layers::OCCUPANCY)];
    const auto& cls = map_[std::string(layers::CLASSIFICATION)];
    const auto& prog = map_[std::string(layers::MOW_PROGRESS)];
    const auto& conf = map_[std::string(layers::CONFIDENCE)];

    for (int r = 0; r < rows; ++r)
    {
      for (int c = 0; c < cols; ++c)
      {
        float vals[4] = {occ(r, c), cls(r, c), prog(r, c), conf(r, c)};
        dat.write(reinterpret_cast<const char*>(vals), sizeof(vals));
      }
    }
    dat.close();

    res->success = true;
    res->message = "Map saved to " + map_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Save failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void MapServerNode::on_load_map(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (map_file_path_.empty())
  {
    res->success = false;
    res->message = "map_file_path parameter is empty; cannot load.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    const std::string yaml_path = map_file_path_ + ".yaml";
    const std::string data_path = map_file_path_ + ".dat";

    std::ifstream yaml(yaml_path);
    if (!yaml.is_open())
    {
      throw std::runtime_error("Cannot open " + yaml_path);
    }

    double res_loaded{}, sx{}, sy{};
    std::string frame_loaded{};
    int rows_loaded{}, cols_loaded{};
    double pos_x{}, pos_y{};

    std::string line;
    while (std::getline(yaml, line))
    {
      std::istringstream ss(line);
      std::string key;
      if (!(ss >> key))
        continue;
      if (key == "resolution:")
        ss >> res_loaded;
      else if (key == "map_size_x:")
        ss >> sx;
      else if (key == "map_size_y:")
        ss >> sy;
      else if (key == "map_frame:")
        ss >> frame_loaded;
      else if (key == "rows:")
        ss >> rows_loaded;
      else if (key == "cols:")
        ss >> cols_loaded;
      else if (key == "pos_x:")
        ss >> pos_x;
      else if (key == "pos_y:")
        ss >> pos_y;
    }
    yaml.close();

    if (rows_loaded <= 0 || cols_loaded <= 0)
    {
      throw std::runtime_error("Invalid map dimensions in " + yaml_path);
    }

    std::lock_guard<std::mutex> lock(map_mutex_);

    resolution_ = res_loaded;
    map_size_x_ = sx;
    map_size_y_ = sy;
    map_frame_ = frame_loaded;

    map_ = grid_map::GridMap({std::string(layers::OCCUPANCY),
                              std::string(layers::CLASSIFICATION),
                              std::string(layers::MOW_PROGRESS),
                              std::string(layers::CONFIDENCE)});

    map_.setFrameId(map_frame_);
    map_.setGeometry(grid_map::Length(map_size_x_, map_size_y_),
                     resolution_,
                     grid_map::Position(pos_x, pos_y));

    std::ifstream dat(data_path, std::ios::binary);
    if (!dat.is_open())
    {
      throw std::runtime_error("Cannot open " + data_path);
    }

    auto& occ = map_[std::string(layers::OCCUPANCY)];
    auto& cls = map_[std::string(layers::CLASSIFICATION)];
    auto& prog = map_[std::string(layers::MOW_PROGRESS)];
    auto& conf = map_[std::string(layers::CONFIDENCE)];

    const int actual_rows = map_.getSize()(0);
    const int actual_cols = map_.getSize()(1);

    for (int r = 0; r < actual_rows && r < rows_loaded; ++r)
    {
      for (int c = 0; c < actual_cols && c < cols_loaded; ++c)
      {
        float vals[4] = {};
        dat.read(reinterpret_cast<char*>(vals), sizeof(vals));
        occ(r, c) = vals[0];
        cls(r, c) = vals[1];
        prog(r, c) = vals[2];
        conf(r, c) = vals[3];
      }
    }
    dat.close();

    last_decay_time_ = now();

    res->success = true;
    res->message = "Map loaded from " + map_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Load failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void MapServerNode::on_clear_map(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                 std_srvs::srv::Trigger::Response::SharedPtr res)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    clear_map_layers();
  }
  areas_.clear();
  obstacle_polygons_.clear();
  docking_pose_set_ = false;
  keepout_filter_info_sent_ = false;
  speed_filter_info_sent_ = false;
  masks_dirty_ = true;

  res->success = true;
  res->message = "All map layers and areas cleared.";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void MapServerNode::on_add_area(const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
                                mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res)
{
  const auto& polygon_msg = req->area.area;

  if (polygon_msg.points.size() < 3)
  {
    res->success = false;
    RCLCPP_WARN(get_logger(), "add_area: polygon must have at least 3 points.");
    return;
  }

  // Build grid_map polygon from geometry_msgs polygon
  grid_map::Polygon gm_polygon;
  for (const auto& pt : polygon_msg.points)
  {
    gm_polygon.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
  }

  // Classify cells inside the area as LAWN (mowable), not NO_GO_ZONE.
  // Only exclusion zones and obstacles should be NO_GO_ZONE.
  const float lawn_val = static_cast<float>(CellType::LAWN);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    for (grid_map::PolygonIterator it(map_, gm_polygon); !it.isPastEnd(); ++it)
    {
      map_.at(std::string(layers::CLASSIFICATION), *it) = lawn_val;
    }
  }

  // Store as an area entry.
  AreaEntry entry;
  entry.name = req->area.name;
  entry.polygon = polygon_msg;
  entry.is_navigation_area = req->is_navigation_area;

  // Store obstacle polygons from the MapArea message.
  // Only store in the area entry (static), NOT in obstacle_polygons_
  // (which is for dynamic LiDAR-detected obstacles).
  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);
  for (const auto& obstacle : req->area.obstacles)
  {
    if (obstacle.points.size() >= 3)
    {
      entry.obstacles.push_back(obstacle);

      grid_map::Polygon obs_gm;
      for (const auto& pt : obstacle.points)
      {
        obs_gm.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
      }
      std::lock_guard<std::mutex> lock(map_mutex_);
      for (grid_map::PolygonIterator it(map_, obs_gm); !it.isPastEnd(); ++it)
      {
        map_.at(std::string(layers::CLASSIFICATION), *it) = no_go_val;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    areas_.push_back(std::move(entry));
  }
  resize_map_to_areas();
  masks_dirty_ = true;

  RCLCPP_INFO(get_logger(),
              "Added area '%s' (%s) with %zu vertices and %zu obstacles.",
              req->area.name.c_str(),
              req->is_navigation_area ? "navigation" : "mowing",
              polygon_msg.points.size(),
              req->area.obstacles.size());

  // Auto-save if persistence path is set.
  if (!areas_file_path_.empty())
  {
    try
    {
      save_areas_to_file(areas_file_path_);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(), "Auto-save after area add failed: %s", ex.what());
    }
  }

  res->success = true;
}

void MapServerNode::on_get_mowing_area(
    const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
    mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  const auto idx = static_cast<std::size_t>(req->index);
  if (idx < areas_.size())
  {
    const auto& entry = areas_[idx];
    res->area.name = entry.name;
    res->area.area = entry.polygon;
    // Only USER-DEFINED static obstacles enter F2C. Auto-tracked
    // obstacles from obstacle_tracker_node are deliberately NOT
    // included here even when "PERSISTENT" — promotion to a planning
    // input is reserved for a future GUI flow where the operator
    // confirms each tracked obstacle. Two reasons:
    //   1. Auto-promotion creates noise: in clean sim runs even with
    //      promotion thresholds tightened (60 s, 8 cluster pts),
    //      transient ground-noise blobs occasionally crossed the
    //      threshold and entered F2C as polygon holes — F2C re-routed,
    //      MPPI followed the new route, robot ended up past the
    //      polygon boundary (sim #14: 9 boundary violations all at
    //      x=-3 west edge with 0 actual obstacles in that area).
    //   2. The tracker's job is detection / visualization, not
    //      planning authority. Real-time avoidance of dynamic
    //      obstacles is handled by Nav2's obstacle_layer feeding
    //      MPPI's ObstaclesCritic — that pipeline doesn't need
    //      persistence to work and doesn't change the F2C plan.
    res->area.obstacles = entry.obstacles;
    res->area.is_navigation_area = entry.is_navigation_area;
    const auto n_static = res->area.obstacles.size();

    // The dock-planning hole — small rectangle covering only the
    // physical dock structure (not the approach corridor). Sent to
    // F2C so the planner cuts strips around the dock without the
    // 1.5 m corridor inflating the cut. The full dock_exclusion_polygon_
    // (including the corridor) is still used for the classification
    // layer + keepout carve-out elsewhere — that's a docking-staging
    // concern, not a coverage-planning concern.
    if (has_dock_exclusion_ && dock_planning_polygon_.points.size() >= 3)
    {
      res->area.obstacles.push_back(dock_planning_polygon_);
    }

    res->success = true;
    RCLCPP_INFO(get_logger(),
                "GetMowingArea[%u]: area='%s', %zu obstacles "
                "(%zu static + %zu dock-exclusion)",
                req->index,
                entry.name.c_str(),
                res->area.obstacles.size(),
                n_static,
                res->area.obstacles.size() - n_static);
  }
  else
  {
    res->success = false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────
void MapServerNode::clear_map_layers()
{
  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);
  map_[std::string(layers::FAIL_COUNT)].setConstant(defaults::FAIL_COUNT);
}
void MapServerNode::on_set_docking_point(
    const mowgli_interfaces::srv::SetDockingPoint::Request::SharedPtr req,
    mowgli_interfaces::srv::SetDockingPoint::Response::SharedPtr res)
{
  docking_pose_ = req->docking_pose;
  docking_pose_set_ = true;

  // Publish the docking pose for other nodes (e.g., behavior tree).
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = now();
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose = docking_pose_;
  docking_pose_pub_->publish(pose_msg);

  RCLCPP_INFO(get_logger(),
              "Docking point set: (%.3f, %.3f, %.3f) orientation (%.3f, %.3f, %.3f, %.3f)",
              docking_pose_.position.x,
              docking_pose_.position.y,
              docking_pose_.position.z,
              docking_pose_.orientation.x,
              docking_pose_.orientation.y,
              docking_pose_.orientation.z,
              docking_pose_.orientation.w);

  // Persist to mowgli_robot.yaml — single source of truth for dock pose.
  // Manual placements via the GUI land here; calibrate_imu_yaw_node writes
  // the same file when its dock pre-phase finishes. A line-regex update
  // preserves the surrounding comments / structure.
  try
  {
    const double yaw_rad =
        2.0 * std::atan2(docking_pose_.orientation.z, docking_pose_.orientation.w);
    if (!update_dock_pose_in_robot_yaml(
            kRuntimeRobotYaml, docking_pose_.position.x, docking_pose_.position.y, yaw_rad))
    {
      RCLCPP_WARN(get_logger(),
                  "Could not persist dock pose to %s — file missing or "
                  "not writable. Pose still applied in-memory.",
                  kRuntimeRobotYaml);
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Persisted dock pose to %s: (%.3f, %.3f) yaw=%.3f rad",
                  kRuntimeRobotYaml,
                  docking_pose_.position.x,
                  docking_pose_.position.y,
                  yaw_rad);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(get_logger(),
                "Failed to persist dock pose to %s: %s",
                kRuntimeRobotYaml,
                ex.what());
  }

  res->success = true;
}
void MapServerNode::on_save_areas(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (areas_file_path_.empty())
  {
    res->success = false;
    res->message = "areas_file_path parameter is empty; cannot save.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    save_areas_to_file(areas_file_path_);
    res->success = true;
    res->message = "Areas saved to " + areas_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Save failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void MapServerNode::on_load_areas(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (areas_file_path_.empty())
  {
    res->success = false;
    res->message = "areas_file_path parameter is empty; cannot load.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    load_areas_from_file(areas_file_path_);
    apply_area_classifications();
    res->success = true;
    res->message = "Areas loaded from " + areas_file_path_;
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
// Area persistence helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string MapServerNode::polygon_to_string(const geometry_msgs::msg::Polygon& poly)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < poly.points.size(); ++i)
  {
    if (i > 0)
    {
      oss << ";";
    }
    oss << poly.points[i].x << "," << poly.points[i].y;
  }
  return oss.str();
}

void MapServerNode::save_areas_to_file(const std::string& path)
{
  std::ofstream out(path);
  if (!out.is_open())
  {
    throw std::runtime_error("Cannot open " + path + " for writing");
  }

  out << "# Mowgli ROS2 — Persisted areas and docking point\n";
  out << "# Auto-generated by map_server_node. Do not edit manually.\n\n";

  out << "area_count: " << areas_.size() << "\n\n";

  for (std::size_t i = 0; i < areas_.size(); ++i)
  {
    const auto& area = areas_[i];
    out << "area_" << i << "_name: " << area.name << "\n";
    out << "area_" << i << "_polygon: " << polygon_to_string(area.polygon) << "\n";
    out << "area_" << i << "_is_navigation: " << (area.is_navigation_area ? 1 : 0) << "\n";
    out << "area_" << i << "_obstacle_count: " << area.obstacles.size() << "\n";
    for (std::size_t j = 0; j < area.obstacles.size(); ++j)
    {
      out << "area_" << i << "_obstacle_" << j << ": " << polygon_to_string(area.obstacles[j])
          << "\n";
    }
    out << "\n";
  }

  // Dock pose intentionally NOT serialized here. The single source of truth
  // is mowgli_robot.yaml — written by calibrate_imu_yaw_node and
  // on_set_docking_point. Storing it in areas.dat too led to a stale
  // all-zero pose taking precedence over the calibrated value.

  out.close();
}

void MapServerNode::load_areas_from_file(const std::string& path)
{
  std::ifstream in(path);
  if (!in.is_open())
  {
    throw std::runtime_error("Cannot open " + path);
  }

  // Parse all key-value pairs into a map.
  std::map<std::string, std::string> kv;
  std::string line;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos)
    {
      continue;
    }
    std::string key = line.substr(0, colon_pos);
    std::string val = line.substr(colon_pos + 1);
    // Trim leading whitespace from value.
    auto start = val.find_first_not_of(" \t");
    if (start != std::string::npos)
    {
      val = val.substr(start);
    }
    else
    {
      val.clear();
    }
    kv[key] = val;
  }
  in.close();

  auto get_int = [&](const std::string& key, int def) -> int
  {
    auto it = kv.find(key);
    return (it != kv.end()) ? std::stoi(it->second) : def;
  };

  auto get_double = [&](const std::string& key, double def) -> double
  {
    auto it = kv.find(key);
    return (it != kv.end()) ? std::stod(it->second) : def;
  };

  auto get_str = [&](const std::string& key) -> std::string
  {
    auto it = kv.find(key);
    return (it != kv.end()) ? it->second : std::string{};
  };

  // Clear existing areas and reload from file.
  areas_.clear();
  obstacle_polygons_.clear();

  const int area_count = get_int("area_count", 0);
  for (int i = 0; i < area_count; ++i)
  {
    const std::string prefix = "area_" + std::to_string(i);
    AreaEntry entry;
    entry.name = get_str(prefix + "_name");
    entry.polygon = parse_polygon_string(get_str(prefix + "_polygon"));
    entry.is_navigation_area = (get_int(prefix + "_is_navigation", 0) != 0);

    const int obs_count = get_int(prefix + "_obstacle_count", 0);
    for (int j = 0; j < obs_count; ++j)
    {
      auto obs_poly = parse_polygon_string(get_str(prefix + "_obstacle_" + std::to_string(j)));
      if (obs_poly.points.size() >= 3)
      {
        entry.obstacles.push_back(obs_poly);
      }
    }

    if (entry.polygon.points.size() >= 3)
    {
      RCLCPP_INFO(get_logger(),
                  "Loaded area '%s': %zu vertices, %s, %zu obstacles",
                  entry.name.c_str(),
                  entry.polygon.points.size(),
                  entry.is_navigation_area ? "navigation" : "mowing",
                  entry.obstacles.size());
      areas_.push_back(std::move(entry));
    }
  }

  // Dock pose is loaded from mowgli_robot.yaml at construction, never
  // from areas.dat. Old areas.dat files may still contain dock_x/dock_qw
  // keys — they are ignored on purpose.

  // Resize map to fit new areas and reset masks.
  resize_map_to_areas();
  keepout_filter_info_sent_ = false;
  speed_filter_info_sent_ = false;
  masks_dirty_ = true;
}

void MapServerNode::apply_area_classifications()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  const float lawn_val = static_cast<float>(CellType::LAWN);
  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);

  for (const auto& area : areas_)
  {
    grid_map::Polygon gm_polygon;
    for (const auto& pt : area.polygon.points)
    {
      gm_polygon.addVertex(
          grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }

    // Mowing areas are LAWN, not NO_GO_ZONE.
    for (grid_map::PolygonIterator it(map_, gm_polygon); !it.isPastEnd(); ++it)
    {
      map_.at(std::string(layers::CLASSIFICATION), *it) = lawn_val;
    }

    for (const auto& obstacle : area.obstacles)
    {
      grid_map::Polygon obs_gm;
      for (const auto& pt : obstacle.points)
      {
        obs_gm.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
      }
      for (grid_map::PolygonIterator it(map_, obs_gm); !it.isPastEnd(); ++it)
      {
        map_.at(std::string(layers::CLASSIFICATION), *it) = no_go_val;
      }
    }
  }

  // Mark dock exclusion zone as DOCKING_AREA — coverage planner skips
  // these cells (they aren't LAWN), but Nav2's keepout filter ignores
  // DOCKING_AREA so opennav_docking's NavigateToPose to the staging
  // pose can plan a path THROUGH the corridor. Marking these cells as
  // NO_GO_ZONE pollutes the keepout mask with LETHAL — the staging
  // pose itself sits inside the corridor and Smac then refuses to
  // route there → "Navigation request to staging pose failed".
  if (has_dock_exclusion_ && dock_exclusion_polygon_.points.size() >= 3)
  {
    const float dock_val = static_cast<float>(CellType::DOCKING_AREA);
    grid_map::Polygon dock_gm;
    for (const auto& pt : dock_exclusion_polygon_.points)
    {
      dock_gm.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }
    for (grid_map::PolygonIterator it(map_, dock_gm); !it.isPastEnd(); ++it)
    {
      map_.at(std::string(layers::CLASSIFICATION), *it) = dock_val;
    }
  }
}
void MapServerNode::tick_once(double elapsed_seconds)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  apply_decay(elapsed_seconds);
}

void MapServerNode::mark_mowed(double x, double y)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  mark_cells_mowed(x, y);
}

void MapServerNode::add_area_for_test(
    const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
    mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res)
{
  on_add_area(req, res);
}

void MapServerNode::get_mowing_area_for_test(
    const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
    mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res)
{
  on_get_mowing_area(req, res);
}

void MapServerNode::save_areas_for_test(const std::string& path)
{
  save_areas_to_file(path);
}

void MapServerNode::load_areas_for_test(const std::string& path)
{
  load_areas_from_file(path);
}

}  // namespace mowgli_map
