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
#include <limits>
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

  map_ = grid_map::GridMap({std::string(layers::OCCUPANCY), std::string(layers::CLASSIFICATION)});

  map_.setFrameId(map_frame_);
  map_.setGeometry(grid_map::Length(map_size_x_, map_size_y_),
                   resolution_,
                   grid_map::Position(0.0, 0.0));

  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);

  // Drop any accumulated mowed-progress overlay: a fresh map (startup or a map
  // delete) means coverage starts over. stamp_mow_progress lazily recreates it.
  mow_progress_map_ = grid_map::GridMap();

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

    for (int r = 0; r < rows; ++r)
    {
      for (int c = 0; c < cols; ++c)
      {
        float vals[2] = {occ(r, c), cls(r, c)};
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

    map_ = grid_map::GridMap({std::string(layers::OCCUPANCY), std::string(layers::CLASSIFICATION)});

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

    const int actual_rows = map_.getSize()(0);
    const int actual_cols = map_.getSize()(1);

    for (int r = 0; r < actual_rows && r < rows_loaded; ++r)
    {
      for (int c = 0; c < actual_cols && c < cols_loaded; ++c)
      {
        float vals[2] = {};
        dat.read(reinterpret_cast<char*>(vals), sizeof(vals));
        occ(r, c) = vals[0];
        cls(r, c) = vals[1];
      }
    }
    dat.close();

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
  // resize_map_to_areas() reallocates the grid and resets every layer
  // (CLASSIFICATION → UNKNOWN), discarding the LAWN/NO_GO cells stamped above.
  // Re-stamp from the full area list — exactly as on_load_areas does — so the
  // keepout/speed costmap filters see the correct classification.
  apply_area_classifications();
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
    // Start with user-defined (static) obstacles from config.
    res->area.obstacles = entry.obstacles;
    res->area.is_navigation_area = entry.is_navigation_area;

    // Also include persistent tracked obstacles from the obstacle tracker
    // so the coverage planner can avoid them in the initial plan.
    const auto n_static = res->area.obstacles.size();
    for (const auto& obs_poly : obstacle_polygons_)
    {
      if (obs_poly.points.size() >= 3)
      {
        res->area.obstacles.push_back(obs_poly);
      }
    }

    res->success = true;
    RCLCPP_INFO(get_logger(),
                "GetMowingArea[%u]: area='%s', %zu obstacles (%zu static + %zu tracked)",
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
}
void MapServerNode::on_set_docking_point(
    const mowgli_interfaces::srv::SetDockingPoint::Request::SharedPtr req,
    mowgli_interfaces::srv::SetDockingPoint::Response::SharedPtr res)
{
  // Sequence of gates protecting dock_pose accuracy. The operator forces
  // the EKF to dock_pose at boot via the fusion_graph gauge reset, so a
  // bad calibration leaks straight into the map-frame anchor for every
  // subsequent session. Reject unless ALL conditions hold:
  //   (1) firmware reports is_charging=true (robot physically on dock)
  //   (2) GPS sample fresh and σ(xy) ≤ dock_set_gps_accuracy_max_m_
  //   (3) EKF yaw converged on the recent rolling window
  //
  // (1) — is_charging gate. Refuse if the last /hardware_bridge/status was
  // not charging or is older than dock_set_status_max_age_s_.
  {
    const double max_age = get_parameter("dock_set_status_max_age_s").as_double();
    const double status_age = (last_status_time_.nanoseconds() == 0)
                                  ? std::numeric_limits<double>::infinity()
                                  : (now() - last_status_time_).seconds();
    if (status_age > max_age || !last_is_charging_)
    {
      res->success = false;
      RCLCPP_WARN(get_logger(),
                  "set_docking_point rejected: robot not detected on dock "
                  "(is_charging=%s, status_age=%.1fs, max=%.1fs). "
                  "Drive onto the dock and wait for the firmware to report "
                  "charging before retrying.",
                  last_is_charging_ ? "true" : "false",
                  status_age,
                  max_age);
      return;
    }
  }

  // (2) — GPS accuracy gate. RTK-Fixed reports σ ≈ 3 mm; RTK-Float is
  // 10-50 cm. Reject when σ(xx) or σ(yy) breaches the threshold, or when
  // /gps/pose_cov is stale (driver dead, USB unplugged, datum unset).
  {
    const double max_acc = get_parameter("dock_set_gps_accuracy_max_m").as_double();
    const double max_age = get_parameter("dock_set_gps_max_age_s").as_double();
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr gps_snap;
    rclcpp::Time gps_time{0, 0, RCL_ROS_TIME};
    {
      std::lock_guard<std::mutex> lk(last_gps_pose_cov_mutex_);
      gps_snap = last_gps_pose_cov_;
      gps_time = last_gps_pose_cov_time_;
    }
    if (!gps_snap)
    {
      res->success = false;
      RCLCPP_WARN(get_logger(),
                  "set_docking_point rejected: no /gps/pose_cov sample yet "
                  "(navsat_to_absolute_pose_node not running, or no GPS fix).");
      return;
    }
    const double gps_age = (now() - gps_time).seconds();
    if (gps_age > max_age)
    {
      res->success = false;
      RCLCPP_WARN(get_logger(),
                  "set_docking_point rejected: /gps/pose_cov stale "
                  "(age %.2fs > %.2fs). Wait for the GPS feed to refresh.",
                  gps_age,
                  max_age);
      return;
    }
    const double sigma_xx = std::sqrt(std::max(gps_snap->pose.covariance[0], 0.0));
    const double sigma_yy = std::sqrt(std::max(gps_snap->pose.covariance[7], 0.0));
    const double sigma_max = std::max(sigma_xx, sigma_yy);
    if (sigma_max > max_acc)
    {
      res->success = false;
      RCLCPP_WARN(get_logger(),
                  "set_docking_point rejected: GPS not accurate enough "
                  "(σ_max=%.3f m > %.3f m). Achieve RTK-Fixed before retrying.",
                  sigma_max,
                  max_acc);
      return;
    }
  }

  // (3) — Yaw convergence gate. Pinning a dock pose during EKF startup
  // transient writes a wildly wrong heading (and therefore a wrong base
  // position via the lever-arm projection on /gps/absolute_pose). The
  // operator should drive the robot forward briefly to lock in COG yaw,
  // then retry, or wait for mag/COG to settle the EKF naturally.
  //
  // Read thresholds live each call so `ros2 param set` works without a
  // node restart — the constructor-cached values were stuck at startup.
  const double threshold_rad = get_parameter("yaw_convergence_threshold_rad").as_double();
  const double window_s = get_parameter("yaw_convergence_window_s").as_double();
  const auto min_samples =
      static_cast<size_t>(get_parameter("yaw_convergence_min_samples").as_int());
  {
    std::lock_guard<std::mutex> lk(recent_yaws_mutex_);
    if (recent_yaws_.size() < min_samples)
    {
      res->success = false;
      RCLCPP_WARN(get_logger(),
                  "set_docking_point rejected: only %zu yaw samples in the last %.1f s "
                  "(need >= %zu). Wait for the EKF to receive more updates.",
                  recent_yaws_.size(),
                  window_s,
                  min_samples);
      return;
    }
    double sum = 0.0;
    double sum_sq = 0.0;
    for (const auto& [t, y] : recent_yaws_)
    {
      sum += y;
      sum_sq += y * y;
    }
    const double n = static_cast<double>(recent_yaws_.size());
    const double mean = sum / n;
    const double var = (sum_sq / n) - (mean * mean);
    const double std_dev = std::sqrt(std::max(var, 0.0));
    if (std_dev > threshold_rad)
    {
      res->success = false;
      RCLCPP_WARN(get_logger(),
                  "set_docking_point rejected: EKF yaw not converged "
                  "(std %.3f° > threshold %.3f° over %.1f s, %zu samples). "
                  "Drive the robot 1 m forward to anchor heading from COG, then retry.",
                  std_dev * 180.0 / M_PI,
                  threshold_rad * 180.0 / M_PI,
                  window_s,
                  recent_yaws_.size());
      return;
    }
  }

  // Position capture mode, selected by req->use_gps_position:
  //   true  — "capture current robot position": the robot is physically
  //           seated on the dock, so take the dock POSITION from the averaged
  //           independent GPS projection (/gps/pose_cov, GPS-vs-datum +
  //           lever-arm), NOT from req->docking_pose (which the GUI fills from
  //           the fused /odometry/filtered_map). While charging, fusion_graph
  //           gauge-resets the fused pose onto the EXISTING dock_pose, so
  //           capturing it would just re-store the old value — a calibration
  //           that can never correct a stale dock_pose. The GPS projection is
  //           free of that circularity; averaging kills the ~1-3 cm RTK jitter.
  //   false — manual map-drag / settings edit: the operator specified the
  //           location directly, so use req->docking_pose.position as given.
  // Yaw always comes from the request (single-antenna GPS gives no heading;
  // the yaw-convergence gate above validated it).
  docking_pose_ = req->docking_pose;  // yaw/orientation (and position if !gps)
  if (req->use_gps_position)
  {
    double gps_x_mean = 0.0;
    double gps_y_mean = 0.0;
    {
      std::lock_guard<std::mutex> lk(last_gps_pose_cov_mutex_);
      if (recent_gps_xy_.size() < dock_set_gps_avg_min_samples_)
      {
        res->success = false;
        RCLCPP_WARN(get_logger(),
                    "set_docking_point rejected: only %zu GPS samples in the last "
                    "%.1f s (need >= %zu) to average the dock position. Wait for "
                    "more /gps/pose_cov updates.",
                    recent_gps_xy_.size(),
                    dock_set_gps_avg_window_s_,
                    dock_set_gps_avg_min_samples_);
        return;
      }
      for (const auto& [t, x, y] : recent_gps_xy_)
      {
        (void)t;
        gps_x_mean += x;
        gps_y_mean += y;
      }
      const double n = static_cast<double>(recent_gps_xy_.size());
      gps_x_mean /= n;
      gps_y_mean /= n;
    }
    docking_pose_.position.x = gps_x_mean;
    docking_pose_.position.y = gps_y_mean;
    docking_pose_.position.z = 0.0;
    RCLCPP_INFO(get_logger(),
                "Docking point captured from averaged GPS: (%.3f, %.3f) over %zu "
                "samples; request fused position was (%.3f, %.3f) — Δ=(%.3f, %.3f) m",
                gps_x_mean,
                gps_y_mean,
                recent_gps_xy_.size(),
                req->docking_pose.position.x,
                req->docking_pose.position.y,
                req->docking_pose.position.x - gps_x_mean,
                req->docking_pose.position.y - gps_y_mean);
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Docking point set from request position (manual): (%.3f, %.3f)",
                docking_pose_.position.x,
                docking_pose_.position.y);
  }
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
// User-driven obstacle promotion
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_promote_obstacle(
    const mowgli_interfaces::srv::PromoteObstacle::Request::SharedPtr req,
    mowgli_interfaces::srv::PromoteObstacle::Response::SharedPtr res)
{
  // Resolve the polygon source: prefer the request's explicit polygon
  // (free-form draw), fall back to looking up the obstacle id in the
  // most recent /obstacle_tracker/obstacles snapshot.
  geometry_msgs::msg::Polygon poly = req->polygon;
  if (poly.points.size() < 3)
  {
    bool found = false;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      for (const auto& obs : last_tracker_snapshot_)
      {
        if (obs.id == req->obstacle_id)
        {
          poly = obs.polygon;
          found = true;
          break;
        }
      }
    }
    if (!found)
    {
      res->success = false;
      res->message = "obstacle_id not found in last tracker snapshot and no polygon supplied";
      return;
    }
  }

  if (!apply_promoted_obstacle(req->area_index, poly))
  {
    res->success = false;
    res->message = "promotion rejected (bad area_index, navigation area, or polygon < 3 points)";
    return;
  }

  // Persist immediately so the keepout survives a restart. Failures
  // here are logged but don't fail the service — the polygon is live
  // in memory; the next save_areas tick will retry.
  if (!areas_file_path_.empty())
  {
    try
    {
      save_areas_to_file(areas_file_path_);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(),
                  "promote_obstacle: applied to live state but YAML save failed: %s",
                  ex.what());
    }
  }

  res->success = true;
  res->message =
      "obstacle promoted to permanent keepout for area " + std::to_string(req->area_index);
  RCLCPP_INFO(get_logger(),
              "promote_obstacle: appended polygon (%zu points) to area %u",
              poly.points.size(),
              req->area_index);
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

  // Dock body cells → OBSTACLE_PERMANENT. The strip planner stops at
  // OBSTACLE_PERMANENT, and Smac sees them as lethal — so the robot
  // never tries to mow into or path through the dock structure.
  if (has_dock_exclusion_ && dock_body_polygon_.points.size() >= 3)
  {
    const float body_val = static_cast<float>(CellType::OBSTACLE_PERMANENT);
    grid_map::Polygon body_gm;
    for (const auto& pt : dock_body_polygon_.points)
    {
      body_gm.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }
    for (grid_map::PolygonIterator it(map_, body_gm); !it.isPastEnd(); ++it)
    {
      map_.at(std::string(layers::CLASSIFICATION), *it) = body_val;
    }
  }

  // Dock approach corridor → DOCKING_AREA. Mowable (strips can traverse),
  // but flagged so the keepout-mask carve-out and reachability analysis
  // can identify these cells. Skip cells already marked OBSTACLE_PERMANENT
  // (body) so the body classification wins where the polygons touch.
  if (has_dock_exclusion_ && dock_corridor_polygon_.points.size() >= 3)
  {
    const float corridor_val = static_cast<float>(CellType::DOCKING_AREA);
    const float perm_val = static_cast<float>(CellType::OBSTACLE_PERMANENT);
    grid_map::Polygon corridor_gm;
    for (const auto& pt : dock_corridor_polygon_.points)
    {
      corridor_gm.addVertex(
          grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }
    auto& cls = map_[std::string(layers::CLASSIFICATION)];
    for (grid_map::PolygonIterator it(map_, corridor_gm); !it.isPastEnd(); ++it)
    {
      if (cls((*it)(0), (*it)(1)) != perm_val)
      {
        cls((*it)(0), (*it)(1)) = corridor_val;
      }
    }
  }
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
