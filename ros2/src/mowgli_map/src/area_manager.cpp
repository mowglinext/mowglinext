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
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mowgli_interfaces/robot_yaml_scalar.hpp"
#include "mowgli_interfaces/wgs84_projection.hpp"
#include "mowgli_map/dock_antenna_capture.hpp"
#include "mowgli_map/dock_set_gates.hpp"
#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace mowgli_map
{

// (kRuntimeRobotYaml moved to map_server_node.hpp — it is now the default of
// the `robot_yaml_path` parameter so tests can redirect dock-pose writes.)

namespace
{

/// A datum is "set" once it is meaningfully away from the 0/0 template
/// default. 1e-9° ≈ 0.1 mm — anything closer to Null Island is unset.
constexpr double kDatumUnsetEpsilonDeg = 1e-9;

/// Two datums this close (≈ 1 mm) are the same site anchor — no migration.
/// Loose enough to absorb the fixed-precision formatting round-trip of the
/// areas.dat stamp and the GUI's geo-scalar formatting.
constexpr double kDatumMatchEpsilonDeg = 1e-8;

bool is_datum_set(double lat, double lon)
{
  return std::abs(lat) > kDatumUnsetEpsilonDeg || std::abs(lon) > kDatumUnsetEpsilonDeg;
}

}  // namespace

// The x/y/yaw splice-in-place writer moved to mowgli_interfaces::
// robot_yaml_scalar::UpdateDockPose (task #42) — was a byte-for-byte-
// identical copy of the same logic duplicated across this file,
// mowgli_localization/calibrate_imu_yaw_node.cpp, and
// mowgli_behavior/calibration_nodes.cpp.

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
        // Dedup on load so pre-existing stacked duplicates (from the old
        // no-dedup promote path) collapse to a single keepout.
        if (obs_poly.points.size() >= 3 &&
            !has_duplicate_obstacle_entry(entry.obstacles, obs_poly, kObstacleDedupEpsilonM) &&
            !has_duplicate_obstacle(obstacle_polygons_, obs_poly, kObstacleDedupEpsilonM))
        {
          entry.obstacles.push_back(make_obstacle_entry(
              obs_poly, {}, mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER, false));
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

  // A fresh map (startup or a map delete) starts coverage over and must not
  // retain an OccupancyGrid cache generated for its previous geometry.
  initialize_mow_progress_map();

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

  initialize_mow_progress_map();

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

    initialize_mow_progress_map();

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
  // mowglinext#637: preserve a caller-supplied id when re-adding an area
  // that already had one — the GUI's edit/delete flow rebuilds the WHOLE
  // area list (clear_map + add_area per area) even when the operator only
  // touched ONE of them, so round-tripping every untouched area's existing
  // id here is what keeps its identity stable across that rebuild. A
  // genuinely new area (id absent, i.e. 0 — the MapArea.msg default for a
  // request that never set it) gets a freshly minted one instead.
  entry.id = (req->area.id != 0) ? req->area.id : next_area_id_++;
  // A round-tripped id must still be UNIQUE. The rebuild flow above replays one
  // add_area per area, so a client whose cached list contains the same id twice
  // — or that replays a stale list against areas this session already minted —
  // would otherwise create two entries sharing an identity that the resume
  // cursor, the mow-progress bookkeeping and the GUI all key on. Mint a fresh
  // id instead of trusting the caller, and say so: silently renaming an area's
  // identity is exactly the kind of thing that is impossible to debug later.
  if (req->area.id != 0 && std::any_of(areas_.begin(),
                                       areas_.end(),
                                       [&entry](const AreaEntry& existing)
                                       {
                                         return existing.id == entry.id;
                                       }))
  {
    RCLCPP_WARN(get_logger(),
                "AddArea('%s'): id %u is already taken by another area — minting %u instead. "
                "The caller replayed a duplicate or stale id.",
                entry.name.c_str(),
                entry.id,
                next_area_id_);
    entry.id = next_area_id_++;
  }
  if (entry.id >= next_area_id_)
  {
    // A round-tripped id can be >= our current counter (e.g. this session
    // already minted past it via some other insert before the client's
    // cached copy was fetched) — advance past it so the NEXT freshly
    // minted id in this session can never collide with it.
    next_area_id_ = entry.id + 1;
  }

  // Store obstacle polygons from the MapArea message.
  // Only store in the area entry (static), NOT in obstacle_polygons_
  // (which is for dynamic LiDAR-detected obstacles).
  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);
  for (std::size_t j = 0; j < req->area.obstacles.size(); ++j)
  {
    const auto& obstacle = req->area.obstacles[j];
    if (obstacle.points.size() >= 3)
    {
      // Identity is optional and index-aligned: a caller that sends bare
      // polygons (or a pre-#502 GUI) gets unnamed user-drawn keepouts.
      // `pending` is deliberately NOT taken from the request — proposals are
      // created by the dig path only, never by drawing an area.
      std::string obs_name;
      uint8_t obs_source = mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER;
      if (j < req->area.obstacle_info.size())
      {
        obs_name = req->area.obstacle_info[j].name;
        obs_source = req->area.obstacle_info[j].source;
      }
      entry.obstacles.push_back(make_obstacle_entry(obstacle, obs_name, obs_source, false));

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
    res->area.is_navigation_area = entry.is_navigation_area;
    res->area.id = entry.id;  // mowglinext#637 — see MapArea.msg's doc comment

    // `obstacles` holds APPLIED keepouts only — it is what PlanCoverageArea
    // turns into coverage holes, so a PENDING proposal must never appear in
    // it: a dig would otherwise change the plan mid-session (9 -> 10
    // sub-paths on 2026-09-17), which breaks the determinism the resume cursor
    // relies on, without the operator having agreed to anything. Proposals go
    // to the separate `proposed_obstacles` list, read only by the GUI. Both
    // lists carry an index-aligned MapObstacleInfo (name / provenance / id).
    for (const auto& obs : entry.obstacles)
    {
      mowgli_interfaces::msg::MapObstacleInfo info;
      info.name = obs.name;
      info.source = obs.source;
      info.pending = obs.pending;
      info.id = obs.id;
      if (obs.pending)
      {
        res->area.proposed_obstacles.push_back(obs.polygon);
        res->area.proposed_obstacle_info.push_back(info);
      }
      else
      {
        res->area.obstacles.push_back(obs.polygon);
        res->area.obstacle_info.push_back(info);
      }
    }

    // Area entries own every obstacle, including proposals and promoted
    // tracker observations. obstacle_polygons_ is only a flat keepout cache
    // for navigation; appending it here leaks other areas' obstacles into
    // this response and loses their id/name provenance. Return the owning
    // area's entries only; the global keepout mask still protects every
    // applied spot.

    res->success = true;
    RCLCPP_INFO(get_logger(),
                "GetMowingArea[%u]: area='%s', %zu obstacles",
                req->index,
                entry.name.c_str(),
                res->area.obstacles.size());
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
  initialize_mow_progress_map();
}
namespace
{
// dock_set_gates.hpp carries the yaw_source constants as plain integers so it
// stays free of generated-message includes — pin them to the .srv here.
using SetDockReqConsts = mowgli_interfaces::srv::SetDockingPoint::Request;
static_assert(kDockYawSourcePreserve == SetDockReqConsts::PRESERVE);
static_assert(kDockYawSourceRequest == SetDockReqConsts::REQUEST);
static_assert(kDockYawSourceMotion == SetDockReqConsts::MOTION);

/// printf-style std::string, so ONE text feeds both the log line and the
/// service response — the caller (the dock calibration) relays it to the
/// operator instead of a generic "rejected".
[[gnu::format(printf, 1, 2)]] std::string FormatRejection(const char* fmt, ...)
{
  char buf[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  return std::string(buf);
}
}  // namespace

std::optional<std::string> MapServerNode::dock_charging_gate_rejection()
{
  // is_charging gate. Refuse if the last /hardware_bridge/status was not
  // charging or is older than dock_set_status_max_age_s_.
  const double max_age = get_parameter("dock_set_status_max_age_s").as_double();
  const double status_age = (last_status_time_.nanoseconds() == 0)
                                ? std::numeric_limits<double>::infinity()
                                : (now() - last_status_time_).seconds();
  if (status_age > max_age || !last_is_charging_)
  {
    return FormatRejection(
        "robot not detected on dock "
        "(is_charging=%s, status_age=%.1fs, max=%.1fs). "
        "Drive onto the dock and wait for the firmware to report "
        "charging before retrying.",
        last_is_charging_ ? "true" : "false",
        status_age,
        max_age);
  }
  return std::nullopt;
}

std::optional<std::string> MapServerNode::dock_gps_accuracy_gate_rejection()
{
  // GPS accuracy gate. RTK-Fixed reports σ ≈ 3 mm; RTK-Float is 10-50 cm.
  // Reject when σ(xx) or σ(yy) breaches the threshold, or when /gps/pose_cov
  // is stale (driver dead, USB unplugged, datum unset). Only the COVARIANCE
  // and the age of that topic are read — never its position, which is
  // lever-arm-corrected with the (on the dock: pinned) fused yaw.
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
    return std::string(
        "no /gps/pose_cov sample yet "
        "(navsat_to_absolute_pose_node not running, or no GPS fix).");
  }
  const double gps_age = (now() - gps_time).seconds();
  if (gps_age > max_age)
  {
    return FormatRejection(
        "/gps/pose_cov stale (age %.2fs > %.2fs). "
        "Wait for the GPS feed to refresh.",
        gps_age,
        max_age);
  }
  const double sigma_xx = std::sqrt(std::max(gps_snap->pose.covariance[0], 0.0));
  const double sigma_yy = std::sqrt(std::max(gps_snap->pose.covariance[7], 0.0));
  const double sigma_max = std::max(sigma_xx, sigma_yy);
  if (sigma_max > max_acc)
  {
    return FormatRejection(
        "GPS not accurate enough (σ_max=%.3f m > %.3f m). "
        "Achieve RTK-Fixed before retrying.",
        sigma_max,
        max_acc);
  }
  return std::nullopt;
}

std::optional<std::string> MapServerNode::average_recent_dock_antenna(Enu& mean,
                                                                      size_t& sample_count)
{
  std::lock_guard<std::mutex> lk(recent_gps_antenna_mutex_);
  // Prune by age HERE too, not only when a new sample arrives: the
  // subscription only trims the window as it pushes, and it only pushes
  // RTK-Fixed epochs. Under the dock canopy the receiver can sit at Float, so
  // nothing is pushed, nothing is trimmed, and the deque would otherwise keep
  // serving samples that are minutes old.
  const rclcpp::Time t_now = now();
  while (!recent_gps_antenna_enu_.empty() &&
         (t_now - std::get<0>(recent_gps_antenna_enu_.front())).seconds() >
             dock_set_gps_avg_window_s_)
  {
    recent_gps_antenna_enu_.pop_front();
  }
  if (recent_gps_antenna_enu_.size() < dock_set_gps_avg_min_samples_)
  {
    return FormatRejection(
        "only %zu RTK-Fixed /gps/fix sample(s) taken on the dock in the "
        "last %.1f s (need >= %zu) to average the dock antenna position. "
        "Wait for more RTK-Fixed GPS updates.",
        recent_gps_antenna_enu_.size(),
        dock_set_gps_avg_window_s_,
        dock_set_gps_avg_min_samples_);
  }
  mean = Enu{};
  for (const auto& [t, east, north] : recent_gps_antenna_enu_)
  {
    (void)t;
    mean.east += east;
    mean.north += north;
  }
  sample_count = recent_gps_antenna_enu_.size();
  const double n = static_cast<double>(sample_count);
  mean.east /= n;
  mean.north /= n;
  return std::nullopt;
}

std::optional<std::string> MapServerNode::resolve_gps_lever_arm()
{
  // Resolve the GPS lever arm from TF — mirrors navsat_to_absolute_pose_node's
  // own resolution, which this node cannot reach into (separate process).
  // Retried on every capture until URDF/TF is up. Fail closed rather than
  // silently treat an unresolved lever arm as (0, 0): that would apply NO
  // correction at all and store the ANTENNA position as the dock.
  if (lever_arm_known_)
  {
    return std::nullopt;
  }
  try
  {
    auto tf = tf_buffer_->lookupTransform("base_footprint", "gps_link", tf2::TimePointZero);
    lever_arm_x_ = tf.transform.translation.x;
    lever_arm_y_ = tf.transform.translation.y;
    lever_arm_known_ = true;
    return std::nullopt;
  }
  catch (const tf2::TransformException& ex)
  {
    return FormatRejection(
        "GPS lever arm not yet resolved from TF "
        "(base_footprint→gps_link: %s). Wait for robot_state_publisher "
        "to come up and retry.",
        ex.what());
  }
}

void MapServerNode::on_capture_dock_antenna(const std_srvs::srv::Trigger::Request::SharedPtr,
                                            std_srvs::srv::Trigger::Response::SharedPtr res)
{
  // Step 1 of the dock calibration's persistence: take the RAW antenna mean
  // while the robot is seated on the dock, charging and RTK-Fixed — the one
  // moment all three hold. Nothing is written: the mean waits in memory for
  // the motion yaw (set_docking_point use_pending_antenna), because without a
  // trustworthy yaw the antenna cannot be turned into a base_footprint
  // position. No yaw is involved here, hence no fused-yaw gate.
  auto reject = [&](const std::string& why)
  {
    res->success = false;
    res->message = why;
    RCLCPP_WARN(get_logger(), "capture_dock_antenna rejected: %s", why.c_str());
  };
  if (const auto why = dock_charging_gate_rejection())
  {
    reject(*why);
    return;
  }
  if (const auto why = dock_gps_accuracy_gate_rejection())
  {
    reject(*why);
    return;
  }
  Enu mean;
  size_t sample_count = 0;
  if (const auto why = average_recent_dock_antenna(mean, sample_count))
  {
    reject(*why);
    return;
  }
  pending_antenna_.valid = true;
  pending_antenna_.antenna = mean;
  pending_antenna_.sample_count = sample_count;
  pending_antenna_.stamp_s = now().seconds();
  res->success = true;
  res->message = FormatRejection(
      "dock antenna captured: (%.3f, %.3f) over %zu RTK-Fixed "
      "sample(s); pending for %.0f s",
      mean.east,
      mean.north,
      sample_count,
      dock_antenna_capture_ttl_s_);
  RCLCPP_INFO(get_logger(), "%s (not persisted — waiting for a motion yaw).", res->message.c_str());
}

void MapServerNode::on_set_docking_point(
    const mowgli_interfaces::srv::SetDockingPoint::Request::SharedPtr req,
    mowgli_interfaces::srv::SetDockingPoint::Response::SharedPtr res)
{
  // Whatever happens, tell the caller what is stored NOW.
  res->stored_pose = docking_pose_;
  auto reject = [&](const std::string& why)
  {
    res->success = false;
    res->message = why;
    RCLCPP_WARN(get_logger(), "set_docking_point rejected: %s", why.c_str());
  };

  // Which gates apply depends on WHAT is being written — see
  // dock_set_gates.hpp. Everything that captures a position NOW or takes one
  // from the caller keeps all three; only the dock calibration's two MOTION
  // writes (robot necessarily OFF the dock) skip the charging + fused-yaw
  // gates, and neither of those lets the caller supply a position.
  const DockSetGates gates = ResolveDockSetGates(req->use_gps_position,
                                                 req->preserve_position,
                                                 req->use_pending_antenna,
                                                 req->yaw_source);
  if (gates.kind == DockSetKind::INVALID)
  {
    reject(FormatRejection("invalid request: %s", gates.invalid_reason));
    return;
  }
  if (gates.require_existing_pose && !docking_pose_set_)
  {
    reject(
        "preserve_position: no dock position is stored yet, so there is nothing to "
        "preserve. Capture the dock position on the dock first.");
    return;
  }
  if (gates.require_pending_antenna)
  {
    const double now_s = now().seconds();
    switch (ClassifyPendingAntenna(pending_antenna_, now_s, dock_antenna_capture_ttl_s_))
    {
      case PendingAntennaState::ABSENT:
        reject(
            "use_pending_antenna: no dock antenna capture is pending "
            "(~/capture_dock_antenna never succeeded, or its result was already used).");
        return;
      case PendingAntennaState::EXPIRED:
        reject(
            FormatRejection("use_pending_antenna: the dock antenna capture is %.0f s old "
                            "(max %.0f s) — the robot may have been moved since.",
                            now_s - pending_antenna_.stamp_s,
                            dock_antenna_capture_ttl_s_));
        pending_antenna_ = PendingAntennaCapture{};
        return;
      case PendingAntennaState::USABLE:
        break;
    }
  }

  // Sequence of gates protecting dock_pose accuracy. The operator forces
  // the EKF to dock_pose at boot via the fusion_graph gauge reset, so a
  // bad calibration leaks straight into the map-frame anchor for every
  // subsequent session. Reject unless ALL conditions hold:
  //   (1) firmware reports is_charging=true (robot physically on dock)
  //   (2) GPS sample fresh and σ(xy) ≤ dock_set_gps_accuracy_max_m_
  //   (3) EKF yaw converged on the recent rolling window
  // (the two off-dock MOTION writes keep (2) only — see dock_set_gates.hpp)
  if (gates.require_charging)
  {
    if (const auto why = dock_charging_gate_rejection())
    {
      reject(*why);
      return;
    }
  }
  if (gates.require_gps_accuracy)
  {
    if (const auto why = dock_gps_accuracy_gate_rejection())
    {
      reject(*why);
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
  if (gates.require_yaw_convergence)
  {
    std::lock_guard<std::mutex> lk(recent_yaws_mutex_);
    if (recent_yaws_.size() < min_samples)
    {
      reject(
          FormatRejection("only %zu yaw samples in the last %.1f s (need >= %zu). "
                          "Wait for the EKF to receive more updates.",
                          recent_yaws_.size(),
                          window_s,
                          min_samples));
      return;
    }
    // Yaw is a wrapping angle in (-π, π]; a linear mean/variance blows up near
    // ±π (a stable west-facing dock straddles the wrap point), which would
    // always reject convergence. Use circular statistics: circular std =
    // sqrt(-2 ln R) where R is the mean resultant length.
    double sum_cos = 0.0;
    double sum_sin = 0.0;
    for (const auto& [t, y] : recent_yaws_)
    {
      sum_cos += std::cos(y);
      sum_sin += std::sin(y);
    }
    const double n = static_cast<double>(recent_yaws_.size());
    const double resultant = std::hypot(sum_cos, sum_sin) / n;
    const double std_dev = std::sqrt(-2.0 * std::log(std::max(resultant, 1e-12)));
    if (std_dev > threshold_rad)
    {
      reject(FormatRejection(
          "EKF yaw not converged (std %.3f° > threshold %.3f° over %.1f s, %zu samples). "
          "Drive the robot 1 m forward to anchor heading from COG, then retry.",
          std_dev * 180.0 / M_PI,
          threshold_rad * 180.0 / M_PI,
          window_s,
          recent_yaws_.size()));
      return;
    }
  }

  // Position capture mode, selected by req->use_gps_position:
  //   true  — "capture current robot position": the robot is physically
  //           seated on the dock, so take the dock POSITION from the averaged
  //           RAW antenna projection (/gps/fix vs datum, lever-arm-corrected
  //           once below), NOT from req->docking_pose (which the GUI fills from
  //           the fused /odometry/filtered_map). While charging, fusion_graph
  //           gauge-resets the fused pose onto the EXISTING dock_pose, so
  //           capturing it would just re-store the old value — a calibration
  //           that can never correct a stale dock_pose. The GPS projection is
  //           free of that circularity; averaging kills the ~1-3 cm RTK jitter.
  //   false — manual map-drag / settings edit: the operator specified the
  //           location directly, so use req->docking_pose.position as given.
  //
  // Orientation (task #45, from #44's circularity trace): the SAME gauge-
  // reset circularity that poisons the fused POSITION while charging also
  // poisons the fused YAW — fusion_graph pins the fused yaw to the EXISTING
  // dock_pose_yaw via a tight (~2°) unary factor plus a periodic gauge
  // reset, so req->docking_pose.orientation is just as circular as its
  // position would be in the use_gps_position=true case (confirmed #44: no
  // independent yaw source is observable at standstill on the dock). A
  // few-degree bad heading could therefore never self-correct via a live
  // GUI re-capture, and #44 traced this as the likely cause of a consistent
  // ~10cm-right dock miss (yaw_err × ~1.5m approach distance). PRESERVE the
  // existing dock_pose_yaw for a live GPS capture — it comes from the
  // motion-derived, RTK-gated writers (calibrate_imu_yaw_node's reverse
  // maneuver + per-undock CalibrateHeadingFromUndock, both confirmed NOT
  // circular in #40/#44). For a manual map-drag (use_gps_position=false)
  // the operator is explicitly setting orientation by hand, so honor the
  // request as before — that path was never circular (no fused-yaw readback
  // involved).
  // ── Orientation capture — keyed on req->yaw_source, DECOUPLED from the
  // position source. PRESERVE keeps the existing persisted dock_pose_yaw
  // (safe default: a live on-dock capture cannot observe an independent yaw —
  // see the circularity note above). REQUEST honours the request quaternion
  // (manual map-drag / settings edit — never circular). MOTION takes the
  // RTK-gated, COG-derived yaw_rad from the one-click dock-calibration action
  // — the ONLY non-circular way to correct a stale dock heading (task #45).
  //
  // The new pose is assembled in a LOCAL and committed only once nothing can
  // reject any more. It used to be written straight into docking_pose_ before
  // the antenna checks: a rejected GPS capture then left the in-memory dock
  // position at the request's (0, 0) — unpublished and unpersisted, but the
  // next yaw-only write would have "preserved" and persisted exactly that.
  using SetDockReq = mowgli_interfaces::srv::SetDockingPoint::Request;
  const geometry_msgs::msg::Pose stored_pose = docking_pose_;
  geometry_msgs::msg::Pose new_pose = req->docking_pose;  // manual: position (+ REQUEST yaw)
  const char* yaw_src_desc = "request";
  switch (req->yaw_source)
  {
    case SetDockReq::MOTION:
      new_pose.orientation.x = 0.0;
      new_pose.orientation.y = 0.0;
      new_pose.orientation.z = std::sin(req->yaw_rad * 0.5);
      new_pose.orientation.w = std::cos(req->yaw_rad * 0.5);
      yaw_src_desc = "motion (COG-derived)";
      break;
    case SetDockReq::REQUEST:
      // orientation already assigned from req->docking_pose above.
      yaw_src_desc = "request (manual)";
      break;
    case SetDockReq::PRESERVE:
    default:
      new_pose.orientation = stored_pose.orientation;
      yaw_src_desc = "preserved (existing dock_pose_yaw)";
      break;
  }
  const double final_yaw = 2.0 * std::atan2(new_pose.orientation.z, new_pose.orientation.w);

  if (req->use_gps_position || req->use_pending_antenna)
  {
    // The dock POSITION comes from the averaged RAW (yaw-independent) GPS
    // antenna position, lever-arm-corrected ONCE with final_yaw — whatever
    // THIS call is about to persist (PRESERVE: the existing dock_pose_yaw;
    // REQUEST: the manually-specified yaw; MOTION: req->yaw_rad, a fresh
    // independently-measured heading). It is never taken from the fused pose,
    // /gps/absolute_pose or /gps/pose_cov: on the dock the fused pose is
    // gauge-pinned onto the STORED dock pose, and those topics are
    // lever-arm-corrected per sample with that pinned yaw (issue #446), so
    // they would re-save the old value or bias it silently. Correcting once
    // with THIS call's own final yaw makes the result right regardless of
    // what the fused yaw was doing during capture — in particular a fresh
    // MOTION yaw fixes a stale stored yaw even though every /gps/fix sample
    // was received before that yaw was known.
    //
    //   use_gps_position    — average the on-dock samples NOW (robot seated).
    //   use_pending_antenna — use the mean ~/capture_dock_antenna took while
    //                         the robot WAS seated; the robot is off the dock
    //                         now, which is the only place the yaw exists.
    Enu antenna_mean;
    size_t antenna_sample_count = 0;
    if (req->use_pending_antenna)
    {
      antenna_mean = pending_antenna_.antenna;
      antenna_sample_count = pending_antenna_.sample_count;
    }
    else if (const auto why = average_recent_dock_antenna(antenna_mean, antenna_sample_count))
    {
      reject(*why);
      return;
    }
    if (const auto why = resolve_gps_lever_arm())
    {
      reject(*why);
      return;
    }

    const Enu base = DockBaseFromAntenna(antenna_mean, final_yaw, lever_arm_x_, lever_arm_y_);
    new_pose.position.x = base.east;
    new_pose.position.y = base.north;
    new_pose.position.z = 0.0;
    RCLCPP_INFO(get_logger(),
                "Docking point captured (%s): raw antenna GPS averaged to (%.3f, %.3f) over "
                "%zu sample(s), lever-arm-corrected with yaw=%.3f rad (source: %s) to "
                "base position (%.3f, %.3f); previously stored (%.3f, %.3f) — "
                "Δ=(%.3f, %.3f) m.",
                req->use_pending_antenna ? "pending on-dock capture" : "live on-dock capture",
                antenna_mean.east,
                antenna_mean.north,
                antenna_sample_count,
                final_yaw,
                yaw_src_desc,
                base.east,
                base.north,
                stored_pose.position.x,
                stored_pose.position.y,
                base.east - stored_pose.position.x,
                base.north - stored_pose.position.y);
  }
  else if (req->preserve_position)
  {
    // Yaw-only MOTION write: the stored X/Y stays exactly as it is. The
    // position is kept HERE, on the server that owns it, rather than echoed
    // back by the caller.
    new_pose.position = stored_pose.position;
    RCLCPP_INFO(get_logger(),
                "Docking point YAW-ONLY update: stored position (%.3f, %.3f) preserved, "
                "yaw %.3f -> %.3f rad (source: %s).",
                new_pose.position.x,
                new_pose.position.y,
                2.0 * std::atan2(stored_pose.orientation.z, stored_pose.orientation.w),
                final_yaw,
                yaw_src_desc);
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Docking point set from request position (manual): (%.3f, %.3f). "
                "Orientation source: %s.",
                new_pose.position.x,
                new_pose.position.y,
                yaw_src_desc);
  }

  // Commit. A pending antenna capture is single-use.
  docking_pose_ = new_pose;
  if (req->use_pending_antenna)
  {
    pending_antenna_ = PendingAntennaCapture{};
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
  std::string persist_warning;
  try
  {
    const double yaw_rad =
        2.0 * std::atan2(docking_pose_.orientation.z, docking_pose_.orientation.w);
    if (!mowgli_interfaces::robot_yaml_scalar::UpdateDockPose(
            robot_yaml_path_, docking_pose_.position.x, docking_pose_.position.y, yaw_rad))
    {
      RCLCPP_WARN(get_logger(),
                  "Could not persist dock pose to %s — file missing or "
                  "not writable. Pose still applied in-memory.",
                  robot_yaml_path_.c_str());
      // success stays true (the pose IS live in this process, and that is the
      // long-standing contract for the manual GUI path) — but say so in the
      // response, so a caller whose whole point is persistence (the dock
      // calibration) can tell the operator instead of reporting "saved".
      persist_warning = "applied in-memory only: could not write " + robot_yaml_path_;
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Persisted dock pose to %s: (%.3f, %.3f) yaw=%.3f rad",
                  robot_yaml_path_.c_str(),
                  docking_pose_.position.x,
                  docking_pose_.position.y,
                  yaw_rad);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(get_logger(),
                "Failed to persist dock pose to %s: %s",
                robot_yaml_path_.c_str(),
                ex.what());
    persist_warning =
        "applied in-memory only: could not write " + robot_yaml_path_ + " (" + ex.what() + ")";
  }

  // Rebuild the lethal dock body + corridor carve-out so they follow the new
  // dock pose immediately. The constructor builds these polygons only at boot
  // (and only when a non-zero dock pose was persisted), so without this a fresh
  // install — or any GUI re-placement — left the dock structure unmarked (or
  // pinned to the old pose) until the node restarted, letting coverage plan a
  // blade-on swath through the physical dock.
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    rebuild_dock_polygons();
    masks_dirty_ = true;
  }
  apply_area_classifications();

  res->success = true;
  res->message = persist_warning;
  res->stored_pose = docking_pose_;
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
  // Path 1: accept a PENDING proposal (a wheel-slip dig report). Until this
  // call the proposal is inert — not in the keepout mask, not a NO_GO cell,
  // not a coverage hole, not in areas.dat. Accepting it is the operator action
  // that APPLIES it (mask + classification + replan) and persists it.
  if (req->pending_id != 0)
  {
    // REFUSE while the robot stands where the resulting keepout would be.
    // Accepting turns polygon + keepout_obstacle_margin into LETHAL cells, and
    // right after a dig the robot is only ~0.2-0.3 m from the dig point: the
    // keepout would sit under it and every plan from its own pose would be
    // START_OCCUPIED — the 2026-09-10 / 2026-09-17 strand, re-created by one
    // click. Refused rather than deferred: a deferred accept is hidden state
    // that applies itself later, mid-mission, when nobody is looking; a refusal
    // tells the operator exactly what to do and changes nothing.
    if (const auto blocked_m = robot_inside_accepted_band(req->pending_id); blocked_m.has_value())
    {
      std::ostringstream why;
      why << std::fixed << std::setprecision(2)
          << "the robot is standing on this proposal: accepting it now would put the robot "
             "INSIDE the new keepout ("
          << *blocked_m << " m from its edge, " << accept_clearance_m()
          << " m needed) and no path could be planned from there. Let the robot drive away "
             "or send it home, then accept again.";
      res->success = false;
      res->message = why.str();
      RCLCPP_WARN(get_logger(),
                  "promote_obstacle: pending %u refused - %s",
                  req->pending_id,
                  res->message.c_str());
      return;
    }

    const auto area_index = accept_pending_obstacle(req->pending_id, req->name);
    if (!area_index.has_value())
    {
      res->success = false;
      res->message =
          "no pending obstacle with id " + std::to_string(req->pending_id) +
          " (already accepted, discarded, or lost to a restart — proposals are session-scoped)";
      return;
    }

    persist_areas_best_effort("promote_obstacle");

    res->success = true;
    res->message = "pending obstacle " + std::to_string(req->pending_id) +
                   " accepted as a permanent keepout for area " + std::to_string(*area_index);
    RCLCPP_INFO(get_logger(),
                "promote_obstacle: operator accepted pending obstacle %u in area %zu",
                req->pending_id,
                *area_index);
    return;
  }

  // Path 2/3: resolve the polygon source: prefer the request's explicit
  // polygon (free-form draw), fall back to looking up the obstacle id in
  // the most recent /obstacle_tracker/obstacles snapshot.
  const uint8_t source = (req->polygon.points.size() >= 3)
                             ? mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER
                             : mowgli_interfaces::msg::MapObstacleInfo::SOURCE_TRACKER;
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

  if (!apply_promoted_obstacle(req->area_index, poly, req->name, source))
  {
    res->success = false;
    res->message = "promotion rejected (bad area_index, navigation area, or polygon < 3 points)";
    return;
  }

  // Persist immediately so the keepout survives a restart.
  persist_areas_best_effort("promote_obstacle");

  res->success = true;
  res->message =
      "obstacle promoted to permanent keepout for area " + std::to_string(req->area_index);
  RCLCPP_INFO(get_logger(),
              "promote_obstacle: appended polygon (%zu points) to area %u",
              poly.points.size(),
              req->area_index);
}

// ─────────────────────────────────────────────────────────────────────────────
// Wheel-slip dig reports
// ─────────────────────────────────────────────────────────────────────────────

std::optional<size_t> MapServerNode::mowing_area_containing(double x, double y) const
{
  geometry_msgs::msg::Point32 pt;
  pt.x = static_cast<float>(x);
  pt.y = static_cast<float>(y);

  for (size_t i = 0; i < areas_.size(); ++i)
  {
    if (areas_[i].is_navigation_area)
    {
      continue;  // obstacles can only be promoted into mowing areas
    }
    if (point_in_polygon(pt, areas_[i].polygon))
    {
      return i;
    }
  }
  return std::nullopt;
}

void MapServerNode::on_dig_event(mowgli_interfaces::msg::DigEvent::ConstSharedPtr msg)
{
  const double x = msg->position.x;
  const double y = msg->position.y;

  RCLCPP_WARN(get_logger(),
              "Dig reported at (%.2f, %.2f): wheels claimed %.2f m, fused pose moved "
              "%.2f m (sigma %.3f m).",
              x,
              y,
              msg->wheel_distance,
              msg->map_distance,
              msg->position_sigma);

  std::optional<size_t> area_index;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    area_index = mowing_area_containing(x, y);
  }

  if (!area_index.has_value())
  {
    // Digs during transit or docking can happen outside every mowing area.
    // There is no area to attach a proposal to, and inventing one would put
    // an obstacle somewhere the operator never drew a boundary. The
    // stop-and-reverse already happened at the bridge. Log it and move on.
    RCLCPP_INFO(get_logger(),
                "Dig at (%.2f, %.2f) is outside every mowing area - no proposal recorded.",
                x,
                y);
    return;
  }

  // A dig INSIDE a hole that already exists there (accepted, or still
  // proposed) is the same hole: the centroid dedup below only catches reports
  // within kObstacleDedupEpsilonM, and issue #500's three latches spanned
  // 0.13 m.
  {
    geometry_msgs::msg::Point32 dig_pt;
    dig_pt.x = static_cast<float>(x);
    dig_pt.y = static_cast<float>(y);
    std::lock_guard<std::mutex> lock(map_mutex_);
    for (const auto& obs : areas_[*area_index].obstacles)
    {
      if (point_in_polygon(dig_pt, obs.polygon))
      {
        RCLCPP_INFO(get_logger(),
                    "Dig at (%.2f, %.2f) lies inside obstacle/proposal %u ('%s') - no new "
                    "proposal.",
                    x,
                    y,
                    obs.id,
                    obs.name.c_str());
        return;
      }
    }
  }

  // The proposal is sized to the PHYSICAL dig — the two drive-wheel ruts —
  // not to the chassis (internal_helpers.hpp). The body clearance is added
  // separately, and exactly once, when an accepted proposal is applied
  // (keepout band, coverage obstacle_margin).
  const double radius = dig_proposal_radius(dig_proposal_radius_m_, msg->map_distance);
  const geometry_msgs::msg::Polygon poly = dig_proposal_polygon(x, y, radius);

  // The name IS the proposal's evidence: it is what the operator reads in the
  // GUI when deciding whether this inferred dig deserves a permanent hole in
  // their map. Keep it short and factual.
  std::ostringstream label;
  label << std::fixed << std::setprecision(2) << "Dig at (" << x << ", " << y << "): wheels "
        << msg->wheel_distance << " m vs pose " << msg->map_distance << " m, sigma "
        << std::setprecision(3) << msg->position_sigma << " m";

  // PROPOSAL ONLY. Nothing here may touch the keepout mask, the
  // classification layer, the coverage holes or areas.dat: the robot is
  // standing ~0.2-0.3 m from this point, and a keepout stamped under or around
  // it made every transit START_OCCUPIED and stranded the robot mid-lawn
  // (2026-09-10, again 2026-09-17). Issue #500's re-dig loop is prevented in
  // the coverage-following layer instead (mowgli_behavior/dig_skip.hpp), which
  // cannot block planning. A single inferred dig is also weaker evidence than
  // a repeatedly-observed tracker obstacle, and those already need operator
  // sign-off (auto_promote_persistent_obstacles defaults false).
  const auto proposal_id = add_obstacle_proposal(
      *area_index, poly, label.str(), mowgli_interfaces::msg::MapObstacleInfo::SOURCE_DIG);
  if (!proposal_id.has_value())
  {
    RCLCPP_INFO(get_logger(),
                "Dig at (%.2f, %.2f): an obstacle or proposal already covers this spot in area "
                "%zu - no new proposal.",
                x,
                y,
                *area_index);
    return;
  }

  RCLCPP_WARN(get_logger(),
              "Dig proposal %u (radius %.2f m, the wheel ruts) recorded for area %zu at (%.2f, "
              "%.2f). It is NOT applied: no keepout, no coverage hole, nothing saved. Accept or "
              "reject it in the GUI.",
              *proposal_id,
              radius,
              *area_index,
              x,
              y);
}

std::optional<uint32_t> MapServerNode::add_obstacle_proposal(
    size_t area_index,
    const geometry_msgs::msg::Polygon& polygon,
    const std::string& name,
    uint8_t source)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (area_index >= areas_.size() || areas_[area_index].is_navigation_area ||
      polygon.points.size() < 3)
  {
    return std::nullopt;
  }
  // One proposal per spot: repeated digs at the same place (and a dig on a
  // keepout that already exists) must not stack entries in the GUI list.
  if (has_duplicate_obstacle_entry(areas_[area_index].obstacles, polygon, kObstacleDedupEpsilonM))
  {
    return std::nullopt;
  }
  // Deliberately NOT obstacle_polygons_, NOT masks_dirty_, NOT
  // apply_area_classifications, NOT replan_needed: a proposal changes nothing
  // that a planner or a controller can see.
  areas_[area_index].obstacles.push_back(make_obstacle_entry(polygon, name, source, true));
  return areas_[area_index].obstacles.back().id;
}

double MapServerNode::accept_clearance_m() const
{
  // The lethal region of an accepted obstacle is polygon + the mask band
  // (Smac 2D is a point check, the mask is not inflated); one cell of slack
  // for the rasterisation.
  return std::max(keepout_obstacle_margin_m_, 0.0) + resolution_;
}

std::optional<double> MapServerNode::robot_inside_accepted_band(uint32_t pending_id)
{
  if (!have_robot_pose_)
  {
    return std::nullopt;  // no pose yet: nothing to protect, do not block the operator
  }
  const double rx = last_robot_x_;
  const double ry = last_robot_y_;
  geometry_msgs::msg::Point32 robot;
  robot.x = static_cast<float>(rx);
  robot.y = static_cast<float>(ry);

  std::lock_guard<std::mutex> lock(map_mutex_);
  for (const auto& area : areas_)
  {
    for (const auto& obs : area.obstacles)
    {
      if (obs.id != pending_id || !obs.pending)
      {
        continue;
      }
      const double dist = point_in_polygon(robot, obs.polygon)
                              ? 0.0
                              : point_to_polygon_distance(rx, ry, obs.polygon);
      return dist <= accept_clearance_m() ? std::optional<double>(dist) : std::nullopt;
    }
  }
  return std::nullopt;  // unknown id: accept_pending_obstacle reports it
}

std::optional<size_t> MapServerNode::accept_pending_obstacle(uint32_t pending_id,
                                                             const std::string& name)
{
  if (pending_id == 0)
  {
    return std::nullopt;
  }

  std::optional<size_t> accepted_area;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    for (size_t i = 0; i < areas_.size() && !accepted_area.has_value(); ++i)
    {
      for (auto& obs : areas_[i].obstacles)
      {
        if (obs.id != pending_id || !obs.pending)
        {
          continue;
        }
        obs.pending = false;
        if (!name.empty())
        {
          obs.name = name;
        }
        // THIS is the moment the polygon becomes a keepout: same stores as
        // apply_promoted_obstacle.
        obstacle_polygons_.push_back(obs.polygon);
        masks_dirty_ = true;
        accepted_area = i;
        break;
      }
    }
  }
  if (!accepted_area.has_value())
  {
    return std::nullopt;
  }

  // Stamp NO_GO_ZONE and tell planners the map changed (the coverage plan has
  // one more hole from now on). apply_area_classifications locks by itself.
  apply_area_classifications();
  std_msgs::msg::Bool replan_msg;
  replan_msg.data = true;
  replan_needed_pub_->publish(replan_msg);
  return accepted_area;
}

bool MapServerNode::discard_pending_obstacle(uint32_t pending_id)
{
  if (pending_id == 0)
  {
    return false;
  }

  // A proposal was never applied, so dropping it touches neither the mask nor
  // the classification layer and needs no replan.
  std::lock_guard<std::mutex> lock(map_mutex_);
  for (auto& area : areas_)
  {
    auto it = std::find_if(area.obstacles.begin(),
                           area.obstacles.end(),
                           [pending_id](const ObstacleEntry& obs)
                           {
                             return obs.id == pending_id && obs.pending;
                           });
    if (it == area.obstacles.end())
    {
      continue;
    }
    area.obstacles.erase(it);
    return true;
  }
  return false;
}

void MapServerNode::on_discard_obstacle(
    const mowgli_interfaces::srv::ClearObstacle::Request::SharedPtr req,
    mowgli_interfaces::srv::ClearObstacle::Response::SharedPtr res)
{
  if (!discard_pending_obstacle(req->obstacle_id))
  {
    res->success = false;
    res->message = "no pending obstacle with id " + std::to_string(req->obstacle_id) +
                   " (accepted keepouts are part of the saved map - edit the area to "
                   "remove one)";
    return;
  }

  // Nothing to persist: a pending obstacle was never in areas.dat, which is
  // also why a discarded proposal cannot come back after a restart.
  res->success = true;
  res->message = "pending obstacle " + std::to_string(req->obstacle_id) + " discarded";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

MapServerNode::ObstacleEntry MapServerNode::make_obstacle_entry(
    const geometry_msgs::msg::Polygon& polygon,
    const std::string& name,
    uint8_t source,
    bool pending)
{
  ObstacleEntry entry;
  entry.polygon = polygon;
  entry.name = name;
  entry.source = source;
  entry.pending = pending;
  entry.id = next_obstacle_id_++;
  return entry;
}

bool MapServerNode::has_duplicate_obstacle_entry(const std::vector<ObstacleEntry>& existing,
                                                 const geometry_msgs::msg::Polygon& candidate,
                                                 double eps,
                                                 bool include_pending)
{
  std::vector<geometry_msgs::msg::Polygon> polygons;
  polygons.reserve(existing.size());
  for (const auto& obs : existing)
  {
    if (include_pending || !obs.pending)
    {
      polygons.push_back(obs.polygon);
    }
  }
  return has_duplicate_obstacle(polygons, candidate, eps);
}

void MapServerNode::persist_areas_best_effort(const char* context)
{
  if (areas_file_path_.empty())
  {
    return;
  }
  try
  {
    save_areas_to_file(areas_file_path_);
  }
  catch (const std::exception& ex)
  {
    // The live state is already updated, so a failed save is a warning, not
    // a failure: the next save_areas tick retries.
    RCLCPP_WARN(get_logger(), "%s: applied to live state but save failed: %s", context, ex.what());
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

  // Datum stamp (issue #216): records which WGS84 datum the metre
  // coordinates below are anchored to. On load, a stamp that differs from
  // the launch datum triggers migrate_areas_datum, which re-projects every
  // polygon + the dock pose into the new datum frame instead of letting the
  // whole map silently shift across the garden.
  if (is_datum_set(datum_lat_, datum_lon_))
  {
    out << std::fixed << std::setprecision(9) << "datum_lat: " << datum_lat_ << "\n"
        << "datum_lon: " << datum_lon_ << "\n\n";
    out.unsetf(std::ios_base::floatfield);
    out << std::setprecision(6);
  }

  out << "area_count: " << areas_.size() << "\n";
  // mowglinext#637: next_area_id_ is written unconditionally (never
  // optional-on-read like the per-area id line below) so a reader always
  // knows where to resume minting, even for a file with zero areas.
  out << "next_area_id: " << next_area_id_ << "\n\n";

  for (std::size_t i = 0; i < areas_.size(); ++i)
  {
    const auto& area = areas_[i];
    out << "area_" << i << "_name: " << area.name << "\n";
    out << "area_" << i << "_polygon: " << polygon_to_string(area.polygon) << "\n";
    out << "area_" << i << "_is_navigation: " << (area.is_navigation_area ? 1 : 0) << "\n";
    // Always written (unlike the obstacle _name/_source lines above, this
    // is never legitimately absent by the time save runs — on_add_area and
    // load's migration below both guarantee area.id != 0 first). The
    // *reader* still treats it as optional (see load_areas_from_file) so a
    // pre-#637 file written by an older binary keeps loading.
    out << "area_" << i << "_id: " << area.id << "\n";
    // PENDING obstacles (wheel-slip dig proposals) are deliberately NOT
    // written: they are inert until the operator accepts them through
    // ~/promote_obstacle. Count only what we actually write, and keep the
    // written indices contiguous so the loader sees no gaps.
    std::size_t persisted = 0;
    for (const auto& obs : area.obstacles)
    {
      if (!obs.pending)
      {
        ++persisted;
      }
    }
    out << "area_" << i << "_obstacle_count: " << persisted << "\n";
    std::size_t j = 0;
    for (const auto& obs : area.obstacles)
    {
      if (obs.pending)
      {
        continue;
      }
      out << "area_" << i << "_obstacle_" << j << ": " << polygon_to_string(obs.polygon) << "\n";
      // Identity lines are OPTIONAL on read (see load_areas_from_file), so an
      // areas.dat written before #502 still loads. Only emit them when they
      // carry information, keeping the file diff-friendly.
      if (!obs.name.empty())
      {
        out << "area_" << i << "_obstacle_" << j << "_name: " << obs.name << "\n";
      }
      if (obs.source != mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER)
      {
        out << "area_" << i << "_obstacle_" << j << "_source: " << static_cast<int>(obs.source)
            << "\n";
      }
      ++j;
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
    // Optional on read (mowglinext#637): absent in any file saved before
    // this field existed. Left at 0 here; the migration block below mints
    // real ids for every area still at 0 once the whole file is loaded, so
    // it can recover next_area_id_ from the highest id ACTUALLY present
    // first, rather than one area at a time.
    entry.id = static_cast<uint32_t>(get_int(prefix + "_id", 0));

    const int obs_count = get_int(prefix + "_obstacle_count", 0);
    for (int j = 0; j < obs_count; ++j)
    {
      const std::string obs_prefix = prefix + "_obstacle_" + std::to_string(j);
      auto obs_poly = parse_polygon_string(get_str(obs_prefix));
      // Identity is optional: a file written before #502 has no _name/_source
      // lines, and every obstacle in it is by definition an operator-drawn
      // keepout. Never pending — nothing pending is ever written.
      const std::string obs_name = get_str(obs_prefix + "_name");
      const auto obs_source = static_cast<uint8_t>(
          get_int(obs_prefix + "_source", mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER));
      // Dedup on load so pre-existing stacked duplicates collapse to one.
      if (obs_poly.points.size() >= 3 &&
          !has_duplicate_obstacle_entry(entry.obstacles, obs_poly, kObstacleDedupEpsilonM))
      {
        entry.obstacles.push_back(make_obstacle_entry(obs_poly, obs_name, obs_source, false));
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

  // Area-id migration (mowglinext#637): recover next_area_id_ as
  // max(loaded ids) + 1 — same recovery shape obstacle_tracker_node uses
  // for its own persisted next_id_ — then mint fresh ids for any area
  // still at 0: either a pre-#637 file, or (defensively) a legacy in-memory
  // entry that reached here some other way. Re-save immediately so the
  // file is stamped from here on, the same "adopt on first load" shape
  // migrate_areas_datum uses below for the datum stamp.
  {
    uint32_t max_id = 0;
    bool any_unassigned = false;
    for (const auto& area : areas_)
    {
      max_id = std::max(max_id, area.id);
      any_unassigned = any_unassigned || (area.id == 0);
    }
    next_area_id_ = static_cast<uint32_t>(get_int("next_area_id", 0));
    next_area_id_ = std::max(next_area_id_, max_id + 1);
    if (any_unassigned)
    {
      for (auto& area : areas_)
      {
        if (area.id == 0)
        {
          area.id = next_area_id_++;
        }
      }
      RCLCPP_INFO(get_logger(),
                  "areas file %s has area(s) with no stable id (mowglinext#637) — "
                  "assigning and re-saving.",
                  path.c_str());
      try
      {
        save_areas_to_file(path);
      }
      catch (const std::exception& ex)
      {
        RCLCPP_WARN(get_logger(),
                    "Could not re-save %s with area ids: %s",
                    path.c_str(),
                    ex.what());
      }
    }
  }

  // Datum-change migration (issue #216): if the file was recorded against a
  // different datum than the one this node was launched with, re-project the
  // just-loaded polygons + the dock pose into the new datum frame and
  // re-stamp the file. Runs BEFORE resize_map_to_areas so the grid is sized
  // around the migrated coordinates.
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    migrate_areas_datum(get_double("datum_lat", nan), get_double("datum_lon", nan), path);
  }

  // Resize map to fit new areas and reset masks.
  resize_map_to_areas();
  keepout_filter_info_sent_ = false;
  speed_filter_info_sent_ = false;
  masks_dirty_ = true;
}

void MapServerNode::migrate_areas_datum(double file_datum_lat,
                                        double file_datum_lon,
                                        const std::string& path)
{
  namespace wgs84 = mowgli_interfaces::wgs84;

  if (!is_datum_set(datum_lat_, datum_lon_))
  {
    // No site datum configured yet (fresh install, sim without GPS) —
    // nothing to anchor the metres against; leave the file as-is.
    return;
  }

  const bool file_has_stamp = std::isfinite(file_datum_lat) && std::isfinite(file_datum_lon) &&
                              is_datum_set(file_datum_lat, file_datum_lon);
  if (!file_has_stamp)
  {
    // Pre-#216 file (recorded before stamping existed). Its metres are by
    // definition anchored to the datum currently in effect — adopt it by
    // re-saving with a stamp so the NEXT datum change migrates from the
    // correct baseline.
    RCLCPP_INFO(get_logger(),
                "areas file %s has no datum stamp — stamping it with the current "
                "datum (%.9f, %.9f).",
                path.c_str(),
                datum_lat_,
                datum_lon_);
    try
    {
      save_areas_to_file(path);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(),
                  "Could not re-save %s with datum stamp: %s",
                  path.c_str(),
                  ex.what());
    }
    return;
  }

  if (std::abs(datum_lat_ - file_datum_lat) < kDatumMatchEpsilonDeg &&
      std::abs(datum_lon_ - file_datum_lon) < kDatumMatchEpsilonDeg)
  {
    return;  // Same site anchor — nothing to migrate.
  }

  // The datum moved (operator relocated the base / re-centred the datum).
  // Re-project every persisted metre coordinate: old-ENU → WGS84 → new-ENU,
  // the exact inverse/forward of the localizer's projection, so each vertex
  // keeps pointing at the same physical spot in the garden.
  auto reproject_polygon = [&](geometry_msgs::msg::Polygon& poly)
  {
    for (auto& pt : poly.points)
    {
      double east = static_cast<double>(pt.x);
      double north = static_cast<double>(pt.y);
      wgs84::ReprojectEnu(file_datum_lat, file_datum_lon, datum_lat_, datum_lon_, east, north);
      pt.x = static_cast<float>(east);
      pt.y = static_cast<float>(north);
    }
  };

  for (auto& area : areas_)
  {
    reproject_polygon(area.polygon);
    for (auto& obstacle : area.obstacles)
    {
      reproject_polygon(obstacle.polygon);
    }
  }

  // Where the old datum origin lands in the new frame == the translation
  // every point just underwent (to first order) — logged so an operator can
  // sanity-check the move against how far they physically moved the base.
  double shift_east = 0.0;
  double shift_north = 0.0;
  wgs84::ReprojectEnu(
      file_datum_lat, file_datum_lon, datum_lat_, datum_lon_, shift_east, shift_north);

  // The dock pose rides with the map. Yaw is unchanged — both old and new
  // frames are north-aligned ENU, so a datum move is a pure translation.
  if (docking_pose_set_)
  {
    double east = docking_pose_.position.x;
    double north = docking_pose_.position.y;
    wgs84::ReprojectEnu(file_datum_lat, file_datum_lon, datum_lat_, datum_lon_, east, north);
    docking_pose_.position.x = east;
    docking_pose_.position.y = north;

    const double yaw_rad =
        2.0 * std::atan2(docking_pose_.orientation.z, docking_pose_.orientation.w);
    if (!mowgli_interfaces::robot_yaml_scalar::UpdateDockPose(
            robot_yaml_path_, docking_pose_.position.x, docking_pose_.position.y, yaw_rad))
    {
      RCLCPP_WARN(get_logger(),
                  "Datum migration: could not persist migrated dock pose to %s — "
                  "file missing or not writable. Pose migrated in-memory only.",
                  robot_yaml_path_.c_str());
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = docking_pose_;
    docking_pose_pub_->publish(pose_msg);

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      rebuild_dock_polygons();
      masks_dirty_ = true;
    }
  }

  // Re-stamp the file so the migration runs exactly once per datum change.
  try
  {
    save_areas_to_file(path);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(),
                 "Datum migration applied in-memory but re-saving %s FAILED: %s — "
                 "the migration will re-run (idempotently) on next load.",
                 path.c_str(),
                 ex.what());
  }

  RCLCPP_WARN(get_logger(),
              "Datum changed (%.9f, %.9f) → (%.9f, %.9f): re-projected %zu area(s) "
              "and %s dock pose by (%.3f, %.3f) m so the map stays anchored to the "
              "physical garden (issue #216).",
              file_datum_lat,
              file_datum_lon,
              datum_lat_,
              datum_lon_,
              areas_.size(),
              docking_pose_set_ ? "the" : "no",
              shift_east,
              shift_north);
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
      if (obstacle.pending)
      {
        continue;  // a proposal is not applied: its cells stay LAWN
      }
      grid_map::Polygon obs_gm;
      for (const auto& pt : obstacle.polygon.points)
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
