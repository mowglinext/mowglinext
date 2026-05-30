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

// Strip planner + cell-based segment selector + their service handlers
// (get_next_strip, get_next_segment, get_coverage_status,
// mark_segment_blocked, clear_dead_cells). This is the bulk of
// map_server_node's planning logic; algorithms (MBR mow-angle,
// Andrew's monotone-chain hull, dead-cell promotion, blocked-strip
// detection) are unchanged from the previous in-place implementation.

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "mowgli_map/map_server_node.hpp"

namespace mowgli_map
{

// ─────────────────────────────────────────────────────────────────────────────
// Strip planner
// ─────────────────────────────────────────────────────────────────────────────

std::vector<std::pair<double, double>> MapServerNode::convex_hull(
    std::vector<std::pair<double, double>> pts)
{
  auto cross = [](const auto& O, const auto& A, const auto& B)
  {
    return (A.first - O.first) * (B.second - O.second) -
           (A.second - O.second) * (B.first - O.first);
  };

  int n = static_cast<int>(pts.size());
  if (n < 3)
    return pts;

  std::sort(pts.begin(), pts.end());

  std::vector<std::pair<double, double>> hull(2 * n);
  int k = 0;

  // Lower hull
  for (int i = 0; i < n; ++i)
  {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
      k--;
    hull[k++] = pts[i];
  }

  // Upper hull
  for (int i = n - 2, t = k + 1; i >= 0; i--)
  {
    while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
      k--;
    hull[k++] = pts[i];
  }

  hull.resize(k - 1);
  return hull;
}

double MapServerNode::compute_optimal_mow_angle(const geometry_msgs::msg::Polygon& poly)
{
  std::vector<std::pair<double, double>> pts;
  pts.reserve(poly.points.size());
  for (const auto& p : poly.points)
    pts.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));

  auto hull = convex_hull(std::move(pts));
  if (hull.size() < 3)
    return 0.0;

  double best_angle = 0.0;
  double min_perp_extent = 1e9;
  int nh = static_cast<int>(hull.size());

  for (int i = 0; i < nh; ++i)
  {
    int j = (i + 1) % nh;
    double edge_dx = hull[j].first - hull[i].first;
    double edge_dy = hull[j].second - hull[i].second;
    double edge_angle = std::atan2(edge_dy, edge_dx);

    double cos_a = std::cos(-edge_angle);
    double sin_a = std::sin(-edge_angle);

    // Compute bounding box of hull rotated to align this edge with X axis
    double min_y = 1e9, max_y = -1e9;
    for (const auto& [hx, hy] : hull)
    {
      double ry = sin_a * hx + cos_a * hy;
      min_y = std::min(min_y, ry);
      max_y = std::max(max_y, ry);
    }

    double perp_extent = max_y - min_y;
    if (perp_extent < min_perp_extent)
    {
      min_perp_extent = perp_extent;
      best_angle = edge_angle;
    }
  }

  return best_angle;
}

void MapServerNode::ensure_strip_layout(size_t area_index)
{
  if (area_index >= areas_.size())
    return;

  // Grow cache if needed
  if (strip_layouts_.size() <= area_index)
    strip_layouts_.resize(area_index + 1);

  auto& layout = strip_layouts_[area_index];
  if (layout.valid)
    return;

  const auto& area = areas_[area_index];
  const auto& poly = area.polygon;
  if (poly.points.size() < 3)
    return;

  // Navigation-only areas: never generate strips. The planner uses the
  // polygon for transit costmap/keepout but the BT must not pick this
  // area as a mowing target.
  if (area.is_navigation_area)
  {
    layout.valid = true;
    layout.strips.clear();
    return;
  }

  // ── 1. Determine mow angle ────────────────────────────────────────────────
  // Auto-compute from polygon MBR or use manual override.
  double mow_angle;
  if (std::isnan(mow_angle_override_deg_))
  {
    mow_angle = compute_optimal_mow_angle(poly);
  }
  else
  {
    mow_angle = mow_angle_override_deg_ * M_PI / 180.0;
  }
  layout.mow_angle = mow_angle;

  // ── 2. Rotate polygon so optimal strip direction aligns with Y axis ───────
  // Strips currently run along Y (vertical scan). Rotation angle:
  //   rot = π/2 - mow_angle
  // maps the desired strip direction onto the Y axis.
  double rot = M_PI / 2.0 - mow_angle;
  double cos_r = std::cos(rot);
  double sin_r = std::sin(rot);

  int n_pts = static_cast<int>(poly.points.size());
  std::vector<std::pair<double, double>> rotated_pts;
  rotated_pts.reserve(n_pts);
  for (const auto& p : poly.points)
  {
    double rx = cos_r * static_cast<double>(p.x) - sin_r * static_cast<double>(p.y);
    double ry = sin_r * static_cast<double>(p.x) + cos_r * static_cast<double>(p.y);
    rotated_pts.emplace_back(rx, ry);
  }

  // Bounding box of rotated polygon (X only — for scan line range)
  double min_x = 1e9, max_x = -1e9;
  for (const auto& [rx, ry] : rotated_pts)
  {
    min_x = std::min(min_x, rx);
    max_x = std::max(max_x, rx);
  }

  // ── 3. Inset and scan ─────────────────────────────────────────────────────
  // Strips must sit INSIDE the headland walk's mower coverage. The
  // headland-walk path is at inset = strip_boundary_margin_m + tool_width/2
  // (see emit_perimeter_ring), so its mower covers from
  // strip_boundary_margin_m to strip_boundary_margin_m + tool_width
  // away from the polygon edge. For the first strip's mower coverage
  // to start at the same depth (continuous coverage with no gap and no
  // strip-extending-past-headland), the strip-row inset is shifted in
  // by one full tool_width — strip path at strip_boundary_margin +
  // tool_width, mower then covers strip_boundary_margin + 0.5*tool_width
  // to strip_boundary_margin + 1.5*tool_width. The 0.5*tool_width
  // overlap with the headland is intentional: it absorbs RTK lateral
  // jitter so a slightly off-track strip still tiles seamlessly.
  double inset = strip_boundary_margin_m_ + tool_width_;
  double inner_min_x = min_x + inset;
  double inner_max_x = max_x - inset;

  if (inner_min_x >= inner_max_x)
  {
    layout.valid = true;
    return;
  }

  // Inverse rotation to map strip endpoints back to the original frame.
  // cos(-rot) = cos(rot), sin(-rot) = -sin(rot)
  double cos_back = cos_r;
  double sin_back = -sin_r;

  layout.strips.clear();
  int col = 0;

  for (double x = inner_min_x + tool_width_ / 2; x <= inner_max_x; x += tool_width_)
  {
    // Find Y intersections of vertical line x=const with rotated polygon edges
    std::vector<double> y_intersections;
    for (int i = 0; i < n_pts; ++i)
    {
      int j = (i + 1) % n_pts;
      double x1 = rotated_pts[i].first;
      double y1 = rotated_pts[i].second;
      double x2 = rotated_pts[j].first;
      double y2 = rotated_pts[j].second;

      if ((x1 < x && x2 >= x) || (x2 < x && x1 >= x))
      {
        double t = (x - x1) / (x2 - x1);
        y_intersections.push_back(y1 + t * (y2 - y1));
      }
    }

    if (y_intersections.size() < 2)
    {
      col++;
      continue;
    }

    std::sort(y_intersections.begin(), y_intersections.end());

    // Even-odd fill: pair consecutive intersections [0,1], [2,3], ...
    // This correctly handles concave polygons (L, U shapes) by producing
    // multiple strip segments per scan line instead of spanning the gap.
    for (size_t k = 0; k + 1 < y_intersections.size(); k += 2)
    {
      double y_lo = y_intersections[k] + inset;
      double y_hi = y_intersections[k + 1] - inset;

      if (y_hi - y_lo < tool_width_)
        continue;

      // Rotate strip endpoints back to map frame
      Strip strip;
      strip.start.x = cos_back * x - sin_back * y_lo;
      strip.start.y = sin_back * x + cos_back * y_lo;
      strip.start.z = 0.0;
      strip.end.x = cos_back * x - sin_back * y_hi;
      strip.end.y = sin_back * x + cos_back * y_hi;
      strip.end.z = 0.0;
      strip.column_index = col;
      layout.strips.push_back(strip);
    }
    col++;
  }

  layout.valid = true;
  RCLCPP_INFO(get_logger(),
              "Strip layout for area '%s': %zu strips, mow_angle=%.1f° (%s), "
              "rotated bbox X=[%.2f, %.2f], inner_x=[%.2f, %.2f]",
              area.name.c_str(),
              layout.strips.size(),
              layout.mow_angle * 180.0 / M_PI,
              std::isnan(mow_angle_override_deg_) ? "auto-MBR" : "manual",
              min_x,
              max_x,
              inner_min_x,
              inner_max_x);
  if (!layout.strips.empty())
  {
    const auto& first = layout.strips.front();
    const auto& last = layout.strips.back();
    RCLCPP_INFO(get_logger(),
                "  First strip: (%.2f,%.2f)→(%.2f,%.2f), "
                "Last strip: (%.2f,%.2f)→(%.2f,%.2f)",
                first.start.x,
                first.start.y,
                first.end.x,
                first.end.y,
                last.start.x,
                last.start.y,
                last.end.x,
                last.end.y);
  }
}

bool MapServerNode::is_strip_mowed(const Strip& strip, double threshold_pct) const
{
  // Sample mow_progress along the strip centerline
  double dx = strip.end.x - strip.start.x;
  double dy = strip.end.y - strip.start.y;
  double length = std::hypot(dx, dy);
  if (length < resolution_)
    return true;

  int samples = std::max(3, static_cast<int>(length / resolution_));
  int mowed_count = 0;
  int total_count = 0;

  const auto& progress_layer = map_[std::string(layers::MOW_PROGRESS)];
  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  for (int i = 0; i <= samples; ++i)
  {
    double t = static_cast<double>(i) / samples;
    double px = strip.start.x + t * dx;
    double py = strip.start.y + t * dy;

    grid_map::Position pos(px, py);
    if (!map_.isInside(pos))
      continue;

    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      continue;

    auto cell_type = static_cast<CellType>(static_cast<int>(class_layer(idx(0), idx(1))));
    // Skip cells the robot must never mow: tracked obstacles AND
    // operator/dock-defined exclusion zones. Without the NO_GO_ZONE
    // skip, strips passing over the dock exclusion polygon (or any
    // exclusion drawn inside a mowing area) count as "not mowed"
    // forever — the robot keeps replanning the same strip and never
    // marks the surrounding cells complete.
    if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY ||
        cell_type == CellType::NO_GO_ZONE)
      continue;

    // Check if inside the mowing area polygon
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(px);
    pt32.y = static_cast<float>(py);

    total_count++;
    if (progress_layer(idx(0), idx(1)) >= 0.3f)
      mowed_count++;
  }

  if (total_count == 0)
    return true;  // No cells to mow

  return static_cast<double>(mowed_count) / total_count >= threshold_pct;
}

bool MapServerNode::is_strip_blocked(const Strip& strip, double blocked_threshold) const
{
  // Check if a strip is mostly blocked by obstacles, making it unreachable.
  // A strip with >blocked_threshold fraction of obstacle cells is "frontier".
  double dx = strip.end.x - strip.start.x;
  double dy = strip.end.y - strip.start.y;
  double length = std::hypot(dx, dy);
  if (length < resolution_)
    return false;

  int samples = std::max(3, static_cast<int>(length / resolution_));
  int obstacle_count = 0;
  int total_count = 0;

  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  for (int i = 0; i <= samples; ++i)
  {
    double t = static_cast<double>(i) / samples;
    double px = strip.start.x + t * dx;
    double py = strip.start.y + t * dy;

    grid_map::Position pos(px, py);
    if (!map_.isInside(pos))
      continue;

    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      continue;

    total_count++;
    auto cell_type = static_cast<CellType>(static_cast<int>(class_layer(idx(0), idx(1))));
    if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY)
      obstacle_count++;
  }

  if (total_count == 0)
    return false;

  return static_cast<double>(obstacle_count) / total_count >= blocked_threshold;
}

void MapServerNode::select_nearest_endpoint_strip(const std::vector<Strip>& strips,
                                                  const std::vector<bool>& eligible,
                                                  double robot_x,
                                                  double robot_y,
                                                  int& out_index,
                                                  Strip& out_strip)
{
  out_index = -1;
  double best_dist = std::numeric_limits<double>::infinity();
  bool best_flip = false;

  const int n = static_cast<int>(strips.size());
  for (int i = 0; i < n; ++i)
  {
    if (i >= static_cast<int>(eligible.size()) || !eligible[i])
      continue;

    const auto& s = strips[i];
    const double d_start = std::hypot(s.start.x - robot_x, s.start.y - robot_y);
    const double d_end = std::hypot(s.end.x - robot_x, s.end.y - robot_y);

    const double d = std::min(d_start, d_end);
    if (d < best_dist)
    {
      best_dist = d;
      out_index = i;
      best_flip = (d_end < d_start);
    }
  }

  if (out_index < 0)
    return;

  out_strip = strips[out_index];
  if (best_flip)
    std::swap(out_strip.start, out_strip.end);
}

bool MapServerNode::find_next_unmowed_strip(
    size_t area_index, double robot_x, double robot_y, Strip& out_strip, bool /*prefer_headland*/)
{
  ensure_strip_layout(area_index);

  if (area_index >= strip_layouts_.size() || !strip_layouts_[area_index].valid)
    return false;

  const auto& layout = strip_layouts_[area_index];
  const int n = static_cast<int>(layout.strips.size());
  if (n == 0)
    return false;

  // Grow tracking vector if needed (kept for compatibility / debugging — the
  // selector itself no longer consumes it).
  if (current_strip_idx_.size() <= area_index)
    current_strip_idx_.resize(area_index + 1, -1);

  // Build eligibility mask: a strip is eligible iff it isn't already mowed and
  // isn't blocked by obstacles (>50% obstacle cells — those are treated as
  // frontier and are skipped during planning).
  std::vector<bool> eligible(n, false);
  for (int i = 0; i < n; ++i)
  {
    const auto& strip = layout.strips[i];
    eligible[i] = !is_strip_mowed(strip) && !is_strip_blocked(strip);
  }

  // Pick the eligible strip whose nearest endpoint is closest to the current
  // robot pose, and orient it so the robot enters from that endpoint. This
  // produces a serpentine/boustrophedon order naturally when adjacent strips
  // are eligible (the previously-mowed strip ended at one column edge, so the
  // adjacent strip's matching endpoint is the nearest by ~one swath width)
  // while gracefully handling skipped or partially-blocked strips.
  int picked = -1;
  select_nearest_endpoint_strip(layout.strips, eligible, robot_x, robot_y, picked, out_strip);
  if (picked < 0)
    return false;

  current_strip_idx_[area_index] = picked;
  return true;
}

nav_msgs::msg::Path MapServerNode::strip_to_path(const Strip& strip, size_t /*area_index*/) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = now();

  double dx = strip.end.x - strip.start.x;
  double dy = strip.end.y - strip.start.y;
  double length = std::hypot(dx, dy);
  double yaw = std::atan2(dy, dx);

  // Quaternion from yaw
  double cy = std::cos(yaw / 2);
  double sy = std::sin(yaw / 2);

  int n_poses = std::max(2, static_cast<int>(length / resolution_) + 1);

  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  for (int i = 0; i < n_poses; ++i)
  {
    double t = static_cast<double>(i) / (n_poses - 1);
    double px = strip.start.x + t * dx;
    double py = strip.start.y + t * dy;

    // Skip cells inside obstacles
    grid_map::Position pos(px, py);
    if (map_.isInside(pos))
    {
      grid_map::Index idx;
      if (map_.getIndex(pos, idx))
      {
        auto cell_type = static_cast<CellType>(static_cast<int>(class_layer(idx(0), idx(1))));
        if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY)
          continue;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = px;
    pose.pose.position.y = py;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = cy;
    pose.pose.orientation.z = sy;
    path.poses.push_back(pose);
  }

  return path;
}

void MapServerNode::compute_coverage_stats(size_t area_index,
                                           uint32_t& total,
                                           uint32_t& mowed,
                                           uint32_t& obstacle_cells) const
{
  total = 0;
  mowed = 0;
  obstacle_cells = 0;

  if (area_index >= areas_.size())
    return;

  const auto& area = areas_[area_index];
  const auto& progress_layer = map_[std::string(layers::MOW_PROGRESS)];
  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  // Iterate all cells and check if inside the area polygon
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position pos;
    map_.getPosition(*it, pos);

    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(pos.x());
    pt.y = static_cast<float>(pos.y());

    if (!point_in_polygon(pt, area.polygon))
      continue;

    auto cell_type = static_cast<CellType>(static_cast<int>(class_layer((*it)(0), (*it)(1))));
    if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY)
    {
      obstacle_cells++;
      continue;
    }

    total++;
    if (progress_layer((*it)(0), (*it)(1)) >= 0.3f)
      mowed++;
  }
}

void MapServerNode::on_get_next_strip(
    const mowgli_interfaces::srv::GetNextStrip::Request::SharedPtr req,
    mowgli_interfaces::srv::GetNextStrip::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    res->coverage_complete = false;
    return;
  }

  Strip strip;
  if (!find_next_unmowed_strip(
          req->area_index, req->robot_x, req->robot_y, strip, req->prefer_headland))
  {
    // All strips mowed
    res->success = true;
    res->coverage_complete = true;
    res->coverage_percent = 100.0f;
    res->strips_remaining = 0;
    res->phase = "complete";
    return;
  }

  res->strip_path = strip_to_path(strip, req->area_index);
  res->success = !res->strip_path.poses.empty();
  res->coverage_complete = false;
  res->phase = "interior";

  // Transit goal = first pose of the strip
  if (!res->strip_path.poses.empty())
  {
    res->transit_goal = res->strip_path.poses.front();
  }

  // Coverage stats
  uint32_t total = 0, mowed_cells = 0, obs = 0;
  compute_coverage_stats(req->area_index, total, mowed_cells, obs);
  res->coverage_percent = total > 0 ? 100.0f * mowed_cells / total : 0.0f;

  // Count remaining strips
  uint32_t remaining = 0;
  if (req->area_index < strip_layouts_.size())
  {
    for (const auto& s : strip_layouts_[req->area_index].strips)
    {
      if (!is_strip_mowed(s))
        remaining++;
    }
  }
  res->strips_remaining = remaining;

  RCLCPP_INFO(get_logger(),
              "GetNextStrip: col=%d, %.1f%% coverage, %u strips remaining",
              strip.column_index,
              res->coverage_percent,
              remaining);
}

void MapServerNode::count_lawn_cells(size_t area_index,
                                     uint32_t& total_lawn,
                                     uint32_t& unmowed_lawn) const
{
  total_lawn = 0;
  unmowed_lawn = 0;
  if (area_index >= areas_.size())
    return;

  const auto& area = areas_[area_index];
  const auto& progress_layer = map_[std::string(layers::MOW_PROGRESS)];
  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position pos;
    map_.getPosition(*it, pos);

    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(pos.x());
    pt.y = static_cast<float>(pos.y());
    if (!point_in_polygon(pt, area.polygon))
      continue;

    // Only plain LAWN is mowable work. LAWN_DEAD (unreachable, per the
    // reachability BFS), NO_GO_ZONE, obstacles and UNKNOWN are excluded.
    const auto cell_type = static_cast<CellType>(static_cast<int>(class_layer((*it)(0), (*it)(1))));
    if (cell_type != CellType::LAWN)
      continue;

    total_lawn++;
    if (progress_layer((*it)(0), (*it)(1)) < 0.3f)
      unmowed_lawn++;
  }
}

void MapServerNode::on_get_coverage_status(
    const mowgli_interfaces::srv::GetCoverageStatus::Request::SharedPtr req,
    mowgli_interfaces::srv::GetCoverageStatus::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    return;
  }

  // Navigation-only areas are transit corridors, not lawn — they
  // exist so the planner has a passage between mowing zones, not so
  // the robot tonds them. Report 0 strips remaining and 100% coverage
  // so GetNextUnmowedArea moves on without ever generating a strip
  // through them.
  if (areas_[req->area_index].is_navigation_area)
  {
    res->success = true;
    res->total_cells = 0;
    res->mowed_cells = 0;
    res->obstacle_cells = 0;
    res->coverage_percent = 100.0f;
    res->strips_remaining = 0;
    return;
  }

  compute_coverage_stats(req->area_index, res->total_cells, res->mowed_cells, res->obstacle_cells);
  res->coverage_percent =
      res->total_cells > 0 ? 100.0f * res->mowed_cells / res->total_cells : 0.0f;

  // Area-completion = remaining work that GetNextUnmowedArea keys on
  // (strips_remaining > 0 → keep planning). Two signals are combined so we
  // neither dock early nor never dock:
  //   (1) STRIPS: the planned coverage passes. is_strip_mowed's DEFAULT 0.2
  //       threshold flagged a strip "mowed" at 20 % centerline coverage, so
  //       the count drained to 0 at ~50 % real coverage and the robot docked
  //       at ~30 % mowed (field 2026-05-30). Use a realistic 0.85 threshold.
  //   (2) CELLS: actual reachable-LAWN coverage. The blade never reaches
  //       every edge cell, so completion is a high FRACTION of reachable
  //       LAWN (>= kCoverageDoneFrac), not zero unmowed cells.
  // The area is complete only when BOTH say done. The no-progress guard in
  // GetNextUnmowedArea retires a genuinely-stuck remainder, so requiring
  // both can never loop forever.
  ensure_strip_layout(req->area_index);
  uint32_t strips_unmowed = 0;
  if (req->area_index < strip_layouts_.size())
  {
    for (const auto& s : strip_layouts_[req->area_index].strips)
    {
      if (!is_strip_mowed(s, 0.85))
        strips_unmowed++;
    }
  }

  uint32_t total_lawn = 0, unmowed_lawn = 0;
  count_lawn_cells(req->area_index, total_lawn, unmowed_lawn);
  constexpr float kCoverageDoneFrac = 0.90f;  // accept the last ~10 % of edges
  const bool cells_done =
      total_lawn == 0 ||
      unmowed_lawn <= static_cast<uint32_t>((1.0f - kCoverageDoneFrac) * total_lawn);

  const bool area_complete = (strips_unmowed == 0) && cells_done;
  // Report a positive count while not complete (GetNextUnmowedArea only tests
  // > 0). Prefer the strip count; fall back to 1 when strips read done but
  // cells do not yet (a region the strip centerlines clipped but didn't cover).
  res->strips_remaining = area_complete ? 0u : (strips_unmowed > 0u ? strips_unmowed : 1u);

  res->success = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Path C — cell-based coverage selector
// ─────────────────────────────────────────────────────────────────────────────
//
// Replaces the strip-based plan with a per-call short segment, picked
// from the live mow_progress + classification grid. Handles obstacles
// in the middle of a row by stopping the segment short, and ignores
// LAWN_DEAD cells entirely (let them decay back to LAWN if the
// blocking obstacle ever clears).
//
// The selector is intentionally simple: walk along prefer_dir from
// the robot's projected row position until we hit a non-mowable cell.
// The BT calls back after each segment, so we pick up obstacle changes
// observed during the previous segment automatically — no need for
// the planner itself to subscribe to obstacle updates.

namespace
{
struct RowBasis
{
  // Unit vector along the mowing row (the direction strips run).
  double ux, uy;
  // Unit vector across rows (perpendicular).
  double vx, vy;
};

// Project (x, y) onto the row basis. u = along-row, v = across-row.
inline void project_to_basis(const RowBasis& b, double x, double y, double& u, double& v)
{
  u = x * b.ux + y * b.uy;
  v = x * b.vx + y * b.vy;
}

// Inverse projection: from (u, v) basis coords back to map (x, y).
inline void basis_to_map(const RowBasis& b, double u, double v, double& x, double& y)
{
  x = u * b.ux + v * b.vx;
  y = u * b.uy + v * b.vy;
}

inline double wrap_pi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

inline RowBasis make_basis(double prefer_dir_yaw)
{
  RowBasis b;
  b.ux = std::cos(prefer_dir_yaw);
  b.uy = std::sin(prefer_dir_yaw);
  b.vx = -b.uy;
  b.vy = b.ux;
  return b;
}

// Find a bypass arc around the obstacle that just blocked the row
// march. Tries both lateral sides (+v and -v at lateral_offset), picks
// the one that returns to the row sooner. Returns true on success and
// fills via points + u-resume position; returns false if neither side
// is viable (offset cell blocked, both sides hit further obstacles, or
// obstacle's u-extent exceeds max_bypass_u_length).
//
// Path layout when successful (all in basis coords):
//   row segment up to (u_entry - dir·step, v_row)        [already in start..end]
//   via 1: (u_entry - dir·step, v_offset)                — lateral move out
//   via 2: (u_resume,            v_offset)                — march at offset
//   via 3: (u_resume,            v_row)                   — lateral return
//   row continues from u_resume (caller advances walk_u and re-enters loop)
//
// Scoring: total path length on the arc (lateral·2 + along-u length).
// The lateral component is identical between sides so the picker
// effectively chooses the side that resumes the row at the smaller
// |u_resume - u_entry| — which is exactly "shorter-resume side".
template <typename MowableUnmowedFn, typename BlockingFn>
inline bool find_bypass_arc(const RowBasis& b,
                                 double u_entry,
                                 double v_row,
                                 double dir,
                                 double lateral_offset,
                                 double max_bypass_u_length,
                                 double step,
                                 MowableUnmowedFn is_mowable_unmowed,
                                 BlockingFn is_blocking,
                                 std::vector<std::pair<double, double>>& out_via,
                                 double& out_u_resume,
                                 double& out_bypass_length)
{
  struct Candidate
  {
    int side;
    double u_resume;
    double via_in_x, via_in_y;
    double via_out_x, via_out_y;
    double via_resume_x, via_resume_y;
    double length;
  };
  Candidate best{};
  bool have_best = false;

  // Try both sides; keep the shorter one.
  for (int side : {+1, -1})
  {
    const double v_offset = v_row + static_cast<double>(side) * lateral_offset;

    // The offset cell at the same u as the obstacle entry must itself
    // be clear — otherwise we can't even sidestep onto the offset
    // row.
    {
      double cx;
      double cy;
      basis_to_map(b, u_entry, v_offset, cx, cy);
      std::string r;
      if (is_blocking(cx, cy, r))
        continue;
    }

    // March along u at v_offset; resume the original row as soon as
    // (u, v_row) becomes mowable+unmowed AND not blocked. Bail out if
    // the offset row itself hits an obstacle (this side is unviable;
    // the obstacle wraps around).
    double u = u_entry;
    bool resumed = false;
    while (std::fabs(u - u_entry) < max_bypass_u_length)
    {
      u += dir * step;
      double offset_x;
      double offset_y;
      basis_to_map(b, u, v_offset, offset_x, offset_y);
      std::string r;
      if (is_blocking(offset_x, offset_y, r))
        break;

      double row_x;
      double row_y;
      basis_to_map(b, u, v_row, row_x, row_y);
      // Resume: the row cell past the obstacle is clear AND not yet
      // mowed (so it's worth resuming). If the row has been mowed on
      // the far side already, we keep marching at v_offset until we
      // either find an unmowed row cell or hit our budget — that
      // way the bypass doesn't leave the offset row prematurely on a
      // long obstacle.
      if (!is_blocking(row_x, row_y, r) && is_mowable_unmowed(row_x, row_y))
      {
        resumed = true;
        break;
      }
    }
    if (!resumed)
      continue;

    Candidate c{};
    c.side = side;
    c.u_resume = u;
    // u_entry is the last clean cell along the row. The bypass pivots
    // off that cell: lateral move to (u_entry, v_offset), forward
    // march to (u_resume, v_offset), lateral return to (u_resume,
    // v_row) which is where the row continues.
    basis_to_map(b, u_entry, v_offset, c.via_in_x, c.via_in_y);
    basis_to_map(b, u, v_offset, c.via_out_x, c.via_out_y);
    basis_to_map(b, u, v_row, c.via_resume_x, c.via_resume_y);
    c.length = std::fabs(u - u_entry) + 2.0 * lateral_offset;
    if (!have_best || c.length < best.length)
    {
      best = c;
      have_best = true;
    }
  }

  if (!have_best)
    return false;

  out_via.clear();
  out_via.emplace_back(best.via_in_x, best.via_in_y);
  out_via.emplace_back(best.via_out_x, best.via_out_y);
  out_via.emplace_back(best.via_resume_x, best.via_resume_y);
  out_u_resume = best.u_resume;
  out_bypass_length = best.length;
  return true;
}

}  // namespace

bool MapServerNode::find_next_segment(size_t area_index,
                                      double robot_x,
                                      double robot_y,
                                      double robot_yaw,
                                      double prefer_dir_yaw,
                                      bool boustrophedon,
                                      double max_segment_length_m,
                                      SegmentResult& out_seg) const
{
  out_seg = SegmentResult{};

  if (area_index >= areas_.size())
    return false;
  const auto& area = areas_[area_index];
  if (area.is_navigation_area)
  {
    out_seg.coverage_complete = true;
    return true;
  }
  const auto& poly = area.polygon;
  if (poly.points.size() < 3)
    return false;

  // Row pitch — same as inter-strip distance: tool_width / tool_width.
  // Falls back to the resolution if tool_width_ wasn't set (sim).
  const double row_pitch = tool_width_ > 1e-3 ? tool_width_ : (resolution_ * 2.0);
  // Step length along u: the resolution gives us per-cell granularity.
  const double step = resolution_;

  // ── 0. Headland-first emission ──────────────────────────────────────
  // Before the boustrophedon row planner runs, emit a single perimeter
  // pass so the robot mows the area's outer ring first. That ring
  // becomes the lane the robot pivots into at every subsequent
  // strip-end headland turn — without it, every U-turn happens right
  // at the polygon edge where the controller can't safely rotate.
  // The pass is offset inward by tool_width/2 + boundary margin so
  // the mower brush kisses the recorded boundary.
  // One-shot per area; reset by area edits / reset_mow_progress.
  if (headland_emitted_areas_.find(area_index) == headland_emitted_areas_.end())
  {
    headland_emitted_areas_.insert(area_index);

    const double inset = strip_boundary_margin_m_ + 0.5 * row_pitch;

    // Build inset perimeter using edge-bisector offset: for each
    // vertex, compute the inward bisector of the two adjacent edges
    // and push the vertex along it by `inset / sin(α/2)` so the
    // resulting offset polygon stays parallel to every original edge
    // at exactly `inset` distance. Centroid-radial shrink (the
    // earlier attempt) over-insets corners on a rectangle — e.g. a
    // 9×6 polygon's SE corner shrinks 1.07 m on x but 0.72 m on y,
    // producing a diamond-ish ring that wanders away from the
    // polygon edge near corners and confuses the row planner.
    //
    // Determine "inward" normal direction: a positive cross product
    // (edge_in × edge_out) indicates a CCW polygon; the inward
    // normal is the left-hand 90° rotation of each edge. We average
    // the two edge normals at each vertex to get the bisector.
    auto signed_area = [&]() -> double
    {
      double a = 0.0;
      const size_t n = poly.points.size();
      for (size_t i = 0; i < n; ++i)
      {
        const auto& p0 = poly.points[i];
        const auto& p1 = poly.points[(i + 1) % n];
        a += static_cast<double>(p0.x) * static_cast<double>(p1.y) -
             static_cast<double>(p1.x) * static_cast<double>(p0.y);
      }
      return 0.5 * a;
    };
    const double sign = signed_area() >= 0.0 ? 1.0 : -1.0;  // +1 = CCW, inward = left

    std::vector<std::pair<double, double>> inset_ring;
    const size_t n_v = poly.points.size();
    inset_ring.reserve(n_v + 1);
    for (size_t i = 0; i < n_v; ++i)
    {
      const auto& prev = poly.points[(i + n_v - 1) % n_v];
      const auto& curr = poly.points[i];
      const auto& next = poly.points[(i + 1) % n_v];

      // Edge vectors and their lengths.
      const double e1x = curr.x - prev.x;
      const double e1y = curr.y - prev.y;
      const double e2x = next.x - curr.x;
      const double e2y = next.y - curr.y;
      const double l1 = std::hypot(e1x, e1y);
      const double l2 = std::hypot(e2x, e2y);
      if (l1 < 1e-6 || l2 < 1e-6)
      {
        inset_ring.emplace_back(curr.x, curr.y);
        continue;
      }
      // Inward unit normals (left-hand rotation × sign).
      const double n1x = -sign * e1y / l1;
      const double n1y = sign * e1x / l1;
      const double n2x = -sign * e2y / l2;
      const double n2y = sign * e2x / l2;
      // Bisector and inset distance along it.
      double bx = n1x + n2x;
      double by = n1y + n2y;
      const double bl = std::hypot(bx, by);
      if (bl < 1e-6)
      {
        // Straight 180° vertex (degenerate); just offset by n1.
        inset_ring.emplace_back(curr.x + inset * n1x, curr.y + inset * n1y);
        continue;
      }
      bx /= bl;
      by /= bl;
      // sin(α/2) = (n1·b + n2·b) / 2 = (bl)/2 since b = (n1+n2)/bl
      const double sin_half = 0.5 * bl;
      // Clamp to avoid huge offsets at near-zero-angle vertices.
      const double scale = inset / std::max(sin_half, 0.2);
      inset_ring.emplace_back(curr.x + bx * scale, curr.y + by * scale);
    }

    if (inset_ring.size() >= 3)
    {
      // Pick the inset vertex closest to the robot as the headland
      // start, walking the ring in whichever direction is the shorter
      // approach. Via points are the remaining vertices in order; the
      // path-builder in on_get_next_segment will linearly interpolate
      // the ring corners at `resolution_` granularity, giving FTC /
      // DWB a continuous headland strip.
      size_t start_idx = 0;
      double best_d = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < inset_ring.size(); ++i)
      {
        const double dx2 = inset_ring[i].first - robot_x;
        const double dy2 = inset_ring[i].second - robot_y;
        const double d2 = dx2 * dx2 + dy2 * dy2;
        if (d2 < best_d)
        {
          best_d = d2;
          start_idx = i;
        }
      }

      // Pick direction: forward (next vertex) vs backward — whichever
      // matches the robot's current yaw better, so the entry into
      // the ring doesn't require an immediate 180° pivot.
      const auto& v_start = inset_ring[start_idx];
      const auto& v_fwd = inset_ring[(start_idx + 1) % inset_ring.size()];
      const auto& v_bwd =
          inset_ring[(start_idx + inset_ring.size() - 1) % inset_ring.size()];
      const double yaw_fwd =
          std::atan2(v_fwd.second - v_start.second, v_fwd.first - v_start.first);
      const double yaw_bwd =
          std::atan2(v_bwd.second - v_start.second, v_bwd.first - v_start.first);
      const bool reverse =
          std::fabs(wrap_pi(yaw_bwd - robot_yaw)) <
          std::fabs(wrap_pi(yaw_fwd - robot_yaw));

      out_seg.start_x = v_start.first;
      out_seg.start_y = v_start.second;
      const size_t n = inset_ring.size();
      // Visit all `n-1` other vertices in order, ending at the
      // vertex JUST BEFORE start (so the path is open, not a closed
      // loop). Closing the loop would put the goal pose AT the start
      // pose, and Nav2's goal-checker fires immediately on tick 1
      // because the robot has not moved away from the start yet —
      // the entire perimeter pass collapses to "Reached the goal!"
      // after one cycle and 0.2 % coverage. Leaving the path open
      // forces DWB to actually drive all the way around. The final
      // ~tool_width gap between the path end and the start is
      // closed organically by the brush radius on subsequent strips.
      const size_t n_via = n - 1;  // skip the final wrap-around back to start
      for (size_t k = 1; k <= n_via; ++k)
      {
        const size_t i = reverse ? (start_idx + n - k) % n : (start_idx + k) % n;
        out_seg.via_points.emplace_back(inset_ring[i].first, inset_ring[i].second);
      }
      // End at the LAST distinct vertex visited (= last via point),
      // not back at start.
      out_seg.end_x = out_seg.via_points.back().first;
      out_seg.end_y = out_seg.via_points.back().second;
      // via_points already contains the end point (it's the last
      // vertex in the walk); pop it so the path-builder doesn't draw
      // a zero-length leg from end back to end.
      out_seg.via_points.pop_back();
      out_seg.cell_count = static_cast<int>(n);
      out_seg.termination_reason = "headland_pass";
      // FALSE so the BT routes this through FollowStrip
      // (NavigateThroughPoses with the full via-point list) instead of
      // TransitToStrip (NavigateToPose to just the end pose, which
      // throws away the perimeter walk and lets Smac plan a straight
      // line to the strip start). Headland's whole point is to walk
      // the perimeter first; if we send it as long_transit, Nav2
      // re-plans direct-to-end and the headland is effectively skipped
      // — observed in run 50 as TRANSIT → RETURNING_HOME within 17 s
      // because Nav2 took the shortest path, the BT thought the
      // segment was done after one Nav2 goal, and AreaLoop unwound
      // when GetNextSegment then returned all-cells-mowed.
      out_seg.is_long_transit = false;
      return true;
    }
    // Degenerate polygon (<3 inset vertices after shrink) — fall through
    // to the regular boustrophedon path.
  }
  // Default cap when caller passes 0 — keeps each FollowSegment short
  // enough that obstacle changes get reflected by the next call.
  const double cap = max_segment_length_m > 0.0 ? max_segment_length_m : 3.0;

  const RowBasis B = make_basis(prefer_dir_yaw);

  // ── 1. Robot's row index (snap to nearest row centreline) ────────────
  double r_u, r_v;
  project_to_basis(B, robot_x, robot_y, r_u, r_v);
  const long current_row = std::lround(r_v / row_pitch);

  // ── 2. Layer accessors. We read mowed/classification through the
  //      grid_map at world positions, NOT cell indices, so no
  //      coordinate-system flipping bugs (see CLAUDE.md note 14). ─────
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const auto& prog = map_[std::string(layers::MOW_PROGRESS)];

  // Returns true when (x, y) is mowable AND not yet mowed AND inside
  // the area polygon. Used both to pick a starting cell and to walk
  // along the row.
  auto is_mowable_unmowed = [&](double x, double y) -> bool
  {
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(x);
    pt32.y = static_cast<float>(y);
    if (!point_in_polygon(pt32, poly))
      return false;
    grid_map::Position pos(x, y);
    if (!map_.isInside(pos))
      return false;
    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      return false;
    auto t = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
    if (t == CellType::OBSTACLE_PERMANENT || t == CellType::OBSTACLE_TEMPORARY ||
        t == CellType::NO_GO_ZONE || t == CellType::LAWN_DEAD)
      return false;
    // Live Nav2 costmap — catches obstacles that haven't been promoted to
    // a CellType::OBSTACLE_* yet (e.g. anything obstacle_tracker hasn't
    // persisted). Reads /scan markings directly via the global costmap.
    if (is_costmap_blocked(x, y))
      return false;
    return prog(idx(0), idx(1)) < 0.3f;
  };

  // Returns true when (x, y) hits a hard stop boundary (outside
  // polygon OR in an obstacle / DEAD cell). MOWED cells are NOT a
  // hard stop — we just walk past them.
  auto is_blocking = [&](double x, double y, std::string& reason) -> bool
  {
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(x);
    pt32.y = static_cast<float>(y);
    if (!point_in_polygon(pt32, poly))
    {
      reason = "boundary";
      return true;
    }
    grid_map::Position pos(x, y);
    if (!map_.isInside(pos))
    {
      reason = "boundary";
      return true;
    }
    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
    {
      reason = "boundary";
      return true;
    }
    auto t = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
    if (t == CellType::OBSTACLE_PERMANENT || t == CellType::OBSTACLE_TEMPORARY ||
        t == CellType::NO_GO_ZONE)
    {
      reason = "obstacle";
      return true;
    }
    if (t == CellType::LAWN_DEAD)
    {
      reason = "dead_zone";
      return true;
    }
    // Live Nav2 costmap obstacle (LiDAR /scan), independent of the
    // obstacle_tracker promotion path which is reserved for user-validated
    // persistent obstacles.
    if (is_costmap_blocked(x, y))
    {
      reason = "costmap";
      return true;
    }
    return false;
  };

  // ── 3. Direction along u for the current row ────────────────────────
  // Boustrophedon: alternate per row index. Otherwise +u always.
  // Heuristic override: if the robot is currently facing closer to -u
  // than +u, flip the sign so we don't force a 180° rotation up front.
  double dir = boustrophedon && (current_row & 1L) ? -1.0 : 1.0;
  {
    const double yaw_to_dir = std::atan2(dir * B.uy, dir * B.ux);  // direction of u in world
    if (std::fabs(wrap_pi(robot_yaw - yaw_to_dir)) > M_PI / 2.0)
      dir = -dir;
  }

  // ── 4. Pick a start point on the current row ────────────────────────
  // Snap robot u to the nearest grid step, then march in dir until we
  // find an unmowed cell (we may have just driven over mowed cells).
  const double row_v = static_cast<double>(current_row) * row_pitch;
  double walk_u = std::round(r_u / step) * step;
  bool found_start = false;
  double start_x = robot_x;
  double start_y = robot_y;
  for (int i = 0; i < static_cast<int>(cap / step) + 1; ++i)
  {
    double cx, cy;
    basis_to_map(B, walk_u, row_v, cx, cy);
    if (is_mowable_unmowed(cx, cy))
    {
      start_x = cx;
      start_y = cy;
      found_start = true;
      break;
    }
    // If we cross a hard block before finding any unmowed cell, the
    // current row in this direction is exhausted — fall through to
    // the row-search path.
    std::string reason;
    if (is_blocking(cx, cy, reason))
      break;
    walk_u += dir * step;
  }

  // ── 5. If the current row is exhausted, scan rows for the closest
  //      unmowed reachable cell. Brute-force iteration over the
  //      polygon-bounded grid is fine — typical area is ≤30×30 m at
  //      0.1 m resolution = 90 k cells, traversed once. ────────────
  if (!found_start)
  {
    double best_dist2 = std::numeric_limits<double>::infinity();
    grid_map::Position best_pos(0.0, 0.0);
    long best_row = current_row;
    grid_map::Polygon gm_poly;
    for (const auto& p : poly.points)
      gm_poly.addVertex(grid_map::Position(static_cast<double>(p.x), static_cast<double>(p.y)));
    for (grid_map::PolygonIterator it(map_, gm_poly); !it.isPastEnd(); ++it)
    {
      grid_map::Position p;
      if (!map_.getPosition(*it, p))
        continue;
      if (!is_mowable_unmowed(p.x(), p.y()))
        continue;
      const double dx = p.x() - robot_x;
      const double dy = p.y() - robot_y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_dist2)
      {
        best_dist2 = d2;
        best_pos = p;
        double cu, cv;
        project_to_basis(B, p.x(), p.y(), cu, cv);
        best_row = std::lround(cv / row_pitch);
      }
    }
    if (!std::isfinite(best_dist2))
    {
      // Last-fallback perimeter-ring pass: close the annulus the bypass
      // arcs leave around each user-promoted obstacle. If every ring is
      // already covered, declare the area complete.
      if (try_emit_perimeter_ring(area_index, out_seg))
        return true;
      out_seg.coverage_complete = true;
      return true;
    }
    start_x = best_pos.x();
    start_y = best_pos.y();
    out_seg.is_long_transit = std::sqrt(best_dist2) > 0.5;
    // Recompute direction for this row (boustrophedon snake).
    dir = boustrophedon && (best_row & 1L) ? -1.0 : 1.0;
    // Reset walk position to the chosen cell's u coordinate.
    project_to_basis(B, best_pos.x(), best_pos.y(), walk_u, r_v);
  }

  out_seg.start_x = start_x;
  out_seg.start_y = start_y;

  // ── 6. Walk along the row in dir until a stop condition fires ─────
  //
  // Bypass arcs: when the row hits a discrete obstacle (OBSTACLE_*,
  // costmap blob, or LAWN_DEAD), try to detour around it and resume
  // the row past the obstacle. This is the cleaning-robot behaviour:
  // the robot mows the row, encounters a tree, hugs around it on
  // whichever side gets back to the row sooner, and continues. Cells
  // along the arc get marked mowed organically as the robot drives,
  // so the obstacle ends up surrounded by a one-tool-width annulus
  // of mowed grass — the queued perimeter pass closes the remaining
  // gap.
  //
  // Hard-stop reasons that DON'T trigger bypass:
  //   * "boundary" — outside the area polygon. No detour exists, by
  //     definition. End the segment so the next call picks a new row.
  //
  // The bypass is rejected (and the row terminates as before) when:
  //   * neither lateral side is reachable (offset cell itself blocked);
  //   * the obstacle's u-extent exceeds bypass_max_length_m_ — at that
  //     scale it's a wall, not a discrete obstacle, and the next-row
  //     scan will pick up the cells on the other side cleanly;
  //   * resuming the row past the bypass exit would require crossing
  //     more obstacles or leaving the polygon.
  const double bypass_lateral_offset = std::max(
      0.5 * chassis_width_m_ + bypass_safety_margin_m_, row_pitch);
  double end_x = start_x;
  double end_y = start_y;
  std::string reason;
  int cells = 0;
  double walked = 0.0;
  std::vector<std::pair<double, double>> via_points;
  while (walked < cap)
  {
    const double next_u = walk_u + dir * step;
    double cx, cy;
    basis_to_map(B, next_u, row_v, cx, cy);
    if (is_blocking(cx, cy, reason))
    {
      // Boundaries don't get a bypass attempt — outside the polygon
      // means no continuation by definition.
      if (reason == "boundary")
        break;

      // Try a bypass arc around this obstacle. The helper returns the
      // u-position past the obstacle if successful, plus the via
      // points to insert (in map frame). On failure (neither side
      // viable), it leaves via_local empty and we honour the original
      // hard stop.
      std::vector<std::pair<double, double>> via_local;
      double u_resume = 0.0;
      double bypass_length = 0.0;
      const double bypass_budget_remaining = std::max(0.0, cap - walked);
      const bool bypassed = find_bypass_arc(B,
                                            walk_u,
                                            row_v,
                                            dir,
                                            bypass_lateral_offset,
                                            std::min(bypass_max_length_m_, bypass_budget_remaining),
                                            step,
                                            is_mowable_unmowed,
                                            is_blocking,
                                            via_local,
                                            u_resume,
                                            bypass_length);
      if (!bypassed)
        break;

      // Commit the arc: append corner points, advance the row cursor
      // past the obstacle. The current end (before the obstacle stays
      // (end_x, end_y) — the via points fan out from there. The new
      // end is the row resume point.
      for (const auto& vp : via_local)
        via_points.push_back(vp);
      walk_u = u_resume;
      basis_to_map(B, walk_u, row_v, end_x, end_y);
      walked += bypass_length;
      reason.clear();
      // Advance one step past the resume point in the next loop iter
      // so the cell at u_resume itself gets validated and marked.
      continue;
    }
    end_x = cx;
    end_y = cy;
    ++cells;
    walked += step;
    walk_u = next_u;
  }
  if (reason.empty())
    reason = walked >= cap ? "max_length" : "row_end";

  // Zero-progress recovery: when the walk in `dir` produces no cells
  // (e.g. the chosen start cell is right at a blocker in that
  // direction), try the opposite direction once before bailing. This
  // catches the common case where the row-search (block at line 1021)
  // picks a cell at the row's boundary edge — boustrophedon picked
  // dir=+1 but the only unmowed run extends the other way, or vice
  // versa. Without it find_next_segment returns success+empty path
  // and the BT escalates to COVERAGE_FAILED_DOCKING after only a few
  // strips even though many rows remain.
  if (cells == 0 && std::hypot(end_x - start_x, end_y - start_y) < 1e-6)
  {
    dir = -dir;
    // Re-snap to the actual start position's u (we may have moved
    // walk_u during row-search in step 5).
    double snap_v;
    project_to_basis(B, start_x, start_y, walk_u, snap_v);
    end_x = start_x;
    end_y = start_y;
    via_points.clear();
    reason.clear();
    walked = 0.0;  // reset budget for the opposite-direction walk
    while (walked < cap)
    {
      const double next_u = walk_u + dir * step;
      double cx, cy;
      basis_to_map(B, next_u, row_v, cx, cy);
      if (is_blocking(cx, cy, reason))
        break;
      end_x = cx;
      end_y = cy;
      ++cells;
      walked += step;
      walk_u = next_u;
    }
    if (reason.empty())
      reason = walked >= cap ? "max_length" : "row_end";

    // Both directions yielded zero progress → the start cell is
    // genuinely isolated. Falling through with cells==0 results in
    // an empty path; the BT's GetSegmentRetry will re-query and the
    // row-search at line ~1021 will pick a different cell next time.
    // We deliberately do NOT declare coverage_complete here — that
    // misfires when the area still has reachable rows further out
    // (run 32 collapsed to 100% complete after 1 strip).
  }

  out_seg.end_x = end_x;
  out_seg.end_y = end_y;
  out_seg.via_points = std::move(via_points);
  out_seg.cell_count = cells;
  out_seg.termination_reason = reason;

  // Long transit when the start point is not the robot's current
  // position (or when we landed on a different row). 0.5 m gap is
  // generous — covers the typical row-to-row jump but not in-row
  // micro-correction.
  if (!out_seg.is_long_transit)
  {
    const double gap = std::hypot(start_x - robot_x, start_y - robot_y);
    out_seg.is_long_transit = gap > 0.5;
  }

  return true;
}

void MapServerNode::on_get_next_segment(
    const mowgli_interfaces::srv::GetNextSegment::Request::SharedPtr req,
    mowgli_interfaces::srv::GetNextSegment::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    return;
  }

  // Navigation-only areas: same short-circuit as on_get_coverage_status.
  if (areas_[req->area_index].is_navigation_area)
  {
    res->success = true;
    res->coverage_complete = true;
    res->coverage_percent = 100.0f;
    res->phase = "complete";
    return;
  }

  SegmentResult seg;
  if (!find_next_segment(req->area_index,
                         req->robot_x,
                         req->robot_y,
                         req->robot_yaw_rad,
                         req->prefer_dir_yaw_rad,
                         req->boustrophedon,
                         req->max_segment_length_m,
                         seg))
  {
    res->success = false;
    return;
  }

  if (seg.coverage_complete)
  {
    res->success = true;
    res->coverage_complete = true;
    res->coverage_percent = 100.0f;
    res->phase = "complete";
    return;
  }

  // Build the path as a polyline through start → via_points... → end.
  // Empty via_points = straight in-row segment (the common case). With
  // bypass arc, via_points carry the arc corners and the path becomes
  // multi-leg, with each leg's poses oriented along its own direction
  // so FTC's PRE_ROTATE aligns at every corner.
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = map_frame_;
  std::vector<std::pair<double, double>> corners;
  corners.reserve(2 + seg.via_points.size());
  corners.emplace_back(seg.start_x, seg.start_y);
  for (const auto& vp : seg.via_points)
    corners.push_back(vp);
  corners.emplace_back(seg.end_x, seg.end_y);
  for (size_t i = 0; i + 1 < corners.size(); ++i)
  {
    const double sx = corners[i].first;
    const double sy = corners[i].second;
    const double ex = corners[i + 1].first;
    const double ey = corners[i + 1].second;
    const double dx = ex - sx;
    const double dy = ey - sy;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6)
      continue;
    const int n_steps = std::max(1, static_cast<int>(std::ceil(length / resolution_)));
    const double leg_yaw = std::atan2(dy, dx);
    // First leg includes its starting pose; subsequent legs skip step 0
    // because that point is the previous leg's terminal pose, already
    // pushed.
    const int j_start = (i == 0) ? 0 : 1;
    for (int j = j_start; j <= n_steps; ++j)
    {
      const double t = static_cast<double>(j) / n_steps;
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = sx + t * dx;
      p.pose.position.y = sy + t * dy;
      p.pose.position.z = 0.0;
      p.pose.orientation.x = 0.0;
      p.pose.orientation.y = 0.0;
      p.pose.orientation.z = std::sin(leg_yaw / 2.0);
      p.pose.orientation.w = std::cos(leg_yaw / 2.0);
      path.poses.push_back(p);
    }
  }

  res->success = true;
  res->coverage_complete = false;
  res->segment_path = path;
  res->target_cell_pose =
      path.poses.empty() ? geometry_msgs::msg::PoseStamped() : path.poses.back();
  res->is_long_transit = seg.is_long_transit;
  res->termination_reason = seg.termination_reason;
  res->phase = seg.is_long_transit ? "transit" : "interior";

  // Coverage stats — reuse the strip planner's per-area accumulator
  // since cell semantics are identical (mow_progress >= 0.3 = mowed).
  uint32_t total = 0, mowed_cells = 0, obs = 0;
  compute_coverage_stats(req->area_index, total, mowed_cells, obs);
  res->coverage_percent = total > 0 ? 100.0f * mowed_cells / total : 0.0f;

  // dead_cells_count: walk the area polygon, count LAWN_DEAD cells.
  // Cheap (one polygon iter per call) and the GUI uses it as a
  // session-quality indicator.
  uint32_t dead = 0;
  grid_map::Polygon gm_poly;
  for (const auto& p : areas_[req->area_index].polygon.points)
    gm_poly.addVertex(grid_map::Position(static_cast<double>(p.x), static_cast<double>(p.y)));
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  for (grid_map::PolygonIterator it(map_, gm_poly); !it.isPastEnd(); ++it)
  {
    auto t = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
    if (t == CellType::LAWN_DEAD)
      ++dead;
  }
  res->dead_cells_count = dead;

  // Rough remaining-segments estimate: unmowed cells / cells_per_segment.
  const uint32_t unmowed = total > mowed_cells ? total - mowed_cells - obs - dead : 0;
  const double cells_per_seg = std::max(1.0, 3.0 / resolution_);  // cap=3m default
  res->segments_remaining_estimate =
      static_cast<uint32_t>(std::ceil(static_cast<double>(unmowed) / cells_per_seg));
}

// Test wrapper for find_next_segment — flattens SegmentResult into
// scalar out-params so unit tests can call the selector without
// pulling the struct definition. Caller holds map_mutex_.
bool MapServerNode::find_next_segment_public(
    size_t area_index,
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
    std::vector<std::pair<double, double>>* out_via_points) const
{
  SegmentResult seg;
  const bool ok = find_next_segment(area_index,
                                    robot_x,
                                    robot_y,
                                    robot_yaw,
                                    prefer_dir_yaw,
                                    boustrophedon,
                                    max_segment_length_m,
                                    seg);
  out_start_x = seg.start_x;
  out_start_y = seg.start_y;
  out_end_x = seg.end_x;
  out_end_y = seg.end_y;
  out_cell_count = seg.cell_count;
  out_termination_reason = seg.termination_reason;
  out_is_long_transit = seg.is_long_transit;
  out_coverage_complete = seg.coverage_complete;
  if (out_via_points)
    *out_via_points = seg.via_points;
  return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// Path C — mark_segment_blocked + clear_dead_cells handlers
//
// DEAD redesign (2026-05-07): DEAD cells are now defined as
// "topologically unreachable from the area's seed cell", computed by
// recompute_reachability_for_area() — not by per-cell failure counting.
// mark_segment_blocked is kept as a no-op stub so the existing BT
// MarkBlockedAndSkip branch keeps compiling and returning SUCCESS;
// the BT's WasRecentlyInCollisionStop guard already routes transient
// dynamic-obstacle failures away from this path. clear_dead_cells
// flips every LAWN_DEAD cell back to LAWN as a manual reset.
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_mark_segment_blocked(
    const mowgli_interfaces::srv::MarkSegmentBlocked::Request::SharedPtr req,
    mowgli_interfaces::srv::MarkSegmentBlocked::Response::SharedPtr res)
{
  // No-op under the topological-reachability DEAD model. A controller
  // failure on a reachable cell is by definition transient (the cell
  // IS reachable, otherwise reachability BFS would already have flipped
  // it DEAD). The BT now skips the segment via the cell-walker's next
  // selection without any per-cell mutation here. Kept as a stub for
  // BT XML compatibility — log, return success, no state change.
  RCLCPP_INFO_THROTTLE(get_logger(),
                       *get_clock(),
                       5000,
                       "mark_segment_blocked: no-op under reachability-based DEAD "
                       "(area %u, %zu poses) — cell-walker will pick a different segment",
                       req->area_index,
                       req->failed_path.poses.size());
  res->success = true;
  res->cells_marked_blocked = 0;
  res->cells_promoted_dead = 0;
}

void MapServerNode::on_clear_dead_cells(const std_srvs::srv::Trigger::Request::SharedPtr,
                                        std_srvs::srv::Trigger::Response::SharedPtr res)
{
  uint32_t cleared = 0;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto& cls = map_[std::string(layers::CLASSIFICATION)];
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
    {
      auto t = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
      if (t == CellType::LAWN_DEAD)
      {
        cls((*it)(0), (*it)(1)) = static_cast<float>(CellType::LAWN);
        ++cleared;
      }
    }
    masks_dirty_ = true;
  }
  // Re-run reachability for every area immediately so anything that
  // really IS unreachable goes back to DEAD before the next segment
  // request.
  for (size_t i = 0; i < areas_.size(); ++i)
    recompute_reachability_for_area(i);

  res->success = true;
  res->message = "cleared " + std::to_string(cleared) +
                 " LAWN_DEAD cells (reachability re-run for all areas)";
  RCLCPP_INFO(get_logger(), "ClearDeadCells: %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Reachability BFS (topological DEAD)
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::recompute_reachability_for_area(size_t area_index)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (area_index >= areas_.size())
    return;
  const auto& area = areas_[area_index];
  if (area.is_navigation_area)
    return;
  const auto& poly = area.polygon;
  if (poly.points.size() < 3)
    return;

  auto& cls = map_[std::string(layers::CLASSIFICATION)];

  // Wall predicate. A cell is a wall if any of:
  //   * outside the area polygon (off-area cells aren't candidates anyway)
  //   * classification = OBSTACLE_PERMANENT / OBSTACLE_TEMPORARY / NO_GO_ZONE
  //   * costmap_blocked (live LiDAR obstacle)
  // DOCKING_AREA cells are NOT walls — corridor lawn is reachable
  // through the corridor in the user's spec.
  auto is_wall = [&](double x, double y) -> bool
  {
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(x);
    pt32.y = static_cast<float>(y);
    if (!point_in_polygon(pt32, poly))
      return true;
    grid_map::Position pos(x, y);
    if (!map_.isInside(pos))
      return true;
    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      return true;
    auto t = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
    if (t == CellType::OBSTACLE_PERMANENT || t == CellType::OBSTACLE_TEMPORARY ||
        t == CellType::NO_GO_ZONE)
      return true;
    if (is_costmap_blocked(x, y))
      return true;
    return false;
  };

  // Build the polygon iterator's (r, c) bounding box and a reached[]
  // grid the same shape.
  grid_map::Polygon gm_poly;
  for (const auto& p : poly.points)
    gm_poly.addVertex(grid_map::Position(static_cast<double>(p.x), static_cast<double>(p.y)));

  // Pick a seed: prefer the robot's last-known position inside this area,
  // else the area centroid, else the first non-wall polygon cell. We
  // use map_-scoped grid indices so neighborhood checks stay O(1).
  auto pick_seed = [&]() -> std::optional<grid_map::Index>
  {
    auto try_cell = [&](double x, double y) -> std::optional<grid_map::Index>
    {
      grid_map::Position pos(x, y);
      grid_map::Index idx;
      if (!map_.isInside(pos))
        return std::nullopt;
      if (!map_.getIndex(pos, idx))
        return std::nullopt;
      if (is_wall(x, y))
        return std::nullopt;
      return idx;
    };
    if (auto s = try_cell(last_robot_x_, last_robot_y_))
      return s;
    // Centroid (mean of polygon vertices — fast, good enough for convex
    // and most concave shapes; spiral-search picks up any blocked cells).
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& p : poly.points)
    {
      cx += p.x;
      cy += p.y;
    }
    cx /= static_cast<double>(poly.points.size());
    cy /= static_cast<double>(poly.points.size());
    if (auto s = try_cell(cx, cy)) return s;
    // Spiral around centroid up to ~2 m at resolution_ steps. Accepts
    // the first non-wall cell.
    const double step = std::max(0.05, resolution_);
    for (int ring = 1; ring <= 40; ++ring)
    {
      const double r = ring * step;
      for (int k = 0; k < 12; ++k)
      {
        const double a = (2.0 * M_PI * k) / 12.0;
        if (auto s = try_cell(cx + r * std::cos(a), cy + r * std::sin(a)))
          return s;
      }
    }
    return std::nullopt;
  };
  const auto seed = pick_seed();

  // Walk every cell inside the polygon's bounding box. If no seed
  // exists, every in-polygon non-wall cell becomes DEAD (the area is
  // an island we can't enter at all).
  uint32_t flipped_dead = 0;
  uint32_t flipped_lawn = 0;

  if (!seed.has_value())
  {
    for (grid_map::PolygonIterator it(map_, gm_poly); !it.isPastEnd(); ++it)
    {
      grid_map::Position pos;
      if (!map_.getPosition(*it, pos))
        continue;
      auto t = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
      if (t == CellType::LAWN || t == CellType::UNKNOWN)
      {
        cls((*it)(0), (*it)(1)) = static_cast<float>(CellType::LAWN_DEAD);
        ++flipped_dead;
      }
    }
    if (flipped_dead > 0)
      masks_dirty_ = true;
    RCLCPP_WARN(get_logger(),
                "reachability(area %zu): no seed cell — area entirely unreachable; "
                "%u cells → DEAD",
                area_index,
                flipped_dead);
    return;
  }

  // BFS over 4-connected grid (diagonals add complexity without much
  // benefit — at typical resolutions the planner won't squeeze through
  // single-cell diagonal gaps anyway).
  const int nx = map_.getSize()(0);
  const int ny = map_.getSize()(1);
  std::vector<uint8_t> visited(static_cast<size_t>(nx * ny), 0);
  auto idx_to_flat = [&](int r, int c) -> size_t
  {
    return static_cast<size_t>(r * ny + c);
  };

  std::vector<grid_map::Index> stack;
  stack.reserve(1024);
  stack.push_back(*seed);
  visited[idx_to_flat((*seed)(0), (*seed)(1))] = 1;

  while (!stack.empty())
  {
    grid_map::Index cur = stack.back();
    stack.pop_back();
    for (const auto& d : {std::pair<int, int>{1, 0},
                          std::pair<int, int>{-1, 0},
                          std::pair<int, int>{0, 1},
                          std::pair<int, int>{0, -1}})
    {
      const int rr = cur(0) + d.first;
      const int cc = cur(1) + d.second;
      if (rr < 0 || rr >= nx || cc < 0 || cc >= ny)
        continue;
      const size_t flat = idx_to_flat(rr, cc);
      if (visited[flat])
        continue;
      grid_map::Index nb(rr, cc);
      grid_map::Position pos;
      if (!map_.getPosition(nb, pos))
        continue;
      if (is_wall(pos.x(), pos.y()))
      {
        visited[flat] = 1;  // mark visited to skip subsequent neighbour visits
        continue;
      }
      visited[flat] = 1;
      stack.push_back(nb);
    }
  }

  // Apply the diff. Walk every in-polygon cell:
  //   * reached & currently DEAD          → flip back to LAWN.
  //   * not reached & currently LAWN/UNKN → flip to DEAD.
  //   * not reached & currently DEAD      → leave (already DEAD).
  //   * reached & currently LAWN/UNKN     → leave (already mowable).
  for (grid_map::PolygonIterator it(map_, gm_poly); !it.isPastEnd(); ++it)
  {
    const auto idx = *it;
    const size_t flat = idx_to_flat(idx(0), idx(1));
    auto t = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
    const bool reached = (visited[flat] != 0) && [&]()
    {
      grid_map::Position pos;
      if (!map_.getPosition(idx, pos))
        return false;
      return !is_wall(pos.x(), pos.y());
    }();

    if (reached && t == CellType::LAWN_DEAD)
    {
      cls(idx(0), idx(1)) = static_cast<float>(CellType::LAWN);
      ++flipped_lawn;
    }
    else if (!reached && (t == CellType::LAWN || t == CellType::UNKNOWN))
    {
      cls(idx(0), idx(1)) = static_cast<float>(CellType::LAWN_DEAD);
      ++flipped_dead;
    }
  }

  if (flipped_dead > 0 || flipped_lawn > 0)
  {
    masks_dirty_ = true;
    RCLCPP_INFO(get_logger(),
                "reachability(area %zu): %u → DEAD, %u → LAWN",
                area_index,
                flipped_dead,
                flipped_lawn);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Perimeter-ring pass — close the annulus around user-promoted obstacles
// ─────────────────────────────────────────────────────────────────────────────

bool MapServerNode::try_emit_perimeter_ring(size_t area_index, SegmentResult& out_seg) const
{
  if (area_index >= areas_.size())
    return false;
  const auto& area = areas_[area_index];
  if (area.is_navigation_area)
    return false;

  // Offset = chassis_half_width + safety_margin. Same value the bypass
  // arc uses, so a ring at this distance closes the gap left by an
  // earlier bypass cleanly.
  const double offset = std::max(0.5 * chassis_width_m_ + bypass_safety_margin_m_, tool_width_);
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const auto& prog = map_[std::string(layers::MOW_PROGRESS)];
  const double step = resolution_;

  // Iterate this area's user-promoted obstacles (single source of truth
  // — only entries the operator validated via ~/promote_obstacle).
  for (const auto& obs : area.obstacles)
  {
    if (obs.points.size() < 3)
      continue;

    // Centroid (mean of vertices). For convex obstacles, pushing each
    // vertex outward from the centroid by `offset` produces a ring at
    // the desired clearance. Concave obstacles can yield self-intersection
    // — acceptable here because user-promoted shapes are typically
    // tree-trunks / pots / rocks, not C-shapes.
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& p : obs.points)
    {
      cx += p.x;
      cy += p.y;
    }
    cx /= static_cast<double>(obs.points.size());
    cy /= static_cast<double>(obs.points.size());

    std::vector<std::pair<double, double>> ring;
    ring.reserve(obs.points.size());
    for (const auto& p : obs.points)
    {
      const double dx = p.x - cx;
      const double dy = p.y - cy;
      const double dist = std::hypot(dx, dy);
      if (dist < 1e-3)
        continue;
      const double scale = (dist + offset) / dist;
      ring.emplace_back(cx + dx * scale, cy + dy * scale);
    }
    if (ring.size() < 3)
      continue;

    // Sample cells along the ring perimeter to decide whether this
    // obstacle still needs a perimeter pass. Count valid (in-area,
    // non-blocked) sample points and how many are already mowed —
    // skip if the ring is already mostly covered.
    int total = 0;
    int mowed = 0;
    for (size_t i = 0; i < ring.size(); ++i)
    {
      const auto& a = ring[i];
      const auto& b = ring[(i + 1) % ring.size()];
      const double dx = b.first - a.first;
      const double dy = b.second - a.second;
      const double leg_len = std::hypot(dx, dy);
      const int nsamples = std::max(1, static_cast<int>(leg_len / step));
      for (int j = 0; j <= nsamples; ++j)
      {
        const double t = static_cast<double>(j) / static_cast<double>(nsamples);
        const double x = a.first + t * dx;
        const double y = a.second + t * dy;
        geometry_msgs::msg::Point32 pt32;
        pt32.x = static_cast<float>(x);
        pt32.y = static_cast<float>(y);
        if (!point_in_polygon(pt32, area.polygon))
          continue;
        grid_map::Position pos(x, y);
        if (!map_.isInside(pos))
          continue;
        grid_map::Index idx;
        if (!map_.getIndex(pos, idx))
          continue;
        const auto t_cell = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
        // Skip ring sample points that fall inside any obstacle/keepout/
        // dead/costmap-blocked cell — those would be unmowable anyway,
        // and don't count toward "needs ring" decision.
        if (t_cell == CellType::OBSTACLE_PERMANENT || t_cell == CellType::OBSTACLE_TEMPORARY ||
            t_cell == CellType::NO_GO_ZONE || t_cell == CellType::LAWN_DEAD)
          continue;
        if (is_costmap_blocked(x, y))
          continue;
        ++total;
        if (prog(idx(0), idx(1)) >= 0.3F)
          ++mowed;
      }
    }

    // If the ring sampled out empty (all points unreachable / outside
    // area), don't try to mow it. If >50 % is already mowed, ring done.
    if (total < 3)
      continue;
    if (mowed * 2 >= total)
      continue;

    // Emit the ring as a single segment. via_points are the ring
    // vertices except the first (which is start_*); end_* closes back
    // to start so FTC drives a complete loop. is_long_transit is true
    // because the robot likely needs to travel from its current pose
    // to the ring start — BT will disengage the blade for the move.
    out_seg.start_x = ring[0].first;
    out_seg.start_y = ring[0].second;
    out_seg.via_points.clear();
    out_seg.via_points.reserve(ring.size());
    for (size_t i = 1; i < ring.size(); ++i)
      out_seg.via_points.emplace_back(ring[i].first, ring[i].second);
    // Close the loop.
    out_seg.via_points.emplace_back(ring[0].first, ring[0].second);
    out_seg.end_x = ring[0].first;
    out_seg.end_y = ring[0].second;
    out_seg.cell_count = total;
    out_seg.termination_reason = "perimeter_ring";
    out_seg.is_long_transit = true;
    out_seg.coverage_complete = false;
    return true;
  }

  return false;
}

}  // namespace mowgli_map
