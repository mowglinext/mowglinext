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
  // Base angle: auto-compute from polygon MBR or use manual override.
  double mow_angle;
  if (std::isnan(mow_angle_override_deg_))
  {
    mow_angle = compute_optimal_mow_angle(poly);
  }
  else
  {
    mow_angle = mow_angle_override_deg_ * M_PI / 180.0;
  }
  // Then apply the operator-tunable offset (constant across all areas) and
  // the per-area increment (multiplied by the area index, so successive
  // areas can be rotated to break up the cut pattern). Both come from
  // mowgli_robot.yaml via map_server_node parameters.
  const double offset_rad = mow_angle_offset_deg_ * M_PI / 180.0;
  const double increment_rad =
      mow_angle_increment_deg_ * static_cast<double>(area_index) * M_PI / 180.0;
  mow_angle += offset_rad + increment_rad;
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
  double inset = strip_boundary_margin_m_;
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

  for (double x = inner_min_x + mower_width_ / 2; x <= inner_max_x; x += mower_width_)
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

      if (y_hi - y_lo < mower_width_)
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

  // Count remaining strips
  ensure_strip_layout(req->area_index);
  res->strips_remaining = 0;
  if (req->area_index < strip_layouts_.size())
  {
    for (const auto& s : strip_layouts_[req->area_index].strips)
    {
      if (!is_strip_mowed(s))
        res->strips_remaining++;
    }
  }

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

  // Row pitch — same as inter-strip distance: tool_width / mower_width.
  // Falls back to the resolution if mower_width_ wasn't set (sim).
  const double row_pitch = mower_width_ > 1e-3 ? mower_width_ : (resolution_ * 2.0);
  // Step length along u: the resolution gives us per-cell granularity.
  const double step = resolution_;
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
  double end_x = start_x;
  double end_y = start_y;
  std::string reason;
  int cells = 0;
  double walked = 0.0;
  while (walked < cap)
  {
    walk_u += dir * step;
    double cx, cy;
    basis_to_map(B, walk_u, row_v, cx, cy);
    if (is_blocking(cx, cy, reason))
      break;
    end_x = cx;
    end_y = cy;
    ++cells;
    walked += step;
  }
  if (reason.empty())
    reason = walked >= cap ? "max_length" : "row_end";

  out_seg.end_x = end_x;
  out_seg.end_y = end_y;
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

  // Build path: dense linspace from start to end at resolution_ steps.
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = map_frame_;
  const double dx = seg.end_x - seg.start_x;
  const double dy = seg.end_y - seg.start_y;
  const double length = std::hypot(dx, dy);
  const int n_steps = std::max(1, static_cast<int>(std::ceil(length / resolution_)));
  const double seg_yaw = std::atan2(dy, dx);
  for (int i = 0; i <= n_steps; ++i)
  {
    const double t = static_cast<double>(i) / n_steps;
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = seg.start_x + t * dx;
    p.pose.position.y = seg.start_y + t * dy;
    p.pose.position.z = 0.0;
    // Orient each pose along the segment direction so FTC's
    // PRE_ROTATE phase aligns the robot before driving.
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = std::sin(seg_yaw / 2.0);
    p.pose.orientation.w = std::cos(seg_yaw / 2.0);
    path.poses.push_back(p);
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
bool MapServerNode::find_next_segment_public(size_t area_index,
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
                                             bool& out_coverage_complete) const
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
  return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// Path C — mark_segment_blocked + clear_dead_cells handlers
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_mark_segment_blocked(
    const mowgli_interfaces::srv::MarkSegmentBlocked::Request::SharedPtr req,
    mowgli_interfaces::srv::MarkSegmentBlocked::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    return;
  }
  const auto& area = areas_[req->area_index];
  if (area.polygon.points.size() < 3)
  {
    res->success = false;
    return;
  }

  auto& fc = map_[std::string(layers::FAIL_COUNT)];
  auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const float promote = static_cast<float>(dead_promote_threshold_);

  // Track which cells we've already bumped this call so a redundant
  // path that loops over the same cell doesn't get charged twice for
  // a single failure event.
  std::set<std::pair<int, int>> bumped;
  uint32_t bumps = 0;
  uint32_t promotions = 0;

  for (const auto& pose : req->failed_path.poses)
  {
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(pose.pose.position.x);
    pt32.y = static_cast<float>(pose.pose.position.y);
    if (!point_in_polygon(pt32, area.polygon))
      continue;
    grid_map::Position pos(pose.pose.position.x, pose.pose.position.y);
    if (!map_.isInside(pos))
      continue;
    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      continue;
    auto key = std::make_pair(idx(0), idx(1));
    if (!bumped.insert(key).second)
      continue;

    auto t = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
    // Don't bump cells that are already non-mowable (real obstacles
    // get their own handling via diff_and_update_obstacles); we only
    // care about LAWN cells the robot couldn't reach.
    if (t != CellType::LAWN && t != CellType::UNKNOWN && t != CellType::LAWN_DEAD)
      continue;

    fc(idx(0), idx(1)) += 1.0F;
    ++bumps;

    if (fc(idx(0), idx(1)) >= promote && t != CellType::LAWN_DEAD)
    {
      cls(idx(0), idx(1)) = static_cast<float>(CellType::LAWN_DEAD);
      ++promotions;
    }
  }

  if (promotions > 0)
  {
    masks_dirty_ = true;
    RCLCPP_WARN(get_logger(),
                "MarkSegmentBlocked: %u cells bumped, %u promoted to LAWN_DEAD "
                "(area %u).",
                bumps,
                promotions,
                req->area_index);
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "MarkSegmentBlocked: %u cells bumped (area %u).",
                bumps,
                req->area_index);
  }

  res->success = true;
  res->cells_marked_blocked = bumps;
  res->cells_promoted_dead = promotions;
}

void MapServerNode::on_clear_dead_cells(const std_srvs::srv::Trigger::Request::SharedPtr,
                                        std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  auto& fc = map_[std::string(layers::FAIL_COUNT)];
  auto& cls = map_[std::string(layers::CLASSIFICATION)];
  uint32_t cleared = 0;
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    auto t = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
    if (t == CellType::LAWN_DEAD)
    {
      cls((*it)(0), (*it)(1)) = static_cast<float>(CellType::LAWN);
      ++cleared;
    }
    fc((*it)(0), (*it)(1)) = 0.0F;
  }
  masks_dirty_ = true;

  res->success = true;
  res->message = "cleared " + std::to_string(cleared) + " LAWN_DEAD cells";
  RCLCPP_INFO(get_logger(), "ClearDeadCells: %s", res->message.c_str());
}

}  // namespace mowgli_map
