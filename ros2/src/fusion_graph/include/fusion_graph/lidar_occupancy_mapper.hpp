// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Georeferenced 2D occupancy grid built from LiDAR scans at TRUSTED poses.
// Pure — no ROS / GTSAM / Beluga — so it is unit-testable, same shape as
// slip_window.hpp and scan_match_dedup.hpp.
//
// Role in the LiDAR map anchor: while RTK is Fixed the fused pose is
// centimetre-accurate, so every scan can be ray-cast into a map-frame grid
// with confidence. When RTK degrades (under a tree, a terrace — exactly where
// the structure worth matching against lives) the particle filter localises
// against THIS grid. The literature is unanimous that matching against an
// accumulated map beats matching against a single stored scan in partial
// structure: a wall or a trunk seen two hundred times is a landmark; the same
// wall in one isolated scan rarely clears an inlier threshold.
//
// Log-odds Bayesian update, fixed extent centred on the map origin (the GNSS
// datum), clamped so a cell can recover after a transient hit (a person, the
// operator). Unknown cells stay unknown until a ray crosses them.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace fusion_graph
{

struct LidarOccupancyMapperParams
{
  double resolution_m = 0.10;
  double half_extent_m = 40.0;  // grid covers [-half_extent, +half_extent] in x and y
  double log_odds_hit = 0.85;  // ≈ p(occ|hit) 0.70
  double log_odds_miss = -0.40;  // ≈ p(occ|miss) 0.40
  double log_odds_min = -2.0;  // clamp so a cell can recover
  double log_odds_max = 3.5;
  double occupied_threshold = 0.85;  // log-odds above which a cell is exported as occupied
  double free_threshold = -0.40;  // log-odds below which a cell is exported as free
  double max_range_m = 12.0;  // beams beyond this are not inserted
};

// Exported grid in the ROS trinary convention: 0 free, 100 occupied, -1 unknown.
struct ExportedOccupancyGrid
{
  double resolution_m = 0.0;
  double origin_x = 0.0;  // world x of cell (0,0)'s lower-left corner
  double origin_y = 0.0;
  std::size_t width = 0;
  std::size_t height = 0;
  std::vector<int8_t> data;  // row-major, data[y * width + x]
  std::size_t occupied = 0;
  std::size_t free = 0;
};

class LidarOccupancyMapper
{
public:
  explicit LidarOccupancyMapper(const LidarOccupancyMapperParams& p) : p_(p)
  {
    const double extent = 2.0 * p_.half_extent_m;
    n_ = static_cast<std::size_t>(std::max(1.0, std::ceil(extent / p_.resolution_m)));
    log_odds_.assign(n_ * n_, 0.0);
    touched_.assign(n_ * n_, 0);
  }

  std::size_t size() const
  {
    return n_;
  }

  // Insert one scan taken at `pose` (map-frame x, y, yaw). `points_body` are
  // the hit points in the robot body frame. Free space is carved along each
  // beam with Bresenham; the end cell gets a hit unless the beam exceeded
  // max_range (then it is a miss-only ray up to max_range).
  void Insert(double px,
              double py,
              double pyaw,
              const std::vector<std::pair<double, double>>& points_body)
  {
    int cx, cy;
    if (!ToCell(px, py, cx, cy))
      return;
    const double c = std::cos(pyaw), s = std::sin(pyaw);
    for (const auto& [bx, by] : points_body)
    {
      const double range = std::hypot(bx, by);
      if (!(range > 0.05))
        continue;
      const bool hit = range <= p_.max_range_m;
      const double scale = hit ? 1.0 : (p_.max_range_m / range);
      const double wx = px + (c * bx - s * by) * scale;
      const double wy = py + (s * bx + c * by) * scale;
      int ex, ey;
      const bool end_inside = ToCell(wx, wy, ex, ey);
      // Walk the ray even if the end is outside: carve free space up to the border.
      TraceFree(cx, cy, wx, wy, end_inside ? std::make_pair(ex, ey) : std::make_pair(-1, -1));
      if (hit && end_inside)
        Update(ex, ey, p_.log_odds_hit);
    }
    ++inserted_;
  }

  std::size_t inserted_scans() const
  {
    return inserted_;
  }

  ExportedOccupancyGrid Export() const
  {
    ExportedOccupancyGrid g;
    g.resolution_m = p_.resolution_m;
    g.origin_x = -static_cast<double>(n_) * p_.resolution_m / 2.0;
    g.origin_y = g.origin_x;
    g.width = n_;
    g.height = n_;
    g.data.assign(n_ * n_, -1);
    for (std::size_t i = 0; i < n_ * n_; ++i)
    {
      if (!touched_[i])
        continue;
      if (log_odds_[i] >= p_.occupied_threshold)
      {
        g.data[i] = 100;
        ++g.occupied;
      }
      else if (log_odds_[i] <= p_.free_threshold)
      {
        g.data[i] = 0;
        ++g.free;
      }
      else
      {
        g.data[i] = -1;
      }
    }
    return g;
  }

  // Test / diagnostics access.
  struct ScanScore
  {
    int hits = 0;  // returns whose 3×3 cell neighbourhood holds an occupied cell
    int total = 0;  // returns inside max_range and inside the grid
  };

  // Scan-vs-map consistency at a candidate pose: how many of this scan's
  // returns land on (or next to) a mapped occupied cell when placed at
  // (px, py, pyaw). Independent of the particle filter's own weighting —
  // it is the witness that catches a tight-but-wrong cloud.
  ScanScore ScoreScan(double px,
                      double py,
                      double pyaw,
                      const std::vector<std::pair<double, double>>& points_body) const
  {
    ScanScore s;
    const double c = std::cos(pyaw), sn = std::sin(pyaw);
    for (const auto& [bx, by] : points_body)
    {
      const double range = std::hypot(bx, by);
      if (!(range > 0.05) || range > p_.max_range_m)
        continue;
      int ex = 0, ey = 0;
      if (!ToCell(px + c * bx - sn * by, py + sn * bx + c * by, ex, ey))
        continue;
      ++s.total;
      bool hit = false;
      for (int dy = -1; dy <= 1 && !hit; ++dy)
        for (int dx = -1; dx <= 1 && !hit; ++dx)
        {
          const int x = ex + dx, y = ey + dy;
          if (x < 0 || y < 0 || static_cast<std::size_t>(x) >= n_ ||
              static_cast<std::size_t>(y) >= n_)
            continue;
          hit = log_odds_[Index(x, y)] >= p_.occupied_threshold;
        }
      if (hit)
        ++s.hits;
    }
    return s;
  }

  // Import a previously exported grid (same convention as Export: 0 free,
  // 100 occupied, -1 unknown; origin = lower-left corner). Cells are placed
  // by WORLD coordinate, so a grid of a different extent or resolution still
  // lands where it belongs. Occupied cells load at the occupied threshold and
  // free cells at the free threshold: known, but not saturated, so live
  // scans can still overturn them. Counts as one inserted scan.
  void ImportCells(double resolution_m,
                   double origin_x,
                   double origin_y,
                   int width,
                   int height,
                   const std::vector<int8_t>& data)
  {
    if (width <= 0 || height <= 0 || data.size() != static_cast<std::size_t>(width) * height)
      return;
    for (int r = 0; r < height; ++r)
      for (int c = 0; c < width; ++c)
      {
        const int8_t v = data[static_cast<std::size_t>(r) * width + c];
        if (v < 0)
          continue;
        const double wx = origin_x + (c + 0.5) * resolution_m;
        const double wy = origin_y + (r + 0.5) * resolution_m;
        int x = 0, y = 0;
        if (!ToCell(wx, wy, x, y))
          continue;
        const std::size_t i = Index(x, y);
        log_odds_[i] = (v >= 50) ? p_.occupied_threshold : p_.free_threshold;
        touched_[i] = 1;
      }
    ++inserted_;
  }

  double LogOddsAt(double wx, double wy) const
  {
    int x, y;
    if (!ToCell(wx, wy, x, y))
      return 0.0;
    return log_odds_[Index(x, y)];
  }

private:
  bool ToCell(double wx, double wy, int& x, int& y) const
  {
    const double origin = -static_cast<double>(n_) * p_.resolution_m / 2.0;
    const double fx = (wx - origin) / p_.resolution_m;
    const double fy = (wy - origin) / p_.resolution_m;
    if (fx < 0.0 || fy < 0.0)
      return false;
    x = static_cast<int>(fx);
    y = static_cast<int>(fy);
    return x >= 0 && y >= 0 && static_cast<std::size_t>(x) < n_ && static_cast<std::size_t>(y) < n_;
  }
  std::size_t Index(int x, int y) const
  {
    return static_cast<std::size_t>(y) * n_ + static_cast<std::size_t>(x);
  }
  void Update(int x, int y, double delta)
  {
    const std::size_t i = Index(x, y);
    log_odds_[i] = std::clamp(log_odds_[i] + delta, p_.log_odds_min, p_.log_odds_max);
    touched_[i] = 1;
  }
  // Bresenham from the sensor cell towards the world endpoint, marking every
  // cell BEFORE the end cell as a miss. Stops at the grid border.
  void TraceFree(int x0, int y0, double wx, double wy, std::pair<int, int> end)
  {
    const double origin = -static_cast<double>(n_) * p_.resolution_m / 2.0;
    // Clip the endpoint to a far point on the same ray so the walk has a
    // finite integer target even when the true end lies outside the grid.
    int x1 = static_cast<int>(std::floor((wx - origin) / p_.resolution_m));
    int y1 = static_cast<int>(std::floor((wy - origin) / p_.resolution_m));
    const int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    const int limit = static_cast<int>(n_) * 3;  // safety bound on steps
    for (int step = 0; step < limit; ++step)
    {
      if (x < 0 || y < 0 || static_cast<std::size_t>(x) >= n_ || static_cast<std::size_t>(y) >= n_)
        return;
      if (x == end.first && y == end.second)
        return;  // the end cell is handled by the caller (hit)
      if (x == x1 && y == y1)
        return;
      Update(x, y, p_.log_odds_miss);
      const int e2 = 2 * err;
      if (e2 >= dy)
      {
        err += dy;
        x += sx;
      }
      if (e2 <= dx)
      {
        err += dx;
        y += sy;
      }
    }
  }

  LidarOccupancyMapperParams p_;
  std::size_t n_ = 0;
  std::vector<double> log_odds_;
  std::vector<uint8_t> touched_;
  std::size_t inserted_ = 0;
};

}  // namespace fusion_graph
