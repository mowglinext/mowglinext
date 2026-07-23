// Copyright 2026 Mowgli Project
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
//
// Pure "detour-and-continue" resume-pose search, factored out of FollowStrip so
// the core decision is unit-testable without ROS, action servers, or a live
// costmap (see coverage_nodes.cpp, test_detour_resume.cpp).
//
// SAFETY-CRITICAL CONTEXT. When the coverage follower (FTCController) meets an
// obstacle it cannot skirt laterally it holds zero velocity, the Nav2 progress
// checker aborts the FollowCoveragePath goal, and FollowStrip used to ABANDON
// the whole swath. Instead we detour: find a clear pose FURTHER ALONG the SAME
// segment, past the obstacle, drive there BLADE-OFF via a Nav2 transit (which
// routes around the obstacle through the costmap), then resume mowing the
// remainder from that pose. This header is only the geometry/policy that decides
// (a) whether the abort really was an obstacle (a lethal cell lies ahead) and
// (b) which forward pose is the clear resume point. It touches no blade/motion
// state; the caller owns the costmap subscription, the blade-off transit, and
// the per-segment detour budget. Anything ambiguous fails safe (no resume →
// caller falls back to the existing abort-to-next-segment behaviour).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// DetourCostmap — a lightweight, ROS-message-free view of an occupancy grid so
// the search is unit-testable without building a nav_msgs/OccupancyGrid. Mirrors
// the OccupancyGrid convention exactly: row-major, data[row * width + col],
// col <-> X (origin_x + (col+0.5)*resolution), row <-> Y. The caller populates
// it from the latest /global_costmap/costmap (both are in the map frame).
// ---------------------------------------------------------------------------
struct DetourCostmap
{
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 0.05;
  uint32_t width = 0;  // cells along X
  uint32_t height = 0;  // cells along Y
  std::vector<int8_t> data;  // OccupancyGrid values: 0..100 cost, -1 unknown

  bool valid() const
  {
    return width > 0 && height > 0 && resolution > 0.0 &&
           data.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }
};

// ---------------------------------------------------------------------------
// DetourResumeCfg — tuning for the forward resume-pose search.
// ---------------------------------------------------------------------------
struct DetourResumeCfg
{
  // The resume pose must be at least this far (euclidean) from the stuck pose so
  // the robot actually clears the obstacle instead of re-hitting it. MUST exceed
  // FollowStrip::kSegmentTransitGap (0.6 m) so reaching the resume pose always
  // triggers the structural blade-off transit rather than a blade-on drive-through.
  double min_skip_dist_m = 0.8;
  // Bounded forward arc-length search. A blockage wider than this yields no clear
  // resume -> the caller falls back (skips the segment) rather than scanning the
  // whole field.
  double max_search_dist_m = 8.0;
  // Robot footprint approximated as a disc of this radius when testing a pose for
  // lethal-cell clearance. Conservative (chassis half-width + margin).
  double footprint_radius_m = 0.25;
  // OccupancyGrid cell value at/above which a cell is treated as lethal. Unknown
  // (-1) is below any positive threshold, so it counts as CLEAR (the global
  // planner handles unmapped space; forbidding resumes near unmapped edges would
  // be too conservative and strand the segment).
  // 100 = TRUE lethal only. The published costmap maps LETHAL(254)->100 and
  // INSCRIBED(253)->99; the inscribed band hugs every keepout wall out to
  // inflation_radius (0.20 m) BY DESIGN, and the outer headland ring rides ON
  // the recorded boundary line — so any threshold <= 99 confirms a phantom
  // obstacle on every outer-ring abort and rejects every ring pose as a resume
  // candidate (whole-sub-path skips; field regression 2026-07-2x).
  int8_t lethal_cost = 100;
  // Radius (m) of the "stalled BESIDE an obstacle" wedge check around the stuck
  // pose. FTC can abort on "failed to make progress" while boxed in next to an
  // obstacle whose lethal cells hug the robot's ACTUAL pose but do NOT lie
  // dead-ahead on the nominal path — so the forward-only obstacle_confirmed scan
  // misses it and the detour never fires (spec Part B). When any lethal cell
  // lies within this radius of the stuck pose, the abort is confirmed
  // obstacle-related and the forward resume search is allowed to pick the first
  // clear pose past min_skip_dist_m even with no dead-ahead blockage. Kept tight
  // (chassis half-width + a small margin) so it fires only on a genuine wedge,
  // never on a localization/goal-checker abort in open space. <= 0 disables.
  double wedge_radius_m = 0.35;
};

// ---------------------------------------------------------------------------
// DetourDecision — result of decideDetour.
// ---------------------------------------------------------------------------
struct DetourDecision
{
  // True when at least one lethal-footprint pose was found ahead of the stuck
  // pose within the search window — i.e. the abort really is obstacle-related.
  // The caller only detours when this is true, so an unrelated failure
  // (localization, goal-checker quirk) never triggers a blade-off detour.
  bool obstacle_confirmed = false;
  // First forward pose that is (a) clear of lethal cells for the footprint,
  // (b) at least min_skip_dist_m from the stuck pose, and (c) beyond the
  // confirmed blockage. Empty when no such pose exists within the search window.
  std::optional<std::size_t> resume_idx;
};

// Returns true when the footprint disc (radius r, centre (x,y)) is clear of
// lethal cells. Cells outside the grid are treated as clear (unmapped). A cell
// value >= lethal (which excludes -1 unknown for any positive lethal) blocks.
inline bool footprintClear(const DetourCostmap& cm, double x, double y, double r, int8_t lethal)
{
  if (!cm.valid())
  {
    return false;  // no map -> cannot assert clearance (caller treats as not clear)
  }
  const double inv = 1.0 / cm.resolution;
  const double r2 = r * r;
  // Bounding box of the disc in cell coordinates, clamped to the grid.
  int64_t col_lo = static_cast<int64_t>(std::floor((x - r - cm.origin_x) * inv));
  int64_t col_hi = static_cast<int64_t>(std::floor((x + r - cm.origin_x) * inv));
  int64_t row_lo = static_cast<int64_t>(std::floor((y - r - cm.origin_y) * inv));
  int64_t row_hi = static_cast<int64_t>(std::floor((y + r - cm.origin_y) * inv));
  col_lo = std::max<int64_t>(col_lo, 0);
  row_lo = std::max<int64_t>(row_lo, 0);
  col_hi = std::min<int64_t>(col_hi, static_cast<int64_t>(cm.width) - 1);
  row_hi = std::min<int64_t>(row_hi, static_cast<int64_t>(cm.height) - 1);
  for (int64_t row = row_lo; row <= row_hi; ++row)
  {
    const double cy = cm.origin_y + (static_cast<double>(row) + 0.5) * cm.resolution;
    for (int64_t col = col_lo; col <= col_hi; ++col)
    {
      const double cx = cm.origin_x + (static_cast<double>(col) + 0.5) * cm.resolution;
      const double dx = cx - x;
      const double dy = cy - y;
      if (dx * dx + dy * dy > r2)
      {
        continue;  // cell centre outside the footprint disc
      }
      const int8_t v =
          cm.data[static_cast<std::size_t>(row) * cm.width + static_cast<std::size_t>(col)];
      if (v >= lethal)
      {
        return false;
      }
    }
  }
  return true;
}

// Pure resume-pose search. Scans `poses` FORWARD from `stuck_idx`, accumulating
// path arc-length, until it exceeds cfg.max_search_dist_m. It:
//   * flags obstacle_confirmed once any scanned pose has a lethal footprint, and
//   * returns the FIRST pose that is footprint-clear, at least min_skip_dist_m
//     (euclidean) from the stuck pose, AND comes after a confirmed blockage
//     (so the resume point is genuinely past the obstacle, not a clear pose
//     reached before it).
// A missing/invalid costmap yields {false, nullopt} -> the caller falls back.
inline DetourDecision decideDetour(const DetourCostmap& cm,
                                   const std::vector<geometry_msgs::msg::PoseStamped>& poses,
                                   std::size_t stuck_idx,
                                   const DetourResumeCfg& cfg)
{
  DetourDecision d;
  if (!cm.valid() || poses.size() < 2 || stuck_idx >= poses.size())
  {
    return d;
  }
  const auto& s = poses[stuck_idx].pose.position;
  // "Stalled BESIDE an obstacle" wedge check (spec Part B): lethal cells hugging
  // the robot's ACTUAL pose confirm the abort is obstacle-related even when
  // nothing lies dead-ahead on the nominal path. Seeding blocked_seen lets the
  // forward search adopt the first clear pose past min_skip_dist_m as the resume
  // point, so a blade-off transit drives the robot OUT of the wedge instead of
  // the coverage loop endlessly re-aborting in place.
  const bool wedged = cfg.wedge_radius_m > 0.0 &&
                      !footprintClear(cm, s.x, s.y, cfg.wedge_radius_m, cfg.lethal_cost);
  bool blocked_seen = wedged;
  double arc = 0.0;
  for (std::size_t i = stuck_idx; i < poses.size(); ++i)
  {
    if (i > stuck_idx)
    {
      const auto& a = poses[i - 1].pose.position;
      const auto& b = poses[i].pose.position;
      arc += std::hypot(b.x - a.x, b.y - a.y);
      if (arc > cfg.max_search_dist_m)
      {
        break;
      }
    }
    const auto& p = poses[i].pose.position;
    const bool clear = footprintClear(cm, p.x, p.y, cfg.footprint_radius_m, cfg.lethal_cost);
    if (!clear)
    {
      blocked_seen = true;
      continue;  // a lethal pose can never be a resume point
    }
    if (!blocked_seen)
    {
      continue;  // still on the clear approach BEFORE the obstacle
    }
    const double euclid = std::hypot(p.x - s.x, p.y - s.y);
    if (euclid >= cfg.min_skip_dist_m)
    {
      d.resume_idx = i;
      break;
    }
  }
  d.obstacle_confirmed = blocked_seen;
  return d;
}

}  // namespace mowgli_behavior
