// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// Implementation of GetRemainingAreaPolygon: returns the area minus the
// already-mowed cells, as a list of disjoint MapArea pieces ready to feed
// to opennav_coverage's ComputeCoveragePath. Boost.Geometry isolated to
// this translation unit to keep the rest of mowgli_map header-clean.

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>

#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/map_types.hpp"

namespace mowgli_map
{

namespace
{
namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
// CCW outer, CW inner — matches both the ROS geometry_msgs/Polygon
// convention used by mowgli_map's area definitions AND F2C's expected
// input (CCW field outer, CW interior cutouts). Boost's default is the
// opposite (CW outer), which silently flipped every Boost.difference
// result polygon and made F2C reject the remaining-area on every
// resume planning call.
using BgPolygon = bg::model::polygon<BgPoint, /*ClockWise=*/false>;
using BgMultiPolygon = bg::model::multi_polygon<BgPolygon>;

/// Convert a ROS Polygon into a Boost ring (closed). Empty input → empty ring.
BgPolygon to_bg_polygon(const geometry_msgs::msg::Polygon& poly)
{
  BgPolygon out;
  for (const auto& p : poly.points)
  {
    bg::append(out.outer(), BgPoint(p.x, p.y));
  }
  if (!poly.points.empty())
  {
    const auto& first = poly.points.front();
    const auto& last = poly.points.back();
    if (first.x != last.x || first.y != last.y)
    {
      bg::append(out.outer(), BgPoint(first.x, first.y));
    }
  }
  bg::correct(out);
  return out;
}

/// Convert a Boost ring back to a ROS Polygon. Drops the duplicated closing
/// vertex so consumers can treat the result as an open ring.
geometry_msgs::msg::Polygon to_ros_polygon(const BgPolygon::ring_type& ring)
{
  geometry_msgs::msg::Polygon out;
  size_t n = ring.size();
  if (n >= 2 && ring.front().x() == ring.back().x() &&
      ring.front().y() == ring.back().y())
  {
    --n;
  }
  out.points.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(ring[i].x());
    pt.y = static_cast<float>(ring[i].y());
    pt.z = 0.0f;
    out.points.push_back(pt);
  }
  return out;
}

}  // namespace

void MapServerNode::on_get_remaining_area_polygon(
    const mowgli_interfaces::srv::GetRemainingAreaPolygon::Request::SharedPtr req,
    mowgli_interfaces::srv::GetRemainingAreaPolygon::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_id >= areas_.size())
  {
    res->success = false;
    res->error = "area_id out of range";
    return;
  }

  const auto& area = areas_[req->area_id];
  if (area.is_navigation_area)
  {
    res->success = false;
    res->error = "area is a navigation area, not mowable";
    return;
  }
  if (area.polygon.points.size() < 3)
  {
    res->success = false;
    res->error = "area polygon has fewer than 3 vertices";
    return;
  }
  if (!map_.exists(std::string(layers::MOW_PROGRESS)))
  {
    res->success = false;
    res->error = "mow_progress layer missing";
    return;
  }

  // 1. Build the original polygon. Outer ring = mowing area; interior
  //    exclusions (operator obstacles + dock body) are punched out.
  //
  //    CRITICAL: an exclusion is added as a HOLE only when it lies strictly
  //    inside the area. When it touches/crosses the boundary (the dock sits
  //    on the lawn edge in the common install) it is instead SUBTRACTED from
  //    the outer ring to carve a clean notch. A hole that touches the outer
  //    boundary makes the polygon non-simple, and F2C's GEOS-backed
  //    TrapezoidalDecomp throws on it (caught as INTERNAL_F2C_ERROR /
  //    code=6) — which silently dropped the entire dock-side chunk of the
  //    area, leaving ~half of it unmowed (field 2026-05-29). Notching keeps
  //    the geometry simple so F2C can plan the whole area.
  //    The corridor is intentionally NOT subtracted: it stays mowable so the
  //    robot can drive across it on the way home (DOCKING_AREA keepout only).
  BgPolygon original = to_bg_polygon(area.polygon);

  // Lobes that a boundary-crossing exclusion pinches off the area. The
  // largest piece stays in `original` (keeping the hole-reattach logic
  // simple); every other above-sliver lobe is collected here and emitted as
  // its own MapArea piece so a large obstacle that partitions the area no
  // longer drops half the garden (see the split branch below).
  std::vector<BgPolygon> split_lobes;

  auto punch_exclusion = [&](BgPolygon excl)
  {
    bg::correct(excl);
    if (excl.outer().empty())
    {
      return;
    }
    BgPolygon outer_only;
    outer_only.outer() = original.outer();
    bg::correct(outer_only);

    BgMultiPolygon overlap;
    bg::intersection(excl, outer_only, overlap);
    if (overlap.empty() || bg::area(overlap) < 1e-6)
    {
      return;  // exclusion lies outside the area — nothing to remove
    }

    if (bg::within(excl, outer_only))
    {
      // Strictly interior — a clean hole.
      original.inners().emplace_back(excl.outer().begin(), excl.outer().end());
      return;
    }

    // Touches/crosses the boundary — subtract to carve a notch so the
    // result stays a simple polygon (no boundary-touching hole).
    BgMultiPolygon notched;
    bg::difference(outer_only, excl, notched);
    if (notched.empty())
    {
      return;  // exclusion covers the whole area — leave original as-is
    }
    size_t best_idx = 0;
    for (size_t ni = 1; ni < notched.size(); ++ni)
    {
      if (bg::area(notched[ni]) > bg::area(notched[best_idx]))
      {
        best_idx = ni;
      }
    }
    const BgPolygon* best = &notched[best_idx];
    if (notched.size() > 1)
    {
      // The notch pinched the area into multiple disjoint pieces. Keep the
      // largest as the working `original` (so the hole-reattach below stays
      // simple) and preserve every other above-sliver lobe in `split_lobes`.
      // Dropping them silently left a whole reachable part of the garden
      // unmowed when a large obstacle split the area (field 2026-05-30) — the
      // GetRemainingAreaPolygon contract is a LIST of pieces and
      // PlanCoverageArea already plans each, so emit them all.
      const double sliver_area_m2 = tool_width_ * tool_width_;
      size_t kept = 1;
      for (size_t ni = 0; ni < notched.size(); ++ni)
      {
        if (ni == best_idx || std::abs(bg::area(notched[ni])) < sliver_area_m2)
        {
          continue;
        }
        split_lobes.push_back(notched[ni]);
        ++kept;
      }
      RCLCPP_WARN(get_logger(),
                  "remaining_polygon: exclusion split the area into %zu pieces; "
                  "keeping %zu (largest %.2f m²), dropping sub-sliver lobes.",
                  notched.size(),
                  kept,
                  bg::area(*best));
    }
    // Replace the outer ring with the notched one, then re-attach the holes
    // punched earlier — but ONLY those still strictly inside the new outer.
    // The notch may have removed or split off the region a previous hole sat
    // in; re-adding such a hole would make the polygon non-simple again (the
    // exact INTERNAL_F2C_ERROR this function exists to avoid).
    auto saved_inners = original.inners();
    original = *best;
    BgPolygon new_outer;
    new_outer.outer() = original.outer();
    bg::correct(new_outer);
    for (auto& ir : saved_inners)
    {
      BgPolygon hole;
      hole.outer().assign(ir.begin(), ir.end());
      bg::correct(hole);
      if (bg::within(hole, new_outer))
      {
        original.inners().push_back(std::move(ir));
      }
    }
  };

  for (const auto& obstacle : area.obstacles)
  {
    if (obstacle.points.size() < 3)
    {
      continue;
    }
    punch_exclusion(to_bg_polygon(obstacle));
  }
  if (has_dock_exclusion_ && dock_body_polygon_.points.size() >= 3)
  {
    punch_exclusion(to_bg_polygon(dock_body_polygon_));
  }
  bg::correct(original);

  // Combine the main (largest) piece with any lobes a notch split off, so
  // every reachable region of the area is planned — not just the biggest.
  // (An exclusion punched AFTER a split applies only to the main piece; that
  // only matters when two separate boundary-crossing exclusions both split
  // the area, which is rare — the lobes are still emitted, just without the
  // later exclusion subtracted.)
  BgMultiPolygon area_mp;
  area_mp.push_back(original);
  for (auto& lobe : split_lobes)
  {
    bg::correct(lobe);
    area_mp.push_back(lobe);
  }

  // 2. Walk mow_progress over the area polygon, collect mowed cell positions.
  //    Rule: cell is "mowed" when mow_progress >= 0.3 (matches the rest of
  //    the planner: see is_strip_mowed / compute_coverage_stats).
  const auto& progress = map_[std::string(layers::MOW_PROGRESS)];

  grid_map::Polygon gm_polygon;
  for (const auto& p : area.polygon.points)
  {
    gm_polygon.addVertex(grid_map::Position(p.x, p.y));
  }

  struct CellHit
  {
    int row;
    int col;
    double cx;
    double cy;
  };
  std::vector<CellHit> mowed;
  mowed.reserve(1024);

  for (grid_map::PolygonIterator it(map_, gm_polygon); !it.isPastEnd(); ++it)
  {
    const auto& idx = *it;
    if (progress(idx(0), idx(1)) >= 0.3f)
    {
      grid_map::Position pos;
      if (map_.getPosition(idx, pos))
      {
        mowed.push_back(CellHit{idx(0), idx(1), pos.x(), pos.y()});
      }
    }
  }

  if (mowed.empty())
  {
    // Nothing mowed yet: return the punched area (dock notch/hole + operator
    // obstacles applied above), NOT the raw area.polygon, so the very first
    // plan already excludes the dock and stays a simple polygon F2C can
    // handle. Emit every split lobe too, so a large obstacle that partitions
    // the area still plans all of it on the first call.
    const double sliver_area_m2 = tool_width_ * tool_width_;
    for (const auto& sub : area_mp)
    {
      if (std::abs(bg::area(sub)) < sliver_area_m2)
      {
        continue;
      }
      BgPolygon simple_orig;
      bg::simplify(sub, simple_orig, 0.08);
      if (simple_orig.outer().size() < 4)
      {
        simple_orig = sub;
      }
      mowgli_interfaces::msg::MapArea piece;
      piece.name = area.name + "_remaining_" + std::to_string(res->pieces.size());
      piece.area = to_ros_polygon(simple_orig.outer());
      for (const auto& inner : simple_orig.inners())
      {
        if (inner.size() < 4)
        {
          continue;
        }
        piece.obstacles.push_back(to_ros_polygon(inner));
      }
      piece.is_navigation_area = false;
      if (piece.area.points.size() >= 3)
      {
        res->pieces.push_back(piece);
      }
    }
    res->success = true;
    return;
  }

  // 3. Compress mowed cells into per-row run-length rectangles, then union
  //    them tree-style (pairwise merge) into a single multipolygon. The
  //    epsilon padding ensures adjacent runs merge cleanly under FP noise.
  std::sort(mowed.begin(), mowed.end(), [](const CellHit& a, const CellHit& b) {
    return std::tie(a.row, a.col) < std::tie(b.row, b.col);
  });

  const double half = 0.5 * resolution_;
  const double eps = 1e-4;  // 0.1 mm overlap to fuse touching rectangles

  std::vector<BgMultiPolygon> rect_mps;
  rect_mps.reserve(mowed.size());

  size_t i = 0;
  while (i < mowed.size())
  {
    size_t j = i + 1;
    while (j < mowed.size() && mowed[j].row == mowed[i].row &&
           mowed[j].col == mowed[j - 1].col + 1)
    {
      ++j;
    }
    const double x_min = std::min(mowed[i].cx, mowed[j - 1].cx) - half - eps;
    const double x_max = std::max(mowed[i].cx, mowed[j - 1].cx) + half + eps;
    const double y_min = std::min(mowed[i].cy, mowed[j - 1].cy) - half - eps;
    const double y_max = std::max(mowed[i].cy, mowed[j - 1].cy) + half + eps;

    BgPolygon rect;
    bg::append(rect.outer(), BgPoint(x_min, y_min));
    bg::append(rect.outer(), BgPoint(x_max, y_min));
    bg::append(rect.outer(), BgPoint(x_max, y_max));
    bg::append(rect.outer(), BgPoint(x_min, y_max));
    bg::append(rect.outer(), BgPoint(x_min, y_min));
    bg::correct(rect);

    BgMultiPolygon mp;
    mp.push_back(std::move(rect));
    rect_mps.push_back(std::move(mp));
    i = j;
  }

  // Pairwise tree merge: O(N log N) unions instead of O(N²) incremental.
  while (rect_mps.size() > 1)
  {
    std::vector<BgMultiPolygon> next;
    next.reserve((rect_mps.size() + 1) / 2);
    for (size_t k = 0; k + 1 < rect_mps.size(); k += 2)
    {
      BgMultiPolygon merged;
      bg::union_(rect_mps[k], rect_mps[k + 1], merged);
      next.push_back(std::move(merged));
    }
    if (rect_mps.size() % 2 == 1)
    {
      next.push_back(std::move(rect_mps.back()));
    }
    rect_mps = std::move(next);
  }
  const BgMultiPolygon& mowed_mp = rect_mps.front();

  // 4. remaining = area (all lobes) - mowed_mp.
  BgMultiPolygon remaining;
  bg::difference(area_mp, mowed_mp, remaining);

  // 5. Convert each output piece to a MapArea. Drop pieces whose area is
  //    smaller than one swath (tool_width²) — they are slivers F2C cannot
  //    cover anyway.
  const double sliver_area_m2 = tool_width_ * tool_width_;
  for (size_t k = 0; k < remaining.size(); ++k)
  {
    const auto& bg_piece = remaining[k];
    if (bg_piece.outer().size() < 4)
    {
      continue;
    }
    const double piece_area = std::abs(bg::area(bg_piece));
    if (piece_area < sliver_area_m2)
    {
      continue;
    }

    // Collapse the 0.05 m staircase the mow_progress difference leaves on
    // the outer ring AND the mowed-region holes (each a 100-800 vertex
    // raster boundary). The tolerance MUST exceed one cell (0.05 m) to
    // actually remove the staircase: at the old 0.02 m it sat *under* the
    // cell pitch and kept ~all the steps, so a resume piece arrived at F2C
    // with a 682-pt outer + 114-pt hole, which TrapezoidalDecomp exploded
    // into 224 sub-cells → 94 s to plan one piece (field 2026-06-01).
    // 0.08 m clears the steps while staying well inside the chassis/headland
    // inset, so the real mowable shape is unchanged. Fall back to the raw
    // piece if simplification collapses it.
    BgPolygon simple_piece;
    bg::simplify(bg_piece, simple_piece, 0.08);
    if (simple_piece.outer().size() < 4)
    {
      simple_piece = bg_piece;
    }

    mowgli_interfaces::msg::MapArea piece;
    piece.name = area.name + "_remaining_" + std::to_string(res->pieces.size());
    piece.area = to_ros_polygon(simple_piece.outer());
    piece.is_navigation_area = false;
    for (const auto& inner : simple_piece.inners())
    {
      if (inner.size() < 4)
      {
        continue;
      }
      piece.obstacles.push_back(to_ros_polygon(inner));
    }
    if (piece.area.points.size() >= 3)
    {
      res->pieces.push_back(piece);
    }
  }

  res->success = true;
}

}  // namespace mowgli_map
