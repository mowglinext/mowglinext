// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
/// @file area_ring_index.hpp
/// @brief Point-in-recorded-area test of costmap_scan_filter_node's LiDAR-ignore
/// corridor filter (shared with its tests).
///
/// The corridor filter asks, for every finite beam of every scan, whether the
/// beam's map-frame endpoint lies inside a recorded area. A plain even-odd test
/// walks every ring vertex per beam, so its cost is beams × vertices at scan
/// rate. AreaRingIndex answers the same question from y-bands built once per
/// area-list update: a query only walks the edges that can straddle its y.
#ifndef MOWGLI_LOCALIZATION__AREA_RING_INDEX_HPP_
#define MOWGLI_LOCALIZATION__AREA_RING_INDEX_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace mowgli_localization
{

/// A point in the MAP frame.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// One even-odd step: does the edge (pi, pj) cross the horizontal ray from
/// (px, py) towards +x? point_in_ring and AreaRingIndex share it, so both
/// evaluate exactly the same arithmetic.
inline bool edge_toggles_parity(double px, double py, const Point2D& pi, const Point2D& pj)
{
  if ((pi.y > py) == (pj.y > py))
    return false;
  const double x_at_py = pi.x + (py - pi.y) * (pj.x - pi.x) / (pj.y - pi.y);
  return px < x_at_py;
}

/// True if (px, py) is inside the polygon `ring` — standard even-odd
/// ray-casting, with an IMPLICIT closing edge between the last and first
/// point (an explicit repeated first-as-last point is harmless). A ring
/// with fewer than 3 points is degenerate and never contains anything.
inline bool point_in_ring(double px, double py, const std::vector<Point2D>& ring)
{
  if (ring.size() < 3)
    return false;
  bool inside = false;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
  {
    if (edge_toggles_parity(px, py, ring[i], ring[j]))
      inside = !inside;
  }
  return inside;
}

/// A ring prepared for repeated point_in_ring queries. Its y-extent is cut
/// into bands, each listing the edges whose y-span touches it. contains()
/// returns exactly point_in_ring's answer: an edge missing from py's band has
/// both endpoints on the same side of py, which the even-odd step skips
/// anyway, and the parity of the remaining toggles does not depend on their
/// order. Correctness rests on band() being monotonic in y (the same
/// arithmetic indexes edges and queries), not on where the band boundaries
/// fall: every py within an edge's y-span maps to a band that lists the edge.
class AreaRingIndex
{
public:
  explicit AreaRingIndex(std::vector<Point2D> ring) : ring_(std::move(ring))
  {
    if (ring_.size() < 3)
      return;  // contains nothing, like point_in_ring
    if (ring_.size() > std::numeric_limits<std::uint32_t>::max())
    {
      unindexed_ = true;
      return;
    }
    y_min_ = y_max_ = ring_.front().y;
    for (const auto& p : ring_)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y))
      {
        unindexed_ = true;  // keep exact IEEE behaviour: walk the whole ring
        return;
      }
      y_min_ = std::min(y_min_, p.y);
      y_max_ = std::max(y_max_, p.y);
    }
    if (!(y_max_ > y_min_))
      return;  // flat: no edge can straddle any py
    const std::size_t bands =
        static_cast<std::size_t>(std::ceil(2.0 * std::sqrt(static_cast<double>(ring_.size()))));
    bands_per_m_ = static_cast<double>(bands) / (y_max_ - y_min_);
    if (!std::isfinite(y_max_ - y_min_) || !std::isfinite(bands_per_m_))
    {
      unindexed_ = true;  // extent too extreme for band arithmetic
      return;
    }
    bands_ = bands;
    // Counting sort of the edges into every band their y-span touches.
    std::vector<std::size_t> next(bands_ + 1, 0);
    const auto for_each_edge_band = [this](auto&& visit)
    {
      for (std::size_t i = 0, j = ring_.size() - 1; i < ring_.size(); j = i++)
      {
        const double lo = std::min(ring_[i].y, ring_[j].y);
        const double hi = std::max(ring_[i].y, ring_[j].y);
        if (!(lo < hi))
          continue;  // horizontal (or NaN): never toggles
        for (std::size_t b = band(lo), last = band(hi); b <= last; ++b)
          visit(b, static_cast<std::uint32_t>(i));
      }
    };
    for_each_edge_band(
        [&next](std::size_t b, std::uint32_t)
        {
          ++next[b + 1];
        });
    for (std::size_t b = 0; b < bands_; ++b)
      next[b + 1] += next[b];
    if (next[bands_] > kMaxEntriesPerEdge * ring_.size())
    {
      // Pathological ring (e.g. a zigzag whose every edge spans the whole
      // extent): the index would not pay for its memory.
      unindexed_ = true;
      bands_ = 0;
      return;
    }
    band_begin_ = next;
    band_edges_.resize(next[bands_]);
    for_each_edge_band(
        [&](std::size_t b, std::uint32_t i)
        {
          band_edges_[next[b]++] = i;
        });
  }

  bool contains(double px, double py) const
  {
    if (unindexed_)
      return point_in_ring(px, py, ring_);
    // Outside [y_min, y_max) — or NaN — no edge straddles py, and
    // point_in_ring returns false there too.
    if (bands_ == 0 || !(py >= y_min_ && py < y_max_))
      return false;
    const std::size_t b = band(py);
    bool inside = false;
    for (std::size_t k = band_begin_[b]; k < band_begin_[b + 1]; ++k)
    {
      // Edge i joins ring[i] and its predecessor, as in point_in_ring.
      const std::uint32_t i = band_edges_[k];
      const Point2D& pj = ring_[i == 0 ? ring_.size() - 1 : i - 1];
      if (edge_toggles_parity(px, py, ring_[i], pj))
        inside = !inside;
    }
    return inside;
  }

private:
  static constexpr std::size_t kMaxEntriesPerEdge = 16;

  // Only called with y in [y_min_, y_max_].
  std::size_t band(double y) const
  {
    const double t = std::floor((y - y_min_) * bands_per_m_);
    return std::min(static_cast<std::size_t>(std::max(t, 0.0)), bands_ - 1);
  }

  std::vector<Point2D> ring_;
  double y_min_{0.0};
  double y_max_{0.0};
  double bands_per_m_{0.0};
  std::size_t bands_{0};
  bool unindexed_{false};
  // Band b lists the edges band_edges_[band_begin_[b], band_begin_[b + 1]);
  // edge i joins ring_[i] and its predecessor.
  std::vector<std::size_t> band_begin_;
  std::vector<std::uint32_t> band_edges_;
};

/// True if (px, py) is inside ANY of `rings`, tested in order. An empty list
/// contains nothing — exactly as the vector-of-rings walk it replaces.
inline bool point_in_any_ring(double px, double py, const std::vector<AreaRingIndex>& rings)
{
  for (const auto& ring : rings)
  {
    if (ring.contains(px, py))
      return true;
  }
  return false;
}

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__AREA_RING_INDEX_HPP_
