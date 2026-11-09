// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <utility>

#include "mowgli_nav2_plugins/transit_geometry.hpp"

namespace mowgli_nav2_plugins::transit
{
// Same arithmetic and closed boundaries as Geometry, evaluated from the
// polygon side instead of repeating every edge test for every garden cell.
struct RasterGrid
{
  unsigned width, height;
  double resolution, origin_x, origin_y;

  Point centre(unsigned x, unsigned y) const
  {
    return {origin_x + (x + 0.5) * resolution, origin_y + (y + 0.5) * resolution};
  }
  std::pair<unsigned, unsigned> range(double lo, double hi, double origin, unsigned size) const
  {
    // At extreme translated origins several cell centres can round to the
    // same coordinate. An index inferred by division then misses candidates.
    if (std::nextafter(origin, INFINITY) - origin >= resolution)
      return {0, size - 1};
    // One extra cell absorbs division/translation rounding. Exact tests below
    // decide membership; an oversized candidate range never grants permission.
    const double last = size - 1;
    return {
        static_cast<unsigned>(std::clamp(std::floor((lo - origin) / resolution) - 1, 0.0, last)),
        static_cast<unsigned>(std::clamp(std::floor((hi - origin) / resolution) + 1, 0.0, last))};
  }
};

template <typename Check, typename Visit>
void rasterInside(const Ring& ring, const RasterGrid& grid, const Check& check, const Visit& visit)
{
  if (ring.size() < 3)
    return;
  double min_x = INFINITY, min_y = INFINITY, max_x = -INFINITY, max_y = -INFINITY;
  for (const auto& p : ring)
  {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
  const auto [x0, x1] = grid.range(min_x, max_x, grid.origin_x, grid.width);
  const auto [y0, y1] = grid.range(min_y, max_y, grid.origin_y, grid.height);
  std::vector<double> crossings;
  crossings.reserve(ring.size());
  for (unsigned y = y0; y <= y1; ++y)
  {
    check();
    const double py = grid.centre(0, y).y;
    crossings.clear();
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
    {
      const auto a = ring[j], b = ring[i];
      if ((a.y > py) != (b.y > py))
        crossings.push_back(a.x + (py - a.y) * (b.x - a.x) / (b.y - a.y));
    }
    const bool finite = std::all_of(crossings.begin(),
                                    crossings.end(),
                                    [](double x)
                                    {
                                      return std::isfinite(x);
                                    });
    if (finite)
      std::sort(crossings.begin(), crossings.end());
    for (unsigned x = x0; x <= x1; ++x)
    {
      const auto p = grid.centre(x, y);
      // Non-finite intermediate arithmetic retains the original classifier.
      const bool in =
          finite
              ? ((crossings.end() - std::upper_bound(crossings.begin(), crossings.end(), p.x)) & 1)
              : inside(p, ring);
      if (in)
        visit(x, y);
    }
  }
}

template <typename Check, typename Visit>
void rasterEdges(const Ring& ring,
                 const RasterGrid& grid,
                 double radius,
                 bool closed_boundary,
                 const Check& check,
                 const Visit& visit)
{
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
  {
    const auto a = ring[j], b = ring[i];
    const auto [x0, x1] = grid.range(std::min(a.x, b.x) - radius,
                                     std::max(a.x, b.x) + radius,
                                     grid.origin_x,
                                     grid.width);
    const auto [y0, y1] = grid.range(std::min(a.y, b.y) - radius,
                                     std::max(a.y, b.y) + radius,
                                     grid.origin_y,
                                     grid.height);
    for (unsigned y = y0; y <= y1; ++y)
    {
      check();
      for (unsigned x = x0; x <= x1; ++x)
      {
        const auto p = grid.centre(x, y);
        const bool in_box =
            p.x >= std::min(a.x, b.x) - radius && p.x <= std::max(a.x, b.x) + radius &&
            p.y >= std::min(a.y, b.y) - radius && p.y <= std::max(a.y, b.y) + radius;
        if ((!closed_boundary || in_box) && edgeDistance(p, a, b) <= radius)
          visit(x, y);
      }
    }
  }
}

template <typename Check>
std::vector<uint8_t> rasterAuthorization(const Geometry& geometry,
                                         const RasterGrid& grid,
                                         const Check& check)
{
  std::vector<uint8_t> allowed(std::size_t{grid.width} * grid.height, 0);
  if (allowed.empty())
    return allowed;
  const auto set = [&](unsigned x, unsigned y)
  {
    allowed[std::size_t{y} * grid.width + x] = 1;
  };
  const auto clear = [&](unsigned x, unsigned y)
  {
    allowed[std::size_t{y} * grid.width + x] = 0;
  };
  for (const auto& ring : geometry.allowed)
  {
    rasterInside(ring, grid, check, set);
    rasterEdges(ring, grid, kEpsilon, true, check, set);
  }
  for (std::size_t i = 0; i < geometry.blocked.size(); ++i)
  {
    rasterInside(geometry.blocked[i], grid, check, clear);
    rasterEdges(geometry.blocked[i], grid, geometry.margin(i) + kEpsilon, false, check, clear);
  }
  return allowed;
}
}  // namespace mowgli_nav2_plugins::transit
