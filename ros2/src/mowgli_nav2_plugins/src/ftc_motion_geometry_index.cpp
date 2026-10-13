// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/ftc_motion_geometry_index.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <utility>

#include "mowgli_interfaces/motion_authorization.hpp"
#include "mowgli_interfaces/motion_geometry_primitives.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace mowgli_nav2_plugins
{
namespace motion = mowgli_interfaces::motion;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using Point = bg::model::d2::point_xy<double>;
using Box = bg::model::box<Point>;
using Value = std::pair<Box, std::size_t>;
struct MotionRegionIndex::Data
{
  std::shared_ptr<const motion::Snapshot> snapshot;
  struct Cache
  {
    static constexpr std::size_t budget = 8 * 1024 * 1024;
    std::size_t bytes = 0;
    std::map<std::pair<std::size_t, double>, std::vector<double>> crossings;
  };
  struct Ring
  {
    motion::Ring vertices;
    bgi::rtree<Value, bgi::quadratic<16>> edges;
    bgi::rtree<Value, bgi::quadratic<16>> parity_edges;
    double min_x = 0, max_x = 0;
    explicit Ring(const motion::Ring& original) : vertices(original)
    {
      std::vector<Value> values;
      std::vector<Value> parity_values;
      if (!vertices.empty())
        min_x = max_x = vertices.front().x;
      for (std::size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++)
      {
        const auto a = vertices[j], b = vertices[i];
        min_x = std::min(min_x, a.x);
        max_x = std::max(max_x, a.x);
        values.emplace_back(Box(Point(std::min(a.x, b.x), std::min(a.y, b.y)),
                                Point(std::max(a.x, b.x), std::max(a.y, b.y))),
                            i);
        if (a.y != b.y)
          parity_values.push_back(values.back());
      }
      edges = bgi::rtree<Value, bgi::quadratic<16>>(values);
      parity_edges = bgi::rtree<Value, bgi::quadratic<16>>(parity_values);
    }
    std::vector<std::size_t> query(const Box& box) const
    {
      std::vector<Value> selected;
      edges.query(bgi::intersects(box), std::back_inserter(selected));
      std::vector<std::size_t> indices;
      for (const auto& value : selected)
        indices.push_back(value.second);
      std::sort(indices.begin(), indices.end());  // Original order, including edge0's wrap.
      return indices;
    }
    bool inside(motion::Point p, Cache* cache, std::size_t ring_id) const
    {
      if (vertices.size() < 3)
        return false;
      // Full ring X range: no unproved ray-X pruning. Outward rounding keeps
      // every edge admitted by the original +/-epsilon boundary comparisons.
      const double infinity = std::numeric_limits<double>::infinity();
      const auto key = std::make_pair(ring_id, p.y);
      const std::vector<double>* thresholds = nullptr;
      if (cache)
      {
        const auto found = cache->crossings.find(key);
        if (found != cache->crossings.end())
          thresholds = &found->second;
      }
      std::vector<Value> selected;
      if (!thresholds)
      {
        // ALL straddling edges across the full ring X range. No ray pruning.
        parity_edges.query(bgi::intersects(Box(Point(min_x, p.y), Point(max_x, p.y))),
                           std::back_inserter(selected));
        // A repeated exact Y can reuse the original comparison thresholds.
        // Budget limits memory only; uncached queries use the exact predicate.
        const std::size_t charge = selected.size() * sizeof(double) + 128;
        if (cache && selected.size() > 256 && charge <= Cache::budget - cache->bytes)
        {
          std::vector<double> crossings;
          crossings.reserve(selected.size());
          for (const auto& value : selected)
          {
            const auto i = value.second;
            const auto a = vertices[i == 0 ? vertices.size() - 1 : i - 1], b = vertices[i];
            if ((a.y > p.y) != (b.y > p.y))
            {
              const double x = motion::parityCrossingX(p.y, a, b);
              if (!std::isnan(x))
                crossings.push_back(x);  // Original p.x<NaN is false; infinities retained.
            }
          }
          std::sort(crossings.begin(), crossings.end());
          auto inserted = cache->crossings.emplace(key, std::move(crossings));
          cache->bytes += charge;
          thresholds = &inserted.first->second;
          selected.clear();
        }
      }
      const Box boundary(Point(std::nextafter(p.x - motion::kEpsilon, -infinity),
                               std::nextafter(p.y - motion::kEpsilon, -infinity)),
                         Point(std::nextafter(p.x + motion::kEpsilon, infinity),
                               std::nextafter(p.y + motion::kEpsilon, infinity)));
      edges.query(bgi::intersects(boundary), std::back_inserter(selected));
      std::vector<std::size_t> indices;
      for (const auto& value : selected)
        indices.push_back(value.second);
      std::sort(indices.begin(), indices.end());
      indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
      bool parity = false;
      for (const auto i : indices)
        if (motion::insideEdge(
                p, vertices[i == 0 ? vertices.size() - 1 : i - 1], vertices[i], parity))
          return true;
      if (thresholds)
        return (static_cast<std::size_t>(thresholds->end() - std::upper_bound(thresholds->begin(),
                                                                              thresholds->end(),
                                                                              p.x)) &
                1) != 0;
      return parity;
    }
  };
  std::vector<Ring> rings;
};
MotionRegionIndex::MotionRegionIndex(const std::vector<motion::Ring>& rings)
{
  auto data = std::make_shared<Data>();
  for (const auto& ring : rings)
    data->rings.emplace_back(ring);
  data_ = std::move(data);
}
MotionRegionIndex::MotionRegionIndex(std::shared_ptr<const motion::Snapshot> snapshot)
{
  auto data = std::make_shared<Data>();
  data->snapshot = std::move(snapshot);
  if (data->snapshot)
    for (const auto& ring : data->snapshot->mowing.allowed)
      data->rings.emplace_back(ring);
  data_ = std::move(data);
}
bool MotionRegionIndex::matches(const motion::Snapshot* snapshot) const
{
  return data_->snapshot && snapshot && data_->snapshot->valid && snapshot->valid &&
         data_->snapshot->identity == snapshot->identity;
}
namespace
{
template <class Data, class Cache>
bool Authorized(const Data& data, motion::Point p, Cache* cache)
{
  if (!std::isfinite(p.x) || !std::isfinite(p.y))
    return false;
  for (std::size_t i = 0; i < data.rings.size(); ++i)
    if (data.rings[i].inside(p, cache, i))
      return true;
  return false;
}
template <class Data, class Cache>
bool SegmentAuthorized(const Data& data, motion::Point a, motion::Point b, Cache* cache)
{
  if (!Authorized(data, a, cache) || !Authorized(data, b, cache))
    return false;
  if (a.x == b.x && a.y == b.y)
    return true;
  const Box box(Point(std::min(a.x, b.x), std::min(a.y, b.y)),
                Point(std::max(a.x, b.x), std::max(a.y, b.y)));
  std::vector<double> cuts{0, 1};
  for (const auto& ring : data.rings)
    for (const auto i : ring.query(box))
      motion::appendEdgeCuts(
          a, b, ring.vertices[i == 0 ? ring.vertices.size() - 1 : i - 1], ring.vertices[i], cuts);
  motion::finishCuts(cuts);
  for (std::size_t i = 1; i < cuts.size(); ++i)
    if (!Authorized(data, motion::interpolate(a, b, (cuts[i - 1] + cuts[i]) / 2), cache))
      return false;
  return true;
}
}  // namespace
bool MotionRegionIndex::authorized(motion::Point p) const
{
  return Authorized(*data_, p, static_cast<Data::Cache*>(nullptr));
}
bool MotionRegionIndex::segmentAuthorized(motion::Point a, motion::Point b) const
{
  return SegmentAuthorized(*data_, a, b, static_cast<Data::Cache*>(nullptr));
}
struct MotionRegionIndex::Query::State
{
  std::shared_ptr<const Data> data;
  Data::Cache cache;
  std::mutex mutex;
};
MotionRegionIndex::Query::Query(std::shared_ptr<const Data> data)
    : state_(std::make_unique<State>())
{
  state_->data = std::move(data);
}
MotionRegionIndex::Query::~Query() = default;
std::shared_ptr<MotionRegionIndex::Query> MotionRegionIndex::newQuery() const
{
  return std::shared_ptr<Query>(new Query(data_));
}
bool MotionRegionIndex::Query::authorized(motion::Point p)
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  return Authorized(*state_->data, p, &state_->cache);
}
bool MotionRegionIndex::Query::segmentAuthorized(motion::Point a, motion::Point b)
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  return SegmentAuthorized(*state_->data, a, b, &state_->cache);
}
}  // namespace mowgli_nav2_plugins
