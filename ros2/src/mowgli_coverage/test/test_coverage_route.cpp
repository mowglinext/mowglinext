// Copyright (C) 2026 Cedric <cedric@mowgli.dev>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "fixtures/isabey.hpp"
#include "mowgli_coverage/coverage_planning.hpp"
#include "ogr_geometry.h"
#include <gtest/gtest.h>

namespace
{
using Point = std::pair<double, double>;
using Path = std::vector<Point>;
using Paths = std::vector<Path>;
namespace coverage = mowgli_coverage;

double distance(const Point& a, const Point& b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}

double length(const Paths& paths)
{
  double result = 0.0;
  for (const auto& path : paths)
    for (std::size_t i = 1; i < path.size(); ++i)
      result += distance(path[i - 1], path[i]);
  return result;
}

f2c::types::LinearRing ring(const Path& points)
{
  f2c::types::LinearRing result;
  for (const auto& [x, y] : points)
    result.addPoint(f2c::types::Point(x, y));
  return coverage::dedupClosedRing(result);
}

TEST(CoverageRoute, IsabeyReducesCrossingsWithoutLosingMowingPrimitives)
{
  // e1fd5b27, F2C 3.0.0 @ 884d895, shipped configuration, float32 goal:
  // 9 paths, 1070.18 m mowing, 22.01 m endpoint links, longest link 7.43 m.
  // This is offline geometry, not a physical mowing or Nav2-transit measurement.
  const auto recorded = isabeyRings();
  f2c::types::Cell field(ring(recorded.front()));
  coverage::PivotJoinLimits limits;
  limits.recorded_boundary = recorded.front();
  limits.recorded_obstacles.assign(recorded.begin() + 1, recorded.end());
  limits.sweep_radius = limits.boundary_margin = std::hypot(0.53, 0.275);
  for (const auto& obstacle : limits.recorded_obstacles)
    field.addRing(coverage::bufferRingOutward(ring(obstacle), 0.389));

  const auto plan = coverage::planBoustrophedon(field, 0.16, 0.18, 5, 0.0, -1.0, 0.15);
  ASSERT_EQ(plan.rings.size(), 16u);
  ASSERT_EQ(plan.swaths.size(), 115u);
  auto build = [&]()
  {
    return coverage::buildContinuousSubPaths(
        plan, plan.connector_clearance_boundary, 0.20, 0.20, 0.03, nullptr, {}, limits);
  };
  const auto paths = build();
  ASSERT_FALSE(paths.empty());
  EXPECT_LE(paths.size(), 6u);
  EXPECT_LT(length(paths), 1030.0);
  double transit = 0.0;
  for (std::size_t i = 1; i < paths.size(); ++i)
  {
    const double gap = distance(paths[i - 1].back(), paths[i].front());
    EXPECT_LT(gap, 5.0);
    transit += gap;
  }
  EXPECT_LT(transit, 15.0);
  EXPECT_EQ(paths, build()) << "resume-by-index requires identical re-plans";

  OGRMultiLineString driven;
  std::size_t outside = 0, in_hole = 0;
  for (const auto& path : paths)
  {
    ASSERT_GE(path.size(), 2u);
    OGRLineString line;
    for (const auto& [x, y] : path)
      line.addPoint(x, y);
    driven.addGeometry(&line);
    // Check the segments as well as the saved poses: endpoints outside a hole
    // cannot establish that the line connecting them is safe.
    for (std::size_t i = 1; i < path.size(); ++i)
    {
      const int n =
          std::max(1, static_cast<int>(std::ceil(distance(path[i - 1], path[i]) / 0.015)));
      for (int k = 0; k <= n; ++k)
      {
        const double t = static_cast<double>(k) / n;
        const double x = path[i - 1].first + t * (path[i].first - path[i - 1].first);
        const double y = path[i - 1].second + t * (path[i].second - path[i - 1].second);
        outside += !coverage::pointInRing(x, y, plan.connector_clearance_boundary) &&
                   coverage::distanceToRing(x, y, plan.connector_clearance_boundary) > 0.001;
        for (const auto& hole : plan.safe_holes)
          in_hole +=
              coverage::pointInRing(x, y, hole) && coverage::distanceToRing(x, y, hole) > 1e-6;
      }
    }
  }
  EXPECT_EQ(outside, 0u);
  EXPECT_EQ(in_hole, 0u);

  // Every original lane and perimeter loop must still be cut. A 9 cm blade
  // radius is the shipped tool_width/2; connectors may change, primitives may
  // not disappear to achieve a shorter route.
  std::unique_ptr<OGRGeometry, decltype(&OGRGeometryFactory::destroyGeometry)> cut(
      driven.Buffer(0.09, 12), OGRGeometryFactory::destroyGeometry);
  ASSERT_NE(cut, nullptr);
  std::size_t missed = 0;
  for (const auto& loop : plan.rings)
    for (const auto& [x, y] : loop)
    {
      OGRPoint point(x, y);
      missed += !cut->Intersects(&point);
    }
  for (const auto& [start, end] : plan.swaths)
  {
    const int n = std::max(1, static_cast<int>(std::ceil(distance(start, end) / 0.05)));
    for (int k = 0; k <= n; ++k)
    {
      const double t = static_cast<double>(k) / n;
      OGRPoint point(start.first + t * (end.first - start.first),
                     start.second + t * (end.second - start.second));
      missed += !cut->Intersects(&point);
    }
  }
  EXPECT_EQ(missed, 0u);
}

TEST(CoverageRoute, AllRingBearingPathsKeepTheirDirection)
{
  // The nearest entry to the SECOND protected path is its back. Protecting
  // only original index 0 (the previous implementation) silently reversed it.
  const Path first{{0, 0}, {0, 1}};
  const Path second{{10, 1}, {0, 1.01}};
  const Path swath{{10, 1.01}, {20, 1}};
  const Paths input{first, second, swath};
  const auto paths = coverage::orderSubPathsForMinimalTransit(input, 2);
  ASSERT_EQ(paths.size(), input.size());
  EXPECT_NE(std::find(paths.begin(), paths.end(), first), paths.end());
  EXPECT_NE(std::find(paths.begin(), paths.end(), second), paths.end());
  EXPECT_EQ(paths, coverage::orderSubPathsForMinimalTransit(input, 2));
}

TEST(CoverageRoute, WiderTurnsSaveDistanceAndVisitEveryRow)
{
  const Path boundary{{-1, -1}, {5, -1}, {5, 10}, {-1, 10}};
  coverage::BoustrophedonPlan plan;
  for (int i = 0; i < 20; ++i)
    plan.swaths.push_back({{i * 0.16, 0}, {i * 0.16, 9}});
  const auto paths = coverage::buildContinuousSubPaths(plan, boundary, 0.20, 0.20, 0.03);
  ASSERT_EQ(paths.size(), 1u);
  // Adjacent-row omega turns require >24 m here. Wider feasible turns remove
  // at least 4 m without shortening or deleting any of the twenty 9 m rows.
  EXPECT_LT(length(paths), 200.0);
  for (const auto& swath : plan.swaths)
  {
    const Point middle{swath.first.first, 4.5};
    EXPECT_TRUE(std::any_of(paths.front().begin(),
                            paths.front().end(),
                            [&](const Point& p)
                            {
                              return distance(p, middle) < 0.03;
                            }));
  }
}
}  // namespace

// Field 2026-10-10 on this lawn: 65 ring corners sharper than 60° were driven
// as curves and FTC wedged against the hedge at three of them. They are now
// in-place pivots wherever the sweep fits — and NEVER where it does not: the
// outer ring's hairpin at (1.99, 11.91) / (2.26, 10.67) lies 0.47 m from a
// drawn obstacle, inside the 0.60 m sweep, and must stay a driven corner.
TEST(CoverageRoute, IsabeyPivotsSharpCornersOnlyWhereTheSweepFits)
{
  const auto recorded = isabeyRings();
  f2c::types::Cell field(ring(recorded.front()));
  coverage::PivotJoinLimits limits;
  limits.recorded_boundary = recorded.front();
  limits.recorded_obstacles.assign(recorded.begin() + 1, recorded.end());
  limits.sweep_radius = limits.boundary_margin = std::hypot(0.53, 0.275);
  for (const auto& obstacle : limits.recorded_obstacles)
    field.addRing(coverage::bufferRingOutward(ring(obstacle), 0.389));
  const auto plan = coverage::planBoustrophedon(field, 0.16, 0.18, 5, 0.0, -1.0, 0.15);
  const auto paths = coverage::buildContinuousSubPaths(
      plan, plan.connector_clearance_boundary, 0.20, 0.20, 0.03, nullptr, {}, limits);

  std::size_t pivots = 0;
  for (const auto& path : paths)
    for (std::size_t i = 1; i < path.size(); ++i)
      if (path[i] == path[i - 1])
      {
        ++pivots;
        EXPECT_TRUE(coverage::pivotSweepFits(path[i].first, path[i].second, limits))
            << "pivot at (" << path[i].first << ", " << path[i].second << ") sweeps outside";
        for (const Point hairpin : {Point{1.99, 11.91}, Point{2.26, 10.67}})
          EXPECT_GT(distance(path[i], hairpin), 0.05) << "pivot beside the drawn obstacle";
      }
  // 54 on the robot's plan of this lawn (field 2026-10-10); 0 before.
  EXPECT_GE(pivots, 40u);
}

namespace
{
// The shipped footprint: 2 x robot_config_util.chassis_half_width (0.45 m
// chassis + 0.05 m margin each side), what coverage_server is injected with.
constexpr double kChassisWidthM = 0.55;

std::vector<Point> exteriorOf(const f2c::types::Cell& cell)
{
  std::vector<Point> points;
  const auto exterior = cell.getGeometry(0);
  for (std::size_t i = 0; i < exterior.size(); ++i)
    points.emplace_back(exterior.getGeometry(i).getX(), exterior.getGeometry(i).getY());
  return points;
}
}  // namespace

TEST(NarrowBoundaryTongues, ADeadEndTongueNarrowerThanTheChassisIsRemoved)
{
  // 6 x 4 m lawn with a 0.30 m-wide, 1.5 m-long wedge out of its top side.
  const f2c::types::Cell field(
      ring({{0, 0}, {6, 0}, {6, 4}, {3.3, 4}, {3.15, 5.5}, {3.0, 4}, {0, 4}}));
  double removed = -1.0;
  const auto cleaned = coverage::removeNarrowBoundaryTongues(field, kChassisWidthM, &removed);
  EXPECT_NEAR(removed, 0.5 * 0.30 * 1.5, 1e-3);
  EXPECT_NEAR(cleaned.area(), 24.0, 1e-3);
  for (const auto& [x, y] : exteriorOf(cleaned))
    EXPECT_LE(y, 4.0 + 1e-6) << "vertex (" << x << ", " << y << ") is still in the tongue";
}

TEST(NarrowBoundaryTongues, OrdinaryFieldsWideTonguesAndNecksAreUnchanged)
{
  const std::vector<Path> unchanged{
      // Plain rectangle: the mitred opening must restore every corner.
      {{0, 0}, {6, 0}, {6, 4}, {0, 4}},
      // A 1.0 m-wide tongue is lawn the robot fits into.
      {{0, 0}, {6, 0}, {6, 4}, {3.5, 4}, {3.5, 5.5}, {2.5, 5.5}, {2.5, 4}, {0, 4}},
      // Two lawns joined by a 0.30 m passage: a neck, not a tongue.
      {{0, 0},
       {4, 0},
       {4, 1.85},
       {5, 1.85},
       {5, 0},
       {9, 0},
       {9, 4},
       {5, 4},
       {5, 2.15},
       {4, 2.15},
       {4, 4},
       {0, 4}},
      // An L with a 60° corner.
      {{0, 0}, {6, 0}, {6, 2}, {2 + 4 / std::tan(M_PI / 3), 2}, {2, 6}, {0, 6}},
  };
  for (const auto& points : unchanged)
  {
    const f2c::types::Cell field(ring(points));
    double removed = -1.0;
    const auto cleaned = coverage::removeNarrowBoundaryTongues(field, kChassisWidthM, &removed);
    EXPECT_EQ(removed, 0.0);
    EXPECT_EQ(exteriorOf(cleaned), exteriorOf(field)) << "an ordinary field was rewritten";
  }
}

TEST(NarrowBoundaryTongues, HolesAreKeptAndZeroWidthIsOff)
{
  f2c::types::Cell field(ring({{0, 0}, {6, 0}, {6, 4}, {3.3, 4}, {3.15, 5.5}, {3.0, 4}, {0, 4}}));
  field.addRing(ring({{1, 1}, {2, 1}, {2, 2}, {1, 2}}));
  const auto cleaned = coverage::removeNarrowBoundaryTongues(field, kChassisWidthM);
  ASSERT_EQ(cleaned.size(), 2u);
  EXPECT_NEAR(cleaned.area(), 23.0, 1e-3);

  double removed = -1.0;
  const auto off = coverage::removeNarrowBoundaryTongues(field, 0.0, &removed);
  EXPECT_EQ(removed, 0.0);
  EXPECT_EQ(exteriorOf(off), exteriorOf(field));
}

// Field 2026-10-10 on this lawn: the recorded line runs a ~0.4 m wedge into the
// charging station, tip at the dock pose (6.27, 2.80). The outer ring followed
// it and pivoted at the tip against the station; the robot stalled there and
// the ring unit was lost twice in one mow. With the chassis width given, no
// driven pose comes near the tip any more.
TEST(CoverageRoute, IsabeyRingsStayOutOfTheDockTongue)
{
  const auto recorded = isabeyRings();
  f2c::types::Cell field(ring(recorded.front()));
  coverage::PivotJoinLimits limits;
  limits.recorded_boundary = recorded.front();
  limits.recorded_obstacles.assign(recorded.begin() + 1, recorded.end());
  limits.sweep_radius = limits.boundary_margin = std::hypot(0.53, 0.275);
  for (const auto& obstacle : limits.recorded_obstacles)
    field.addRing(coverage::bufferRingOutward(ring(obstacle), 0.389));
  const Point tip{6.27278, 2.79832};

  // Closest driven pose, and closest pivot corner, to the tongue's tip.
  auto nearest = [&](const Paths& paths, bool pivots_only)
  {
    double best = 1e9;
    for (const auto& path : paths)
      for (std::size_t i = 0; i < path.size(); ++i)
        if (!pivots_only || (i > 0 && path[i] == path[i - 1]))
          best = std::min(best, distance(path[i], tip));
    return best;
  };
  const auto before = coverage::planBoustrophedon(field, 0.16, 0.18, 5, 0.0, -1.0, 0.15);
  const auto before_paths = coverage::buildContinuousSubPaths(
      before, before.connector_clearance_boundary, 0.20, 0.20, 0.03, nullptr, {}, limits);
  ASSERT_LT(nearest(before_paths, false), 0.01) << "fixture no longer reproduces the field case";
  ASSERT_LT(nearest(before_paths, true), 0.01) << "the field case pivoted at the tip";

  const auto plan = coverage::planBoustrophedon(
      field, 0.16, 0.18, 5, 0.0, -1.0, 0.15, 0, 0.20, false, 0, std::nullopt, kChassisWidthM);
  const auto paths = coverage::buildContinuousSubPaths(
      plan, plan.connector_clearance_boundary, 0.20, 0.20, 0.03, nullptr, {}, limits);
  ASSERT_FALSE(paths.empty());
  // The wedge is cut back to where it is as wide as the chassis (0.22 m from
  // the tip here), and nothing turns in place inside what is left of it.
  EXPECT_GT(nearest(paths, false), 0.20);
  EXPECT_GT(nearest(paths, true), 0.40);
  for (const auto& loop : plan.rings)
    for (const auto& p : loop)
      EXPECT_GT(distance(p, tip), 0.20) << "ring vertex in the dock tongue";
  EXPECT_EQ(paths,
            coverage::buildContinuousSubPaths(
                plan, plan.connector_clearance_boundary, 0.20, 0.20, 0.03, nullptr, {}, limits))
      << "resume-by-index requires identical re-plans";
  for (const auto& note : plan.diagnostics.notes)
    std::printf("[isabey] %s\n", note.c_str());
  std::printf("[isabey] without: %zu paths %.1f m; with tongue removal: %zu paths %.1f m\n",
              before_paths.size(),
              length(before_paths),
              paths.size(),
              length(paths));
}
