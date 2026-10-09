// Copyright (C) 2026 Cedric <cedric@mowgli.dev>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "mowgli_coverage/coverage_planning.hpp"
#include <gtest/gtest.h>

namespace
{
namespace coverage = mowgli_coverage;
using Paths = std::vector<std::vector<std::pair<double, double>>>;

std::vector<uint64_t> bits(const Paths& paths)
{
  std::vector<uint64_t> result{paths.size()};
  for (const auto& path : paths)
  {
    result.push_back(path.size());
    for (const auto& [x, y] : path)
    {
      result.push_back(std::bit_cast<uint64_t>(x));
      result.push_back(std::bit_cast<uint64_t>(y));
    }
  }
  return result;
}

Paths swaths(const coverage::BoustrophedonPlan& plan)
{
  Paths result;
  for (const auto& swath : plan.swaths)
    result.push_back({swath.first, swath.second});
  return result;
}

Paths quaternions(const Paths& paths)
{
  Paths result;
  for (const auto& path : paths)
  {
    std::vector<std::pair<double, double>> angles;
    for (double yaw : coverage::pathHeadings(path))
      angles.emplace_back(std::sin(yaw * 0.5), std::cos(yaw * 0.5));
    result.push_back(std::move(angles));
  }
  return result;
}

f2c::types::Cell field(bool split)
{
  std::vector<std::pair<double, double>> points = {{0, 0}, {8, 0}, {8, 6}, {0, 6}, {0, 0}};
  if (split)
    points = {{0, 0},
              {4, 0},
              {4, 2.7},
              {7, 2.7},
              {7, 0},
              {11, 0},
              {11, 6},
              {7, 6},
              {7, 3.3},
              {4, 3.3},
              {4, 6},
              {0, 6},
              {0, 0}};
  f2c::types::LinearRing ring;
  for (auto [x, y] : points)
    ring.addPoint(f2c::types::Point(x, y));
  return f2c::types::Cell(ring);
}

TEST(PlanningTrace, ObservingAndReusingTraceDoesNotChangeExecution)
{
  for (int geometry : {0, 1, 2})
    for (double angle : {-1.0, 0.3})
      for (bool perpendicular : {false, true})
      {
        const bool split = geometry == 1;
        auto cell = field(split);
        if (geometry == 2)
        {
          f2c::types::LinearRing hole;
          for (auto [x, y] :
               std::vector<std::pair<double, double>>{{3, 2}, {4, 2}, {4, 3}, {3, 3}, {3, 2}})
            hole.addPoint(f2c::types::Point(x, y));
          cell.addRing(hole);
        }
        coverage::PlanningTrace trace;
        auto plan = [&](coverage::PlanningTrace* observer)
        {
          return coverage::planBoustrophedon(cell,
                                             0.16,
                                             0.18,
                                             5,
                                             0.0,
                                             angle,
                                             0.15,
                                             0,
                                             0.20,
                                             perpendicular,
                                             0,
                                             std::nullopt,
                                             observer);
        };
        const auto plain = plan(nullptr);
        const auto observed = plan(&trace);
        EXPECT_EQ(bits(plain.rings), bits(observed.rings));
        EXPECT_EQ(bits(swaths(plain)), bits(swaths(observed)));
        EXPECT_EQ(std::bit_cast<uint64_t>(plain.swath_angle_rad),
                  std::bit_cast<uint64_t>(observed.swath_angle_rad));
        EXPECT_EQ(bits(plain.safe_holes), bits(observed.safe_holes));
        std::size_t cells = 0, angles = 0;
        for (const auto& stage : trace.stages)
        {
          cells += stage.name.starts_with("mainland.");
          if (stage.name.starts_with(angle < 0 ? "auto_angle." : "fixed_angle."))
          {
            ++angles;
            ASSERT_EQ(stage.angles.size(), 1u);
            EXPECT_TRUE(std::isfinite(stage.angles.front()));
          }
        }
        EXPECT_GT(cells, 0u);
        EXPECT_EQ(cells, angles) << "every mainland cell must record its actual selected angle";
        const auto expected = coverage::buildContinuousSubPaths(
            plain, plain.connector_clearance_boundary, 0.20, 0.20, 0.03);
        auto paths = coverage::buildContinuousSubPaths(observed,
                                                       observed.connector_clearance_boundary,
                                                       0.20,
                                                       0.20,
                                                       0.03,
                                                       nullptr,
                                                       {},
                                                       {},
                                                       false,
                                                       &trace);
        EXPECT_EQ(bits(expected), bits(paths));
        EXPECT_EQ(bits(quaternions(expected)), bits(quaternions(paths)));
        ASSERT_FALSE(trace.stages.empty());
        EXPECT_EQ(trace.stages.back().name, "connected_subpaths");
        const auto before = trace.stages.size();
        plan(&trace);
        EXPECT_LT(trace.stages.size(), before) << "a new planning call must clear a reused trace";
        if (split)
        {
          EXPECT_TRUE(std::any_of(trace.stages.begin(),
                                  trace.stages.end(),
                                  [](const auto& stage)
                                  {
                                    return stage.name == "mainland.1";
                                  }));
        }
      }
}

TEST(PlanningTrace, AutoRecordIsTheActualUpstreamArgminAndPivotObservationIsPassive)
{
  const auto cell = field(false);
  f2c::sg::BruteForce bf;
  bf.setStepAngle(5.0 * M_PI / 180.0);
  f2c::obj::NSwath objective;
  const double expected = bf.computeBestAngle(objective, 0.16, cell, bf.getOverlapType());
  coverage::PlanningTrace trace;
  const auto plain = coverage::planBoustrophedon(cell, 0.16, 0.18, -1, 0.0, -1.0, 0.15);
  const auto observed = coverage::planBoustrophedon(
      cell, 0.16, 0.18, -1, 0.0, -1.0, 0.15, 0, 0.20, false, 0, std::nullopt, &trace);
  const auto angle = std::find_if(trace.stages.begin(),
                                  trace.stages.end(),
                                  [](const auto& stage)
                                  {
                                    return stage.name == "auto_angle.0";
                                  });
  ASSERT_NE(angle, trace.stages.end());
  ASSERT_EQ(angle->angles.size(), 1u);
  EXPECT_EQ(std::bit_cast<uint64_t>(expected), std::bit_cast<uint64_t>(angle->angles.front()));
  coverage::PivotJoinLimits limits;
  limits.sweep_radius = limits.boundary_margin = 0.597;
  limits.recorded_boundary = {{0, 0}, {8, 0}, {8, 6}, {0, 6}, {0, 0}};
  coverage::ConnectorStats plain_stats, traced_stats;
  const auto normal = coverage::buildContinuousSubPaths(
      plain, plain.connector_clearance_boundary, 0.20, 0.20, 0.03, &plain_stats, {}, limits);
  const auto traced = coverage::buildContinuousSubPaths(observed,
                                                        observed.connector_clearance_boundary,
                                                        0.20,
                                                        0.20,
                                                        0.03,
                                                        &traced_stats,
                                                        {},
                                                        limits,
                                                        false,
                                                        &trace);
  EXPECT_GT(plain_stats.pivot, 0u);
  EXPECT_EQ(plain_stats.pivot, traced_stats.pivot);
  EXPECT_EQ(bits(normal), bits(traced));
  EXPECT_EQ(bits(quaternions(normal)), bits(quaternions(traced)));
}

TEST(PlanningTrace, PreservesSubmillimetreInputBits)
{
  const auto cell = field(false);
  const auto plain = coverage::planBoustrophedon(cell, 0.16, 0.18, -1, 0.123456789, 0.0, 0.15);
  coverage::PlanningTrace trace;
  coverage::planBoustrophedon(
      cell, 0.16, 0.18, -1, 0.123456789, 0.0, 0.15, 0, 0.20, false, 0, std::nullopt, &trace);
  ASSERT_FALSE(trace.stages.empty());
  ASSERT_EQ(trace.stages.front().name, "safe_cell.0");
  EXPECT_EQ(bits(trace.stages.front().paths), bits({plain.safe_boundary}));
  EXPECT_TRUE(std::any_of(plain.safe_boundary.begin(),
                          plain.safe_boundary.end(),
                          [](const auto& p)
                          {
                            return p.first * 1000.0 != std::round(p.first * 1000.0);
                          }));
}
}  // namespace
