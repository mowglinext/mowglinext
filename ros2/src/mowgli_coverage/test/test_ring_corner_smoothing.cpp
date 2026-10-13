// Copyright 2026 Mowgli Project
#include <algorithm>
#include <cmath>
#include <limits>

#include "mowgli_coverage/coverage_planning.hpp"
#include "ring_corner_rounding.hpp"
#include <gtest/gtest.h>

namespace
{
using Pts = std::vector<std::pair<double, double>>;
namespace mc = mowgli_coverage;
Pts capturedSparseOuterRing()
{
  // Exact XY vertices at captured initial-plan indices
  // 0,45,46,106,107,197,198,258,259,304 (9899-pose path, attempt23).
  // Collinear intermediate points are omitted; no vertex is moved.
  return {{0.003030736729810357, 6.79696363235767},
          {4.497455247307262, 6.79393289562786},
          {4.499991271691718, 6.791396871243404},
          {4.493943816998613, 0.802555876719279},
          {4.491393773847642, 0.8000058335683081},
          {-4.4974552473072595, 0.8060673070279263},
          {-4.499991271691718, 0.8086033314123844},
          {-4.493943816998613, 6.797444325936509},
          {-4.4913937738476415, 6.7999943690874805},
          {0.003030736729810357, 6.79696363235767}};
}
Pts densify(const Pts& pts, double step)
{
  Pts out;
  for (std::size_t i = 1; i < pts.size(); ++i)
  {
    auto a = pts[i - 1], b = pts[i];
    int n = std::max(1,
                     static_cast<int>(
                         std::ceil(std::hypot(b.first - a.first, b.second - a.second) / step)));
    for (int k = 0; k < n; ++k)
    {
      double t = static_cast<double>(k) / n;
      out.emplace_back(a.first + t * (b.first - a.first), a.second + t * (b.second - a.second));
    }
  }
  out.push_back(pts.back());
  return out;
}
std::size_t unsupported(const Pts& pts)
{
  std::size_t count = 0;
  for (std::size_t i = 1; i + 1 < pts.size(); ++i)
  {
    double ax = pts[i].first - pts[i - 1].first, ay = pts[i].second - pts[i - 1].second;
    double bx = pts[i + 1].first - pts[i].first, by = pts[i + 1].second - pts[i].second;
    double a = std::hypot(ax, ay), b = std::hypot(bx, by);
    if (a < 1e-4 || b < 1e-4)
      continue;
    double turn = std::acos(std::clamp((ax * bx + ay * by) / (a * b), -1.0, 1.0));
    double chord = std::hypot(ax + bx, ay + by);
    double radius = std::abs(std::sin(turn)) > 1e-9 ? chord / (2 * std::abs(std::sin(turn))) : 1e9;
    if (turn > 15 * M_PI / 180 && radius < .2 - 1e-6)
      ++count;
  }
  return count;
}
const Pts kBoundary{{-4.5, .8}, {4.5, .8}, {4.5, 6.8}, {-4.5, 6.8}};
TEST(RingCornerSmoothing, CapturedMicroscopicBevelsGetSupportedFillets)
{
  auto sparse = capturedSparseOuterRing();
  const auto before = densify(sparse, .1);
  EXPECT_EQ(unsupported(before), 8u);
  auto rounded =
      mc::detail::roundBeveledCorners(sparse, kBoundary, {}, 30 * M_PI / 180, .4, .2, .03);
  auto dense = densify(rounded, .1);
  EXPECT_EQ(unsupported(dense), 0u);
  EXPECT_TRUE(std::all_of(dense.begin(),
                          dense.end(),
                          [](const auto& p)
                          {
                            return mc::pointInRing(p.first, p.second, kBoundary) ||
                                   mc::distanceToRing(p.first, p.second, kBoundary) < 1e-8;
                          }));
  EXPECT_EQ(rounded.front(), sparse.front());
  EXPECT_EQ(rounded.back(), sparse.back());
}
TEST(RingCornerSmoothing, ExistingPivotTwinsAndHeadingsArePreserved)
{
  const Pts pts{{0, 2}, {2, 2}, {2, 2}, {2, 4}};
  auto result = mc::detail::roundBeveledCorners(
      pts, {{-1, -1}, {5, -1}, {5, 5}, {-1, 5}}, {}, 30 * M_PI / 180, .4, .2, .03);
  EXPECT_EQ(result, pts);
  EXPECT_EQ(mc::pathHeadings(result), mc::pathHeadings(pts));
  Pts near = pts;
  near[2].second += .00005;
  EXPECT_EQ(mc::detail::roundBeveledCorners(
                near, {{-1, -1}, {5, -1}, {5, 5}, {-1, 5}}, {}, 30 * M_PI / 180, .4, .2, .03),
            near);
}
TEST(RingCornerSmoothing, LegalPivotsAtActualBevelVerticesRemainSharp)
{
  const auto sparse = capturedSparseOuterRing();
  std::vector<std::pair<double, double>> evaluated;
  const auto keep = [&](const auto& corner, double turn)
  {
    evaluated.push_back(corner);
    EXPECT_GT(turn, 30 * M_PI / 180);
    EXPECT_NE(std::find(sparse.begin(), sparse.end(), corner), sparse.end());
    return true;
  };
  EXPECT_EQ(
      mc::detail::roundBeveledCorners(sparse, kBoundary, {}, 30 * M_PI / 180, .4, .2, .03, keep),
      sparse);
  EXPECT_FALSE(evaluated.empty());
}
TEST(RingCornerSmoothing, RejectedPivotPreservesSupportedBevelFillet)
{
  const auto sparse = capturedSparseOuterRing();
  const auto without_pivots =
      mc::detail::roundBeveledCorners(sparse, kBoundary, {}, 30 * M_PI / 180, .4, .2, .03);
  const auto rejected = mc::detail::roundBeveledCorners(sparse,
                                                        kBoundary,
                                                        {},
                                                        30 * M_PI / 180,
                                                        .4,
                                                        .2,
                                                        .03,
                                                        [](const auto&, double)
                                                        {
                                                          return false;
                                                        });
  EXPECT_EQ(rejected, without_pivots);
  EXPECT_EQ(unsupported(densify(rejected, .1)), 0u);
}
TEST(RingCornerSmoothing, RejectedNarrowBevelRollsBack)
{
  const Pts pts{{0, 0}, {.04, 0}, {.043, .003}, {.043, .04}};
  EXPECT_EQ(mc::detail::roundBeveledCorners(
                pts, {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}, {}, 30 * M_PI / 180, .4, .2, .03),
            pts);
}
TEST(RingCornerSmoothing, HairpinAndZigzagAreNotMerged)
{
  const Pts hairpin{{0, 0}, {2, 0}, {2, .003}, {0, .003}};
  const Pts zigzag{{0, 0}, {2, 0}, {2.003, .003}, {4, .003}};
  EXPECT_EQ(mc::detail::roundBeveledCorners(
                hairpin, {{-1, -1}, {5, -1}, {5, 5}, {-1, 5}}, {}, 30 * M_PI / 180, .4, .2, .03),
            hairpin);
  EXPECT_EQ(mc::detail::roundBeveledCorners(
                zigzag, {{-1, -1}, {5, -1}, {5, 5}, {-1, 5}}, {}, 30 * M_PI / 180, .4, .2, .03),
            zigzag);
}
TEST(RingCornerSmoothing, TinyHoleBetweenSamplesRejectsBevel)
{
  const Pts pts{{0, 2}, {2, 2}, {2.003, 2.003}, {2.003, 4}};
  const Pts boundary{{-1, -1}, {5, -1}, {5, 5}, {-1, 5}};
  auto accepted = mc::detail::roundBeveledCorners(pts, boundary, {}, 30 * M_PI / 180, .4, .2, .03);
  ASSERT_GT(accepted.size(), 4u);
  // A 20-micrometre obstacle at the midpoint of one emitted arc chord.
  auto k = accepted.size() / 2;
  const auto a = accepted[k], b = accepted[k + 1];
  double x = (a.first + b.first) / 2, y = (a.second + b.second) / 2, e = 1e-5;
  Pts hole{{x - e, y - e}, {x + e, y - e}, {x + e, y + e}, {x - e, y + e}};
  EXPECT_EQ(mc::detail::roundBeveledCorners(pts, boundary, {hole}, 30 * M_PI / 180, .4, .2, .03),
            pts);
}
TEST(RingCornerSmoothing, ConcaveBoundaryRejectsCrossingChord)
{
  const Pts pts{{0, 2}, {2, 2}, {2.003, 2.003}, {2.003, 4}};
  auto accepted = mc::detail::roundBeveledCorners(
      pts, {{-1, -1}, {5, -1}, {5, 5}, {-1, 5}}, {}, 30 * M_PI / 180, .4, .2, .03);
  auto k = accepted.size() / 2;
  const auto a = accepted[k], b = accepted[k + 1];
  double x = (a.first + b.first) / 2, y = (a.second + b.second) / 2, e = 1e-5;
  Pts notched{
      {-1, -1}, {5, -1}, {5, 5}, {x + e, 5}, {x + e, y - e}, {x - e, y - e}, {x - e, 5}, {-1, 5}};
  EXPECT_EQ(mc::detail::roundBeveledCorners(pts, notched, {}, 30 * M_PI / 180, .4, .2, .03), pts);
}
TEST(RingCornerSmoothing, RoundingIsIdempotentAndClosureStable)
{
  auto sparse = capturedSparseOuterRing();
  auto first = mc::detail::roundBeveledCorners(sparse, kBoundary, {}, 30 * M_PI / 180, .4, .2, .03);
  auto twice = mc::detail::roundBeveledCorners(first, kBoundary, {}, 30 * M_PI / 180, .4, .2, .03);
  EXPECT_EQ(first, twice);
  EXPECT_EQ(first.front(), sparse.front());
  EXPECT_EQ(first.back(), sparse.back());
}

TEST(RingCornerSmoothing, SameSignedReflexBevelRollsBack)
{
  double in = 170 * M_PI / 180, out = 260 * M_PI / 180;
  const std::pair<double, double> b{2, 2}, c{2 + .001 * std::cos(in), 2 + .001 * std::sin(in)};
  const Pts pts{{1, 2}, b, c, {c.first + std::cos(out), c.second + std::sin(out)}};
  EXPECT_EQ(mc::detail::roundBeveledCorners(
                pts, {{-1, -1}, {5, -1}, {5, 5}, {-1, 5}}, {}, 30 * M_PI / 180, .4, .2, .03),
            pts);
}
TEST(RingCornerSmoothing, InvalidSamplingStepKeepsInput)
{
  auto pts = capturedSparseOuterRing();
  for (double step : {0.0, -.1, std::numeric_limits<double>::quiet_NaN()})
    EXPECT_EQ(mc::detail::roundBeveledCorners(pts, kBoundary, {}, 30 * M_PI / 180, .4, .2, step),
              pts);
}

}  // namespace
