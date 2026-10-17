// Copyright 2026 Mowgli Project
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>

#include "mowgli_coverage/coverage_planning.hpp"
#include "ring_corner_rounding.hpp"
#include <gtest/gtest.h>
#include <ogr_geometry.h>

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
// Test the emitted chords continuously, rather than checking only sampled points.
bool contained(const Pts& path, const Pts& boundary, const std::vector<Pts>& holes)
{
  OGRLineString line;
  for (const auto& p : path)
    line.addPoint(p.first, p.second);
  const auto polygon = [](const Pts& points)
  {
    OGRLinearRing ring;
    for (const auto& p : points)
      ring.addPoint(p.first, p.second);
    ring.closeRings();
    OGRPolygon result;
    result.addRing(&ring);
    return result;
  };
  auto shell = polygon(boundary);
  std::unique_ptr<OGRGeometry> outside(line.Difference(&shell));
  if (!outside || !outside->IsEmpty())
    return false;
  for (const auto& points : holes)
  {
    auto hole = polygon(points);
    if (line.Intersects(&hole))
      return false;
  }
  return true;
}
TEST(RingCornerSmoothing, RetunedNeighbourHoleRollsBackEntireBevelChange)
{
  const Pts pts{{0, 0}, {2, 0}, {2.003, .003}, {2.003, 1}, {0, 1}, {0, 0}};
  const Pts boundary{{-1, -1}, {5, -1}, {5, 5}, {-1, 5}};
  mc::detail::BevelTrace accepted_trace;
  const auto proposed = mc::detail::roundBeveledCorners(
      pts, boundary, {}, M_PI / 6, .4, .2, .03, {}, &accepted_trace);
  ASSERT_EQ(accepted_trace.replacements.size(), 1u);
  const auto dense = densify(proposed, .10);
  ASSERT_TRUE(contained(dense, boundary, {}));
  std::size_t k = 0;
  for (std::size_t i = 1; i + 1 < dense.size(); ++i)
    if (dense[i].first > 1.72 && dense[i].first < 1.99 && dense[i].second > .72 &&
        dense[i].second < .99 && dense[i + 1].second > .72 && dense[i + 1].second < 1.0)
    {
      k = i;
      break;
    }
  ASSERT_GT(k, 0u) << "must select the retuned neighbouring ordinary fillet";
  const double x = (dense[k].first + dense[k + 1].first) / 2;
  const double y = (dense[k].second + dense[k + 1].second) / 2;
  const double e = 1e-5;
  const Pts hole{{x - e, y - e}, {x + e, y - e}, {x + e, y + e}, {x - e, y + e}};
  ASSERT_TRUE(contained(accepted_trace.replacements.front().local, boundary, {hole}));
  ASSERT_FALSE(contained(dense, boundary, {hole}));

  // Keeping only the two microscopic bevel vertices disables their replacement.
  // Their sub-4mm trim budgets already prevent ordinary radius >= .2 fillets;
  // all other corners use the unchanged ordinary-rounding path and arguments.
  const auto preserve_bevel = [&](const auto& point, double)
  {
    return point == pts[1] || point == pts[2];
  };
  const auto legacy =
      mc::detail::roundBeveledCorners(pts, boundary, {hole}, M_PI / 6, .4, .2, .03, preserve_bevel);
  ASSERT_EQ(
      legacy,
      mc::detail::roundBeveledCorners(pts, boundary, {}, M_PI / 6, .4, .2, .03, preserve_bevel));
  ASSERT_TRUE(contained(densify(legacy, .10), boundary, {hole}));
  ASSERT_LT(unsupported(dense), unsupported(densify(legacy, .10)));

  mc::detail::BevelTrace rejected_trace;
  const auto actual = mc::detail::roundBeveledCorners(
      pts, boundary, {hole}, M_PI / 6, .4, .2, .03, {}, &rejected_trace);
  EXPECT_TRUE(contained(densify(actual, .10), boundary, {hole}));
  EXPECT_EQ(actual, legacy);
  EXPECT_EQ(rejected_trace.source, pts);
  EXPECT_EQ(rejected_trace.intermediate, pts);
  EXPECT_EQ(rejected_trace.rounded, legacy);
  EXPECT_TRUE(rejected_trace.replacements.empty());
  EXPECT_EQ(actual, mc::detail::roundBeveledCorners(pts, boundary, {hole}, M_PI / 6, .4, .2, .03));
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
TEST(RingCornerSmoothing, HostedCapturedBevelsAreSingleCirclesWithoutDubinsSeams)
{
  struct Captured
  {
    const char* name;
    Pts source, boundary;
  };
  // Exact S and safe-boundary doubles from hosted run 38993303690, Release,
  // artifact 12086422140. These isolate geometry from F2C/input generation.
  const std::vector<Captured> captured{{"multi_polygon",
                                        {{10.503019531776916, 3.9969699972664756},
                                         {12.997437486710702, 3.993950465489558},
                                         {12.999986916983707, 3.991401035216553},
                                         {12.993960411938465, 0.0025724597382089784},
                                         {12.991398423156863, 1.0470956606782714e-05},
                                         {8.002562513289298, 0.00604953451043948},
                                         {8.000013083016292, 0.008598964783445986},
                                         {8.006039588061535, 3.997427540261797},
                                         {8.00860157684313, 3.999989529043393},
                                         {10.503019531776916, 3.9969699972664756}},
                                        {{7.96938532541079, -0.073910362600903},
                                         {7.9260896373991, -0.030614674589207},
                                         {7.92, 0.0},
                                         {7.9260896373991, 4.03061467458921},
                                         {7.96938532541079, 4.0739103626009},
                                         {8.0, 4.08},
                                         {13.0306146745892, 4.0739103626009},
                                         {13.0739103626009, 4.03061467458921},
                                         {13.08, 4.0},
                                         {13.0739103626009, -0.030614674589207},
                                         {13.0306146745892, -0.073910362600903},
                                         {13.0, -0.08},
                                         {7.96938532541079, -0.073910362600903}}},
                                       {"near_degenerate",
                                        {{4.003028982415013, 5.996964469010116},
                                         {7.997452465605159, 5.993935486595103},
                                         {7.999991271691458, 5.991396680508804},
                                         {7.993943816999605, 0.002555864799343048},
                                         {7.991394500775132, 6.548574870030244e-06},
                                         {0.0025475343948440886, 0.006064513404893529},
                                         {8.728308542183383e-06, 0.008603319491195435},
                                         {0.006056183000392006, 5.997444135200655},
                                         {0.00860549922486762, 5.999993451425129},
                                         {4.003028982415013, 5.996964469010116}},
                                        {{-0.030614674589207, -0.073910362600903},
                                         {-0.073910362600903, -0.030614674589207},
                                         {-0.08, 0.0},
                                         {-0.073910362600903, 6.03061467458921},
                                         {-0.030614674589207, 6.0739103626009},
                                         {0.0, 6.08},
                                         {8.03061467458921, 6.0739103626009},
                                         {8.0739103626009, 6.03061467458921},
                                         {8.08, 6.0},
                                         {8.0739103626009, -0.030614674589207},
                                         {8.03061467458921, -0.073910362600903},
                                         {8.0, -0.08},
                                         {-0.030614674589207, -0.073910362600903}}}};
  for (const auto& fixture : captured)
  {
    SCOPED_TRACE(fixture.name);
    mc::detail::BevelTrace trace;
    const auto rounded = mc::detail::roundBeveledCorners(
        fixture.source, fixture.boundary, {}, M_PI / 6, .4, .2, .03, {}, &trace);
    ASSERT_FALSE(trace.replacements.empty());
    const auto dense = densify(rounded, .10);
    if (unsupported(dense) != 0)
    {
      std::cerr << std::setprecision(17);
      const auto dump = [&](const char* stage, const Pts& points)
      {
        std::cerr << "GEOMETRY " << fixture.name << " " << stage << " [";
        for (std::size_t i = 0; i < points.size(); ++i)
        {
          if (i)
            std::cerr << ",";
          std::cerr << "[" << points[i].first << "," << points[i].second << "]";
        }
        std::cerr << "]\n";
      };
      dump("S", trace.source);
      dump("T", trace.intermediate);
      dump("R", rounded);
      dump("D", dense);
      for (const auto& item : trace.replacements)
      {
        std::cerr << "REPLACEMENT " << fixture.name << " source=" << item.source_vertex
                  << " intermediate=" << item.output_begin << "\n";
        dump("local", item.local);
      }
      for (std::size_t i = 1; i + 1 < dense.size(); ++i)
      {
        // Reuse the unchanged predicate to identify only the failing triples.
        if (unsupported({dense[i - 1], dense[i], dense[i + 1]}) == 0)
          continue;
        const auto a = dense[i - 1], b = dense[i], c = dense[i + 1];
        const double ax = b.first - a.first, ay = b.second - a.second;
        const double bx = c.first - b.first, by = c.second - b.second;
        const double al = std::hypot(ax, ay), bl = std::hypot(bx, by);
        const double turn = std::acos(std::clamp((ax * bx + ay * by) / (al * bl), -1.0, 1.0));
        const double radius = std::hypot(ax + bx, ay + by) / (2 * std::abs(std::sin(turn)));
        std::cerr << "UNSUPPORTED " << fixture.name << " D index=" << i << " lengths=[" << al << ","
                  << bl << "] headings=[" << std::atan2(ay, ax) << "," << std::atan2(by, bx)
                  << "] turn=" << turn << " radius=" << radius << "\n";
        dump("D_triple", {a, b, c});
      }
    }
    EXPECT_EQ(unsupported(densify(rounded, .10)), 0u);
    for (const auto& replacement : trace.replacements)
    {
      const auto& local = replacement.local;
      const auto a = local.front(), b = fixture.source[replacement.source_vertex];
      const auto c = fixture.source[replacement.source_vertex + 1];
      const auto d = fixture.source[replacement.source_vertex + 2];
      const double ix = b.first - a.first, iy = b.second - a.second;
      const double ox = d.first - c.first, oy = d.second - c.second;
      const double t =
          ((c.first - a.first) * oy - (c.second - a.second) * ox) / (ix * oy - iy * ox);
      const std::pair<double, double> corner{a.first + t * ix, a.second + t * iy};
      const double il = std::hypot(corner.first - a.first, corner.second - a.second);
      const double ol = std::hypot(d.first - corner.first, d.second - corner.second);
      const double ux = (corner.first - a.first) / il, uy = (corner.second - a.second) / il;
      const double vx = (d.first - corner.first) / ol, vy = (d.second - corner.second) / ol;
      const double angle = std::acos(std::clamp(ux * vx + uy * vy, -1.0, 1.0));
      const double sign = ux * vy - uy * vx > 0 ? 1.0 : -1.0;
      const double trim =
          (corner.first - local[1].first) * ux + (corner.second - local[1].second) * uy;
      const double radius = trim * std::tan((M_PI - angle) / 2.0);
      EXPECT_GE(radius, .2 - 1e-6);
      EXPECT_LE(radius, .4 + 1e-8);
      const std::pair<double, double> center{local[1].first - sign * uy * radius,
                                             local[1].second + sign * ux * radius};
      const double limit = std::min(.03 / radius, 15 * M_PI / 180);
      const auto segments = static_cast<std::size_t>(std::ceil(angle / limit));
      EXPECT_EQ(local.size(), segments + 3);
      EXPECT_EQ(local.back(), d);
      for (std::size_t i = 1; i + 1 < local.size(); ++i)
      {
        EXPECT_LT(std::abs(
                      std::hypot(local[i].first - center.first, local[i].second - center.second) -
                      radius),
                  1e-8);
        if (i > 1)
        {
          const double px = local[i - 1].first - center.first;
          const double py = local[i - 1].second - center.second;
          const double x = local[i].first - center.first, y = local[i].second - center.second;
          const double delta = sign * std::atan2(px * y - py * x, px * x + py * y);
          EXPECT_GT(delta, 0.0);
          EXPECT_LT(std::abs(delta - angle / segments), 1e-8);
        }
      }
    }
  }
}
TEST(RingCornerSmoothing, CircularSamplerRetainsExistingTinyStepBound)
{
  const Pts points{{0, 2}, {2, 2}, {2.003, 2.003}, {2.003, 4}};
  mc::detail::BevelTrace trace;
  mc::detail::roundBeveledCorners(
      points, {{-1, -1}, {5, -1}, {5, 5}, {-1, 5}}, {}, M_PI / 6, .4, .2, 1e-12, {}, &trace);
  ASSERT_EQ(trace.replacements.size(), 1u);
  EXPECT_LE(trace.replacements.front().local.size(),
            static_cast<std::size_t>(std::ceil(M_PI / 1e-3)) + 3);
}
TEST(RingCornerSmoothing, InvalidSamplingStepKeepsInput)
{
  auto pts = capturedSparseOuterRing();
  for (double step : {0.0, -.1, std::numeric_limits<double>::quiet_NaN()})
    EXPECT_EQ(mc::detail::roundBeveledCorners(pts, kBoundary, {}, 30 * M_PI / 180, .4, .2, step),
              pts);
}

}  // namespace
