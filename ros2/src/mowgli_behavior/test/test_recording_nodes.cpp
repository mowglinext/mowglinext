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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_recording_nodes.cpp
 * @brief Unit tests for RecordArea pure algorithm methods.
 *
 * Tests the static helper functions (perpendicular_distance, polygon_area,
 * douglas_peucker) in isolation without any BT tick() or ROS2 spin.
 * The test class is declared as a friend of RecordArea to access the private
 * static methods directly.
 */

#include <cmath>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point32.hpp"
#include "mowgli_behavior/recording_nodes.hpp"
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Global ROS2 init/shutdown
// ---------------------------------------------------------------------------

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

static geometry_msgs::msg::Point32 make_point(float x, float y)
{
  geometry_msgs::msg::Point32 pt;
  pt.x = x;
  pt.y = y;
  pt.z = 0.0f;
  return pt;
}

// ---------------------------------------------------------------------------
// Test fixture — friend of RecordArea so we can call private static methods
// ---------------------------------------------------------------------------

class RecordAreaAlgorithmTest : public ::testing::Test
{
protected:
  // Wrappers that forward to RecordArea's private static methods.

  static double perpendicular_distance(const geometry_msgs::msg::Point32& pt,
                                       const geometry_msgs::msg::Point32& line_start,
                                       const geometry_msgs::msg::Point32& line_end)
  {
    return mowgli_behavior::RecordArea::perpendicular_distance(pt, line_start, line_end);
  }

  static double polygon_area(const std::vector<geometry_msgs::msg::Point32>& points)
  {
    return mowgli_behavior::RecordArea::polygon_area(points);
  }

  static std::vector<geometry_msgs::msg::Point32> douglas_peucker(
      const std::vector<geometry_msgs::msg::Point32>& points, double tolerance)
  {
    return mowgli_behavior::RecordArea::douglas_peucker(points, tolerance);
  }

  static bool is_sample_far_enough(const geometry_msgs::msg::Point32& last,
                                   const geometry_msgs::msg::Point32& candidate)
  {
    return mowgli_behavior::RecordArea::is_sample_far_enough(last, candidate);
  }

  /// Replay a densely-sampled drive through RecordArea's minimum-spacing gate,
  /// exactly as record_position() does, and return the kept trajectory.
  static std::vector<geometry_msgs::msg::Point32> apply_spacing_gate(
      const std::vector<geometry_msgs::msg::Point32>& samples)
  {
    std::vector<geometry_msgs::msg::Point32> kept;
    for (const auto& s : samples)
    {
      if (kept.empty() || is_sample_far_enough(kept.back(), s))
      {
        kept.push_back(s);
      }
    }
    return kept;
  }
};

namespace
{
constexpr double kTolerance = mowgli_behavior::RecordArea::kDefaultSimplificationToleranceM;
constexpr double kOldTolerance = 0.2;  // the pre-fix hardcoded value
constexpr double kMinSpacing = mowgli_behavior::RecordArea::kMinSampleSpacingM;

/// Sample a closed rectangular boundary at `step` metres, as a recording drive
/// at `speed` and a given sample rate would. Perimeter = 2 * (w + h).
std::vector<geometry_msgs::msg::Point32> sample_rectangle(float w, float h, double step)
{
  std::vector<geometry_msgs::msg::Point32> pts;
  const std::vector<std::pair<geometry_msgs::msg::Point32, geometry_msgs::msg::Point32>> edges = {
      {make_point(0.0f, 0.0f), make_point(w, 0.0f)},
      {make_point(w, 0.0f), make_point(w, h)},
      {make_point(w, h), make_point(0.0f, h)},
      {make_point(0.0f, h), make_point(0.0f, 0.0f)},
  };
  for (const auto& e : edges)
  {
    const double len = std::hypot(e.second.x - e.first.x, e.second.y - e.first.y);
    const int n = static_cast<int>(len / step);
    for (int i = 0; i < n; ++i)
    {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      pts.push_back(make_point(static_cast<float>(e.first.x + t * (e.second.x - e.first.x)),
                               static_cast<float>(e.first.y + t * (e.second.y - e.first.y))));
    }
  }
  return pts;
}
}  // namespace

// ===========================================================================
// Boundary-resolution regression tests
//
// Field report: a recorded 38.13 m perimeter came back with only 24 vertices
// (~1.6 m apart) because simplification_tolerance was 0.2 m — wider than
// tool_width — and record_rate_hz was 2 Hz, so the 0.05 m spacing gate never
// bound and the sample rate was the real limiter.
// ===========================================================================

TEST_F(RecordAreaAlgorithmTest, DefaultToleranceIsBelowToolWidthAndAtSamplingFloor)
{
  // The tolerance IS the worst-case boundary error (DP's guarantee), so it must
  // stay well under one cutting swath. tool_width defaults to 0.18 m and is
  // 0.15 m on some builds; the tolerance must clear the smaller of those.
  constexpr double kNarrowestToolWidth = 0.15;
  EXPECT_LT(kTolerance, kNarrowestToolWidth)
      << "boundary error must never exceed a full cutting swath";
  EXPECT_LE(kTolerance / kNarrowestToolWidth, 0.5)
      << "tolerance should be comfortably below tool_width, not merely under it";

  // Below the sampling floor DP cannot recover detail that was never sampled.
  EXPECT_GE(kTolerance, kMinSpacing) << "a tolerance finer than the sample spacing buys nothing";
}

TEST_F(RecordAreaAlgorithmTest, DefaultRateMakesSpacingGateBindAtMaxSpeed)
{
  // max_mps (0.5 m/s) is a hard runtime wheel-speed ceiling pushed to the STM32,
  // so it bounds how fast the robot can be driven while recording. The gate
  // binds only when the per-sample travel is at or below the spacing floor.
  constexpr double kMaxMps = 0.5;
  const double rate = mowgli_behavior::RecordArea::kDefaultRecordRateHz;
  EXPECT_LE(kMaxMps / rate, kMinSpacing)
      << "at max_mps the sample rate, not the 5 cm gate, would be the limiter";

  // And the old 2 Hz default did NOT bind — this is the bug being fixed.
  EXPECT_GT(kMaxMps / 2.0, kMinSpacing);

  // onRunning() runs inside the BT tick, so tick_rate is the achievable ceiling.
  // A default above it would be silently unattainable.
  constexpr double kDefaultTickRate = 10.0;
  EXPECT_LE(rate, kDefaultTickRate) << "default record rate must be achievable at the BT tick rate";
}

TEST_F(RecordAreaAlgorithmTest, NewTolerancePreservesFeatureThatOldToleranceDestroyed)
{
  // A 0.12 m bump on an otherwise straight 4 m run — smaller than the old 0.2 m
  // tolerance, larger than the new 0.05 m one. This is exactly the class of
  // garden-boundary detail (a shrub, a bed corner) that used to vanish.
  std::vector<geometry_msgs::msg::Point32> path;
  for (int i = 0; i <= 40; ++i)
  {
    const float x = static_cast<float>(i) * 0.1f;
    const float y = (i == 20) ? 0.12f : 0.0f;
    path.push_back(make_point(x, y));
  }

  const auto old_result = douglas_peucker(path, kOldTolerance);
  EXPECT_EQ(old_result.size(), 2u) << "old tolerance flattened the bump into a straight line";

  const auto new_result = douglas_peucker(path, kTolerance);
  ASSERT_GT(new_result.size(), 2u) << "new tolerance must retain the bump";

  bool bump_kept = false;
  for (const auto& p : new_result)
  {
    if (std::abs(p.y - 0.12f) < 1e-4f)
    {
      bump_kept = true;
    }
  }
  EXPECT_TRUE(bump_kept) << "the apex of the feature must survive simplification";
}

TEST_F(RecordAreaAlgorithmTest, SpacingGateCollapsesDuplicatesAndSubThresholdSamples)
{
  const auto origin = make_point(1.0f, 2.0f);

  // Exact duplicate and near-duplicates below the floor are rejected.
  EXPECT_FALSE(is_sample_far_enough(origin, origin));
  EXPECT_FALSE(is_sample_far_enough(origin, make_point(1.02f, 2.0f)));
  EXPECT_FALSE(is_sample_far_enough(origin, make_point(1.0f, 2.049f)));

  // At/above the floor they are kept — including on a diagonal.
  EXPECT_TRUE(is_sample_far_enough(origin, make_point(1.06f, 2.0f)));
  EXPECT_TRUE(is_sample_far_enough(origin, make_point(1.04f, 2.04f)));

  // Replaying a stationary robot (TF jitter only) must not grow the trajectory.
  std::vector<geometry_msgs::msg::Point32> jitter;
  for (int i = 0; i < 100; ++i)
  {
    jitter.push_back(make_point(1.0f + 0.003f * static_cast<float>(i % 3), 2.0f));
  }
  EXPECT_EQ(apply_spacing_gate(jitter).size(), 1u)
      << "a parked robot must record exactly one point";
}

TEST_F(RecordAreaAlgorithmTest, RealisticPerimeterYieldsUsefulButBoundedVertexCount)
{
  // A 38.13 m perimeter, matching the operator's recorded_area_1. Sampled at
  // 10 Hz while driving 0.3 m/s => 0.03 m raw steps, which the gate thins.
  const float w = 11.0f;
  const float h = 8.065f;  // 2 * (11 + 8.065) = 38.13 m
  const auto raw = sample_rectangle(w, h, 0.03);
  const auto kept = apply_spacing_gate(raw);

  // The gate — not the sample rate — is what sets the spacing.
  for (size_t i = 1; i < kept.size(); ++i)
  {
    const double d = std::hypot(kept[i].x - kept[i - 1].x, kept[i].y - kept[i - 1].y);
    EXPECT_GE(d, kMinSpacing - 1e-6) << "gate must enforce the floor at index " << i;
    EXPECT_LT(d, kMinSpacing + 0.03 + 1e-6) << "spacing must not exceed floor + one sample step";
  }
  EXPECT_GT(kept.size(), 600u) << "38 m at ~6 cm spacing should retain hundreds of raw samples";

  // DP then collapses the straight runs. A boundary this simple must come back
  // near its true corner count — and must NOT explode into hundreds of vertices
  // that would bloat areas.dat and slow F2C/costmap polygon work.
  const auto simplified = douglas_peucker(kept, kTolerance);
  EXPECT_LE(simplified.size(), 40u) << "straight runs must collapse; no vertex explosion";

  // The old settings are what produced the 24-vertex field result; the new
  // pipeline must do better than ~1.6 m between vertices on a real boundary.
  const double perimeter = 2.0 * (static_cast<double>(w) + static_cast<double>(h));
  EXPECT_LT(perimeter / static_cast<double>(kept.size()), 0.1)
      << "recorded resolution must be well under a cutting swath";
}

TEST_F(RecordAreaAlgorithmTest, DegenerateTrajectoryStillTrippedByMinVerticesAndMinArea)
{
  // Guard 1: a trajectory that cannot form a polygon. douglas_peucker returns
  // it untouched (< 3 points), so the min_vertices check in onRunning() sees a
  // sub-minimum count and bails rather than saving garbage.
  const std::vector<geometry_msgs::msg::Point32> two_points = {make_point(0.0f, 0.0f),
                                                               make_point(1.0f, 0.0f)};
  const auto simplified_pair = douglas_peucker(two_points, kTolerance);
  EXPECT_EQ(simplified_pair.size(), 2u);
  EXPECT_LT(simplified_pair.size(), 3u) << "must fail the min_vertices=3 guard";
  EXPECT_DOUBLE_EQ(polygon_area(simplified_pair), 0.0);

  // Guard 2: a collinear drive (robot pushed in a straight line, never closing
  // a loop) simplifies to its endpoints and has zero area.
  std::vector<geometry_msgs::msg::Point32> straight;
  for (int i = 0; i <= 30; ++i)
  {
    straight.push_back(make_point(static_cast<float>(i) * 0.06f, 0.0f));
  }
  const auto simplified_straight = douglas_peucker(straight, kTolerance);
  EXPECT_EQ(simplified_straight.size(), 2u);
  EXPECT_NEAR(polygon_area(simplified_straight), 0.0, 1e-6) << "must fail the min_area=1.0 guard";

  // Guard 3: a real but tiny loop keeps its vertices yet is still under
  // min_area, so it is rejected on area rather than slipping through.
  const auto tiny = sample_rectangle(0.4f, 0.4f, 0.03);
  const auto simplified_tiny = douglas_peucker(apply_spacing_gate(tiny), kTolerance);
  EXPECT_GE(simplified_tiny.size(), 3u);
  EXPECT_LT(polygon_area(simplified_tiny), 1.0) << "~0.15 m^2 must fail the min_area=1.0 guard";
}

// ===========================================================================
// perpendicular_distance tests
// ===========================================================================

TEST_F(RecordAreaAlgorithmTest, PerpendicularDistance_PointOnLine)
{
  // Midpoint of line from (0,0) to (10,0) is (5,0) — distance should be 0
  auto pt = make_point(5.0f, 0.0f);
  auto start = make_point(0.0f, 0.0f);
  auto end = make_point(10.0f, 0.0f);
  EXPECT_NEAR(perpendicular_distance(pt, start, end), 0.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PerpendicularDistance_PerpendicularToMidpoint)
{
  // Point (5, 3) is 3 units above the line from (0,0) to (10,0)
  auto pt = make_point(5.0f, 3.0f);
  auto start = make_point(0.0f, 0.0f);
  auto end = make_point(10.0f, 0.0f);
  EXPECT_NEAR(perpendicular_distance(pt, start, end), 3.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PerpendicularDistance_DegenerateLine)
{
  // When start == end, distance should be Euclidean distance to the point
  auto pt = make_point(3.0f, 4.0f);
  auto start = make_point(0.0f, 0.0f);
  auto end = make_point(0.0f, 0.0f);
  EXPECT_NEAR(perpendicular_distance(pt, start, end), 5.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PerpendicularDistance_PointPastEndpoints)
{
  // Point (15, 3) is past the end of line from (0,0) to (10,0).
  // The perpendicular distance to the *infinite line* is still 3.
  auto pt = make_point(15.0f, 3.0f);
  auto start = make_point(0.0f, 0.0f);
  auto end = make_point(10.0f, 0.0f);
  EXPECT_NEAR(perpendicular_distance(pt, start, end), 3.0, 1e-6);
}

// ===========================================================================
// polygon_area tests
// ===========================================================================

TEST_F(RecordAreaAlgorithmTest, PolygonArea_UnitSquare)
{
  // Counter-clockwise unit square
  std::vector<geometry_msgs::msg::Point32> square = {
      make_point(0.0f, 0.0f),
      make_point(1.0f, 0.0f),
      make_point(1.0f, 1.0f),
      make_point(0.0f, 1.0f),
  };
  EXPECT_NEAR(polygon_area(square), 1.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PolygonArea_Triangle)
{
  // Triangle with vertices (0,0), (4,0), (0,3) — area = 0.5 * 4 * 3 = 6
  std::vector<geometry_msgs::msg::Point32> triangle = {
      make_point(0.0f, 0.0f),
      make_point(4.0f, 0.0f),
      make_point(0.0f, 3.0f),
  };
  EXPECT_NEAR(polygon_area(triangle), 6.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PolygonArea_LargerRectangle)
{
  // 5 x 3 rectangle = area 15
  std::vector<geometry_msgs::msg::Point32> rect = {
      make_point(0.0f, 0.0f),
      make_point(5.0f, 0.0f),
      make_point(5.0f, 3.0f),
      make_point(0.0f, 3.0f),
  };
  EXPECT_NEAR(polygon_area(rect), 15.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PolygonArea_LessThan3Points)
{
  // 0 points
  EXPECT_NEAR(polygon_area({}), 0.0, 1e-6);

  // 1 point
  std::vector<geometry_msgs::msg::Point32> one = {make_point(1.0f, 2.0f)};
  EXPECT_NEAR(polygon_area(one), 0.0, 1e-6);

  // 2 points
  std::vector<geometry_msgs::msg::Point32> two = {
      make_point(0.0f, 0.0f),
      make_point(1.0f, 1.0f),
  };
  EXPECT_NEAR(polygon_area(two), 0.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PolygonArea_ClockwiseVsCounterclockwise)
{
  // CCW unit square
  std::vector<geometry_msgs::msg::Point32> ccw = {
      make_point(0.0f, 0.0f),
      make_point(1.0f, 0.0f),
      make_point(1.0f, 1.0f),
      make_point(0.0f, 1.0f),
  };

  // CW unit square (reversed winding)
  std::vector<geometry_msgs::msg::Point32> cw = {
      make_point(0.0f, 0.0f),
      make_point(0.0f, 1.0f),
      make_point(1.0f, 1.0f),
      make_point(1.0f, 0.0f),
  };

  EXPECT_NEAR(polygon_area(ccw), polygon_area(cw), 1e-6);
  EXPECT_NEAR(polygon_area(ccw), 1.0, 1e-6);
}

// ===========================================================================
// douglas_peucker tests
// ===========================================================================

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_StraightLine)
{
  // Collinear points along the x-axis — should simplify to just endpoints
  std::vector<geometry_msgs::msg::Point32> line;
  for (int i = 0; i <= 10; ++i)
  {
    line.push_back(make_point(static_cast<float>(i), 0.0f));
  }

  auto result = douglas_peucker(line, 0.1);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_FLOAT_EQ(result.front().x, 0.0f);
  EXPECT_FLOAT_EQ(result.back().x, 10.0f);
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_LShapedPath)
{
  // L-shape: (0,0) -> (5,0) -> (5,5) with intermediate collinear points
  std::vector<geometry_msgs::msg::Point32> path = {
      make_point(0.0f, 0.0f),
      make_point(1.0f, 0.0f),
      make_point(2.0f, 0.0f),
      make_point(3.0f, 0.0f),
      make_point(4.0f, 0.0f),
      make_point(5.0f, 0.0f),
      make_point(5.0f, 1.0f),
      make_point(5.0f, 2.0f),
      make_point(5.0f, 3.0f),
      make_point(5.0f, 4.0f),
      make_point(5.0f, 5.0f),
  };

  auto result = douglas_peucker(path, 0.1);
  // Should keep at least 3 points: start, corner, end
  ASSERT_GE(result.size(), 3u);

  // First and last must be preserved
  EXPECT_FLOAT_EQ(result.front().x, 0.0f);
  EXPECT_FLOAT_EQ(result.front().y, 0.0f);
  EXPECT_FLOAT_EQ(result.back().x, 5.0f);
  EXPECT_FLOAT_EQ(result.back().y, 5.0f);

  // The corner point (5,0) must be retained
  bool corner_found = false;
  for (const auto& pt : result)
  {
    if (std::abs(pt.x - 5.0f) < 1e-3f && std::abs(pt.y - 0.0f) < 1e-3f)
    {
      corner_found = true;
      break;
    }
  }
  EXPECT_TRUE(corner_found) << "Corner point (5,0) should be retained";
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_LessThan3Points)
{
  // 0 points
  auto r0 = douglas_peucker({}, 0.1);
  EXPECT_TRUE(r0.empty());

  // 1 point
  std::vector<geometry_msgs::msg::Point32> one = {make_point(1.0f, 2.0f)};
  auto r1 = douglas_peucker(one, 0.1);
  ASSERT_EQ(r1.size(), 1u);
  EXPECT_FLOAT_EQ(r1[0].x, 1.0f);

  // 2 points
  std::vector<geometry_msgs::msg::Point32> two = {
      make_point(0.0f, 0.0f),
      make_point(5.0f, 5.0f),
  };
  auto r2 = douglas_peucker(two, 0.1);
  ASSERT_EQ(r2.size(), 2u);
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_HighTolerance)
{
  // L-shape with very high tolerance — should aggressively simplify to 2 endpoints
  std::vector<geometry_msgs::msg::Point32> path = {
      make_point(0.0f, 0.0f),
      make_point(5.0f, 0.0f),
      make_point(5.0f, 5.0f),
  };

  auto result = douglas_peucker(path, 100.0);
  // With tolerance larger than the corner deviation, only endpoints are kept
  EXPECT_EQ(result.size(), 2u);
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_ZeroTolerance)
{
  // With zero tolerance, every point must be kept
  std::vector<geometry_msgs::msg::Point32> path;
  for (int i = 0; i <= 5; ++i)
  {
    // Slight zigzag so each point deviates from the line between its neighbors
    float y = (i % 2 == 0) ? 0.0f : 0.01f;
    path.push_back(make_point(static_cast<float>(i), y));
  }

  auto result = douglas_peucker(path, 0.0);
  EXPECT_EQ(result.size(), path.size());
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_CircleLikeShape)
{
  // Approximate a circle with 36 points (every 10 degrees)
  const float radius = 5.0f;
  const int n_points = 36;
  std::vector<geometry_msgs::msg::Point32> circle;
  for (int i = 0; i < n_points; ++i)
  {
    float angle =
        static_cast<float>(i) * 2.0f * static_cast<float>(M_PI) / static_cast<float>(n_points);
    circle.push_back(make_point(radius * std::cos(angle), radius * std::sin(angle)));
  }

  // Moderate simplification — should retain some points but fewer than original
  auto result = douglas_peucker(circle, 0.5);
  EXPECT_GT(result.size(), 4u) << "Circle should retain enough points for shape";
  EXPECT_LT(result.size(), circle.size()) << "Simplification should reduce point count";

  // Verify first and last points are preserved
  EXPECT_FLOAT_EQ(result.front().x, circle.front().x);
  EXPECT_FLOAT_EQ(result.front().y, circle.front().y);
  EXPECT_FLOAT_EQ(result.back().x, circle.back().x);
  EXPECT_FLOAT_EQ(result.back().y, circle.back().y);
}
