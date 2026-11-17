// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "mowgli_map/ring_repair.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_map::repair_ring;
using mowgli_map::RingRepair;

geometry_msgs::msg::Polygon make(const std::vector<std::pair<double, double>>& xy)
{
  geometry_msgs::msg::Polygon polygon;
  for (const auto& [x, y] : xy)
  {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    polygon.points.push_back(p);
  }
  return polygon;
}

bool valid(const geometry_msgs::msg::Polygon& polygon)
{
  return mowgli_map::ring_repair_detail::is_valid(
      mowgli_map::ring_repair_detail::open_vertices(polygon));
}

bool same(const geometry_msgs::msg::Polygon& a, const geometry_msgs::msg::Polygon& b)
{
  return a.points == b.points;
}

bool contains_vertex(const geometry_msgs::msg::Polygon& polygon, double x, double y)
{
  return std::any_of(polygon.points.begin(),
                     polygon.points.end(),
                     [&](const auto& p)
                     {
                       return std::hypot(p.x - x, p.y - y) < 1e-4;
                     });
}

// The field boundary of 2026-11-09 around its zigzag (vertices 47-58 of the recorded
// ring, map frame), closed with two far corners. Vertices 51-54 go back and forth by
// a few centimetres: edge 51-52 crosses edge 54-55 near (2.27, 10.92).
const std::vector<std::pair<double, double>> kFieldZigzag{
    {2.852370, 7.330140},
    {2.492600, 8.163420},
    {2.256210, 10.616900},
    {2.309440, 11.086600},
    {2.269350, 10.919400},
    {2.294300, 11.018100},
    {2.277000, 10.901600},
    {1.980830, 11.933900},
    {0.116907, 13.707600},
    {-4.0, 13.0},
    {-4.0, 7.0},
};

TEST(RingRepair, LeavesAValidRingUntouched)
{
  const auto square = make({{0, 0}, {4, 0}, {4, 3}, {0, 3}});
  const auto result = repair_ring(square);
  EXPECT_EQ(result.status, RingRepair::kAlreadyValid);
  EXPECT_TRUE(same(result.polygon, square));
}

TEST(RingRepair, AcceptsAnExplicitlyClosedRingAsIs)
{
  const auto closed = make({{0, 0}, {4, 0}, {4, 3}, {0, 3}, {0, 0}});
  const auto result = repair_ring(closed);
  EXPECT_EQ(result.status, RingRepair::kAlreadyValid);
  EXPECT_TRUE(same(result.polygon, closed));
}

TEST(RingRepair, FieldZigzagIsInvalidAsRecorded)
{
  EXPECT_FALSE(valid(make(kFieldZigzag)));
}

TEST(RingRepair, RemovesTheFieldZigzagLoop)
{
  const auto recorded = make(kFieldZigzag);
  const auto result = repair_ring(recorded);
  ASSERT_EQ(result.status, RingRepair::kRepaired);
  EXPECT_EQ(result.loops_removed, 1u);
  EXPECT_TRUE(valid(result.polygon));
  // Only the loop's vertices went; every other recorded vertex is still there.
  for (std::size_t i : {0u, 1u, 2u, 3u, 7u, 8u, 9u, 10u})
    EXPECT_TRUE(contains_vertex(result.polygon, kFieldZigzag[i].first, kFieldZigzag[i].second))
        << "vertex " << i;
  EXPECT_LT(result.polygon.points.size(), recorded.points.size());
}

TEST(RingRepair, RemovesALoopThatWrapsAroundTheFirstVertex)
{
  // Same ring, rotated so the zigzag straddles index 0 / n-1: the small side of the
  // crossing is then the chain through the closing edge.
  auto rotated = kFieldZigzag;
  std::rotate(rotated.begin(), rotated.begin() + 5, rotated.end());
  const auto result = repair_ring(make(rotated));
  ASSERT_EQ(result.status, RingRepair::kRepaired);
  EXPECT_TRUE(valid(result.polygon));
  for (std::size_t i : {0u, 1u, 2u, 3u, 7u, 8u, 9u, 10u})
    EXPECT_TRUE(contains_vertex(result.polygon, kFieldZigzag[i].first, kFieldZigzag[i].second))
        << "vertex " << i;
}

TEST(RingRepair, RefusesToGuessAtALargeCrossing)
{
  // A bow-tie: both sides of the crossing are metres long. Not a recording artefact.
  const auto bow_tie = make({{0, 0}, {4, 3}, {4, 0}, {0, 3}});
  const auto result = repair_ring(bow_tie);
  EXPECT_EQ(result.status, RingRepair::kUnrepairable);
  EXPECT_TRUE(same(result.polygon, bow_tie));
}

TEST(RingRepair, RefusesALoopJustOverTheBound)
{
  // A rectangular detour whose loop perimeter is 0.6 m (> kMaxRepairLoopPerimeterM).
  const auto ring = make({{0, 0},
                          {4, 0},
                          {4, 3},
                          {2.0, 3},
                          {2.0, 3.2},
                          {2.1, 3.2},
                          {2.1, 2.9},
                          {1.9, 2.9},
                          {1.9, 3},
                          {0, 3}});
  EXPECT_FALSE(valid(ring));
  const auto result = repair_ring(ring);
  EXPECT_NE(result.status, RingRepair::kRepaired);
}

}  // namespace
