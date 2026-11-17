// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
#include "mowgli_map/peer_dock_rings.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_map::classify_peer_dock;
using mowgli_map::convex_ring_clearance;
using mowgli_map::convex_rings_overlap;
using mowgli_map::PeerDockUse;

geometry_msgs::msg::Polygon rect(float x0, float y0, float x1, float y1)
{
  geometry_msgs::msg::Polygon polygon;
  for (const auto& xy : {std::pair<float, float>{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}})
  {
    geometry_msgs::msg::Point32 p;
    p.x = xy.first;
    p.y = xy.second;
    polygon.points.push_back(p);
  }
  return polygon;
}

const auto kOwnBody = rect(-0.12F, -0.275F, 0.68F, 0.275F);
const auto kOwnCorridor = rect(-1.5F, -0.4F, -0.12F, 0.4F);
constexpr double kMargin = 0.275;

TEST(PeerDockRings, OverlapAndClearanceOfConvexRings)
{
  EXPECT_TRUE(convex_rings_overlap(rect(0, 0, 1, 1), rect(0.5F, 0.5F, 2, 2)));
  // A cross: overlapping with no vertex of either inside the other.
  EXPECT_TRUE(convex_rings_overlap(rect(-2, -0.1F, 2, 0.1F), rect(-0.1F, -2, 0.1F, 2)));
  EXPECT_FALSE(convex_rings_overlap(rect(0, 0, 1, 1), rect(1.5F, 0, 2, 1)));
  EXPECT_NEAR(convex_ring_clearance(rect(0, 0, 1, 1), rect(1.5F, 0, 2, 1)), 0.5, 1e-6);
  EXPECT_DOUBLE_EQ(convex_ring_clearance(rect(0, 0, 1, 1), rect(0.5F, 0.5F, 2, 2)), 0.0);
}

TEST(PeerDockRings, AStationFarFromOurDockIsForbiddenWithItsMargin)
{
  EXPECT_EQ(classify_peer_dock(rect(5, 5, 5.8F, 5.55F), kOwnBody, kOwnCorridor, kMargin),
            PeerDockUse::kFull);
}

// The 2026-10-28 site: docks 0.53 m apart. The margin would cover our own approach axis.
TEST(PeerDockRings, DocksSideBySideKeepOnlyTheObjectSoOurApproachStaysOpen)
{
  const auto peer = rect(-0.12F, 0.45F, 0.68F, 1.0F);
  EXPECT_EQ(classify_peer_dock(peer, kOwnBody, kOwnCorridor, kMargin), PeerDockUse::kBareRing);
}

// A bad peer pose (or a copied config) must never block our own dock or undock.
TEST(PeerDockRings, AStationOnOurDockOrCorridorIsIgnored)
{
  EXPECT_EQ(classify_peer_dock(kOwnBody, kOwnBody, kOwnCorridor, kMargin), PeerDockUse::kIgnored);
  EXPECT_EQ(classify_peer_dock(rect(-1.0F, 0, -0.5F, 0.3F), kOwnBody, kOwnCorridor, kMargin),
            PeerDockUse::kIgnored);
}

TEST(PeerDockRings, WithoutADockOfOurOwnNothingCanBeBlocked)
{
  const geometry_msgs::msg::Polygon none;
  EXPECT_EQ(classify_peer_dock(kOwnBody, none, none, kMargin), PeerDockUse::kFull);
}

}  // namespace
