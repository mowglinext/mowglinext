// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// Regression guard for the coverage join/transit gap (task #14). The PLANNING
// side (mowgli_coverage/coverage_planning.cpp) splits a continuous drivable
// sub-path into a separate transit whenever a connector join exceeds this
// gap; the EXECUTION side (FollowStrip, coverage_nodes.hpp) independently
// re-decides transit-vs-drive-through at runtime using the same threshold.
// Both now read mowgli_interfaces::coverage_geometry::kSegmentTransitGapM, so
// they cannot drift apart at compile time — this test pins that wiring (and
// the value itself) so a future edit that reintroduces a local literal on
// either side is caught immediately rather than silently reopening the
// blade-on-diagonal-crossing / redundant-transit failure modes described in
// coverage_geometry.hpp.

#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_interfaces/coverage_geometry.hpp"
#include <gtest/gtest.h>

TEST(CoverageTransitGap, ExecutionSideMatchesSharedConstant)
{
  EXPECT_DOUBLE_EQ(mowgli_behavior::FollowStrip::kSegmentTransitGap,
                   mowgli_interfaces::coverage_geometry::kSegmentTransitGapM);
}

TEST(CoverageTransitGap, ValueIsPinnedAndSane)
{
  // Pin the actual value (0.6 m) so a change here is a deliberate, reviewed
  // edit rather than an accidental one — and sanity-bound it: a gap this
  // large already relies on the connector-arc / relocation split described
  // in coverage_geometry.hpp, and adjacent swaths are ~one operation_width
  // (commonly ~0.16-0.20 m) apart, so the threshold must comfortably clear
  // that spacing without approaching a size that would swallow real
  // relocations as if they were ordinary turn-arounds.
  EXPECT_DOUBLE_EQ(mowgli_interfaces::coverage_geometry::kSegmentTransitGapM, 0.6);
  EXPECT_GT(mowgli_interfaces::coverage_geometry::kSegmentTransitGapM, 0.0);
  EXPECT_LT(mowgli_interfaces::coverage_geometry::kSegmentTransitGapM, 2.0);
}
