// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pins that a freshly dispatched plan is tracked from its start, and that a
// CLOSED headland ring can never resolve to its end index.

#include <cmath>
#include <utility>
#include <vector>

#include "mowgli_nav2_plugins/ftc_start_index.hpp"
#include <gtest/gtest.h>

using mowgli_nav2_plugins::ChooseStartIndex;

namespace
{

/// A closed headland ring: start == end, exactly as the coverage server emits.
/// Modelled on the real one — 436 poses, start=end=(1.95, 9.50).
std::vector<std::pair<double, double>> ClosedRing(std::size_t n = 436)
{
  std::vector<std::pair<double, double>> ring;
  ring.reserve(n);
  const double cx = 1.95, cy = 8.50, r = 1.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    // Sweep a full turn so the last point lands back on the first.
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n - 1);
    ring.emplace_back(cx + r * std::sin(a), cy + r * std::cos(a));
  }
  return ring;
}

}  // namespace

TEST(FtcStartIndex, FreshPlanAlwaysStartsAtZero)
{
  const auto ring = ClosedRing();
  // Robot sitting on the ring's start/end point — the exact 2026-08-24 case.
  EXPECT_EQ(ChooseStartIndex(false, ring, ring.front().first, ring.front().second), 0u);
  // ...and even parked deep inside the ring, a dispatched plan starts at 0.
  EXPECT_EQ(ChooseStartIndex(false, ring, ring[300].first, ring[300].second), 0u);
}

// The regression. Both 2026-08-24 runs snapped to the ring's END index
// (432 and 433 of 436) and skipped the entire ring while reporting it MOWED.
TEST(FtcStartIndex, ClosedRingNeverResolvesToItsEndIndex)
{
  const auto ring = ClosedRing();
  ASSERT_NEAR(ring.front().first, ring.back().first, 1e-9) << "fixture must be closed";
  ASSERT_NEAR(ring.front().second, ring.back().second, 1e-9) << "fixture must be closed";

  // Even with the legacy snap enabled, the tie must break toward the start.
  const std::size_t idx = ChooseStartIndex(true, ring, ring.front().first, ring.front().second);
  EXPECT_EQ(idx, 0u) << "a closed ring must start at its beginning, not its end";
  EXPECT_LT(idx, ring.size() - 2) << "must never skip essentially the whole ring";
}

TEST(FtcStartIndex, LegacySnapStillFindsAGenuineMidPathPoint)
{
  // An open path — legacy behaviour is unambiguous here and must be preserved.
  std::vector<std::pair<double, double>> line;
  for (int i = 0; i < 100; ++i)
  {
    line.emplace_back(static_cast<double>(i) * 0.1, 0.0);
  }
  EXPECT_EQ(ChooseStartIndex(true, line, 5.0, 0.01), 50u);
  EXPECT_EQ(ChooseStartIndex(false, line, 5.0, 0.01), 0u) << "default ignores the snap";
}

TEST(FtcStartIndex, EmptyPlanIsSafe)
{
  const std::vector<std::pair<double, double>> empty;
  EXPECT_EQ(ChooseStartIndex(true, empty, 1.0, 2.0), 0u);
  EXPECT_EQ(ChooseStartIndex(false, empty, 1.0, 2.0), 0u);
}
