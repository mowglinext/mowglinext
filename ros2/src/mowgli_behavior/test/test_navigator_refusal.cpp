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
/**
 * @file test_navigator_refusal.cpp
 * @brief A refused navigation goal is re-sent after a pause, a bounded number of times.
 *
 * Issue #943: instant refusals from a busy navigator spent the area's whole dispatch budget in
 * one second. See navigator_refusal.hpp.
 */

#include <chrono>

#include "mowgli_behavior/navigator_refusal.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::NavigatorRefusalRetry;
using Clock = NavigatorRefusalRetry::Clock;

namespace
{
Clock::duration seconds(double s)
{
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
}
}  // namespace

TEST(NavigatorRefusal, NothingIsPendingBeforeARefusal)
{
  const NavigatorRefusalRetry retry;
  EXPECT_FALSE(retry.pending());
  EXPECT_FALSE(retry.due(Clock::now()));
  EXPECT_EQ(retry.retries(), 0);
}

TEST(NavigatorRefusal, ARefusalSchedulesAReSendOnlyAfterThePause)
{
  NavigatorRefusalRetry retry;
  const auto t0 = Clock::now();

  ASSERT_TRUE(retry.onRefused(t0));

  EXPECT_TRUE(retry.pending());
  EXPECT_FALSE(retry.due(t0));
  EXPECT_FALSE(retry.due(t0 + seconds(NavigatorRefusalRetry::kRetryDelaySec - 0.1)));
  EXPECT_TRUE(retry.due(t0 + seconds(NavigatorRefusalRetry::kRetryDelaySec)));
  EXPECT_EQ(retry.retries(), 1);
}

TEST(NavigatorRefusal, ConsumingTheReSendClearsIt)
{
  NavigatorRefusalRetry retry;
  const auto t0 = Clock::now();
  ASSERT_TRUE(retry.onRefused(t0));

  retry.consume();

  EXPECT_FALSE(retry.pending());
  EXPECT_FALSE(retry.due(t0 + seconds(60.0)));
  EXPECT_EQ(retry.retries(), 1);
}

TEST(NavigatorRefusal, TheBudgetEndsAfterTheMaximumReSends)
{
  NavigatorRefusalRetry retry;
  auto t = Clock::now();
  for (int i = 0; i < NavigatorRefusalRetry::kMaxRetries; ++i)
  {
    ASSERT_TRUE(retry.onRefused(t)) << "re-send " << i + 1;
    t += seconds(NavigatorRefusalRetry::kRetryDelaySec);
    ASSERT_TRUE(retry.due(t));
    retry.consume();
  }

  EXPECT_FALSE(retry.onRefused(t));
  EXPECT_FALSE(retry.pending());
}

TEST(NavigatorRefusal, TheWholeBudgetOutlastsANavigatorWindingDownAGoal)
{
  // The point of the policy: the attempt now lasts long enough for a cancelled goal to stop
  // (its BT, a running recovery), instead of milliseconds.
  EXPECT_GE(NavigatorRefusalRetry::kRetryDelaySec * NavigatorRefusalRetry::kMaxRetries, 10.0);
}

TEST(NavigatorRefusal, ResetGivesANewDispatchTheFullBudget)
{
  NavigatorRefusalRetry retry;
  const auto t0 = Clock::now();
  for (int i = 0; i < NavigatorRefusalRetry::kMaxRetries; ++i)
  {
    ASSERT_TRUE(retry.onRefused(t0));
  }
  ASSERT_FALSE(retry.onRefused(t0));

  retry.reset();

  EXPECT_FALSE(retry.pending());
  EXPECT_EQ(retry.retries(), 0);
  EXPECT_TRUE(retry.onRefused(t0));
}
