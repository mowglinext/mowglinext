// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_request_stamps.cpp
 * @brief Reply sample times of the request/response xESC mini.
 */

#include <chrono>

#include "mowgli_openmower_bridge/request_stamps.hpp"
#include <gtest/gtest.h>

using mowgli_openmower_bridge::RequestStampQueue;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

TEST(RequestStampQueue, AReplyCarriesTheTimeOfItsRequestNotOfThePoll)
{
  RequestStampQueue q;
  const auto t0 = Clock::time_point{} + milliseconds(1000);
  q.OnRequest(t0);
  EXPECT_EQ(q.OnResponse(t0 + milliseconds(20)), t0);
  EXPECT_EQ(q.outstanding(), 0u);
}

// After a 350 ms stall the reply read first was asked for BEFORE the stall:
// the interval between the two sample times must be the real one.
TEST(RequestStampQueue, AStallDoesNotShiftTheIntervalByOnePoll)
{
  RequestStampQueue q;
  const auto t0 = Clock::time_point{} + milliseconds(1000);
  q.OnRequest(t0);  // tick k-1, then the process is frozen
  const auto after_stall = t0 + milliseconds(350);
  const auto first = q.OnResponse(after_stall);  // read at tick k
  q.OnRequest(after_stall);
  const auto second = q.OnResponse(after_stall + milliseconds(20));  // tick k+1
  EXPECT_EQ(first, t0);
  EXPECT_EQ(second - first, milliseconds(350));
}

TEST(RequestStampQueue, RepliesPairWithRequestsInOrder)
{
  RequestStampQueue q;
  const auto t0 = Clock::time_point{} + milliseconds(1000);
  q.OnRequest(t0);
  q.OnRequest(t0 + milliseconds(20));
  EXPECT_EQ(q.OnResponse(t0 + milliseconds(45)), t0);
  EXPECT_EQ(q.OnResponse(t0 + milliseconds(45)), t0 + milliseconds(20));
}

TEST(RequestStampQueue, UnsolicitedReplyUsesThePollTime)
{
  RequestStampQueue q;
  const auto now = Clock::time_point{} + milliseconds(5);
  EXPECT_EQ(q.OnResponse(now), now);
}

TEST(RequestStampQueue, LostRepliesNeverGrowTheQueueWithoutBound)
{
  RequestStampQueue q;
  const auto t0 = Clock::time_point{} + milliseconds(1000);
  for (int i = 0; i < 100; ++i)
  {
    q.OnRequest(t0 + milliseconds(20 * i));
  }
  EXPECT_EQ(q.outstanding(), RequestStampQueue::kCapacity);
  // The oldest kept request is the most recent kCapacity ones' first.
  EXPECT_EQ(q.OnResponse(t0 + milliseconds(5000)), t0 + milliseconds(20 * (100 - 8)));
}
