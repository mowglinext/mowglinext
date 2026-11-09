// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdint>

#include "fusion_graph/motion_input_support.hpp"
#include <gtest/gtest.h>

namespace
{
using fusion_graph::kMotionInputFutureToleranceNs;
using fusion_graph::MotionInputSupport;
constexpr int64_t kMs = 1000000;
constexpr int64_t kLead = 50 * kMs;  // fusion_graph tf_publish_lead_s

MotionInputSupport Established(int64_t t)
{
  MotionInputSupport s;
  s.Accept(true, t, t, 1);
  s.Accept(false, t, t, 1);
  EXPECT_TRUE(s.Allows(t, t + kLead, 1));
  return s;
}
}  // namespace

TEST(MotionInputSupport, InputsStampedJustAheadOfNowStillAllowPublication)
{
  // Field 2026-11-09: IMU and wheel samples arrive stamped up to 6 ms after
  // their receipt; every one of them used to cancel a TF tick.
  auto s = Established(10000 * kMs);
  for (int64_t k = 1; k <= 50; ++k)
  {
    const int64_t now = 10000 * kMs + k * 10 * kMs;
    s.Accept(false, now + 6 * kMs, now, 1);  // IMU, 100 Hz
    if (k % 7 == 0)
      s.Accept(true, now + 5 * kMs, now, 1);  // wheel odom
    EXPECT_TRUE(s.Allows(now, now + kLead, 1)) << "tick " << k;
  }
  EXPECT_FALSE(s.lost());
}

TEST(MotionInputSupport, InputsFarInTheFutureAreStillRefused)
{
  auto s = Established(10000 * kMs);
  const int64_t now = 10010 * kMs;
  s.Accept(false, now + kMotionInputFutureToleranceNs + kMs, now, 1);
  EXPECT_FALSE(s.Allows(now, now + kLead, 1));
}

TEST(MotionInputSupport, AGapStillWithholdsPublication)
{
  auto s = Established(10000 * kMs);
  const int64_t now = 10000 * kMs + fusion_graph::kMotionSupportGapNs + kMs;
  EXPECT_FALSE(s.Allows(now, now + kLead, 1));
  EXPECT_TRUE(s.lost());
}
