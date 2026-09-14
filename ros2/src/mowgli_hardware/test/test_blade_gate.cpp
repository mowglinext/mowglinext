// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the `mowing_enabled` blade gate. Pure logic, no ROS —
// mirrors test_dig_detector.cpp.

#include "mowgli_hardware/blade_gate.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

// Normal operation: mowing_enabled = true is a pass-through in both directions.
TEST(BladeGate, EnablePassesWhenMowingEnabled)
{
  EXPECT_TRUE(mh::blade_enable_allowed(/*requested_enable=*/true, /*mowing_enabled=*/true));
}

// Dry run: an ENABLE is suppressed so a full mission can be driven with the
// blade never spinning.
TEST(BladeGate, EnableSuppressedWhenMowingDisabled)
{
  EXPECT_FALSE(mh::blade_enable_allowed(/*requested_enable=*/true, /*mowing_enabled=*/false));
}

// SAFETY-CRITICAL: a DISABLE must never be swallowed, in EITHER state. If this
// ever fails, the convenience inhibit has become a path that can drop a stop —
// the exact false-safety-promise bug it exists to avoid. The firmware remains
// the sole blade safety authority regardless.
TEST(BladeGate, DisableAlwaysPassesThrough)
{
  EXPECT_FALSE(mh::blade_enable_allowed(/*requested_enable=*/false, /*mowing_enabled=*/true));
  EXPECT_FALSE(mh::blade_enable_allowed(/*requested_enable=*/false, /*mowing_enabled=*/false));
}

// The gate is constexpr — a compile-time check that it stays pure (no state, no
// I/O), which is what makes it safe to call from the service handler.
TEST(BladeGate, IsConstexpr)
{
  static_assert(mh::blade_enable_allowed(true, true), "enable+enabled must be allowed");
  static_assert(!mh::blade_enable_allowed(true, false), "enable+disabled must be suppressed");
  static_assert(!mh::blade_enable_allowed(false, true), "disable must never be flipped to enable");
  SUCCEED();
}
