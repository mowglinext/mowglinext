// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "process_window_policy.hpp"
#include <gtest/gtest.h>

namespace
{
using namespace fusion_graph;

ProcessWindowPolicyState AlmostFull()
{
  ProcessWindowPolicyState state;
  state.slip_window = SlipWindow(5);
  for (int i = 0; i < 4; ++i)
    state.slip_window.Push(.05, 0.0);
  return state;
}

GraphParams Params()
{
  GraphParams params;
  params.node_period_s = .1;
  return params;
}

ProcessWindowPolicyInput Moving()
{
  ProcessWindowPolicyInput input;
  input.dx = input.wheel_travel_m = .02;
  input.dtheta_wheel = .05;
  input.dt_total = .1;
  input.last_node_time_s = 1.0;
  input.now_s = 1.1;
  return input;
}

TEST(ProcessWindowPolicy, SourceDurationDoesNotClearAStillCurrentScheduleWindow)
{
  const auto original = AlmostFull();
  auto input = Moving();
  input.dt_total = 1.0;
  const auto selected = SelectProcessWindowPolicy(Params(), input, original);
  EXPECT_EQ(original.slip_window.Size(), 4u);
  EXPECT_EQ(original.slip_veto_count, 0u);
  EXPECT_EQ(selected.state.slip_window.Size(), 5u);
  EXPECT_TRUE(selected.slip_detected);
  EXPECT_FALSE(selected.wheel_translation_enabled);
  EXPECT_EQ(selected.state.slip_veto_count, 1u);
  EXPECT_DOUBLE_EQ(selected.fallback_mean.x(), 0.0);
}

TEST(ProcessWindowPolicy, ScheduleGapClearsEvenAShortSourceWindow)
{
  const auto original = AlmostFull();
  auto input = Moving();
  input.now_s = 2.0;
  const auto selected = SelectProcessWindowPolicy(Params(), input, original);
  EXPECT_EQ(original.slip_window.Size(), 4u);
  EXPECT_EQ(selected.state.slip_window.Size(), 1u);
  EXPECT_FALSE(selected.slip_detected);
  EXPECT_TRUE(selected.wheel_translation_enabled);
  EXPECT_EQ(selected.state.slip_veto_count, 0u);
  EXPECT_DOUBLE_EQ(selected.fallback_mean.x(), input.dx);
}

TEST(ProcessWindowPolicy, RepeatedPrivateSelectionDoesNotAdvanceLiveHistoryOrFeedback)
{
  const auto original = AlmostFull();
  for (int i = 0; i < 10; ++i)
  {
    const auto selected = SelectProcessWindowPolicy(Params(), Moving(), original);
    EXPECT_TRUE(selected.slip_detected);
    EXPECT_EQ(selected.state.slip_window.Size(), 5u);
    EXPECT_EQ(selected.state.slip_veto_count, 1u);
    EXPECT_EQ(original.slip_window.Size(), 4u);
    EXPECT_EQ(original.slip_veto_count, 0u);
    EXPECT_DOUBLE_EQ(original.residual_ema, 0.0);
  }
}

TEST(ProcessWindowPolicy, PreintegrationRetainsSourceRotationOnStationarySelection)
{
  ProcessWindowPolicyInput input;
  input.dt_total = .1;
  input.dtheta_gyro = .001;
  input.max_abs_gyro_rad_per_s = .01;
  input.now_s = .1;
  auto params = Params();
  const ProcessWindowPolicyState state;
  const auto ordinary = SelectProcessWindowPolicy(params, input, state);
  params.use_imu_preint = true;
  const auto preintegrated = SelectProcessWindowPolicy(params, input, state);
  EXPECT_TRUE(ordinary.state.wheel_stationary_now);
  EXPECT_TRUE(preintegrated.state.wheel_stationary_now);
  EXPECT_FALSE(ordinary.gyro_rotation_enabled);
  EXPECT_TRUE(preintegrated.gyro_rotation_enabled);
  EXPECT_DOUBLE_EQ(ordinary.wheel_sigma_theta, params.stationary_sigma_theta);
  EXPECT_DOUBLE_EQ(preintegrated.wheel_sigma_theta, .5);
}
}  // namespace
