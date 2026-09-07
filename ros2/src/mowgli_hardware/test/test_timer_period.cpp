// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <limits>
#include <stdexcept>

#include "mowgli_hardware/timer_period.hpp"
#include <gtest/gtest.h>

namespace mowgli_hardware
{

TEST(TimerPeriod, NormalHardwareBridgeDefaultsKeepTheirPeriods)
{
  EXPECT_EQ(timer_period_from_rate_hz(4.0), std::chrono::milliseconds(250));
  EXPECT_EQ(timer_period_from_rate_hz(100.0), std::chrono::milliseconds(10));
  EXPECT_EQ(timer_period_from_rate_hz(2.0), std::chrono::milliseconds(500));
  EXPECT_EQ(timer_period_from_rate_hz(10.0), std::chrono::milliseconds(100));
}

TEST(TimerPeriod, RejectsZeroNegativeAndNonFiniteRates)
{
  EXPECT_THROW(timer_period_from_rate_hz(0.0), std::invalid_argument);
  EXPECT_THROW(timer_period_from_rate_hz(-1.0), std::invalid_argument);
  EXPECT_THROW(timer_period_from_rate_hz(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(timer_period_from_rate_hz(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_THROW(timer_period_from_rate_hz(-std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST(TimerPeriod, OneNanosecondPeriodIsTheFastestAcceptedCadence)
{
  EXPECT_EQ(timer_period_from_rate_hz(1.0e9).count(), 1);
  EXPECT_THROW(timer_period_from_rate_hz(std::nextafter(maximum_timer_rate_hz(),
                                                        std::numeric_limits<double>::infinity())),
               std::invalid_argument);
  EXPECT_THROW(timer_period_from_rate_hz(std::numeric_limits<double>::max()),
               std::invalid_argument);
}

TEST(TimerPeriod, HandlesRepresentablePeriodBoundaryWithoutOverflow)
{
  const double minimum = minimum_timer_rate_hz();
  EXPECT_GT(timer_period_from_rate_hz(minimum).count(), 0);
  EXPECT_LE(timer_period_from_rate_hz(minimum), TimerDuration::max());
  EXPECT_NO_THROW(
      timer_period_from_rate_hz(std::nextafter(minimum, std::numeric_limits<double>::infinity())));
  EXPECT_THROW(timer_period_from_rate_hz(std::nextafter(minimum, 0.0)), std::invalid_argument);
}

TEST(TimerPeriod, ClampsOperationalTimerRateBounds)
{
  EXPECT_EQ(normalize_operational_timer_rate(4.0, kHeartbeatRateRange), 4.0);
  EXPECT_EQ(normalize_operational_timer_rate(100.0, kPublishRateRange), 100.0);
  EXPECT_EQ(normalize_operational_timer_rate(2.0, kHighLevelRateRange), 2.0);
  EXPECT_EQ(normalize_operational_timer_rate(10.0, kDigMonitorRateRange), 10.0);
  EXPECT_EQ(normalize_operational_timer_rate(5.0, kPublishRateRange), kPublishRateRange.minimum_hz);
  EXPECT_EQ(normalize_operational_timer_rate(501.0, kPublishRateRange),
            kPublishRateRange.maximum_hz);
}

TEST(TimerPeriod, RejectsInvalidDivisorsAndTimeouts)
{
  EXPECT_EQ(normalize_runtime_wheel_track(0.325), 0.325);
  EXPECT_EQ(normalize_runtime_wheel_track(0.14), kMinRuntimeWheelTrackM);
  EXPECT_EQ(normalize_runtime_wheel_track(0.61), kMaxRuntimeWheelTrackM);
  EXPECT_THROW(normalize_runtime_wheel_track(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(normalize_runtime_wheel_track(std::numeric_limits<double>::infinity()),
               std::invalid_argument);

  EXPECT_THROW(normalize_operational_timer_rate(0.0, kPublishRateRange), std::invalid_argument);
  EXPECT_THROW(normalize_operational_timer_rate(-1.0, kPublishRateRange), std::invalid_argument);
  EXPECT_THROW(normalize_operational_timer_rate(std::numeric_limits<double>::quiet_NaN(),
                                                kPublishRateRange),
               std::invalid_argument);
  EXPECT_THROW(normalize_operational_timer_rate(std::numeric_limits<double>::infinity(),
                                                kPublishRateRange),
               std::invalid_argument);

  EXPECT_NO_THROW(require_finite_nonnegative_timeout(0.0, "serial_rx_timeout_s"));
  EXPECT_NO_THROW(require_finite_positive(1.0, "dig_pose_timeout_s"));
  EXPECT_THROW(require_finite_nonnegative_timeout(-1.0, "serial_rx_timeout_s"),
               std::invalid_argument);
  EXPECT_THROW(require_finite_nonnegative_timeout(std::numeric_limits<double>::quiet_NaN(),
                                                  "serial_rx_timeout_s"),
               std::invalid_argument);
  EXPECT_THROW(require_finite_nonnegative_timeout(std::numeric_limits<double>::infinity(),
                                                  "serial_rx_timeout_s"),
               std::invalid_argument);
  EXPECT_THROW(require_finite_positive(0.0, "dig_pose_timeout_s"), std::invalid_argument);
  EXPECT_THROW(require_finite_positive(-1.0, "dig_pose_timeout_s"), std::invalid_argument);
  EXPECT_THROW(require_finite_positive(std::numeric_limits<double>::quiet_NaN(),
                                       "dig_pose_timeout_s"),
               std::invalid_argument);
  EXPECT_THROW(require_finite_positive(std::numeric_limits<double>::infinity(),
                                       "dig_pose_timeout_s"),
               std::invalid_argument);
}

}  // namespace mowgli_hardware
