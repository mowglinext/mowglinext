// SPDX-License-Identifier: GPL-3.0

#include <cmath>

#include "mowgli_openmower_bridge/imu_bias_estimator.hpp"
#include <gtest/gtest.h>

using mowgli_openmower_bridge::ImuBiasEstimator;
using mowgli_openmower_bridge::ImuSample;

TEST(ImuBiasEstimator, NotCollectingIgnoresSamples)
{
  ImuBiasEstimator e(4u);
  EXPECT_FALSE(e.Feed(ImuSample{0, 0, 9.81, 0.01, 0, 0}));
  EXPECT_FALSE(e.ready());
  const ImuSample raw{1, 2, 3, 4, 5, 6};
  const auto out = e.Apply(raw);
  EXPECT_DOUBLE_EQ(out.gx, 4.0);
}

TEST(ImuBiasEstimator, AveragesAndSubtractsButKeepsGravity)
{
  ImuBiasEstimator e(2u);
  e.Start();
  EXPECT_TRUE(e.collecting());
  EXPECT_FALSE(e.Feed(ImuSample{0.1, -0.2, 9.8, 0.01, 0.02, 0.03}));
  EXPECT_TRUE(e.Feed(ImuSample{0.3, -0.4, 9.9, 0.03, 0.02, 0.01}));
  EXPECT_TRUE(e.ready());
  EXPECT_FALSE(e.collecting());
  EXPECT_DOUBLE_EQ(e.bias().ax, 0.2);
  EXPECT_DOUBLE_EQ(e.bias().ay, -0.3);
  EXPECT_DOUBLE_EQ(e.bias().gz, 0.02);
  const auto out = e.Apply(ImuSample{0.2, -0.3, 9.85, 0.02, 0.02, 0.02});
  EXPECT_NEAR(out.ax, 0.0, 1e-12);
  EXPECT_NEAR(out.ay, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(out.az, 9.85);  // never corrected
  EXPECT_NEAR(out.gz, 0.0, 1e-12);
  EXPECT_GT(e.bias().gz_variance, 0.0);
}

TEST(ImuBiasEstimator, AbortDropsPartialCollection)
{
  ImuBiasEstimator e(3u);
  e.Start();
  (void)e.Feed(ImuSample{});
  e.Abort();
  EXPECT_FALSE(e.collecting());
  EXPECT_FALSE(e.ready());
  EXPECT_EQ(e.count(), 0u);
}

TEST(ImuBiasEstimator, DefaultCalibrationRequiresTwoHundredSamples)
{
  ImuBiasEstimator e;
  const ImuSample raw{0.1, -0.2, 9.81, 0.01, -0.02, 0.03};
  e.Start();
  for (std::size_t count = 1u; count < 200u; ++count)
  {
    SCOPED_TRACE(count);
    EXPECT_FALSE(e.Feed(raw));
    EXPECT_TRUE(e.collecting());
    EXPECT_FALSE(e.ready());
    EXPECT_EQ(e.count(), count);
    const auto out = e.Apply(raw);
    EXPECT_DOUBLE_EQ(out.ax, raw.ax);
    EXPECT_DOUBLE_EQ(out.ay, raw.ay);
    EXPECT_DOUBLE_EQ(out.gx, raw.gx);
    EXPECT_DOUBLE_EQ(out.gy, raw.gy);
    EXPECT_DOUBLE_EQ(out.gz, raw.gz);
  }

  EXPECT_TRUE(e.Feed(raw));
  EXPECT_EQ(e.count(), 200u);
  EXPECT_TRUE(e.ready());
  EXPECT_FALSE(e.collecting());
  EXPECT_NEAR(e.bias().gz, raw.gz, 1e-12);
  const auto out = e.Apply(raw);
  EXPECT_NEAR(out.ax, 0.0, 1e-12);
  EXPECT_NEAR(out.ay, 0.0, 1e-12);
  EXPECT_NEAR(out.gx, 0.0, 1e-12);
  EXPECT_NEAR(out.gy, 0.0, 1e-12);
  EXPECT_NEAR(out.gz, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(out.az, raw.az);
}

TEST(ImuBiasEstimator, AbortedDefaultCalibrationRequiresTwoHundredNewSamples)
{
  ImuBiasEstimator e;
  e.Start();
  for (std::size_t count = 0u; count < 150u; ++count)
  {
    EXPECT_FALSE(e.Feed(ImuSample{0, 0, 9.81, 0, 0, 0.5}));
  }
  e.Abort();
  EXPECT_EQ(e.count(), 0u);
  EXPECT_FALSE(e.collecting());
  EXPECT_FALSE(e.ready());

  const ImuSample raw{0, 0, 9.81, 0, 0, 0.03};
  e.Start();
  for (std::size_t count = 1u; count < 200u; ++count)
  {
    SCOPED_TRACE(count);
    EXPECT_FALSE(e.Feed(raw));
    EXPECT_TRUE(e.collecting());
    EXPECT_FALSE(e.ready());
    EXPECT_EQ(e.count(), count);
    EXPECT_DOUBLE_EQ(e.Apply(raw).gz, raw.gz);
  }
  EXPECT_TRUE(e.Feed(raw));
  EXPECT_EQ(e.count(), 200u);
  EXPECT_TRUE(e.ready());
  EXPECT_FALSE(e.collecting());
  EXPECT_NEAR(e.bias().gz, raw.gz, 1e-12);
  EXPECT_NEAR(e.Apply(raw).gz, 0.0, 1e-12);
}

TEST(ImuBiasEstimator, NonFiniteSamplesAreSkipped)
{
  ImuBiasEstimator e(1u);
  e.Start();
  EXPECT_FALSE(e.Feed(ImuSample{std::nan(""), 0, 0, 0, 0, 0}));
  EXPECT_TRUE(e.Feed(ImuSample{0, 0, 0, 0, 0, 0.5}));
  EXPECT_DOUBLE_EQ(e.bias().gz, 0.5);
}
