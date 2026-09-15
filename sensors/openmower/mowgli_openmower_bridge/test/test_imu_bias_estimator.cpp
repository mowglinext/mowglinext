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

TEST(ImuBiasEstimator, NonFiniteSamplesAreSkipped)
{
  ImuBiasEstimator e(1u);
  e.Start();
  EXPECT_FALSE(e.Feed(ImuSample{std::nan(""), 0, 0, 0, 0, 0}));
  EXPECT_TRUE(e.Feed(ImuSample{0, 0, 0, 0, 0, 0.5}));
  EXPECT_DOUBLE_EQ(e.bias().gz, 0.5);
}
