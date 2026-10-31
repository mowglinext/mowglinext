// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdlib>
#include <filesystem>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace
{
class GyroPreintBias : public ::testing::TestWithParam<std::pair<double, int>>
{
};

TEST_P(GyroPreintBias, LearnedAbsoluteBiasIsSubtractedOnceInNextWindow)
{
  const auto [bias, samples] = GetParam();
  fusion_graph::GraphParams params;
  params.node_period_s = 0.1;
  params.stationary_node_period_s = 0.0;
  params.use_imu_preint = true;
  params.prior_sigma_theta = 1e-6;
  params.gyro_noise_density_rad_per_s = 1e-4;
  params.gyro_bias_prior_sigma_rad_per_s = .5;
  params.gyro_bias_rw_rad_per_s = 1e-6;
  params.adaptive_noise_enabled_gain = 0.0;
  fusion_graph::GraphManager graph(params);
  graph.Initialize(gtsam::Pose2(), 0.0);
  const double dt = 1.0 / samples;
  const double true_yaw_rate = .1;
  for (int window = 1; window <= 2; ++window)
  {
    for (int sample = 0; sample < samples; ++sample)
    {
      graph.AddWheelTwist(.15, 0.0, true_yaw_rate, dt);
      graph.AddGyroDelta(true_yaw_rate + bias, dt);
    }
    // One independent yaw observation identifies the absolute bias in the
    // first window. The second has no absolute observation to hide a repeated
    // subtraction of that same bias during preintegration and factor scoring.
    if (window == 1)
      graph.QueueYaw(true_yaw_rate, 1e-6, false);
    const auto result = graph.Tick(window);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->pose.theta(), window * true_yaw_rate, 1e-6);
    EXPECT_TRUE(graph.IsInitialized());
  }
}

struct TemporaryDirectory
{
  char path[64] = "/tmp/mowgli-preint-lifecycle-XXXXXX";
  TemporaryDirectory()
  {
    if (!mkdtemp(path))
      throw std::runtime_error("mkdtemp failed");
  }
  ~TemporaryDirectory()
  {
    std::filesystem::remove_all(path);
  }
};

fusion_graph::GraphParams LifecycleParams()
{
  fusion_graph::GraphParams params;
  params.node_period_s = .01;
  params.stationary_node_period_s = 0.0;
  params.use_imu_preint = true;
  return params;
}

void DriveLifecycle(fusion_graph::GraphManager& graph, double stamp)
{
  graph.AddWheelTwist(.15, 0.0, .1, .1);
  graph.AddGyroDelta(.1, .1);
  ASSERT_TRUE(graph.Tick(stamp));
}

TEST(GyroPreintLifecycle, ReanchorRetainsAValidBiasPredecessor)
{
  fusion_graph::GraphManager graph(LifecycleParams());
  graph.Initialize(gtsam::Pose2(), 0.0);
  DriveLifecycle(graph, .1);
  graph.RigidTransformAll(gtsam::Pose2(.1, .2, .01));
  DriveLifecycle(graph, .2);
  EXPECT_TRUE(graph.IsInitialized());
}

TEST(GyroPreintLifecycle, ReloadRetainsAValidBiasPredecessor)
{
  TemporaryDirectory temporary;
  const auto prefix = std::string(temporary.path) + "/graph";
  fusion_graph::GraphManager before(LifecycleParams());
  before.Initialize(gtsam::Pose2(), 0.0);
  DriveLifecycle(before, .1);
  ASSERT_TRUE(before.Save(prefix));
  fusion_graph::GraphManager restored(LifecycleParams());
  ASSERT_TRUE(restored.Load(prefix));
  DriveLifecycle(restored, .2);
  EXPECT_TRUE(restored.IsInitialized());
}

TEST(GyroPreintLifecycle, ResetThenReloadDoesNotReusePreviousSessionBias)
{
  TemporaryDirectory temporary;
  const auto prefix = std::string(temporary.path) + "/graph";
  auto params = LifecycleParams();
  params.prior_sigma_theta = 1e-6;
  params.gyro_noise_density_rad_per_s = 1e-4;
  params.gyro_bias_prior_sigma_rad_per_s = .5;
  params.gyro_bias_rw_rad_per_s = 1e-6;
  params.adaptive_noise_enabled_gain = 0.0;
  fusion_graph::GraphManager reused(params);
  reused.Initialize(gtsam::Pose2(), 0.0);
  reused.AddWheelTwist(.15, 0.0, .1, 1.0);
  reused.AddGyroDelta(.12, 1.0);
  reused.QueueYaw(.1, 1e-6, false);
  ASSERT_TRUE(reused.Tick(1.0));
  ASSERT_TRUE(reused.Save(prefix));
  reused.Reset();
  ASSERT_TRUE(reused.Load(prefix));
  fusion_graph::GraphManager fresh(params);
  ASSERT_TRUE(fresh.Load(prefix));
  for (auto* graph : {&reused, &fresh})
  {
    // New session's independently calibrated gyro is unbiased. Persisted
    // trajectory poses do not authorize carrying the previous sensor bias.
    graph->AddWheelTwist(.15, 0.0, .1, 1.0);
    graph->AddGyroDelta(.1, 1.0);
    ASSERT_TRUE(graph->Tick(2.0));
  }
  ASSERT_TRUE(reused.LatestSnapshot());
  ASSERT_TRUE(fresh.LatestSnapshot());
  EXPECT_NEAR(reused.LatestSnapshot()->pose.theta(), .2, 1e-6);
  EXPECT_NEAR(reused.LatestSnapshot()->pose.theta(), fresh.LatestSnapshot()->pose.theta(), 1e-9);
}

INSTANTIATE_TEST_SUITE_P(PositiveNegativeAndSplitSamples,
                         GyroPreintBias,
                         ::testing::Values(std::pair(.02, 1),
                                           std::pair(-.03, 1),
                                           std::pair(.02, 25),
                                           std::pair(-.03, 25)));
}  // namespace
