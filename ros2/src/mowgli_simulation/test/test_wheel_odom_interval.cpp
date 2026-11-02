// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <array>
#include <cmath>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

// The upstream header selects its accumulator using these version macros.
// clang-format off
#include <rcpputils/version.h>
#include <diff_drive_controller/odometry.hpp>
// clang-format on

namespace
{
constexpr double kDt = 0.02;
constexpr double kTolerance = 1e-10;

// Recover the body Twist from the independently integrated pose interval.
// A trailing velocity mean describes older intervals during acceleration,
// braking and turns, despite carrying the current odometry header.
int CountIntervalMismatches(diff_drive_controller::Odometry& odometry, double track, double radius)
{
  odometry.setWheelParams(track, radius, radius);
  double left = 0;
  double right = 0;
  const std::array<std::array<double, 2>, 6> commands{
      {{0, 0}, {0.15, 0}, {0.15, -0.6}, {0, -0.6}, {-0.1, 0.2}, {0, 0}}};
  int mismatches = 0;
  for (const auto& command : commands)
  {
    for (int sample = 0; sample < 5; ++sample)
    {
      const double x = odometry.getX();
      const double y = odometry.getY();
      const double yaw = odometry.getHeading();
      left += (command[0] - command[1] * track / 2) * kDt / radius;
      right += (command[0] + command[1] * track / 2) * kDt / radius;
      EXPECT_TRUE(odometry.update_from_pos(left, right, kDt));
      const double turn = odometry.getHeading() - yaw;
      const double half = turn / 2;
      const double sinc = half == 0 ? 1 : std::sin(half) / half;
      const double velocity = ((odometry.getX() - x) * std::cos(yaw + half) +
                               (odometry.getY() - y) * std::sin(yaw + half)) /
                              (kDt * sinc);
      if (std::abs(odometry.getLinear() - velocity) > kTolerance ||
          std::abs(odometry.getAngular() - turn / kDt) > kTolerance)
        ++mismatches;
    }
  }
  return mismatches;
}

TEST(WheelOdometryInterval, ConfiguredTwistMatchesItsPoseInterval)
{
  const auto config = YAML::LoadFile(WHEEL_ODOM_CONFIG)["diffdrive_controller"]["ros__parameters"];
  diff_drive_controller::Odometry odometry;
  if (config["velocity_rolling_window_size"])
    odometry.setVelocityRollingWindowSize(config["velocity_rolling_window_size"].as<std::size_t>());
  EXPECT_EQ(CountIntervalMismatches(odometry,
                                    config["wheel_separation"].as<double>(),
                                    config["wheel_radius"].as<double>()),
            0);
}

TEST(WheelOdometryInterval, TrailingMeanControlDetectsTemporalMismatch)
{
  const auto config = YAML::LoadFile(WHEEL_ODOM_CONFIG)["diffdrive_controller"]["ros__parameters"];
  diff_drive_controller::Odometry odometry(10);
  EXPECT_GT(CountIntervalMismatches(odometry,
                                    config["wheel_separation"].as<double>(),
                                    config["wheel_radius"].as<double>()),
            0);
}
}  // namespace
