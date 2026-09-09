// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>

#include "mowgli_nav2_plugins/ftc_carrot.hpp"
#include <gtest/gtest.h>

namespace mnp = mowgli_nav2_plugins;

TEST(FtcCarrot, DeviationDoesNotMutateOrAccumulateInNominalCarrot)
{
  Eigen::Affine3d nominal = Eigen::Affine3d::Identity();
  nominal.translation() = Eigen::Vector3d(1.0, 2.0, 0.0);

  const Eigen::Affine3d first = mnp::LaterallyDeviatedCarrot(nominal, 0.5);
  const Eigen::Affine3d second = mnp::LaterallyDeviatedCarrot(nominal, 0.5);

  EXPECT_NEAR(nominal.translation().x(), 1.0, 1e-12);
  EXPECT_NEAR(nominal.translation().y(), 2.0, 1e-12);
  EXPECT_NEAR(first.translation().x(), 1.0, 1e-12);
  EXPECT_NEAR(first.translation().y(), 2.5, 1e-12);
  EXPECT_TRUE(first.isApprox(second));
}

TEST(FtcCarrot, DeviationUsesNominalCarrotHeading)
{
  Eigen::Affine3d nominal = Eigen::Affine3d::Identity();
  nominal.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const Eigen::Affine3d deviated = mnp::LaterallyDeviatedCarrot(nominal, 0.5);

  EXPECT_NEAR(deviated.translation().x(), -0.5, 1e-12);
  EXPECT_NEAR(deviated.translation().y(), 0.0, 1e-12);
}
