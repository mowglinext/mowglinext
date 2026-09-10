// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pins the body → map rotation of the Pose2 marginal covariance, and the
// frame-invariance of MaxPositionSigma. See covariance_frame.hpp.

#include <cmath>

#include "fusion_graph/covariance_frame.hpp"
#include <Eigen/Core>
#include <gtest/gtest.h>

using fusion_graph::BodyToMapCovariance;
using fusion_graph::MaxPositionSigma;

namespace
{

/// The non-holonomic wheel between-factor shape this robot actually runs:
/// wheel_sigma_x 0.05 along-track, wheel_sigma_y 0.005 cross-track.
Eigen::Matrix3d NonHolonomicBodyCov()
{
  Eigen::Matrix3d c = Eigen::Matrix3d::Zero();
  c(0, 0) = 0.05 * 0.05;  // along-track variance
  c(1, 1) = 0.005 * 0.005;  // cross-track variance
  c(2, 2) = 0.0008;  // yaw variance
  return c;
}

}  // namespace

TEST(CovarianceFrame, ZeroYawIsIdentity)
{
  const Eigen::Matrix3d body = NonHolonomicBodyCov();
  const Eigen::Matrix3d map = BodyToMapCovariance(body, 0.0);

  EXPECT_TRUE(map.isApprox(body, 1e-12));
}

TEST(CovarianceFrame, QuarterTurnSwapsAlongAndCrossTrack)
{
  const Eigen::Matrix3d body = NonHolonomicBodyCov();
  const Eigen::Matrix3d map = BodyToMapCovariance(body, M_PI / 2.0);

  // Heading due north: along-track uncertainty now lies on map Y.
  EXPECT_NEAR(map(0, 0), body(1, 1), 1e-12);
  EXPECT_NEAR(map(1, 1), body(0, 0), 1e-12);
  EXPECT_NEAR(map(0, 1), 0.0, 1e-12);
}

// The regression this whole change exists for. Live on 2026-08-24 the robot
// held ~132° for 22 s and the published matrix carried var_xy == 0.0000 with
// var_xx ~100x var_yy — the signature of an unrotated body-frame matrix. A
// correctly rotated one MUST show a substantial cross term at that heading.
TEST(CovarianceFrame, ObliqueHeadingProducesCrossTerm)
{
  const Eigen::Matrix3d body = NonHolonomicBodyCov();
  const double yaw = 132.0 * M_PI / 180.0;
  const Eigen::Matrix3d map = BodyToMapCovariance(body, yaw);

  // cos(132°)sin(132°) < 0, and the along-track term dominates, so the cross
  // term is clearly negative — not the 0.0000 we observed on the robot.
  EXPECT_LT(map(0, 1), -1e-4);
  EXPECT_NEAR(map(0, 1), map(1, 0), 1e-12);
}

TEST(CovarianceFrame, RotationPreservesTraceAndYaw)
{
  const Eigen::Matrix3d body = NonHolonomicBodyCov();

  for (double deg = 0.0; deg < 360.0; deg += 17.0)
  {
    const Eigen::Matrix3d map = BodyToMapCovariance(body, deg * M_PI / 180.0);

    // Similarity transform: xy trace and the heading block are untouched.
    EXPECT_NEAR(map(0, 0) + map(1, 1), body(0, 0) + body(1, 1), 1e-12);
    EXPECT_NEAR(map(2, 2), body(2, 2), 1e-12);
    // Still symmetric.
    EXPECT_NEAR(map(0, 1), map(1, 0), 1e-12);
  }
}

TEST(CovarianceFrame, PositionHeadingCrossTermsRotate)
{
  Eigen::Matrix3d body = NonHolonomicBodyCov();
  body(0, 2) = 0.002;  // along-track / yaw coupling
  body(2, 0) = 0.002;

  const Eigen::Matrix3d map = BodyToMapCovariance(body, M_PI / 2.0);

  // A quarter turn moves the along-track/yaw coupling onto map Y.
  EXPECT_NEAR(map(0, 2), 0.0, 1e-12);
  EXPECT_NEAR(map(1, 2), 0.002, 1e-12);
  EXPECT_NEAR(map(2, 1), map(1, 2), 1e-12);
}

// MaxPositionSigma is the quantity a trust gate wants: it must read the same
// no matter which way the robot is pointing. sqrt(max(var_xx, var_yy)) does
// NOT have that property once the matrix is correctly rotated.
TEST(CovarianceFrame, MaxPositionSigmaIsFrameInvariant)
{
  const Eigen::Matrix3d body = NonHolonomicBodyCov();
  const double expected = 0.05;  // the along-track sigma is the major axis

  for (double deg = 0.0; deg < 360.0; deg += 13.0)
  {
    const Eigen::Matrix3d map = BodyToMapCovariance(body, deg * M_PI / 180.0);
    EXPECT_NEAR(MaxPositionSigma(map), expected, 1e-9) << "heading " << deg << " deg";
  }
}

TEST(CovarianceFrame, MaxPositionSigmaHandlesIsotropicAndDegenerate)
{
  Eigen::Matrix3d iso = Eigen::Matrix3d::Zero();
  iso(0, 0) = 0.04;
  iso(1, 1) = 0.04;
  EXPECT_NEAR(MaxPositionSigma(iso), 0.2, 1e-12);

  // All-zero block must yield 0, not NaN.
  EXPECT_NEAR(MaxPositionSigma(Eigen::Matrix3d::Zero()), 0.0, 1e-12);
}
