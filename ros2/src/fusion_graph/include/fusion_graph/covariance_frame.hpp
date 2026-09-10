// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Body → map frame conversion for the Pose2 marginal covariance. Pure, so it
// is unit-testable without ROS/GTSAM (see test_covariance_frame.cpp).
//
// ── Why this exists ─────────────────────────────────────────────────────────
// GTSAM's `marginalCovariance(PoseKey)` returns the marginal in the pose's
// LOCAL tangent coordinates — i.e. the robot's body frame, where x is
// along-track and y is cross-track. `nav_msgs/Odometry.pose.covariance` is
// defined in the message's `header.frame_id`, which for /odometry/filtered_map
// is `map`. Copying one into the other publishes a body-frame matrix labelled
// map-frame.
//
// The wheel between-factor is deliberately non-holonomic (σ_x >> σ_y, 0.05 vs
// 0.005 by default), so the body-frame marginal is strongly anisotropic and
// the mislabelling is not a rounding detail: measured live on 2026-08-24 with
// the robot holding a ~132° heading for 22 s, the published matrix showed
// var_xx ≈ 100 × var_yy with var_xy pinned at 0.0000. A genuine map-frame
// ellipse at that heading would carry a large negative var_xy; the absence of
// any cross term regardless of heading is the signature of the raw body-frame
// matrix going out unrotated.
//
// ── What this does NOT fix ──────────────────────────────────────────────────
// Rotation is a similarity transform, so it does not change the eigenvalues:
// the ~0.09 m along-track sigma is still there afterwards, just distributed
// across map x and y according to heading. Any consumer wanting a single
// "how well do we know where we are" number needs the MAJOR AXIS
// (`MaxPositionSigma` below), which is frame-invariant — reading
// `sqrt(max(var_xx, var_yy))` off the rotated matrix yields a
// heading-dependent value that swings between the minor and major axis as the
// robot turns.

#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace fusion_graph
{

/// Rotate a Pose2 tangent-space covariance from the pose's body frame into the
/// map frame. Tangent ordering is GTSAM's: (x, y, theta).
///
/// The Jacobian is block-diagonal — R(yaw) on the translation block, identity
/// on heading — so heading variance is preserved exactly and the
/// position/heading cross terms rotate with the position block.
///
/// @param cov_body  3x3 marginal in body coordinates
/// @param yaw       map-frame heading of the pose the marginal belongs to [rad]
inline Eigen::Matrix3d BodyToMapCovariance(const Eigen::Matrix3d& cov_body, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  Eigen::Matrix3d j = Eigen::Matrix3d::Identity();
  j(0, 0) = c;
  j(0, 1) = -s;
  j(1, 0) = s;
  j(1, 1) = c;

  return j * cov_body * j.transpose();
}

/// Largest 1-sigma position uncertainty [m] — the major axis of the xy
/// covariance ellipse. Frame-invariant, so it reads the same before and after
/// BodyToMapCovariance, which is exactly what makes it the right quantity for
/// a trust gate.
///
/// Closed-form largest eigenvalue of the symmetric 2x2 block; the radicand is
/// clamped at zero so floating-point noise on a near-degenerate block cannot
/// produce NaN.
inline double MaxPositionSigma(const Eigen::Matrix3d& cov)
{
  const double vxx = cov(0, 0);
  const double vyy = cov(1, 1);
  const double vxy = 0.5 * (cov(0, 1) + cov(1, 0));

  const double mean = 0.5 * (vxx + vyy);
  const double diff = 0.5 * (vxx - vyy);
  const double radicand = std::max(0.0, diff * diff + vxy * vxy);
  const double lambda_max = mean + std::sqrt(radicand);

  return std::sqrt(std::max(0.0, lambda_max));
}

}  // namespace fusion_graph
