// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// ScanMatcher — 2D point-to-point ICP for LaserScan-to-LaserScan
// alignment. Used by the factor graph to add a relative-motion factor
// between consecutive nodes from LiDAR (in addition to the wheel
// between-factor).
//
// Design choices:
//   - Point-to-point (not point-to-line). Marginally noisier on flat
//     walls but trivially robust and fast for a flat-grass mower whose
//     environment is mostly trees + dock + chassis.
//   - Brute-force NN with subsampled source. With src ≤ 60 pts and
//     tgt ≤ 360 pts that's 22k ops/iter — under 5 ms on RK3588.
//   - 2D rigid alignment via Kabsch / SVD on the centered point sets.
//   - max_iterations + delta-pose convergence threshold; outlier
//     rejection by per-correspondence distance gate.

#pragma once

#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>

namespace fusion_graph
{

struct ScanMatcherParams
{
  int max_iterations = 15;
  double convergence_eps = 1e-3;  // rad / m, sum
  double max_correspondence_dist = 0.5;  // m
  int min_inliers = 30;
  size_t source_subsample = 60;  // cap on #source points used per iter
  // Per-axis sigma scaling — final factor sigma is base + scale * rmse.
  double sigma_xy_base = 0.02;  // m floor
  double sigma_xy_scale = 1.0;
  double sigma_theta_base = 0.005;  // rad floor
  double sigma_theta_scale = 0.5;
};

struct ScanMatcherResult
{
  gtsam::Pose2 delta;  // T such that target = T * source
  bool ok = false;
  int iterations = 0;
  int inliers = 0;
  double rmse = 0.0;  // m, RMS over inliers
  double sigma_xy = 0.5;  // suggested noise model sigma
  double sigma_theta = 0.5;
};

class ScanMatcher
{
public:
  explicit ScanMatcher(const ScanMatcherParams& params = {});

  // Align source onto target. init_guess is the prior relative motion
  // (e.g. from the wheel between-factor); ICP refines it.
  //
  // Both scans are in the same body frame (no map/odom transforms
  // applied — the caller is expected to have pulled raw 2D points in
  // base_footprint with the lidar_link extrinsic).
  ScanMatcherResult Match(const std::vector<Eigen::Vector2d>& source,
                          const std::vector<Eigen::Vector2d>& target,
                          const gtsam::Pose2& init_guess) const;

private:
  ScanMatcherParams p_;

  // Single-iteration helpers.
  static Eigen::Vector2d Transform(const gtsam::Pose2& T, const Eigen::Vector2d& p);

  // SVD-based 2D rigid alignment: returns Pose2 such that
  //   sum_i ||(R*src[i] + t) - tgt[i]||^2 is minimized.
  // Inputs must be aligned (src[i] <-> tgt[i]) and non-empty.
  static gtsam::Pose2 RigidAlign2D(const std::vector<Eigen::Vector2d>& src_corr,
                                   const std::vector<Eigen::Vector2d>& tgt_corr);
};

}  // namespace fusion_graph
