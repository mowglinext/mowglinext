// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace fusion_graph
{

ScanMatcher::ScanMatcher(const ScanMatcherParams& params) : p_(params)
{
}

Eigen::Vector2d ScanMatcher::Transform(const gtsam::Pose2& T, const Eigen::Vector2d& p)
{
  const double c = std::cos(T.theta());
  const double s = std::sin(T.theta());
  return Eigen::Vector2d(T.x() + c * p.x() - s * p.y(), T.y() + s * p.x() + c * p.y());
}

gtsam::Pose2 ScanMatcher::RigidAlign2D(const std::vector<Eigen::Vector2d>& src_corr,
                                       const std::vector<Eigen::Vector2d>& tgt_corr)
{
  // Kabsch in 2D: centroids -> SVD on covariance.
  const size_t n = src_corr.size();
  Eigen::Vector2d cs = Eigen::Vector2d::Zero();
  Eigen::Vector2d ct = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < n; ++i)
  {
    cs += src_corr[i];
    ct += tgt_corr[i];
  }
  cs /= static_cast<double>(n);
  ct /= static_cast<double>(n);

  Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < n; ++i)
  {
    const Eigen::Vector2d s = src_corr[i] - cs;
    const Eigen::Vector2d t = tgt_corr[i] - ct;
    H += s * t.transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();

  // Reflection guard: ensure det(R) = +1 for a proper rotation.
  Eigen::Matrix2d D = Eigen::Matrix2d::Identity();
  if ((V * U.transpose()).determinant() < 0.0)
    D(1, 1) = -1.0;

  Eigen::Matrix2d R = V * D * U.transpose();
  Eigen::Vector2d t = ct - R * cs;
  const double theta = std::atan2(R(1, 0), R(0, 0));
  return gtsam::Pose2(t.x(), t.y(), theta);
}

ScanMatcherResult ScanMatcher::Match(const std::vector<Eigen::Vector2d>& source,
                                     const std::vector<Eigen::Vector2d>& target,
                                     const gtsam::Pose2& init_guess,
                                     int min_inliers_override) const
{
  ScanMatcherResult res;
  if (source.empty() || target.empty())
    return res;

  // Per-call inlier threshold: keyframe matching passes a looser value than
  // the shared scan-to-scan default (see ScanMatcherParams::min_inliers).
  const int min_inliers = (min_inliers_override >= 0) ? min_inliers_override : p_.min_inliers;

  // Subsample source for speed. Stride-pick keeps angular coverage.
  std::vector<Eigen::Vector2d> src;
  if (source.size() <= p_.source_subsample)
  {
    src = source;
  }
  else
  {
    src.reserve(p_.source_subsample);
    const double step = static_cast<double>(source.size()) / p_.source_subsample;
    for (size_t i = 0; i < p_.source_subsample; ++i)
    {
      const size_t idx =
          std::min<size_t>(source.size() - 1, static_cast<size_t>(static_cast<double>(i) * step));
      src.push_back(source[idx]);
    }
  }

  gtsam::Pose2 T = init_guess;
  std::vector<Eigen::Vector2d> src_corr, tgt_corr;
  src_corr.reserve(src.size());
  tgt_corr.reserve(src.size());

  const double max_d2 = p_.max_correspondence_dist * p_.max_correspondence_dist;

  double last_rmse = std::numeric_limits<double>::infinity();

  for (int iter = 0; iter < p_.max_iterations; ++iter)
  {
    src_corr.clear();
    tgt_corr.clear();
    double sse = 0.0;

    // Brute-force NN: for each transformed source point, find the
    // closest target point, gate by max_correspondence_dist.
    for (const auto& p : src)
    {
      const Eigen::Vector2d ps = Transform(T, p);
      double best = max_d2;
      int best_idx = -1;
      for (size_t j = 0; j < target.size(); ++j)
      {
        const double d2 = (target[j] - ps).squaredNorm();
        if (d2 < best)
        {
          best = d2;
          best_idx = static_cast<int>(j);
        }
      }
      if (best_idx >= 0)
      {
        src_corr.push_back(p);  // pre-transform — RigidAlign re-applies
        tgt_corr.push_back(target[best_idx]);
        sse += best;
      }
    }

    if (static_cast<int>(src_corr.size()) < min_inliers)
    {
      // Too few correspondences. Bail out with current T but flag fail.
      res.iterations = iter;
      res.inliers = static_cast<int>(src_corr.size());
      return res;
    }

    const gtsam::Pose2 T_new = RigidAlign2D(src_corr, tgt_corr);

    // Convergence check — composed delta from previous T.
    const gtsam::Pose2 dT = T.between(T_new);
    const double change = std::abs(dT.x()) + std::abs(dT.y()) + std::abs(dT.theta());
    T = T_new;
    last_rmse = std::sqrt(sse / src_corr.size());

    if (change < p_.convergence_eps)
    {
      res.iterations = iter + 1;
      break;
    }
    res.iterations = iter + 1;
  }

  res.delta = T;
  res.inliers = static_cast<int>(src_corr.size());
  res.rmse = last_rmse;
  res.ok = res.inliers >= min_inliers;

  // Noise model: sigma scales with rmse. Tight floors so we don't
  // outdrive the wheel between-factor when ICP looks great.
  res.sigma_xy = p_.sigma_xy_base + p_.sigma_xy_scale * res.rmse;
  res.sigma_theta = p_.sigma_theta_base + p_.sigma_theta_scale * res.rmse;

  return res;
}

}  // namespace fusion_graph
