// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <optional>

#include <Eigen/Eigenvalues>
namespace fusion_graph
{
// Preserve the weak direction; floor every principal variance, not just X/Y.
// An invalid estimate is not evidence and must not turn into a tight prior.
inline std::optional<Eigen::Matrix2d> FloorLidarCovariance(const Eigen::Matrix2d& covariance,
                                                           double sigma_floor)
{
  if (!covariance.allFinite() || !std::isfinite(sigma_floor) || sigma_floor <= 0.0)
    return std::nullopt;
  const Eigen::Matrix2d sym = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(sym);
  if (es.info() != Eigen::Success || es.eigenvalues().minCoeff() < -1e-10)
    return std::nullopt;
  return (es.eigenvectors() * es.eigenvalues().cwiseMax(sigma_floor * sigma_floor).asDiagonal() *
          es.eigenvectors().transpose())
      .eval();
}
}  // namespace fusion_graph
