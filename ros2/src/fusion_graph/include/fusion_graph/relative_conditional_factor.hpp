// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <optional>
#include <stdexcept>
#include <vector>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

namespace fusion_graph
{

/// A source query outside this local model's chart is unsupported, not a corrupt solver.
class UnsupportedRelativeConditional : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};

// A Gaussian least-squares potential in shared relative-error coordinates, with frozen
// source-derived coefficients and noise/rank. It agrees to first order with the frozen
// affine bridge at canonical references. Away from those references it deliberately
// reevaluates relative geometry; it is not the former affine likelihood, nor a globally
// normalized density under product Pose2 Haar measure. Schur/no-added-endpoint-information
// statements apply locally in these error coordinates, on compatible constrained support.
// Bias uses the declared first-order source mean M_ref Exp(J_bias * (b - b_ref)); it does
// not reintegrate source history. Kernel, gains and source bias reference never move.
class RelativeConditionalFactor : public gtsam::NoiseModelFactor
{
public:
  struct Term
  {
    gtsam::Key key;
    gtsam::Pose2 mean;
    gtsam::Vector3 bias_jacobian;
    gtsam::Matrix3 coefficient;
  };

  RelativeConditionalFactor(gtsam::Key left_key,
                            std::optional<gtsam::Key> bias_key,
                            double bias_reference,
                            std::vector<Term> terms,
                            const gtsam::SharedNoiseModel& noise);

  static gtsam::Pose2 Mean(const Term& term, double bias_delta);

  static gtsam::Vector3 Coordinate(const Term& term,
                                   const gtsam::Pose2& left,
                                   const gtsam::Pose2& target,
                                   double bias_delta);

  gtsam::Vector unwhitenedError(const gtsam::Values& values,
                                gtsam::OptionalMatrixVecType H = nullptr) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

  bool equals(const gtsam::NonlinearFactor& other, double tolerance = 1e-9) const override;

private:
  static gtsam::KeyVector Keys(gtsam::Key left,
                               std::optional<gtsam::Key> bias,
                               const std::vector<Term>& terms);

  gtsam::Key left_key_;
  std::optional<gtsam::Key> bias_key_;
  double bias_reference_;
  std::vector<Term> terms_;
};

}  // namespace fusion_graph
