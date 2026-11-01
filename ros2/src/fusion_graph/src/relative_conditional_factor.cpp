// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "fusion_graph/relative_conditional_factor.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <utility>

namespace fusion_graph
{

RelativeConditionalFactor::RelativeConditionalFactor(gtsam::Key left_key,
                                                     std::optional<gtsam::Key> bias_key,
                                                     double bias_reference,
                                                     std::vector<Term> terms,
                                                     const gtsam::SharedNoiseModel& noise)
    : gtsam::NoiseModelFactor(noise, Keys(left_key, bias_key, terms)),
      left_key_(left_key),
      bias_key_(bias_key),
      bias_reference_(bias_reference),
      terms_(std::move(terms))
{
  if (!noise || noise->dim() != 3 || !std::isfinite(bias_reference_))
    throw std::invalid_argument("invalid relative conditional noise or bias reference");
  for (const auto& term : terms_)
    if (!term.mean.matrix().allFinite() || !term.bias_jacobian.allFinite() ||
        !term.coefficient.allFinite())
      throw std::invalid_argument("invalid relative conditional term");
}

gtsam::Pose2 RelativeConditionalFactor::Mean(const Term& term, double bias_delta)
{
  if (!std::isfinite(bias_delta))
    throw std::invalid_argument("invalid relative conditional bias");
  return term.mean.compose(gtsam::Pose2::Expmap(term.bias_jacobian * bias_delta));
}

gtsam::Vector3 RelativeConditionalFactor::Coordinate(const Term& term,
                                                     const gtsam::Pose2& left,
                                                     const gtsam::Pose2& target,
                                                     double bias_delta)
{
  if (!left.matrix().allFinite() || !target.matrix().allFinite())
    throw std::invalid_argument("invalid relative conditional pose");
  return gtsam::Pose2::Logmap(Mean(term, bias_delta).between(left.between(target)));
}

gtsam::Vector RelativeConditionalFactor::unwhitenedError(const gtsam::Values& values,
                                                         gtsam::OptionalMatrixVecType H) const
{
  const auto& left = values.at<gtsam::Pose2>(left_key_);
  const double delta = bias_key_ ? values.at<double>(*bias_key_) - bias_reference_ : 0.0;
  if (!left.matrix().allFinite() || !std::isfinite(delta))
    throw std::invalid_argument("invalid relative conditional state");
  std::map<gtsam::Key, gtsam::Matrix> jacobians;
  if (H)
    for (const auto key : keys())
      jacobians.emplace(key, gtsam::Matrix::Zero(3, bias_key_ && key == *bias_key_ ? 1 : 3));
  gtsam::Vector3 residual = gtsam::Vector3::Zero();
  for (const auto& term : terms_)
  {
    const auto& target = values.at<gtsam::Pose2>(term.key);
    if (!target.matrix().allFinite())
      throw std::invalid_argument("invalid relative conditional target");
    gtsam::Matrix3 hexp, hcompose, hleft, htarget, hmean, hrelative, hlog;
    const auto perturbation = gtsam::Pose2::Expmap(term.bias_jacobian * delta, hexp);
    const auto mean = term.mean.compose(perturbation, {}, hcompose);
    const auto relative = left.between(target, hleft, htarget);
    const auto discrepancy = mean.between(relative, hmean, hrelative);
    residual += term.coefficient * gtsam::Pose2::Logmap(discrepancy, hlog);
    if (H)
    {
      jacobians.at(left_key_) += term.coefficient * hlog * hrelative * hleft;
      jacobians.at(term.key) += term.coefficient * hlog * hrelative * htarget;
      if (bias_key_)
        jacobians.at(*bias_key_) +=
            term.coefficient * hlog * hmean * hcompose * hexp * term.bias_jacobian;
    }
  }
  if (H)
  {
    H->resize(keys().size());
    for (size_t i = 0; i < keys().size(); ++i)
      (*H)[i] = jacobians.at(keys()[i]);
  }
  return residual;
}

gtsam::NonlinearFactor::shared_ptr RelativeConditionalFactor::clone() const
{
  return std::make_shared<RelativeConditionalFactor>(*this);
}

bool RelativeConditionalFactor::equals(const gtsam::NonlinearFactor& other, double tolerance) const
{
  const auto* factor = dynamic_cast<const RelativeConditionalFactor*>(&other);
  if (!factor || !gtsam::NoiseModelFactor::equals(other, tolerance) ||
      left_key_ != factor->left_key_ || bias_key_ != factor->bias_key_ ||
      std::abs(bias_reference_ - factor->bias_reference_) > tolerance ||
      terms_.size() != factor->terms_.size())
    return false;
  for (size_t i = 0; i < terms_.size(); ++i)
  {
    const auto& a = terms_[i];
    const auto& b = factor->terms_[i];
    if (a.key != b.key || !a.mean.equals(b.mean, tolerance) ||
        (a.bias_jacobian - b.bias_jacobian).cwiseAbs().maxCoeff() > tolerance ||
        (a.coefficient - b.coefficient).cwiseAbs().maxCoeff() > tolerance)
      return false;
  }
  return true;
}

gtsam::KeyVector RelativeConditionalFactor::Keys(gtsam::Key left,
                                                 std::optional<gtsam::Key> bias,
                                                 const std::vector<Term>& terms)
{
  if (terms.empty() || (bias && left == *bias))
    throw std::invalid_argument("invalid relative conditional keys");
  gtsam::KeyVector result{left};
  if (bias)
    result.push_back(*bias);
  for (const auto& term : terms)
    result.push_back(term.key);
  std::sort(result.begin(), result.end());
  if (std::adjacent_find(result.begin(), result.end()) != result.end())
    throw std::invalid_argument("duplicate relative conditional key");
  return result;
}

}  // namespace fusion_graph
