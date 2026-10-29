// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "fusion_graph/motion_source_history.hpp"
#include <Eigen/Eigenvalues>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace fusion_graph
{

// FIRST-ORDER conditional extension of an existing process edge, never its replacement.
// The callback must own an immutable snapshot, including the edge-wide mean/gate policy.
// Controls are conditioned inputs. Default uncertainty is independent per-edge process
// diffusion, uniformly allocated in SOURCE time in a frozen terminal tangent chart.
// This is neither raw-wheel measurement covariance nor exact sensor preintegration.
struct FrozenProcessEdge
{
  using TrajectoryQuery = std::function<MotionTrajectory(int64_t)>;
  gtsam::Key left_key = 0;
  gtsam::Key right_key = 0;
  std::optional<gtsam::Key> bias_key;
  uint64_t source_epoch = 0;
  int64_t start_ns = 0;
  int64_t end_ns = 0;
  gtsam::Matrix3 endpoint_covariance = gtsam::Matrix3::Zero();
  gtsam::Values reference_values;
  std::shared_ptr<const TrajectoryQuery> trajectory_query;
  // Empty selects default diffusion alone. Otherwise every queried gyro ID must occur here.
  // Bias remains a graph variable; it is never included in this noise covariance.
  std::map<MotionSourceId, double> gyro_rate_variances;

  static std::shared_ptr<const TrajectoryQuery> HistoryQuery(
      std::shared_ptr<const MotionSourceHistory> history, uint64_t epoch, int64_t start_ns)
  {
    if (!history)
      throw std::invalid_argument("null motion snapshot");
    return std::make_shared<const TrajectoryQuery>(
        [history = std::move(history), epoch, start_ns](int64_t end_ns)
        {
          return history->Query(epoch, start_ns, end_ns);
        });
  }
};

class ConditionalProcessBridge
{
public:
  struct Insertion
  {
    gtsam::Key key;
    // Aliases carry neither a new value nor a process factor. Observation ID dedup is separate.
    std::optional<gtsam::Pose2> initial_value;
    gtsam::LinearContainerFactor::shared_ptr factor;
  };

  explicit ConditionalProcessBridge(FrozenProcessEdge edge) : edge_(std::move(edge))
  {
    if (edge_.end_ns <= edge_.start_ns || edge_.left_key == edge_.right_key ||
        !edge_.trajectory_query ||
        (edge_.bias_key &&
         (*edge_.bias_key == edge_.left_key || *edge_.bias_key == edge_.right_key)))
      throw std::invalid_argument("invalid frozen process edge");
    (void)edge_.reference_values.at<gtsam::Pose2>(edge_.left_key);
    (void)edge_.reference_values.at<gtsam::Pose2>(edge_.right_key);
    if (edge_.bias_key)
      if (!std::isfinite(edge_.reference_values.at<double>(*edge_.bias_key)))
        throw std::invalid_argument("invalid bias reference");
    Spectrum(edge_.endpoint_covariance);
    for (const auto& [id, variance] : edge_.gyro_rate_variances)
      if (id.kind != MotionSourceKind::Gyro || !std::isfinite(variance) || variance < 0.0)
        throw std::invalid_argument("invalid gyro source variance");
    endpoint_ = Query(edge_.end_ns);
    endpoint_affine_ = Coordinates(endpoint_, edge_.right_key, edge_.reference_values);
    references_ = edge_.reference_values;
    endpoint_kernel_ = Kernel(edge_.end_ns, endpoint_, edge_.end_ns, endpoint_);
  }

  // Caller serializes insertion. References, transports and covariance NEVER follow optimized
  // poses. current_values only chooses a useful initial Y from the frozen affine conditional.
  // Rebuilding a reference requires rebuilding the complete edge, including all its acquisitions.
  Insertion Insert(int64_t source_ns,
                   gtsam::Key new_key,
                   const gtsam::Values* current_values = nullptr)
  {
    if (source_ns < edge_.start_ns || source_ns > edge_.end_ns)
      throw std::out_of_range("acquisition outside frozen process edge");
    if (source_ns == edge_.start_ns)
      return {edge_.left_key, std::nullopt, nullptr};
    if (source_ns == edge_.end_ns)
      return {edge_.right_key, std::nullopt, nullptr};
    for (const auto& acquisition : acquisitions_)
      if (acquisition.time == source_ns)
        return {acquisition.key, std::nullopt, nullptr};
    if (references_.exists(new_key))
      throw std::invalid_argument("acquisition key already in frozen references");

    const auto trajectory = Query(source_ns);
    gtsam::Values refs = references_;
    const auto canonical = refs.at<gtsam::Pose2>(edge_.left_key).compose(trajectory.mean);
    refs.insert(new_key, canonical);
    const Affine target = Coordinates(trajectory, new_key, refs);
    const size_t count = acquisitions_.size() + 1;
    gtsam::Matrix conditioning = gtsam::Matrix::Zero(3 * count, 3 * count);
    gtsam::Matrix cross(3, 3 * count);
    conditioning.topLeftCorner<3, 3>() = endpoint_kernel_;
    cross.leftCols<3>() = Kernel(source_ns, trajectory, edge_.end_ns, endpoint_);
    std::vector<const Affine*> affines{&endpoint_affine_};
    for (size_t i = 0; i < acquisitions_.size(); ++i)
    {
      const auto& a = acquisitions_[i];
      affines.push_back(&a.affine);
      cross.middleCols<3>(3 * (i + 1)) = Kernel(source_ns, trajectory, a.time, a.trajectory);
      conditioning.block<3, 3>(0, 3 * (i + 1)) =
          Kernel(edge_.end_ns, endpoint_, a.time, a.trajectory);
      conditioning.block<3, 3>(3 * (i + 1), 0) =
          conditioning.block<3, 3>(0, 3 * (i + 1)).transpose();
      for (size_t j = 0; j <= i; ++j)
      {
        const auto& b = acquisitions_[j];
        conditioning.block<3, 3>(3 * (i + 1), 3 * (j + 1)) =
            Kernel(a.time, a.trajectory, b.time, b.trajectory);
        conditioning.block<3, 3>(3 * (j + 1), 3 * (i + 1)) =
            conditioning.block<3, 3>(3 * (i + 1), 3 * (j + 1)).transpose().eval();
      }
    }
    const auto spectrum = Spectrum(conditioning);
    const gtsam::Matrix gain = SolveRight(cross, spectrum);
    const gtsam::Matrix3 unconditional = Kernel(source_ns, trajectory, source_ns, trajectory);
    gtsam::Matrix3 covariance = unconditional - gain * cross.transpose();
    covariance = (covariance + covariance.transpose()).eval() * 0.5;
    // Cancellation of a pure shared sample produces exact constraints, not tiny fake noise.
    // Scale each coordinate by its own unconditional variance; highly anisotropic D is safe.
    const auto conditional_spectrum = Spectrum(covariance, unconditional.diagonal());

    Affine residual = target;
    for (size_t i = 0; i < affines.size(); ++i)
    {
      const gtsam::Matrix3 block = gain.middleCols<3>(3 * i);
      residual.constant -= block * affines[i]->constant;
      for (const auto& [key, jacobian] : affines[i]->jacobians)
      {
        auto [entry, inserted] =
            residual.jacobians.try_emplace(key, gtsam::Matrix::Zero(3, jacobian.cols()));
        entry->second -= block * jacobian;
      }
    }
    const gtsam::Matrix rotation =
        conditional_spectrum.vectors.transpose() * conditional_spectrum.inverse_scale.asDiagonal();
    gtsam::Vector sigmas = conditional_spectrum.values.cwiseSqrt();
    std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
    gtsam::Values factor_refs;
    for (const auto& [key, jacobian] : residual.jacobians)
    {
      terms.emplace_back(key, rotation * jacobian);
      factor_refs.insert(key, refs.at(key));
    }
    const gtsam::SharedDiagonal noise = (sigmas.array() == 0.0).any()
                                            ? gtsam::noiseModel::Constrained::MixedSigmas(sigmas)
                                            : gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    const gtsam::JacobianFactor gaussian(terms, -rotation * residual.constant, noise);
    auto factor = std::make_shared<gtsam::LinearContainerFactor>(gaussian, factor_refs);
    gtsam::Vector3 offset = residual.constant;
    if (current_values)
      for (const auto& [key, jacobian] : residual.jacobians)
        if (key != new_key)
          offset += jacobian * refs.at(key).localCoordinates_(current_values->at(key));
    const gtsam::Vector3 delta = residual.jacobians.at(new_key).fullPivLu().solve(-offset);
    if (!delta.allFinite())
      throw std::invalid_argument("invalid conditional initialization");
    const auto initial = canonical.retract(delta);
    acquisitions_.push_back({source_ns, new_key, trajectory, target});
    references_.insert(new_key, canonical);
    return {new_key, initial, factor};
  }

  const FrozenProcessEdge& edge() const
  {
    return edge_;
  }

private:
  struct Affine
  {
    gtsam::Vector3 constant;
    std::map<gtsam::Key, gtsam::Matrix> jacobians;
  };
  struct Acquisition
  {
    int64_t time;
    gtsam::Key key;
    MotionTrajectory trajectory;
    Affine affine;
  };
  struct ScaledSpectrum
  {
    gtsam::Vector values, inverse_scale;
    gtsam::Matrix vectors;
  };

  // Work in a correlation chart before deciding numerical rank. A yaw variance of 1e-20
  // alongside metre-scale translation variance must not become a constraint just for its units.
  static ScaledSpectrum Spectrum(const gtsam::Matrix& matrix,
                                 const gtsam::Vector& reference_diagonal = gtsam::Vector())
  {
    if (!matrix.allFinite() || matrix.rows() != matrix.cols())
      throw std::invalid_argument("invalid process covariance");
    const double tolerance = 128.0 * std::numeric_limits<double>::epsilon() * matrix.rows();
    gtsam::Vector scale(matrix.rows());
    for (Eigen::Index i = 0; i < matrix.rows(); ++i)
    {
      const double reference =
          reference_diagonal.size() ? reference_diagonal(i) : std::abs(matrix(i, i));
      if (!std::isfinite(reference) || reference < 0.0 || matrix(i, i) < -tolerance * reference)
        throw std::invalid_argument("negative process variance");
      if (reference == 0.0 && (matrix.row(i).array() != 0.0).any())
        throw std::invalid_argument("nonzero covariance outside zero-variance support");
      scale(i) = reference > 0.0 ? 1.0 / std::sqrt(reference) : 1.0;
    }
    const gtsam::Matrix normalized = scale.asDiagonal() * matrix * scale.asDiagonal();
    if (!normalized.allFinite() ||
        (normalized - normalized.transpose()).norm() > tolerance * std::max(1.0, normalized.norm()))
      throw std::invalid_argument("asymmetric process covariance");
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix> solver(normalized);
    if (solver.info() != Eigen::Success)
      throw std::invalid_argument("process covariance eigensolve failed");
    auto values = solver.eigenvalues().eval();
    const double threshold = tolerance * std::max(1.0, values.cwiseAbs().maxCoeff());
    if (values.minCoeff() < -threshold)
      throw std::invalid_argument("non PSD process covariance");
    for (Eigen::Index i = 0; i < values.size(); ++i)
      if (values(i) <= threshold)
        values(i) = 0.0;
    return {values, scale, solver.eigenvectors()};
  }

  static gtsam::Matrix SolveRight(const gtsam::Matrix& cross, const ScaledSpectrum& spectrum)
  {
    const gtsam::Matrix transformed =
        cross * spectrum.inverse_scale.asDiagonal() * spectrum.vectors;
    gtsam::Matrix solved = transformed;
    const double tolerance = 512.0 * std::numeric_limits<double>::epsilon() * cross.cols();
    for (Eigen::Index i = 0; i < spectrum.values.size(); ++i)
      if (spectrum.values(i) > 0.0)
        solved.col(i) /= spectrum.values(i);
      else
      {
        if (transformed.col(i).norm() > tolerance * transformed.norm())
          throw std::invalid_argument("incompatible singular process covariance");
        solved.col(i).setZero();
      }
    return solved * spectrum.vectors.transpose() * spectrum.inverse_scale.asDiagonal();
  }

  MotionTrajectory Query(int64_t time) const
  {
    auto result = (*edge_.trajectory_query)(time);
    if (result.status != MotionHistoryStatus::Ok || result.epoch != edge_.source_epoch ||
        !result.mean.matrix().allFinite() || !result.bias_jacobian.allFinite())
      throw std::invalid_argument("uncovered or invalid frozen trajectory");
    if (!edge_.gyro_rate_variances.empty())
      for (const auto& [id, jacobian] : result.gyro_jacobians)
        if (!jacobian.allFinite() || !edge_.gyro_rate_variances.count(id))
          throw std::invalid_argument("missing gyro source variance");
    return result;
  }

  gtsam::Matrix3 Kernel(int64_t a,
                        const MotionTrajectory& ta,
                        int64_t b,
                        const MotionTrajectory& tb) const
  {
    const double progress =
        static_cast<double>(static_cast<long double>(std::min(a, b)) - edge_.start_ns) /
        static_cast<double>(static_cast<long double>(edge_.end_ns) - edge_.start_ns);
    const gtsam::Matrix3 pa = ta.mean.between(endpoint_.mean).AdjointMap();
    const gtsam::Matrix3 pb = tb.mean.between(endpoint_.mean).AdjointMap();
    gtsam::Matrix3 result = pa * (progress * edge_.endpoint_covariance) * pb.transpose();
    for (const auto& [id, variance] : edge_.gyro_rate_variances)
    {
      const auto ja = ta.gyro_jacobians.find(id), jb = tb.gyro_jacobians.find(id);
      if (ja != ta.gyro_jacobians.end() && jb != tb.gyro_jacobians.end())
        result += ja->second * variance * jb->second.transpose();
    }
    return result;
  }

  Affine Coordinates(const MotionTrajectory& trajectory,
                     gtsam::Key target,
                     const gtsam::Values& refs) const
  {
    gtsam::Matrix3 hleft, htarget, hmean, hrelative, hlog;
    const auto relative = refs.at<gtsam::Pose2>(edge_.left_key)
                              .between(refs.at<gtsam::Pose2>(target), hleft, htarget);
    const auto discrepancy = trajectory.mean.between(relative, hmean, hrelative);
    Affine result;
    result.constant = gtsam::Pose2::Logmap(discrepancy, hlog);
    result.jacobians.emplace(edge_.left_key, hlog * hrelative * hleft);
    result.jacobians.emplace(target, hlog * hrelative * htarget);
    if (edge_.bias_key)
      result.jacobians.emplace(*edge_.bias_key, hlog * hmean * trajectory.bias_jacobian);
    return result;
  }

  const FrozenProcessEdge edge_;
  MotionTrajectory endpoint_;
  Affine endpoint_affine_;
  gtsam::Matrix3 endpoint_kernel_;
  gtsam::Values references_;
  std::vector<Acquisition> acquisitions_;
};

}  // namespace fusion_graph
