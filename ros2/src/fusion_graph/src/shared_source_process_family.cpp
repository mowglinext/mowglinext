// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "fusion_graph/shared_source_process_family.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <set>

#include "fusion_graph/factors.hpp"
#include <Eigen/Eigenvalues>
#include <gtsam/linear/NoiseModel.h>

namespace fusion_graph
{
namespace
{
using gtsam::Key;
using gtsam::Matrix;
using gtsam::Matrix3;
using gtsam::Pose2;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using Jacobians = std::map<Key, Matrix>;

struct Spectrum
{
  Vector values, scale;
  Matrix vectors;
};

Spectrum Decompose(const Matrix& covariance, const Vector& reference = Vector())
{
  if (!covariance.allFinite() || covariance.rows() == 0 || covariance.rows() != covariance.cols() ||
      (reference.size() && reference.size() != covariance.rows()))
    throw std::invalid_argument("invalid shared-source covariance");
  const double tolerance = 128.0 * std::numeric_limits<double>::epsilon() * covariance.rows();
  Vector scale(covariance.rows());
  for (Eigen::Index i = 0; i < covariance.rows(); ++i)
  {
    const double variance = reference.size() ? reference(i) : std::abs(covariance(i, i));
    if (!std::isfinite(variance) || variance < 0.0 || covariance(i, i) < -tolerance * variance ||
        (variance == 0.0 && (covariance.row(i).array() != 0.0).any()))
      throw std::invalid_argument("invalid shared-source variance support");
    scale(i) = variance > 0.0 ? 1.0 / std::sqrt(variance) : 1.0;
  }
  const Matrix normalized = scale.asDiagonal() * covariance * scale.asDiagonal();
  if (!normalized.allFinite() ||
      (normalized - normalized.transpose()).norm() > tolerance * std::max(1.0, normalized.norm()))
    throw std::invalid_argument("asymmetric shared-source covariance");
  Eigen::SelfAdjointEigenSolver<Matrix> solver(normalized);
  if (solver.info() != Eigen::Success)
    throw std::invalid_argument("shared-source covariance eigensolve failed");
  Vector values = solver.eigenvalues();
  const double threshold = tolerance * std::max(1.0, values.cwiseAbs().maxCoeff());
  if (values.minCoeff() < -threshold)
    throw std::invalid_argument("non-PSD shared-source covariance");
  for (Eigen::Index i = 0; i < values.size(); ++i)
    if (values(i) <= threshold)
      values(i) = 0.0;
  return {values, scale, solver.eigenvectors()};
}

Matrix SolveRight(const Matrix& cross, const Spectrum& spectrum)
{
  const Matrix transformed = cross * spectrum.scale.asDiagonal() * spectrum.vectors;
  Matrix solved = transformed;
  const double tolerance = 512.0 * std::numeric_limits<double>::epsilon() * cross.cols();
  for (Eigen::Index i = 0; i < spectrum.values.size(); ++i)
    if (spectrum.values(i) > 0.0)
      solved.col(i) /= spectrum.values(i);
    else
    {
      if (transformed.col(i).norm() > tolerance * transformed.norm())
        throw std::invalid_argument("incompatible shared-source covariance support");
      solved.col(i).setZero();
    }
  return solved * spectrum.vectors.transpose() * spectrum.scale.asDiagonal();
}

void CheckSupport(const Vector& value, const Spectrum& spectrum)
{
  if (!value.allFinite())
    throw UnsupportedSharedSourceConditional("nonfinite conditioning state");
  const Vector normalized = spectrum.scale.asDiagonal() * value;
  const Vector projected = spectrum.vectors.transpose() * normalized;
  const double tolerance = 512.0 * std::numeric_limits<double>::epsilon() * value.size() *
                           std::max(1.0, normalized.norm());
  for (Eigen::Index i = 0; i < projected.size(); ++i)
    if (spectrum.values(i) == 0.0 && std::abs(projected(i)) > tolerance)
      throw UnsupportedSharedSourceConditional("conditioning state outside exact noise support");
}

gtsam::SharedDiagonal Noise(const Spectrum& spectrum)
{
  const Vector sigmas = spectrum.values.cwiseSqrt();
  return (sigmas.array() == 0.0).any() ? gtsam::noiseModel::Constrained::MixedSigmas(sigmas)
                                       : gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

Matrix Rotation(const Spectrum& spectrum)
{
  return spectrum.vectors.transpose() * spectrum.scale.asDiagonal();
}

bool Near(double a, double b)
{
  return std::isfinite(a) && std::isfinite(b) &&
         std::abs(a - b) <= 128.0 * std::numeric_limits<double>::epsilon() *
                                std::max({1.0, std::abs(a), std::abs(b)});
}

struct Recipe
{
  int64_t source_ns = 0;
  Key key = 0;
  size_t window = 0;
  MotionTrajectory trajectory;
  double progress = 0.0;
  Matrix3 transport, h;
  Matrix f;
};

struct FamilyData
{
  std::vector<SharedSourceProcessWindow> windows;
  std::vector<MotionTrajectory> endpoints;
  std::map<MotionSourceId, size_t> ids;
  std::map<Key, size_t> dimensions;
  Values references;
  Vector q;
  Matrix a, c;
  bool gyro = false;

  MotionTrajectory Query(size_t index, int64_t time) const
  {
    const auto& edge = windows[index].edge;
    auto trajectory = (*edge.trajectory_query)(time);
    if (trajectory.status != MotionHistoryStatus::Ok || trajectory.epoch != edge.source_epoch ||
        !trajectory.mean.matrix().allFinite() || !trajectory.bias_jacobian.allFinite())
      throw std::invalid_argument("uncovered or invalid shared-source trajectory");
    for (const auto& [id, jacobian] : trajectory.gyro_jacobians)
      if (!jacobian.allFinite() ||
          (gyro && (!ids.count(id) || !edge.gyro_rate_variances.count(id))))
        throw std::invalid_argument("unregistered shared-source trajectory sensitivity");
    if (gyro)
    {
      // The on-mode recipe always rotates from the gyro over its full consumed
      // window. A comes from metadata; this checks that F's history describes
      // those same uses, without deriving consumption from producer support.
      const double elapsed =
          static_cast<double>(static_cast<long double>(time) - edge.start_ns) * 1e-9;
      double total = 0.0;
      for (const auto& [id, jacobian] : trajectory.gyro_jacobians)
      {
        const double duration = a(index, ids.at(id)), partial = jacobian.z();
        if ((partial < 0.0 && !Near(partial, 0.0)) ||
            (partial > duration && !Near(partial, duration)) ||
            (time == edge.end_ns && !Near(partial, duration)))
          throw std::invalid_argument("gyro trajectory sensitivity contradicts consumed duration");
        total += partial;
      }
      if ((time == edge.end_ns &&
           trajectory.gyro_jacobians.size() != edge.gyro_rate_variances.size()) ||
          !Near(total, elapsed) || !Near(trajectory.bias_jacobian.z(), -elapsed))
        throw std::invalid_argument("gyro trajectory does not cover its full consumed rotation");
    }
    return trajectory;
  }

  Recipe MakeRecipe(int64_t time, Key key) const
  {
    size_t index = 0;
    while (time > windows[index].edge.end_ns)
      ++index;
    const auto& edge = windows[index].edge;
    Recipe recipe;
    recipe.source_ns = time;
    recipe.key = key;
    recipe.window = index;
    recipe.trajectory = Query(index, time);
    recipe.progress = static_cast<double>(static_cast<long double>(time) - edge.start_ns) /
                      static_cast<double>(static_cast<long double>(edge.end_ns) - edge.start_ns);
    recipe.transport = recipe.trajectory.mean.between(endpoints[index].mean).AdjointMap();
    recipe.h = recipe.progress * recipe.transport;
    recipe.f = Matrix::Zero(3, ids.size());
    if (gyro)
      for (const auto& [id, column] : ids)
      {
        const auto query = recipe.trajectory.gyro_jacobians.find(id);
        const auto end = endpoints[index].gyro_jacobians.find(id);
        if (query != recipe.trajectory.gyro_jacobians.end())
          recipe.f.col(column) += query->second;
        if (end != endpoints[index].gyro_jacobians.end())
          recipe.f.col(column) -= recipe.h * end->second;
      }
    return recipe;
  }

  Matrix3 W(const Recipe& a_query, const Recipe& b_query) const
  {
    if (a_query.window != b_query.window)
      return Matrix3::Zero();
    const double progress =
        std::min(a_query.progress, b_query.progress) - a_query.progress * b_query.progress;
    return a_query.transport * (progress * windows[a_query.window].edge.endpoint_covariance) *
           b_query.transport.transpose();
  }
};

void AddJacobian(Jacobians& target, Key key, const Matrix& value)
{
  const auto [it, inserted] = target.try_emplace(key, value);
  if (!inserted)
    it->second += value;
}

Vector GyroResidual(const FamilyData& data, const Values& values, Jacobians* jacobians = nullptr)
{
  Vector residual(data.c.rows());
  for (size_t i = 0; i < static_cast<size_t>(data.c.rows()); ++i)
  {
    const auto& window = data.windows[i];
    const auto& edge = window.edge;
    GyroPreintFactor factor(edge.left_key,
                            edge.right_key,
                            *edge.bias_key,
                            window.gyro->raw_integral_rad,
                            window.gyro->duration_s,
                            gtsam::noiseModel::Unit::Create(1));
    Matrix left, right, bias;
    residual(i) = factor.evaluateError(values.at<Pose2>(edge.left_key),
                                       values.at<Pose2>(edge.right_key),
                                       values.at<double>(*edge.bias_key),
                                       jacobians ? &left : nullptr,
                                       jacobians ? &right : nullptr,
                                       jacobians ? &bias : nullptr)(0);
    if (jacobians)
      for (const auto& [key, block] : std::vector<std::pair<Key, Matrix>>{{edge.left_key, left},
                                                                          {edge.right_key, right},
                                                                          {*edge.bias_key, bias}})
      {
        Matrix full = Matrix::Zero(data.c.rows(), block.cols());
        full.row(i) = block;
        AddJacobian(*jacobians, key, full);
      }
  }
  return residual;
}

Vector3 R0(const FamilyData& data,
           const Recipe& recipe,
           const Values& values,
           Jacobians* jacobians = nullptr)
{
  const auto& edge = data.windows[recipe.window].edge;
  const auto& endpoint = data.endpoints[recipe.window];
  const double reference = edge.bias_key ? edge.reference_values.at<double>(*edge.bias_key) : 0.0;
  RelativeConditionalFactor factor(
      edge.left_key,
      edge.bias_key,
      reference,
      {{recipe.key, recipe.trajectory.mean, recipe.trajectory.bias_jacobian, Matrix3::Identity()},
       {edge.right_key, endpoint.mean, endpoint.bias_jacobian, -recipe.h}},
      gtsam::noiseModel::Unit::Create(3));
  std::vector<Matrix> blocks;
  const Vector3 residual = factor.unwhitenedError(values, jacobians ? &blocks : nullptr);
  if (jacobians)
    for (size_t i = 0; i < factor.keys().size(); ++i)
      AddJacobian(*jacobians, factor.keys()[i], blocks[i]);
  return residual;
}

gtsam::KeyVector FactorKeys(const FamilyData& data,
                            const Recipe* target = nullptr,
                            const std::vector<Recipe>& previous = {})
{
  std::set<Key> keys;
  const auto add_window = [&](size_t i)
  {
    const auto& edge = data.windows[i].edge;
    keys.insert(edge.left_key);
    keys.insert(edge.right_key);
    if (edge.bias_key)
      keys.insert(*edge.bias_key);
  };
  if (data.gyro)
    for (size_t i = 0; i < data.windows.size(); ++i)
      add_window(i);
  if (target)
  {
    add_window(target->window);
    keys.insert(target->key);
  }
  for (const auto& recipe : previous)
  {
    add_window(recipe.window);
    keys.insert(recipe.key);
  }
  return {keys.begin(), keys.end()};
}

bool SameData(const FamilyData& a, const FamilyData& b, double tolerance)
{
  if (a.gyro != b.gyro || a.windows.size() != b.windows.size())
    return false;
  for (size_t i = 0; i < a.windows.size(); ++i)
  {
    const auto& x = a.windows[i];
    const auto& y = b.windows[i];
    if (x.edge.left_key != y.edge.left_key || x.edge.right_key != y.edge.right_key ||
        x.edge.bias_key != y.edge.bias_key ||
        !a.endpoints[i].mean.equals(b.endpoints[i].mean, tolerance) ||
        (a.endpoints[i].bias_jacobian - b.endpoints[i].bias_jacobian).norm() > tolerance)
      return false;
    if (a.gyro && (std::abs(x.gyro->raw_integral_rad - y.gyro->raw_integral_rad) > tolerance ||
                   std::abs(x.gyro->duration_s - y.gyro->duration_s) > tolerance ||
                   std::abs(x.edge.reference_values.at<double>(*x.edge.bias_key) -
                            y.edge.reference_values.at<double>(*y.edge.bias_key)) > tolerance))
      return false;
  }
  return true;
}

bool SameRecipe(const Recipe& a, const Recipe& b, double tolerance)
{
  return a.key == b.key && a.window == b.window && a.source_ns == b.source_ns &&
         a.trajectory.mean.equals(b.trajectory.mean, tolerance) &&
         (a.trajectory.bias_jacobian - b.trajectory.bias_jacobian).norm() <= tolerance &&
         (a.h - b.h).norm() <= tolerance;
}

class GyroFamilyFactor : public gtsam::NoiseModelFactor
{
public:
  GyroFamilyFactor(std::shared_ptr<const FamilyData> data, const Spectrum& spectrum)
      : NoiseModelFactor(Noise(spectrum), FactorKeys(*data)),
        data_(std::move(data)),
        rotation_(Rotation(spectrum))
  {
  }
  Vector unwhitenedError(const Values& values,
                         gtsam::OptionalMatrixVecType h = nullptr) const override
  {
    Jacobians blocks;
    const Vector residual = GyroResidual(*data_, values, h ? &blocks : nullptr);
    if (h)
    {
      h->resize(keys().size());
      for (size_t i = 0; i < keys().size(); ++i)
        (*h)[i] = rotation_ * blocks.at(keys()[i]);
    }
    return rotation_ * residual;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return std::make_shared<GyroFamilyFactor>(*this);
  }
  bool equals(const gtsam::NonlinearFactor& other, double tolerance = 1e-9) const override
  {
    const auto* factor = dynamic_cast<const GyroFamilyFactor*>(&other);
    return factor && NoiseModelFactor::equals(other, tolerance) &&
           SameData(*data_, *factor->data_, tolerance) &&
           (rotation_ - factor->rotation_).norm() <= tolerance;
  }

private:
  std::shared_ptr<const FamilyData> data_;
  Matrix rotation_;
};

class AcquisitionFactor : public gtsam::NoiseModelFactor
{
public:
  AcquisitionFactor(std::shared_ptr<const FamilyData> data,
                    Recipe target,
                    std::vector<Recipe> previous,
                    Matrix gain,
                    const Spectrum& spectrum)
      : NoiseModelFactor(Noise(spectrum), FactorKeys(*data, &target, previous)),
        data_(std::move(data)),
        target_(std::move(target)),
        previous_(std::move(previous)),
        gain_(std::move(gain)),
        rotation_(Rotation(spectrum))
  {
  }
  Vector unwhitenedError(const Values& values,
                         gtsam::OptionalMatrixVecType h = nullptr) const override
  {
    Jacobians blocks;
    Vector3 residual = R0(*data_, target_, values, h ? &blocks : nullptr);
    Jacobians gyro_blocks;
    if (data_->gyro)
    {
      const Vector g = GyroResidual(*data_, values, h ? &gyro_blocks : nullptr);
      const Matrix gain = gain_.leftCols(data_->c.rows());
      residual -= gain * g;
      if (h)
        for (const auto& [key, block] : gyro_blocks)
          AddJacobian(blocks, key, -gain * block);
    }
    for (size_t i = 0; i < previous_.size(); ++i)
    {
      Jacobians prior_blocks;
      const Matrix3 gain = gain_.middleCols<3>(data_->c.rows() + 3 * i);
      residual -= gain * R0(*data_, previous_[i], values, h ? &prior_blocks : nullptr);
      if (h)
        for (const auto& [key, block] : prior_blocks)
          AddJacobian(blocks, key, -gain * block);
    }
    if (h)
    {
      h->resize(keys().size());
      for (size_t i = 0; i < keys().size(); ++i)
        (*h)[i] = rotation_ * blocks.at(keys()[i]);
    }
    return rotation_ * residual;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return std::make_shared<AcquisitionFactor>(*this);
  }
  bool equals(const gtsam::NonlinearFactor& other, double tolerance = 1e-9) const override
  {
    const auto* factor = dynamic_cast<const AcquisitionFactor*>(&other);
    if (!factor || !NoiseModelFactor::equals(other, tolerance) ||
        !SameData(*data_, *factor->data_, tolerance) ||
        !SameRecipe(target_, factor->target_, tolerance) ||
        previous_.size() != factor->previous_.size() ||
        (gain_ - factor->gain_).norm() > tolerance ||
        (rotation_ - factor->rotation_).norm() > tolerance)
      return false;
    for (size_t i = 0; i < previous_.size(); ++i)
      if (!SameRecipe(previous_[i], factor->previous_[i], tolerance))
        return false;
    return true;
  }

private:
  std::shared_ptr<const FamilyData> data_;
  Recipe target_;
  std::vector<Recipe> previous_;
  Matrix gain_, rotation_;
};

std::shared_ptr<const FamilyData> Build(std::vector<SharedSourceProcessWindow> windows)
{
  if (windows.empty() || windows.size() > 2)
    throw std::invalid_argument("shared-source family requires one or two windows");
  auto data = std::make_shared<FamilyData>();
  data->windows = std::move(windows);
  data->gyro = data->windows.front().gyro.has_value();
  std::map<MotionSourceId, MotionGyroSource> sources;
  std::map<MotionSourceId, double> variances;
  std::map<Key, int64_t> endpoint_times;
  for (size_t i = 0; i < data->windows.size(); ++i)
  {
    const auto& window = data->windows[i];
    const auto& edge = window.edge;
    if (edge.end_ns <= edge.start_ns || edge.left_key == edge.right_key || !edge.trajectory_query ||
        window.gyro.has_value() != data->gyro || edge.bias_key.has_value() != data->gyro ||
        (!data->gyro && !edge.gyro_rate_variances.empty()))
      throw std::invalid_argument("invalid shared-source window");
    if (i && (data->windows[i - 1].edge.end_ns != edge.start_ns ||
              data->windows[i - 1].edge.right_key != edge.left_key ||
              data->windows[i - 1].edge.source_epoch != edge.source_epoch))
      throw std::invalid_argument("noncontiguous shared-source windows");
    Decompose(edge.endpoint_covariance);
    for (Key key : {edge.left_key, edge.right_key})
    {
      const int64_t time = key == edge.left_key ? edge.start_ns : edge.end_ns;
      const auto [binding, inserted] = endpoint_times.emplace(key, time);
      if (!inserted && binding->second != time)
        throw std::invalid_argument("pose key aliases different source endpoints");
      const auto& pose = edge.reference_values.at<Pose2>(key);
      if (!pose.matrix().allFinite() ||
          (data->dimensions.count(key) && data->dimensions.at(key) != 3))
        throw std::invalid_argument("invalid shared-source pose reference");
      if (data->references.exists(key))
      {
        if (!data->references.at<Pose2>(key).equals(pose, 1e-12))
          throw std::invalid_argument("inconsistent shared endpoint reference");
      }
      else
        data->references.insert(key, pose);
      data->dimensions[key] = 3;
    }
    if (!data->gyro)
      continue;
    const auto& gyro = *window.gyro;
    const double dt =
        static_cast<double>(static_cast<long double>(edge.end_ns) - edge.start_ns) * 1e-9;
    if (!std::isfinite(gyro.raw_integral_rad) || !std::isfinite(gyro.duration_s) ||
        gyro.duration_s <= 0.0 || !Near(gyro.duration_s, dt) ||
        !std::isfinite(gyro.sigma_floor_rad) || gyro.sigma_floor_rad < 0.0)
      throw std::invalid_argument("invalid shared-source gyro window");
    const Key bias = *edge.bias_key;
    if (data->dimensions.count(bias) && data->dimensions.at(bias) != 1)
      throw std::invalid_argument("shared-source bias key aliases a pose");
    const double reference = edge.reference_values.at<double>(bias);
    if (!std::isfinite(reference))
      throw std::invalid_argument("invalid shared-source bias reference");
    if (data->references.exists(bias))
    {
      if (data->references.at<double>(bias) != reference)
        throw std::invalid_argument("inconsistent shared bias reference");
    }
    else
      data->references.insert(bias, reference);
    data->dimensions[bias] = 1;
    double duration = 0.0, integral = 0.0;
    std::set<MotionSourceId> local_ids;
    for (const auto& use : gyro.sources)
    {
      const auto& source = use.source;
      const auto variance = edge.gyro_rate_variances.find(source.id);
      if (source.id.kind != MotionSourceKind::Gyro || !std::isfinite(source.raw_z) ||
          source.support.end_ns < source.support.start_ns || !std::isfinite(use.duration_s) ||
          use.duration_s <= 0.0 || variance == edge.gyro_rate_variances.end() ||
          !std::isfinite(variance->second) || variance->second < 0.0)
        throw std::invalid_argument("invalid shared-source gyro identity/variance/use");
      const auto [record, fresh] = sources.emplace(source.id, source);
      const auto [noise, fresh_noise] = variances.emplace(source.id, variance->second);
      if ((!fresh && record->second != source) ||
          (!fresh_noise && noise->second != variance->second))
        throw std::invalid_argument("changed payload/support/variance for shared gyro ID");
      local_ids.insert(source.id);
      duration += use.duration_s;
      integral += use.duration_s * source.raw_z;
    }
    if (!Near(duration, gyro.duration_s) || !Near(integral, gyro.raw_integral_rad) ||
        local_ids.size() != edge.gyro_rate_variances.size())
      throw std::invalid_argument("incoherent shared-source gyro integral or identity table");
  }
  for (const auto& [id, variance] : variances)
    data->ids.emplace(id, data->ids.size());
  data->q = Vector::Zero(data->ids.size());
  for (const auto& [id, column] : data->ids)
    data->q(column) = variances.at(id);
  const size_t count = data->gyro ? data->windows.size() : 0;
  data->a = Matrix::Zero(count, data->ids.size());
  for (size_t i = 0; i < count; ++i)
    for (const auto& use : data->windows[i].gyro->sources)
      data->a(i, data->ids.at(use.source.id)) += use.duration_s;
  data->c = data->a * data->q.asDiagonal() * data->a.transpose();
  for (size_t i = 0; i < count; ++i)
  {
    const double floor = data->windows[i].gyro->sigma_floor_rad;
    data->c(i, i) = std::max(data->c(i, i), floor * floor);
  }
  if (count)
    Decompose(data->c);
  for (size_t i = 0; i < data->windows.size(); ++i)
    data->endpoints.push_back(data->Query(i, data->windows[i].edge.end_ns));
  return data;
}
}  // namespace

struct SharedSourceProcessFamily::Impl
{
  std::shared_ptr<const FamilyData> data;
  std::vector<Recipe> previous;
  Values references;
  gtsam::NonlinearFactor::shared_ptr gyro_factor;

  explicit Impl(std::vector<SharedSourceProcessWindow> windows) : data(Build(std::move(windows)))
  {
    references = data->references;
    if (data->gyro)
      gyro_factor = std::make_shared<GyroFamilyFactor>(data, Decompose(data->c));
  }

  std::optional<Key> Alias(int64_t time) const
  {
    if (time < data->windows.front().edge.start_ns || time > data->windows.back().edge.end_ns)
      throw std::out_of_range("source outside shared-source family");
    for (const auto& window : data->windows)
    {
      if (time == window.edge.start_ns)
        return window.edge.left_key;
      if (time == window.edge.end_ns)
        return window.edge.right_key;
    }
    for (const auto& query : previous)
      if (time == query.source_ns)
        return query.key;
    return std::nullopt;
  }

  struct Conditional
  {
    Matrix gain;
    Spectrum noise;
    std::optional<Spectrum> conditioning;
  };
  Conditional Condition(const Recipe& target) const
  {
    const size_t gyro_rows = data->c.rows(), rows = gyro_rows + 3 * previous.size();
    Matrix covariance = Matrix::Zero(rows, rows), cross = Matrix::Zero(3, rows);
    if (gyro_rows)
    {
      covariance.topLeftCorner(gyro_rows, gyro_rows) = data->c;
      cross.leftCols(gyro_rows) = target.f * data->q.asDiagonal() * data->a.transpose();
    }
    for (size_t i = 0; i < previous.size(); ++i)
    {
      const auto& a_query = previous[i];
      const size_t offset = gyro_rows + 3 * i;
      if (gyro_rows)
      {
        covariance.block(0, offset, gyro_rows, 3) =
            data->a * data->q.asDiagonal() * a_query.f.transpose();
        covariance.block(offset, 0, 3, gyro_rows) =
            covariance.block(0, offset, gyro_rows, 3).transpose().eval();
      }
      cross.middleCols<3>(offset) =
          data->W(target, a_query) + target.f * data->q.asDiagonal() * a_query.f.transpose();
      for (size_t j = 0; j <= i; ++j)
      {
        const auto& b_query = previous[j];
        covariance.block<3, 3>(offset, gyro_rows + 3 * j) =
            data->W(a_query, b_query) + a_query.f * data->q.asDiagonal() * b_query.f.transpose();
        covariance.block<3, 3>(gyro_rows + 3 * j, offset) =
            covariance.block<3, 3>(offset, gyro_rows + 3 * j).transpose().eval();
      }
    }
    const Matrix3 unconditional =
        data->W(target, target) + target.f * data->q.asDiagonal() * target.f.transpose();
    std::optional<Spectrum> conditioning;
    Matrix gain = Matrix::Zero(3, rows);
    if (rows)
    {
      conditioning = Decompose(covariance);
      gain = SolveRight(cross, *conditioning);
    }
    Matrix3 residual = unconditional - gain * cross.transpose();
    residual = (residual + residual.transpose()).eval() * 0.5;
    return {gain, Decompose(residual, unconditional.diagonal()), conditioning};
  }

  Pose2 Mean(const Recipe& target, const Conditional& conditional, const Values& values) const
  {
    Vector conditioning(data->c.rows() + 3 * previous.size());
    if (data->gyro)
      conditioning.head(data->c.rows()) = GyroResidual(*data, values);
    for (size_t i = 0; i < previous.size(); ++i)
      conditioning.segment<3>(data->c.rows() + 3 * i) = R0(*data, previous[i], values);
    if (conditional.conditioning)
      CheckSupport(conditioning, *conditional.conditioning);
    const auto& edge = data->windows[target.window].edge;
    const double delta = edge.bias_key ? values.at<double>(*edge.bias_key) -
                                             edge.reference_values.at<double>(*edge.bias_key)
                                       : 0.0;
    const auto& left = values.at<Pose2>(edge.left_key);
    const auto& end = data->endpoints[target.window];
    const RelativeConditionalFactor::Term endpoint{edge.right_key,
                                                   end.mean,
                                                   end.bias_jacobian,
                                                   Matrix3::Identity()};
    const Vector3 predicted =
        target.h * RelativeConditionalFactor::Coordinate(endpoint,
                                                         left,
                                                         values.at<Pose2>(edge.right_key),
                                                         delta) +
        conditional.gain * conditioning;
    constexpr double margin = 128.0 * std::numeric_limits<double>::epsilon();
    if (!predicted.allFinite() || std::abs(predicted.z()) >= std::numbers::pi - margin)
      throw UnsupportedSharedSourceConditional("shared-source mean outside principal Log branch");
    const RelativeConditionalFactor::Term term{target.key,
                                               target.trajectory.mean,
                                               target.trajectory.bias_jacobian,
                                               Matrix3::Identity()};
    const Pose2 initial = left.compose(RelativeConditionalFactor::Mean(term, delta))
                              .compose(Pose2::Expmap(predicted));
    const Vector3 recovered = RelativeConditionalFactor::Coordinate(term, left, initial, delta);
    if (!initial.matrix().allFinite() ||
        (recovered - predicted).norm() > 1e-10 * std::max(1.0, predicted.norm()))
      throw UnsupportedSharedSourceConditional("invalid shared-source mean inversion");
    return initial;
  }
};

SharedSourceProcessFamily::SharedSourceProcessFamily(std::vector<SharedSourceProcessWindow> windows)
    : impl_(std::make_unique<Impl>(std::move(windows)))
{
}
SharedSourceProcessFamily::~SharedSourceProcessFamily() = default;
SharedSourceProcessFamily::SharedSourceProcessFamily(const SharedSourceProcessFamily& other)
    : impl_(std::make_unique<Impl>(*other.impl_))
{
}
SharedSourceProcessFamily& SharedSourceProcessFamily::operator=(
    const SharedSourceProcessFamily& other)
{
  if (this != &other)
  {
    auto copy = std::make_unique<Impl>(*other.impl_);
    impl_.swap(copy);
  }
  return *this;
}
SharedSourceProcessFamily::SharedSourceProcessFamily(SharedSourceProcessFamily&&) noexcept =
    default;
SharedSourceProcessFamily& SharedSourceProcessFamily::operator=(
    SharedSourceProcessFamily&&) noexcept = default;
gtsam::NonlinearFactor::shared_ptr SharedSourceProcessFamily::endpoint_gyro_factor() const
{
  return impl_->gyro_factor;
}
SharedSourceProcessFamily::Insertion SharedSourceProcessFamily::Insert(int64_t source_ns,
                                                                       Key new_key,
                                                                       const Values* current_values)
{
  if (const auto alias = impl_->Alias(source_ns))
    return {*alias, std::nullopt, nullptr};
  if (impl_->references.exists(new_key) || (current_values && current_values->exists(new_key)))
    throw std::invalid_argument("shared-source acquisition key already registered");
  const auto target = impl_->data->MakeRecipe(source_ns, new_key);
  const auto conditional = impl_->Condition(target);
  const Pose2 initial =
      impl_->Mean(target, conditional, current_values ? *current_values : impl_->references);
  auto factor = std::make_shared<AcquisitionFactor>(
      impl_->data, target, impl_->previous, conditional.gain, conditional.noise);
  // Both containers and the factor are fully allocated before either live registry changes.
  auto staged_previous = impl_->previous;
  auto staged_references = impl_->references;
  staged_previous.push_back(target);
  staged_references.insert(new_key, initial);
  impl_->previous.swap(staged_previous);
  impl_->references.swap(staged_references);
  return {new_key, initial, factor};
}
Pose2 SharedSourceProcessFamily::ConditionalMean(int64_t source_ns, const Values& values) const
{
  if (const auto alias = impl_->Alias(source_ns))
    return values.at<Pose2>(*alias);
  const auto target = impl_->data->MakeRecipe(source_ns, 0);
  return impl_->Mean(target, impl_->Condition(target), values);
}
const std::vector<SharedSourceProcessWindow>& SharedSourceProcessFamily::windows() const
{
  return impl_->data->windows;
}
std::vector<SharedSourceProcessFamily::Acquisition> SharedSourceProcessFamily::acquisitions() const
{
  std::vector<Acquisition> result;
  result.reserve(impl_->previous.size());
  for (const auto& query : impl_->previous)
    result.push_back({query.source_ns, query.key});
  return result;
}
uint64_t SharedSourceProcessFamily::source_epoch() const
{
  return impl_->data->windows.front().edge.source_epoch;
}
int64_t SharedSourceProcessFamily::start_ns() const
{
  return impl_->data->windows.front().edge.start_ns;
}
int64_t SharedSourceProcessFamily::end_ns() const
{
  return impl_->data->windows.back().edge.end_ns;
}
}  // namespace fusion_graph
