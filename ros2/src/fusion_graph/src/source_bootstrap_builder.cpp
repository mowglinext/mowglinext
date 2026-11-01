// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "source_bootstrap_builder.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include "fusion_graph/factors.hpp"
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
namespace
{
using Observation = GraphMotionState::Observation;
using gtsam::Key;
using gtsam::Pose2;
using gtsam::Symbol;

double SpanSeconds(int64_t start, int64_t end)
{
  return static_cast<double>(static_cast<long double>(end) - start) * 1e-9;
}

bool Near(double a, double b)
{
  return std::isfinite(a) && std::isfinite(b) &&
         std::abs(a - b) <= 1e-12 * std::max({1.0, std::abs(a), std::abs(b)});
}

gtsam::SharedNoiseModel Diagonal(const gtsam::Vector& sigmas)
{
  if (!sigmas.allFinite() || (sigmas.array() <= 0.0).any())
    throw std::invalid_argument("invalid bootstrap sigma");
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::SharedNoiseModel ObservationNoise(const GraphParams& params, const Observation& observation)
{
  auto noise = Diagonal(gtsam::Vector::Constant(observation.yaw ? 1 : 2, observation.sigma));
  if (observation.robust)
  {
    const double huber = observation.yaw ? params.huber_k_yaw : params.huber_k_gps;
    if (!std::isfinite(huber) || huber <= 0.0)
      throw std::invalid_argument("invalid bootstrap Huber parameter");
    noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(huber),
                                              noise);
  }
  return noise;
}

void ValidateObservation(const Observation& observation)
{
  if (!std::isfinite(observation.sigma) || observation.sigma <= 0.0 ||
      (observation.yaw ? !std::isfinite(*observation.yaw) : !observation.xy.allFinite()))
    throw std::invalid_argument("invalid bootstrap observation");
}

void ValidateSupport(const SourceBootstrapBuildRequest& request, int64_t start)
{
  MotionSourceHistory validation(request.original_steps.size());
  validation.Reset(request.epoch);
  for (const auto& step : request.original_steps)
    if (step.end_ns > start && step.start_ns < request.frontier_ns &&
        validation.Append(step) != MotionHistoryStatus::Ok)
      throw std::invalid_argument("invalid original bootstrap source history");
  if (validation.Query(request.epoch, start, request.frontier_ns).status != MotionHistoryStatus::Ok)
    throw std::invalid_argument("uncovered bootstrap source history");
}

ProcessWindowPolicyInput Accumulate(const SourceBootstrapBuildRequest& request,
                                    int64_t start,
                                    int64_t end)
{
  ProcessWindowPolicyInput input;
  for (const auto& step : request.original_steps)
  {
    const int64_t begin = std::max(start, step.start_ns);
    const int64_t finish = std::min(end, step.end_ns);
    if (finish <= begin)
      continue;
    const double dt = SpanSeconds(begin, finish);
    input.dx += step.wheel.vx * dt;
    input.dy += step.wheel.vy * dt;
    input.wheel_travel_m += std::hypot(step.wheel.vx, step.wheel.vy) * dt;
    input.dtheta_wheel += step.wheel.wz * dt;
    input.dtheta_gyro += (step.gyro.raw_z - step.bias_linearization) * dt;
    input.dt_total += dt;
    input.max_abs_gyro_rad_per_s =
        std::max(input.max_abs_gyro_rad_per_s, std::abs(step.gyro.raw_z));
  }
  return input;
}

void ValidateSuffixRecipe(const SourceBootstrapBuildRequest& request)
{
  const auto raw = Accumulate(request, request.source_s_ns, request.frontier_ns);
  const auto& input = *request.suffix_policy_input;
  if (!Near(input.dx, raw.dx) || !Near(input.dy, raw.dy) ||
      !Near(input.wheel_travel_m, raw.wheel_travel_m) ||
      !Near(input.dtheta_wheel, raw.dtheta_wheel) || !Near(input.dtheta_gyro, raw.dtheta_gyro) ||
      !Near(input.dt_total, raw.dt_total) ||
      !Near(input.max_abs_gyro_rad_per_s, raw.max_abs_gyro_rad_per_s) ||
      input.last_node_time_s != static_cast<double>(request.initial_schedule_ns) * 1e-9 ||
      input.now_s != static_cast<double>(request.schedule_ns) * 1e-9)
    throw std::invalid_argument("bootstrap suffix policy recipe differs from raw window/schedule");
}

struct PreparedWindow
{
  SharedSourceProcessWindow window;
  Pose2 between;
  double bias_reference = 0.0;
};

PreparedWindow PrepareWindow(const SourceBootstrapBuildRequest& request,
                             int64_t start,
                             int64_t end,
                             Key left,
                             Key right,
                             Key bias,
                             const ProcessWindowPolicyResult& policy)
{
  PreparedWindow result;
  auto& edge = result.window.edge;
  edge.left_key = left;
  edge.right_key = right;
  edge.source_epoch = request.epoch;
  edge.start_ns = start;
  edge.end_ns = end;
  const gtsam::Vector3 sigmas(policy.wheel_sigma_x, policy.wheel_sigma_y, policy.wheel_sigma_theta);
  (void)Diagonal(sigmas);
  edge.endpoint_covariance = sigmas.array().square().matrix().asDiagonal();
  auto history = std::make_shared<MotionSourceHistory>(request.original_steps.size());
  history->Reset(request.epoch);
  if (request.params.use_imu_preint)
  {
    edge.bias_key = bias;
    result.window.gyro.emplace();
  }
  std::optional<double> recorded_bias;
  const double density = request.params.gyro_noise_density_rad_per_s;
  if (request.params.use_imu_preint && (!std::isfinite(density) || density < 0.0))
    throw std::invalid_argument("invalid bootstrap gyro noise density");
  for (auto step : request.original_steps)
  {
    step.start_ns = std::max(start, step.start_ns);
    step.end_ns = std::min(end, step.end_ns);
    if (step.end_ns <= step.start_ns)
      continue;
    if (request.params.use_imu_preint && recorded_bias && *recorded_bias != step.bias_linearization)
      throw std::invalid_argument("bias reference changed inside bootstrap process window");
    recorded_bias = step.bias_linearization;
    step.wheel_translation_enabled = policy.wheel_translation_enabled;
    step.gyro_rotation_enabled = policy.gyro_rotation_enabled;
    if (history->Append(step) != MotionHistoryStatus::Ok)
      throw std::invalid_argument("incoherent clipped bootstrap process window");
    if (result.window.gyro)
    {
      const double dt = SpanSeconds(step.start_ns, step.end_ns);
      auto& gyro = *result.window.gyro;
      gyro.sources.push_back({step.gyro, dt});
      gyro.duration_s += dt;
      gyro.raw_integral_rad += step.gyro.raw_z * dt;
      edge.gyro_rate_variances.emplace(step.gyro.id, density * density);
    }
  }
  if (!recorded_bias)
    throw std::invalid_argument("empty bootstrap process window");
  // Off mode retains each step's recorded EMA correction in the history;
  // there is no common graph-bias variable or frozen scalar reference.
  result.bias_reference = request.params.use_imu_preint ? *recorded_bias : 0.0;
  const auto trajectory = history->Query(request.epoch, start, end);
  if (trajectory.status != MotionHistoryStatus::Ok)
    throw std::invalid_argument("uncovered clipped bootstrap process window");
  result.between = trajectory.mean;
  edge.trajectory_query = FrozenProcessEdge::HistoryQuery(history, request.epoch, start);
  return result;
}

void SetReferences(PreparedWindow& window, const Pose2& left, const Pose2& right)
{
  auto& edge = window.window.edge;
  edge.reference_values.insert(edge.left_key, left);
  edge.reference_values.insert(edge.right_key, right);
  if (edge.bias_key)
    edge.reference_values.insert(*edge.bias_key, window.bias_reference);
}

gtsam::NonlinearFactor::shared_ptr PositionFactor(const SourceBootstrapBuildRequest& request,
                                                  const Pose2& initial)
{
  auto factor =
      std::make_shared<GnssLeverArmFactor>(Symbol('x', 0),
                                           request.position.xy,
                                           gtsam::Vector2(request.params.lever_arm_x,
                                                          request.params.lever_arm_y),
                                           ObservationNoise(request.params, request.position));
  if (!request.preserved_primary_gnss)
    return factor;
  const auto* preserved =
      dynamic_cast<const GnssLeverArmFactor*>(request.preserved_primary_gnss.get());
  if (!preserved || !preserved->equals(*factor, 1e-12))
    throw std::invalid_argument("invalid preserved bootstrap GNSS factor");
  for (const Pose2& pose : {initial, initial.compose(Pose2(.31, -.2, .7))})
    if ((preserved->evaluateError(pose) - factor->evaluateError(pose)).norm() > 1e-12)
      throw std::invalid_argument("preserved bootstrap GNSS likelihood changed");
  return request.preserved_primary_gnss;
}
}  // namespace

SourceBootstrapCandidate BuildSourceBootstrapCandidate(const SourceBootstrapBuildRequest& request)
{
  const bool prefix = request.heading.source_ns < request.source_s_ns;
  const bool suffix = request.frontier_ns > request.source_s_ns;
  if (request.position.yaw || !request.heading.yaw ||
      request.position.source_ns != request.source_s_ns ||
      request.heading.source_ns == request.source_s_ns ||
      request.heading.source_ns > request.frontier_ns ||
      request.frontier_ns < request.source_s_ns ||
      request.schedule_ns < request.initial_schedule_ns ||
      request.suffix_policy_input.has_value() != suffix || (!prefix && request.prefix_left_key) ||
      !std::isfinite(request.params.lever_arm_x) || !std::isfinite(request.params.lever_arm_y) ||
      !std::isfinite(request.public_policy_baseline.residual_ema) ||
      !std::isfinite(request.public_policy_baseline.last_wheel_sigma_x_eff))
    throw std::invalid_argument("invalid cross-time bootstrap request");
  ValidateObservation(request.position);
  ValidateObservation(request.heading);
  ValidateSupport(request, std::min(request.source_s_ns, request.heading.source_ns));
  SourceBootstrapCandidate result;
  result.next_acquisition = request.next_acquisition;
  auto allocate = [&](int64_t source_ns, std::optional<Key> reuse = std::nullopt)
  {
    if (reuse && Symbol(*reuse).index() >= result.next_acquisition)
      throw std::invalid_argument("bootstrap reused auxiliary key exceeds allocation frontier");
    const Key key = reuse ? *reuse : Key(Symbol('a', result.next_acquisition++));
    if (Symbol(key).chr() != 'a' || !result.auxiliary_poses.emplace(key, source_ns).second)
      throw std::invalid_argument("invalid bootstrap auxiliary key allocation");
    return key;
  };
  if (prefix)
  {
    result.prefix_key = allocate(request.heading.source_ns, request.prefix_left_key);
    auto input = Accumulate(request, request.heading.source_ns, request.source_s_ns);
    input.now_s = SpanSeconds(request.heading.source_ns, request.source_s_ns);
    auto policy = SelectProcessWindowPolicy(request.params, input, request.public_policy_baseline);
    result.prefix_policy.emplace(SourceBootstrapPrefixPolicy{input, std::move(policy)});
  }
  result.frontier_key =
      suffix ? (request.promote ? Key(Symbol('x', 1)) : allocate(request.frontier_ns))
             : Key(Symbol('x', 0));
  if (suffix)
  {
    ValidateSuffixRecipe(request);
    result.suffix_policy = SelectProcessWindowPolicy(request.params,
                                                     *request.suffix_policy_input,
                                                     request.public_policy_baseline);
  }
  std::optional<PreparedWindow> prefix_window, suffix_window;
  if (prefix)
    prefix_window = PrepareWindow(request,
                                  request.heading.source_ns,
                                  request.source_s_ns,
                                  *result.prefix_key,
                                  Symbol('x', 0),
                                  Symbol('b', 0),
                                  result.prefix_policy->selection);
  if (suffix)
    suffix_window = PrepareWindow(request,
                                  request.source_s_ns,
                                  request.frontier_ns,
                                  Symbol('x', 0),
                                  result.frontier_key,
                                  Symbol('b', 1),
                                  *result.suffix_policy);

  // This is only a guess. Every original likelihood, the cold bias prior and
  // nuisance pose remains in the joint nonlinear optimization below.
  double yaw_s = *request.heading.yaw;
  if (prefix_window)
    yaw_s += prefix_window->between.theta();
  else
  {
    const auto transport =
        (*suffix_window->window.edge.trajectory_query)(request.heading.source_ns);
    if (transport.status != MotionHistoryStatus::Ok)
      throw std::invalid_argument("uncovered bootstrap heading transport");
    yaw_s -= transport.mean.theta();
  }
  const gtsam::Vector2 lever(request.params.lever_arm_x, request.params.lever_arm_y);
  const gtsam::Vector2 base = request.position.xy - gtsam::Rot2(yaw_s).rotate(lever);
  const Pose2 source_s(base.x(), base.y(), yaw_s);
  gtsam::Values initial;
  initial.insert(Symbol('x', 0), source_s);
  std::vector<SharedSourceProcessWindow> windows;
  if (prefix_window)
  {
    const Pose2 left = source_s.compose(prefix_window->between.inverse());
    initial.insert(*result.prefix_key, left);
    SetReferences(*prefix_window, left, source_s);
    windows.push_back(prefix_window->window);
  }
  if (suffix_window)
  {
    const Pose2 right = source_s.compose(suffix_window->between);
    initial.insert(result.frontier_key, right);
    SetReferences(*suffix_window, source_s, right);
    windows.push_back(suffix_window->window);
  }
  gtsam::NonlinearFactorGraph factors;
  if (request.params.use_imu_preint)
  {
    initial.insert(Symbol('b', 0), prefix_window ? prefix_window->bias_reference : 0.0);
    factors.emplace_shared<gtsam::PriorFactor<double>>(
        Symbol('b', 0),
        0.0,
        Diagonal(gtsam::Vector1(request.params.gyro_bias_prior_sigma_rad_per_s)));
    if (suffix_window)
    {
      initial.insert(Symbol('b', 1), suffix_window->bias_reference);
      const double sigma_rw =
          request.params.gyro_bias_rw_rad_per_s * std::sqrt(suffix_window->window.gyro->duration_s);
      factors.emplace_shared<gtsam::BetweenFactor<double>>(
          Symbol('b', 0), Symbol('b', 1), 0.0, Diagonal(gtsam::Vector1(std::max(sigma_rw, 1e-6))));
    }
  }
  for (const auto& window : windows)
  {
    const auto& edge = window.edge;
    const auto mean = (*edge.trajectory_query)(edge.end_ns).mean;
    factors.emplace_shared<gtsam::BetweenFactor<Pose2>>(
        edge.left_key,
        edge.right_key,
        mean,
        Diagonal(edge.endpoint_covariance.diagonal().cwiseSqrt()));
  }
  result.family = std::make_shared<SharedSourceProcessFamily>(std::move(windows));
  if (auto gyro = result.family->endpoint_gyro_factor())
    factors.push_back(gyro);
  auto position = PositionFactor(request, source_s);
  factors.push_back(position);
  result.raw_bindings.push_back(
      {false, request.position.id, request.source_s_ns, Symbol('x', 0), position});
  std::set<std::pair<bool, uint64_t>> identities{{false, request.position.id}};
  auto add_observation = [&](const Observation& observation)
  {
    ValidateObservation(observation);
    if (!identities.emplace(observation.yaw.has_value(), observation.id).second)
      throw std::invalid_argument("duplicate raw bootstrap observation");
    const auto inserted = result.family->Insert(observation.source_ns,
                                                Symbol('a', result.next_acquisition),
                                                &initial);
    if (inserted.initial_value)
    {
      const Key allocated = allocate(observation.source_ns);
      if (allocated != inserted.key || !inserted.factor)
        throw std::invalid_argument("invalid bootstrap conditional insertion");
      initial.insert(inserted.key, *inserted.initial_value);
      factors.push_back(inserted.factor);
    }
    gtsam::NonlinearFactor::shared_ptr factor;
    if (observation.yaw)
      factor = std::make_shared<YawUnaryFactor>(inserted.key,
                                                *observation.yaw,
                                                ObservationNoise(request.params, observation));
    else
      factor = std::make_shared<GnssLeverArmFactor>(inserted.key,
                                                    observation.xy,
                                                    lever,
                                                    ObservationNoise(request.params, observation));
    factors.push_back(factor);
    result.raw_bindings.push_back(
        {observation.yaw.has_value(), observation.id, observation.source_ns, inserted.key, factor});
  };
  add_observation(request.heading);
  for (const auto& observation : request.additional_observations)
  {
    if (observation.source_ns < request.source_s_ns || observation.source_ns > request.frontier_ns)
      throw std::invalid_argument("additional bootstrap observation outside public source window");
    add_observation(observation);
  }
  std::set<const gtsam::NonlinearFactor*> unique;
  for (const auto& factor : factors)
    unique.insert(factor.get());
  for (const auto& factor : request.extra_factors)
  {
    if (!factor || !unique.insert(factor.get()).second)
      throw std::invalid_argument("duplicate/null extra bootstrap factor");
    for (Key key : factor->keys())
      if (!initial.exists(key))
        throw std::invalid_argument("extra bootstrap factor references an unavailable state");
    factors.push_back(factor);
  }
  const auto optimized = gtsam::LevenbergMarquardtOptimizer(factors, initial).optimize();
  gtsam::ISAM2Params parameters;
  parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  parameters.relinearizeThreshold = 0.05;
  parameters.relinearizeSkip = std::max(1, request.params.isam2_relinearize_skip);
  result.isam = std::make_unique<gtsam::ISAM2>(parameters);
  result.isam->update(factors, optimized);
  result.values = result.isam->calculateEstimate();
  for (const auto& value : result.values)
  {
    const bool bias = Symbol(value.key).chr() == 'b';
    if (bias ? !std::isfinite(value.value.cast<double>())
             : !value.value.cast<Pose2>().matrix().allFinite())
      throw std::runtime_error("nonfinite bootstrap candidate estimate");
  }
  result.source_s_pose = result.values.at<Pose2>(Symbol('x', 0));
  result.source_s_covariance = result.isam->marginalCovariance(Symbol('x', 0));
  result.frontier_pose = result.values.at<Pose2>(result.frontier_key);
  result.frontier_covariance = result.isam->marginalCovariance(result.frontier_key);
  if (!result.source_s_covariance.allFinite() || !result.frontier_covariance.allFinite())
    throw std::runtime_error("nonfinite bootstrap candidate marginal");
  return result;
}
}  // namespace fusion_graph
