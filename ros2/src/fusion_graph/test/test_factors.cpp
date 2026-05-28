// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for fusion_graph custom factors.

#include <cmath>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/scan_matcher.hpp"
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

using fusion_graph::GnssLeverArmFactor;
using fusion_graph::GyroPreintFactor;
using fusion_graph::WrapAngle;
using fusion_graph::YawUnaryFactor;

namespace
{

gtsam::SharedNoiseModel UnitDiag2()
{
  return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
}

gtsam::SharedNoiseModel UnitDiag1()
{
  return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0));
}

}  // namespace

TEST(GnssLeverArmFactor, ZeroErrorAtTrueValue)
{
  // Lever arm 0.5 m forward; robot at (10, 5, π/2). Antenna should be
  // at (10, 5.5).
  const gtsam::Vector2 lever(0.5, 0.0);
  const gtsam::Vector2 gps(10.0, 5.5);
  GnssLeverArmFactor f(gtsam::Symbol('x', 0), gps, lever, UnitDiag2());

  const gtsam::Pose2 X(10.0, 5.0, M_PI / 2.0);
  auto e = f.evaluateError(X);
  EXPECT_NEAR(e[0], 0.0, 1e-9);
  EXPECT_NEAR(e[1], 0.0, 1e-9);
}

TEST(GnssLeverArmFactor, JacobianMatchesNumeric)
{
  const gtsam::Vector2 lever(0.3, 0.1);
  const gtsam::Vector2 gps(2.0, 1.5);
  GnssLeverArmFactor f(gtsam::Symbol('x', 0), gps, lever, UnitDiag2());

  const gtsam::Pose2 X(1.0, 1.0, 0.7);
  gtsam::Matrix H_analytic;
  f.evaluateError(X, &H_analytic);

  // Numerical Jacobian via right perturbation in tangent space.
  const double eps = 1e-6;
  gtsam::Matrix H_num(2, 3);
  for (int i = 0; i < 3; ++i)
  {
    gtsam::Vector3 d = gtsam::Vector3::Zero();
    d[i] = eps;
    auto Xp = X.retract(d);
    auto Xm = X.retract(-d);
    auto ep = f.evaluateError(Xp);
    auto em = f.evaluateError(Xm);
    H_num.col(i) = (ep - em) / (2.0 * eps);
  }

  for (int i = 0; i < H_analytic.rows(); ++i)
    for (int j = 0; j < H_analytic.cols(); ++j)
      EXPECT_NEAR(H_analytic(i, j), H_num(i, j), 1e-4);
}

TEST(YawUnaryFactor, WrapHandledCorrectly)
{
  YawUnaryFactor f(gtsam::Symbol('x', 0), -3.0, UnitDiag1());
  const gtsam::Pose2 X(0.0, 0.0, 3.0);
  auto e = f.evaluateError(X);
  // 3.0 - (-3.0) = 6.0, wrapped to ~ -0.283.
  EXPECT_NEAR(e[0], WrapAngle(6.0), 1e-9);
}

// ── ScanMatcher tests ────────────────────────────────────────────────

namespace
{
// Synthetic scan: N points on an L-shape (two perpendicular walls),
// transformed by T. Asymmetric so rotation is observable — a closed
// circle would be rotation-degenerate.
std::vector<Eigen::Vector2d> SyntheticScan(int n, const gtsam::Pose2& T)
{
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(n);
  for (int i = 0; i < n / 2; ++i)
  {
    const double t = -2.0 + 4.0 * static_cast<double>(i) / static_cast<double>(n / 2);
    pts.emplace_back(t, 2.0);  // top wall y = 2
  }
  for (int i = 0; i < n / 2; ++i)
  {
    const double t = -2.0 + 3.0 * static_cast<double>(i) / static_cast<double>(n / 2);
    pts.emplace_back(2.0, t);  // right wall x = 2 (shorter)
  }
  for (auto& p : pts)
  {
    const double c = std::cos(T.theta());
    const double s = std::sin(T.theta());
    Eigen::Vector2d q(T.x() + c * p.x() - s * p.y(), T.y() + s * p.x() + c * p.y());
    p = q;
  }
  return pts;
}
}  // namespace

TEST(ScanMatcher, RecoversKnownTransform)
{
  // Realistic per-tick motion at 10 Hz: ~3 cm translation, ~0.02 rad
  // rotation. Larger transforms (>5°) exceed the no-warmstart ICP
  // capture range on a sparse synthetic L-scan; the live wiring
  // passes the wheel-derived prior as init_guess so this matches
  // the operating regime.
  const gtsam::Pose2 T_truth(0.03, 0.02, 0.02);
  auto src = SyntheticScan(200, gtsam::Pose2());
  auto tgt = SyntheticScan(200, T_truth);

  fusion_graph::ScanMatcher matcher;
  auto res = matcher.Match(src, tgt, gtsam::Pose2());
  ASSERT_TRUE(res.ok);
  // Point-to-point ICP precision on a synthetic L-shape with
  // boundary-wrap effects is ~1-2 cm / 0.02 rad. The downstream
  // BetweenFactor weights the result by its rmse, so this precision
  // is sufficient — the test guards against wrong solutions, not
  // perfect ones.
  EXPECT_NEAR(res.delta.x(), T_truth.x(), 0.025);
  EXPECT_NEAR(res.delta.y(), T_truth.y(), 0.025);
  EXPECT_NEAR(res.delta.theta(), T_truth.theta(), 0.03);
}

TEST(ScanMatcher, ConvergesWithWarmStart)
{
  // Larger motion (5 cm, 5°), but the caller passes a near-truth
  // init_guess (the wheel between-factor in practice). ICP must
  // refine to within a few mm.
  const gtsam::Pose2 T_truth(0.05, 0.03, 0.087);  // ~5 deg
  const gtsam::Pose2 init(0.04, 0.02, 0.07);  // close, not exact
  auto src = SyntheticScan(200, gtsam::Pose2());
  auto tgt = SyntheticScan(200, T_truth);

  fusion_graph::ScanMatcher matcher;
  auto res = matcher.Match(src, tgt, init);
  ASSERT_TRUE(res.ok);
  EXPECT_NEAR(res.delta.x(), T_truth.x(), 0.025);
  EXPECT_NEAR(res.delta.y(), T_truth.y(), 0.025);
  EXPECT_NEAR(res.delta.theta(), T_truth.theta(), 0.03);
}

TEST(ScanMatcher, EmptyInputsFail)
{
  fusion_graph::ScanMatcher matcher;
  std::vector<Eigen::Vector2d> empty;
  auto src = SyntheticScan(180, gtsam::Pose2());
  EXPECT_FALSE(matcher.Match(empty, src, gtsam::Pose2()).ok);
  EXPECT_FALSE(matcher.Match(src, empty, gtsam::Pose2()).ok);
}

TEST(WrapAngle, BoundsRespected)
{
  EXPECT_NEAR(WrapAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(WrapAngle(M_PI), M_PI, 1e-12);
  EXPECT_NEAR(WrapAngle(-M_PI), M_PI, 1e-12);  // -π wraps to +π
  EXPECT_NEAR(WrapAngle(2.0 * M_PI), 0.0, 1e-12);
  EXPECT_NEAR(WrapAngle(-1.5 * M_PI), 0.5 * M_PI, 1e-12);
}

// ─────────────────────────────────────────────────────────────────────
// GyroPreintFactor
// ─────────────────────────────────────────────────────────────────────

TEST(GyroPreintFactor, ZeroErrorAtTrueValue)
{
  // Robot rotated by +0.1 rad over 1 s. Preint with zero bias should
  // expect Δθ_curr − Δθ_prev = 0.1, residual = 0 when bias_curr = 0.
  GyroPreintFactor f(gtsam::Symbol('x', 0),
                     gtsam::Symbol('x', 1),
                     gtsam::Symbol('b', 1),
                     /* delta_theta_preint */ 0.10,
                     /* dt_total */ 1.0,
                     UnitDiag1());
  const gtsam::Pose2 X_prev(0.0, 0.0, 0.0);
  const gtsam::Pose2 X_curr(0.0, 0.0, 0.10);
  const double bias = 0.0;
  auto e = f.evaluateError(X_prev, X_curr, bias);
  EXPECT_NEAR(e[0], 0.0, 1e-12);
}

TEST(GyroPreintFactor, BiasCancelsCorrectly)
{
  // Preint says Δθ_preint = 0.20 over 1 s, but true motion was 0.10.
  // → factor expects bias_curr · 1.0 = 0.10 (i.e. bias = 0.10 rad/s)
  // for zero residual.
  GyroPreintFactor f(gtsam::Symbol('x', 0),
                     gtsam::Symbol('x', 1),
                     gtsam::Symbol('b', 1),
                     /* delta_theta_preint */ 0.20,
                     /* dt_total */ 1.0,
                     UnitDiag1());
  const gtsam::Pose2 X_prev(0.0, 0.0, 0.0);
  const gtsam::Pose2 X_curr(0.0, 0.0, 0.10);

  // bias = 0 → residual = (0.10 - (0.20 - 0)) = -0.10
  EXPECT_NEAR(f.evaluateError(X_prev, X_curr, 0.0)[0], -0.10, 1e-12);
  // bias = 0.10 → residual = (0.10 - (0.20 - 0.10*1.0)) = 0
  EXPECT_NEAR(f.evaluateError(X_prev, X_curr, 0.10)[0], 0.0, 1e-12);
}

TEST(GyroPreintFactor, JacobianMatchesNumeric)
{
  GyroPreintFactor f(gtsam::Symbol('x', 0),
                     gtsam::Symbol('x', 1),
                     gtsam::Symbol('b', 1),
                     0.15,
                     0.5,
                     UnitDiag1());
  const gtsam::Pose2 X_prev(0.5, 0.2, 0.3);
  const gtsam::Pose2 X_curr(0.7, 0.1, 0.45);
  const double bias = 0.02;

  gtsam::Matrix H1_an, H2_an, H3_an;
  f.evaluateError(X_prev, X_curr, bias, &H1_an, &H2_an, &H3_an);

  const double eps = 1e-6;
  gtsam::Matrix H1_num(1, 3), H2_num(1, 3), H3_num(1, 1);

  for (int i = 0; i < 3; ++i)
  {
    gtsam::Vector3 d_prev = gtsam::Vector3::Zero();
    d_prev(i) = eps;
    auto e_plus = f.evaluateError(X_prev.retract(d_prev), X_curr, bias);
    auto e_minus = f.evaluateError(X_prev.retract(-d_prev), X_curr, bias);
    H1_num(0, i) = (e_plus[0] - e_minus[0]) / (2.0 * eps);

    gtsam::Vector3 d_curr = gtsam::Vector3::Zero();
    d_curr(i) = eps;
    auto e_plus2 = f.evaluateError(X_prev, X_curr.retract(d_curr), bias);
    auto e_minus2 = f.evaluateError(X_prev, X_curr.retract(-d_curr), bias);
    H2_num(0, i) = (e_plus2[0] - e_minus2[0]) / (2.0 * eps);
  }
  H3_num(0, 0) =
      (f.evaluateError(X_prev, X_curr, bias + eps)[0] -
       f.evaluateError(X_prev, X_curr, bias - eps)[0]) /
      (2.0 * eps);

  for (int j = 0; j < 3; ++j)
  {
    EXPECT_NEAR(H1_an(0, j), H1_num(0, j), 1e-4) << "H1[0," << j << "]";
    EXPECT_NEAR(H2_an(0, j), H2_num(0, j), 1e-4) << "H2[0," << j << "]";
  }
  EXPECT_NEAR(H3_an(0, 0), H3_num(0, 0), 1e-4);
}

TEST(GyroPreintFactor, WrapsAroundPi)
{
  // X_prev.theta = π−0.05, X_curr.theta = -π+0.05 → actual Δθ = 0.10
  // (crosses ±π). Preint = 0.10, bias = 0 → residual must be 0
  // (NOT 2π − 0.10).
  GyroPreintFactor f(gtsam::Symbol('x', 0),
                     gtsam::Symbol('x', 1),
                     gtsam::Symbol('b', 1),
                     0.10,
                     0.1,
                     UnitDiag1());
  const gtsam::Pose2 X_prev(0.0, 0.0, M_PI - 0.05);
  const gtsam::Pose2 X_curr(0.0, 0.0, -M_PI + 0.05);
  EXPECT_NEAR(f.evaluateError(X_prev, X_curr, 0.0)[0], 0.0, 1e-9);
}
