// Wheel-vs-gyro disagreement during an attempted pivot.
//
// Scenario: the robot is commanded to rotate. The grass is wet enough
// that both drive wheels spin in the same direction (slip), producing
// encoder readings that say "the robot moved forward AND rotated a
// little" — but the chassis sits still. The IMU gyro, which measures
// chassis-frame angular velocity directly, shows zero rotation.
//
// What the localizer must do: the wheels are not ground truth here.
// Trust the gyro for yaw (we already did this — gyro between-factor
// dominates when |dtheta_gyro| > 0) AND drop the wheel-derived
// translation, since wheels reporting motion while the gyro confirms
// the chassis is stationary means the wheels are skating.
//
// Failure mode this catches: previously the wheel's per-tick
// `accum_.dx` flowed straight into the between-factor regardless of
// what the gyro said. A 0.1 m/s phantom vx during a 6 s pivot pushed
// the map-frame pose 0.3-0.6 m sideways — the controller then chased
// the spurious drift with real cmd_vel corrections, which produced
// real slip, which the wheels reported, which moved the pose, etc.

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

fg::GraphParams MakeParams()
{
  fg::GraphParams gp;
  // Run at the 25 Hz tuning reference so the per-tick gate scaling factor is
  // exactly 1.0 and the slip thresholds below apply at face value (the twist is
  // still fed at kDt=0.1 s per tick — node_period only gates the MINIMUM node
  // spacing, so one node is created per Tick as before).
  gp.node_period_s = fg::kTunedNodePeriodS;  // 0.04
  gp.wheel_sigma_x_per_sqrt_m = 0.05;
  gp.wheel_sigma_y_per_sqrt_m = 0.005;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_thresh_xy_m = 1.0e-3;
  gp.stationary_thresh_theta = 2.0e-3;
  gp.stationary_sigma_theta = 1.0e-3;
  // Multi-source gate threshold — gyro stays well under this when the
  // chassis is genuinely still.
  gp.stationary_gyro_thresh_rad_per_s = 0.10;
  // Disable the throttle so every Tick gives a node.
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  // Disable adaptive_noise so the test isolates the slip-detection path
  // rather than the EMA's σ_x inflation, which has its own dedicated
  // test in test_adaptive_noise.
  gp.adaptive_noise_enabled_gain = 0.0;
  // Slip-veto thresholds: must hold for both scenarios. Twist fed at kDt=0.1 s,
  // kPhantomWz=0.30 rad/s, gyro residual=0.02 rad/s; scaling factor 1.0 at the
  // 25 Hz reference so thresholds apply at face value:
  //   wheel_dtheta = 0.030 / tick, gyro_dtheta = 0.002 / tick
  //   residual = 0.028 / tick → comfortably above 0.01 threshold.
  //   gyro_dtheta = 0.002 < 0.005 slip_gyro_max.
  //   wheel_dtheta = 0.030 > 0.005 slip_wheel_min.
  gp.slip_residual_thresh_rad = 0.01;
  gp.slip_gyro_max_rad = 0.005;
  gp.slip_wheel_min_rad = 0.005;
  // Window of ONE node: the three scenarios above were written against the
  // per-node gate and stay its regression test — slip_window_s = 0 must
  // reproduce it exactly (issue #516). The windowed default is exercised by
  // the Windowed* cases below.
  gp.slip_window_s = 0.0;
  return gp;
}

// One encoder tick of L/R asymmetry, as wheel dθ per node
// (1 / 280.44 / 0.325 — the 2026-08-24 quantization note).
constexpr double kOneTickRad = 0.01097;
// Twist feed period. Above node_period_s (0.04) so every Tick creates a node
// (the other tests do the same); the wheel yaw rate is chosen so each node's
// wheel dθ is exactly one tick, whatever the feed period.
constexpr double kJitterDt = 0.1;
constexpr double kJitterVx = 0.20;  // m/s — mowing speed

// Straight swath with the gyro still and the wheels reporting alternating
// ±1-tick asymmetry: the #516 field signature. Returns the slip_veto count.
uint64_t DriveStraightWithTickJitter(fg::GraphManager& gm, int ticks)
{
  const double one_tick_wz = kOneTickRad / kJitterDt;
  for (int i = 0; i < ticks; ++i)
  {
    const double wz = (i % 2 == 0) ? one_tick_wz : -one_tick_wz;
    gm.AddWheelTwist(kJitterVx, 0.0, wz, kJitterDt);
    gm.AddGyroDelta(0.0, kJitterDt);
    gm.Tick(kJitterDt * (i + 1));
  }
  return gm.Stats().slip_veto;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// Issue #516: with the per-node gate (window of 1) EVERY jitter node is
// vetoed — one tick is 0.011 rad > slip_wheel_min AND > slip_residual —
// and the straight swath's wheel translation is thrown away. This pins the
// old behaviour so slip_window_s = 0 stays an exact reproduction of it.
// ─────────────────────────────────────────────────────────────────────
TEST(SlipPivot, PerNodeGateVetoesEveryTickJitterNode)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  constexpr int kTicks = 100;
  const uint64_t vetoed = DriveStraightWithTickJitter(gm, kTicks);
  EXPECT_EQ(vetoed, static_cast<uint64_t>(kTicks));
}

// ─────────────────────────────────────────────────────────────────────
// With the 0.5 s window (12 nodes) the same jitter sums to ~0 and never
// vetoes, so dead reckoning keeps the full 0.20 m/s × 10 s = 2.0 m.
// ─────────────────────────────────────────────────────────────────────
TEST(SlipPivot, WindowedGateIgnoresTickJitterOnStraights)
{
  auto gp = MakeParams();
  gp.slip_window_s = 0.5;
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  constexpr int kTicks = 100;
  const uint64_t vetoed = DriveStraightWithTickJitter(gm, kTicks);
  EXPECT_EQ(vetoed, 0u);

  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const double travelled = std::hypot(snap->pose.x(), snap->pose.y());
  const double expected_m = kJitterVx * kJitterDt * kTicks;
  std::printf("[SlipPivot] windowed gate, straight with tick jitter: travelled=%.3f m\n",
              travelled);
  EXPECT_NEAR(travelled, expected_m, 0.05 * expected_m);
}

// ─────────────────────────────────────────────────────────────────────
// The windowed gate still catches the 2026-05-27 sustained slip, but only
// once the window has warmed up: the first (window − 1) nodes leak their
// wheel translation by design (a cold start never vetoes), every node
// after that is vetoed, and the counter reflects exactly that.
// ─────────────────────────────────────────────────────────────────────
TEST(SlipPivot, WindowedGateWarmsUpThenHoldsSustainedSlip)
{
  auto gp = MakeParams();
  gp.slip_window_s = 0.5;  // 12 nodes at the 0.04 s reference
  fg::GraphManager gm(gp);
  const gtsam::Pose2 X0(5.0, 3.0, 0.7);
  gm.Initialize(X0, 0.0);

  constexpr int kTicks = 60;
  constexpr double kDt = 0.1;
  constexpr double kPhantomVx = 0.10;
  constexpr double kPhantomWz = 0.30;
  constexpr double kGyroResidual = 0.02;
  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(kPhantomVx, 0.0, kPhantomWz, kDt);
    gm.AddGyroDelta(kGyroResidual, kDt);
    gm.Tick(kDt * (i + 1));
  }

  const uint64_t window_nodes = fg::SlipWindowNodes(gp.slip_window_s, gp.node_period_s);
  ASSERT_EQ(window_nodes, 12u);
  EXPECT_EQ(gm.Stats().slip_veto, static_cast<uint64_t>(kTicks) - (window_nodes - 1));

  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const double drift_m = std::hypot(snap->pose.x() - X0.x(), snap->pose.y() - X0.y());
  const double warmup_leak_m = static_cast<double>(window_nodes - 1) * kPhantomVx * kDt;
  std::printf("[SlipPivot] windowed veto: drift=%.3f m (warm-up leak %.3f m)\n",
              drift_m,
              warmup_leak_m);
  // Bounded by the warm-up leak — NOT the 0.6 m an un-vetoed integrator
  // would accumulate.
  EXPECT_LT(drift_m, warmup_leak_m + 0.05);
}

// ─────────────────────────────────────────────────────────────────────
// Slipping pivot attempt: wheels report continuous forward + rotation,
// gyro says the chassis didn't actually move or rotate. The localizer
// must hold position — moving more than ~5 cm here means the wheel
// integration silently drove the pose despite the gyro vetoing it.
// ─────────────────────────────────────────────────────────────────────
TEST(SlipPivot, WheelOnlyMotionIsRejectedByGyroVeto)
{
  fg::GraphManager gm(MakeParams());
  const gtsam::Pose2 X0(5.0, 3.0, 0.7);
  gm.Initialize(X0, 0.0);

  constexpr int kTicks = 60;  // 6 s @ 10 Hz
  constexpr double kDt = 0.1;
  constexpr double kPhantomVx = 0.10;  // m/s — slipping wheels
  constexpr double kPhantomWz = 0.30;  // rad/s — diff-drive guess
  // Gyro residual within stationary_gyro_thresh_rad_per_s — chassis
  // is really still.
  constexpr double kGyroResidual = 0.02;

  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(kPhantomVx, 0.0, kPhantomWz, kDt);
    gm.AddGyroDelta(kGyroResidual, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());

  const double dx = snap->pose.x() - X0.x();
  const double dy = snap->pose.y() - X0.y();
  const double drift_m = std::hypot(dx, dy);
  std::printf("[SlipPivot] 6 s wheel-only slip, drift=%.3f m\n", drift_m);

  // Without slip detection the wheel integrator pulls the pose by
  // ~kPhantomVx * kTicks * kDt = 0.6 m. With the gyro veto in place
  // the between-factor's sigma_x is loose enough that the GPS-less
  // graph relaxes to near zero drift; allow 5 cm of residual.
  EXPECT_LT(drift_m, 0.05);
}

// ─────────────────────────────────────────────────────────────────────
// Sanity check the opposite: when the wheels and the gyro AGREE that
// the chassis rotated in place (forward-velocity components from the
// two drive wheels cancel out, so vx ≈ 0), the localizer must still
// track the rotation without translation.
// ─────────────────────────────────────────────────────────────────────
TEST(SlipPivot, TruePivotStillTracksWithoutPhantomTranslation)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr int kTicks = 60;
  constexpr double kDt = 0.1;
  constexpr double kWz = 0.5;

  for (int i = 0; i < kTicks; ++i)
  {
    // Pure in-place rotation: wheels report wz only.
    gm.AddWheelTwist(0.0, 0.0, kWz, kDt);
    gm.AddGyroDelta(kWz, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const double drift_m = std::hypot(snap->pose.x(), snap->pose.y());
  const double yaw = snap->pose.theta();
  std::printf("[SlipPivot] 6 s pivot @ %.2f rad/s: drift=%.3f m, yaw=%.3f rad\n",
              kWz,
              drift_m,
              yaw);

  EXPECT_LT(drift_m, 0.05);
  EXPECT_NEAR(yaw, kWz * kDt * kTicks, 0.1 * kWz * kDt * kTicks);
}

// ─────────────────────────────────────────────────────────────────────
// Cadence invariance of the per-tick slip gate. The thresholds are tuned at
// 25 Hz but the robot can run at 50 Hz. This slip's RATE (0.30 rad/s residual)
// is above the tuned trip point (0.01 rad / 0.04 s = 0.25 rad/s), but at 50 Hz
// its PER-TICK residual (0.006 rad) is BELOW the raw 0.01 rad threshold — so it
// is vetoed ONLY because CreateNodeLocked scales the threshold down with the
// cadence (0.01 * 0.02/0.04 = 0.005 rad). Without the scaling the phantom vx
// would leak straight through and drive the pose ~0.20 m.
// ─────────────────────────────────────────────────────────────────────
TEST(SlipPivot, SlipVetoScalesWithCadence)
{
  auto gp = MakeParams();
  gp.node_period_s = 0.02;  // 50 Hz — half the tuned reference (scale 0.5)
  fg::GraphManager gm(gp);
  const gtsam::Pose2 X0(5.0, 3.0, 0.7);
  gm.Initialize(X0, 0.0);

  constexpr int kTicks = 100;  // 2 s @ 50 Hz
  constexpr double kDt = 0.02;
  constexpr double kPhantomVx = 0.10;  // m/s — slipping wheels
  constexpr double kPhantomWz = 0.32;  // rad/s — diff-drive guess
  constexpr double kGyroResidual = 0.02;  // rad/s — chassis really still
  // Per tick: wheel_dtheta=0.0064, gyro_dtheta=0.0004, residual=0.0060 rad.
  //   scaled slip_residual = 0.01  * 0.5 = 0.0050 → 0.0060 > 0.0050 → detected.
  //   scaled slip_gyro_max = 0.005 * 0.5 = 0.0025 → 0.0004 < 0.0025 ✓.
  //   scaled slip_wheel_min = 0.005 * 0.5 = 0.0025 → 0.0064 > 0.0025 ✓.
  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(kPhantomVx, 0.0, kPhantomWz, kDt);
    gm.AddGyroDelta(kGyroResidual, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const double drift_m = std::hypot(snap->pose.x() - X0.x(), snap->pose.y() - X0.y());
  std::printf("[SlipPivot] 50 Hz cadence-scaled slip veto, drift=%.3f m\n", drift_m);
  EXPECT_LT(drift_m, 0.05);
}
