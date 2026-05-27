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
  gp.node_period_s = 0.1;
  gp.wheel_sigma_x = 0.05;
  gp.wheel_sigma_y = 0.005;
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
  // Slip-veto thresholds: must hold for both scenarios. With node_period
  // 0.1 s, kPhantomWz=0.30 rad/s, gyro residual=0.02 rad/s:
  //   wheel_dtheta = 0.030 / tick, gyro_dtheta = 0.002 / tick
  //   residual = 0.028 / tick → comfortably above 0.01 threshold.
  //   gyro_dtheta = 0.002 < 0.005 slip_gyro_max.
  //   wheel_dtheta = 0.030 > 0.005 slip_wheel_min.
  gp.slip_residual_thresh_rad = 0.01;
  gp.slip_gyro_max_rad = 0.005;
  gp.slip_wheel_min_rad = 0.005;
  return gp;
}

}  // namespace

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

  constexpr int kTicks = 60;       // 6 s @ 10 Hz
  constexpr double kDt = 0.1;
  constexpr double kPhantomVx = 0.10;       // m/s — slipping wheels
  constexpr double kPhantomWz = 0.30;       // rad/s — diff-drive guess
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
