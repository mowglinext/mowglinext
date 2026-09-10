// Dead-reckoning slip veto vs. encoder quantization (issue #488).
//
// The veto drops wheel-derived translation when the wheels claim a yaw rate
// the gyro does not corroborate. Its threshold must sit ABOVE the resolution
// of the wheel yaw rate, which /wheel_odom derives from a tick DIFFERENCE
// over one aggregation window. Below that floor the veto cannot tell one tick
// of encoder rounding from a skating pivot, and fires during ordinary
// straight driving — which is what made odom→base_footprint under-report
// travel by ~25-30 % and Nav2's BackUp drive 2.1 m for a 1.50 m command.

#include "fusion_graph/dr_slip_veto.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{
// This robot, from mowgli_robot.yaml + odometry_publisher.cpp.
constexpr double kTicksPerMeter = 280.4408894780635;
constexpr double kWheelTrack = 0.325;
// /wheel_odom publishes once the accumulated firmware dt reaches kAggregateMs
// = 50; the observed field windows are 65-67 ms. The SHORTEST window gives the
// LARGEST quantum, so it is the one the threshold must clear.
constexpr double kShortestWindowS = 0.050;
constexpr double kTypicalWindowS = 0.066;

// Shipped defaults — read from the header, never re-typed here, so this test
// tracks the real value if someone edits it.
constexpr double kWheelMin = fg::kDrSlipWheelMinDefaultRadPerS;
constexpr double kGyroMax = fg::kDrSlipGyroMaxDefaultRadPerS;

double Quantum(double window_s)
{
  return fg::WheelYawRateQuantumRadPerS(kTicksPerMeter, kWheelTrack, window_s);
}
}  // namespace

TEST(DrSlipVeto, QuantumMatchesHandDerivation)
{
  // (1 / 280.44) / 0.325 / dt
  EXPECT_NEAR(Quantum(kTypicalWindowS), 0.1662, 1e-3);
  EXPECT_NEAR(Quantum(kShortestWindowS), 0.2194, 1e-3);
}

TEST(DrSlipVeto, QuantumIsZeroForDegenerateGeometry)
{
  EXPECT_EQ(fg::WheelYawRateQuantumRadPerS(0.0, kWheelTrack, kTypicalWindowS), 0.0);
  EXPECT_EQ(fg::WheelYawRateQuantumRadPerS(kTicksPerMeter, 0.0, kTypicalWindowS), 0.0);
  EXPECT_EQ(fg::WheelYawRateQuantumRadPerS(kTicksPerMeter, kWheelTrack, 0.0), 0.0);
}

// The regression guard. A future edit that lowers the threshold back under the
// quantization floor re-opens issue #488, so pin the invariant, not the value.
TEST(DrSlipVeto, ThresholdClearsTwoTicksOfEncoderAsymmetry)
{
  EXPECT_GT(kWheelMin, 2.0 * Quantum(kShortestWindowS));
}

// Slow STRAIGHT reverse, gyro ~0, one tick of left/right asymmetry: this is
// encoder rounding, not slip. Must NOT be vetoed at any observed window length.
TEST(DrSlipVeto, OneTickAsymmetryDuringStraightDriveIsNotSlip)
{
  for (double window : {kShortestWindowS, kTypicalWindowS})
  {
    const double wz = Quantum(window);
    EXPECT_FALSE(fg::DrSlipVetoed(wz, 0.0, kWheelMin, kGyroMax)) << "window " << window;
    EXPECT_FALSE(fg::DrSlipVetoed(-wz, 0.0, kWheelMin, kGyroMax)) << "window " << window;
  }
}

TEST(DrSlipVeto, TwoTickAsymmetryDuringStraightDriveIsNotSlip)
{
  const double wz = 2.0 * Quantum(kTypicalWindowS);  // 0.332 rad/s, observed in the field logs
  EXPECT_FALSE(fg::DrSlipVetoed(wz, 0.0, kWheelMin, kGyroMax));
}

// The failure this veto exists for: wheels claim a pivot, gyro says still.
TEST(DrSlipVeto, SkatingPivotIsVetoed)
{
  for (double wheel_wz : {1.0, 2.0, 3.0})
  {
    EXPECT_TRUE(fg::DrSlipVetoed(wheel_wz, 0.0, kWheelMin, kGyroMax)) << wheel_wz;
    EXPECT_TRUE(fg::DrSlipVetoed(-wheel_wz, 0.0, kWheelMin, kGyroMax)) << wheel_wz;
  }
}

// A real coordinated turn: both sensors agree. Never vetoed, however fast.
TEST(DrSlipVeto, CoordinatedTurnIsNotVetoed)
{
  EXPECT_FALSE(fg::DrSlipVetoed(1.0, 1.0, kWheelMin, kGyroMax));
  EXPECT_FALSE(fg::DrSlipVetoed(2.0, 1.95, kWheelMin, kGyroMax));
}

// Gyro moving disqualifies the veto even when the wheels disagree — the gyro
// gate exists so a turn where the wheels merely mis-scale is not called slip.
TEST(DrSlipVeto, MovingGyroDisqualifiesVeto)
{
  EXPECT_FALSE(fg::DrSlipVetoed(2.0, 0.5, kWheelMin, kGyroMax));
}

// Sanity: the veto still catches a slow skate, just above the new floor.
TEST(DrSlipVeto, SlowSkateJustAboveThresholdIsVetoed)
{
  EXPECT_TRUE(fg::DrSlipVetoed(kWheelMin + 0.01, 0.0, kWheelMin, kGyroMax));
  EXPECT_FALSE(fg::DrSlipVetoed(kWheelMin - 0.01, 0.0, kWheelMin, kGyroMax));
}
