#include <cmath>
#include <limits>

#include "mowgli_hardware/cmd_vel_validation.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::is_finite_velocity_command;
using mowgli_hardware::is_float32_representable_velocity_command;

TEST(CmdVelValidation, AcceptsFiniteCommands)
{
  EXPECT_TRUE(is_finite_velocity_command(0.25, -0.8));
  EXPECT_TRUE(is_finite_velocity_command(0.0, 0.0));
  EXPECT_TRUE(is_finite_velocity_command(-0.0, 0.0));
  EXPECT_TRUE(is_finite_velocity_command(0.0, -0.0));
}

TEST(CmdVelValidation, RejectsNonFiniteLinearVelocity)
{
  EXPECT_FALSE(is_finite_velocity_command(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_FALSE(is_finite_velocity_command(std::numeric_limits<double>::infinity(), 0.0));
  EXPECT_FALSE(is_finite_velocity_command(-std::numeric_limits<double>::infinity(), 0.0));
}

TEST(CmdVelValidation, RejectsNonFiniteAngularVelocity)
{
  EXPECT_FALSE(is_finite_velocity_command(0.0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(is_finite_velocity_command(0.0, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(is_finite_velocity_command(0.0, -std::numeric_limits<double>::infinity()));
}

TEST(CmdVelValidation, RejectsBothNonFinite)
{
  EXPECT_FALSE(is_finite_velocity_command(std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::infinity()));
}

TEST(CmdVelValidation, RejectsValuesOutsideFloatWireRangeBeforeNarrowing)
{
  const double max_wire_float = static_cast<double>(std::numeric_limits<float>::max());
  const double above_max_wire_float =
      std::nextafter(max_wire_float, std::numeric_limits<double>::infinity());
  const double below_min_wire_float =
      std::nextafter(-max_wire_float, -std::numeric_limits<double>::infinity());

  EXPECT_FALSE(is_float32_representable_velocity_command(std::numeric_limits<double>::max(), 0.0));
  EXPECT_FALSE(is_float32_representable_velocity_command(-std::numeric_limits<double>::max(), 0.0));
  EXPECT_FALSE(is_float32_representable_velocity_command(above_max_wire_float, 0.0));
  EXPECT_FALSE(is_float32_representable_velocity_command(0.0, below_min_wire_float));
}

TEST(CmdVelValidation, AcceptsMaximumFiniteFloatWireValue)
{
  const double max_wire_float = static_cast<double>(std::numeric_limits<float>::max());
  EXPECT_TRUE(is_float32_representable_velocity_command(max_wire_float, -max_wire_float));
  EXPECT_TRUE(is_float32_representable_velocity_command(0.0, -0.0));
}

TEST(CmdVelValidation, RejectsNonFiniteFloatWireValues)
{
  EXPECT_FALSE(
      is_float32_representable_velocity_command(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_FALSE(
      is_float32_representable_velocity_command(0.0, std::numeric_limits<double>::infinity()));
}
