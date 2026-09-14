#pragma once

#include <cmath>
#include <limits>

namespace mowgli_hardware
{

/// A velocity command is usable only when both values are IEEE-754 finite.
/// In particular, comparisons against limits do not reject NaN.
inline bool is_finite_velocity_command(double linear_x, double angular_z)
{
  return std::isfinite(linear_x) && std::isfinite(angular_z);
}

/// A velocity command can be narrowed to the float32 wire format only when
/// both finite double values are within float32's finite range.  This check
/// must happen before the floating-point conversion.
inline bool is_float32_representable_velocity_command(double linear_x, double angular_z)
{
  constexpr double kMaxFiniteFloat = static_cast<double>(std::numeric_limits<float>::max());
  return is_finite_velocity_command(linear_x, angular_z) && linear_x >= -kMaxFiniteFloat &&
         linear_x <= kMaxFiniteFloat && angular_z >= -kMaxFiniteFloat &&
         angular_z <= kMaxFiniteFloat;
}

}  // namespace mowgli_hardware
