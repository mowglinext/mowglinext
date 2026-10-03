// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "mowgli_interfaces/msg/absolute_pose.hpp"

namespace mowgli_localization
{

enum class LocalizationMode : std::int32_t
{
  DEAD_RECKONING = 0,
  GPS_ONLY = 1,
  RTK_FLOAT = 2,
  RTK_FIXED = 3,
};

inline bool RtkFixedFromAbsolutePoseFlags(const std::uint16_t flags)
{
  using AbsolutePose = mowgli_interfaces::msg::AbsolutePose;
  return (flags & AbsolutePose::FLAG_GPS_RTK_FIXED) != 0u;
}

inline bool RtkActiveFromAbsolutePoseFlags(const std::uint16_t flags)
{
  using AbsolutePose = mowgli_interfaces::msg::AbsolutePose;

  // FLAG_GPS_RTK is the historical "GPS fix present" bit. It does NOT mean
  // that the solution is RTK. Only the explicit FLOAT/FIXED bits authorize
  // an RTK localization mode.
  return (flags & (AbsolutePose::FLAG_GPS_RTK_FLOAT |
                   AbsolutePose::FLAG_GPS_RTK_FIXED)) != 0u;
}

inline LocalizationMode EvaluateLocalizationMode(const bool observation_fresh,
                                                 const bool rtk_active,
                                                 const bool rtk_fixed)
{
  if (!observation_fresh)
  {
    return LocalizationMode::DEAD_RECKONING;
  }
  if (rtk_fixed)
  {
    return LocalizationMode::RTK_FIXED;
  }
  if (rtk_active)
  {
    return LocalizationMode::RTK_FLOAT;
  }
  return LocalizationMode::GPS_ONLY;
}

}  // namespace mowgli_localization
