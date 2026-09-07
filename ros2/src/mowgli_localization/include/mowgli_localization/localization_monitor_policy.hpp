// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

namespace mowgli_localization
{

enum class LocalizationMode : std::int32_t
{
  DEAD_RECKONING = 0,
  GPS_ONLY = 1,
  RTK_FLOAT = 2,
  RTK_FIXED = 3,
};

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
