// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

namespace mowgli_hardware
{

// The STM32's existing no-current-GNSS indication is quality zero: firmware
// turns PANEL_LED_LOCK off below 90. Preserve every fresh quality mapping and
// only substitute that established safe value when physical provenance ages
// out.
inline std::uint8_t GnssQualityForFirmware(const bool observation_fresh,
                                           const std::uint8_t quality_percent)
{
  return observation_fresh ? quality_percent : 0U;
}

}  // namespace mowgli_hardware
