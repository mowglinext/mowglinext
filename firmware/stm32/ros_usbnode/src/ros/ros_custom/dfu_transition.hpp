#pragma once

#include <cstdint>

#include "dfu_decision.h"

namespace mowgli_dfu
{

constexpr uint32_t kStopWindowMs = DFU_STOP_WINDOW_MS;
constexpr uint32_t kMarkerMagic = DFU_MARKER_MAGIC;  // "DFU1"

enum class Phase : uint8_t
{
  Idle,
  Requested,
  Stopping,
};

inline bool request_is_accepted(
    bool f401_capable, bool is_idle, uint8_t magic, uint8_t expected_magic, Phase phase)
{
  return f401_capable && is_idle && magic == expected_magic && phase == Phase::Idle;
}

inline bool inputs_are_locked(Phase phase)
{
  return phase != Phase::Idle;
}

/* Unsigned subtraction deliberately handles the HAL tick counter wrapping. */
inline bool ready_to_reset(uint32_t start_tick, uint32_t now_tick)
{
  return DFU_ReadyToReset(start_tick, now_tick);
}

inline bool marker_is_valid(uint32_t marker,
                            uint32_t inverse,
                            bool software_reset,
                            bool conflicting_reset_cause = false)
{
  return DFU_MarkerIsValid(marker, inverse,
                           software_reset && !conflicting_reset_cause);
}

}  // namespace mowgli_dfu
