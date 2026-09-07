#pragma once

#include <cmath>
#include <cstdint>

namespace mowgli_cmd_vel
{

struct SafetyState
{
  float cmd_wz;
  float left_target_mps;
  float right_target_mps;
  uint32_t last_valid_tick;
};

/// Accept only finite wire values; invalid input clears targets but preserves
/// the last-valid timestamp so it cannot refresh the motion watchdog.
inline bool apply_safety(float vx, float wz, uint32_t tick, SafetyState &state)
{
  if (!std::isfinite(vx) || !std::isfinite(wz)) {
    state.cmd_wz = 0.0f;
    state.left_target_mps = 0.0f;
    state.right_target_mps = 0.0f;
    return false;
  }
  state.cmd_wz = wz;
  state.last_valid_tick = tick;
  return true;
}

}  // namespace mowgli_cmd_vel
