#pragma once

#include <cmath>
#include <cstdint>

namespace mowgli_motor_safety {

struct LinkRearmState {
  bool required = true;
  std::uint32_t zero_phase_baseline = 0u;
  std::uint32_t drive_fault_sequence = 0u;
  std::uint32_t blade_fault_sequence = 0u;
  // One-cycle event: the zero/off phase has just authorized inhibit clearing.
  bool rearm_completed = false;
};

// A fault is sticky across a transient link recovery. Only a new, validated
// zero phase received while both links are healthy can clear the inhibit.
inline bool update_link_rearm(LinkRearmState &state, bool links_healthy,
                              std::uint32_t zero_phase_sequence,
                              bool host_zero_intent, bool blade_off,
                              std::uint32_t drive_fault_sequence,
                              std::uint32_t blade_fault_sequence,
                              bool link_inhibited) {
  state.rearm_completed = false;
  const bool new_fault =
      drive_fault_sequence != state.drive_fault_sequence ||
      blade_fault_sequence != state.blade_fault_sequence ||
      (!state.required && link_inhibited);
  state.drive_fault_sequence = drive_fault_sequence;
  state.blade_fault_sequence = blade_fault_sequence;

  if (!links_healthy || new_fault) {
    state.required = true;
    state.zero_phase_baseline = zero_phase_sequence;
  } else if (state.required && host_zero_intent && blade_off &&
             zero_phase_sequence != state.zero_phase_baseline) {
    state.required = false;
    state.rearm_completed = true;
  }
  return state.required;
}

struct WheelTargets {
  float left_mps;
  float right_mps;
};

inline bool yaw_loop_active(bool enabled, bool hard_stop,
                            bool host_yaw_inhibited) {
  return enabled && !hard_stop && !host_yaw_inhibited;
}

// Explicit host zero intent is a hard boundary for yaw correction. A hard stop
// also clears targets; otherwise a turn's requested wheel targets are preserved.
inline WheelTargets apply_yaw_trim(float left_mps, float right_mps,
                                   float yaw_trim_mps,
                                   bool host_zero_intent, bool hard_stop) {
  if (hard_stop) return {0.0f, 0.0f};
  if (host_zero_intent) return {left_mps, right_mps};
  return {left_mps - yaw_trim_mps, right_mps + yaw_trim_mps};
}

// Preserve controller braking while exactly stationary zero intent remains
// non-propulsive. This mirrors the final filter immediately before the driver.
inline std::int16_t suppress_stationary_zero_output(float target_mps,
                                                    float measured_mps,
                                                    std::int16_t pwm) {
  return target_mps == 0.0f && std::fabs(measured_mps) < 0.02f ? 0 : pwm;
}

struct Pac5210DriveRequest {
  std::uint8_t direction;
  std::uint8_t left_speed;
  std::uint8_t right_speed;
};

inline Pac5210DriveRequest encode_pac5210_drive_request(
    std::int16_t left_pwm, std::int16_t right_pwm) {
  if (left_pwm > 255) left_pwm = 255;
  if (left_pwm < -255) left_pwm = -255;
  if (right_pwm > 255) right_pwm = 255;
  if (right_pwm < -255) right_pwm = -255;
  const bool left_forward = left_pwm > 0;
  const bool right_forward = right_pwm > 0;
  const std::uint8_t direction = static_cast<std::uint8_t>(
      (right_forward ? 0x30u : 0x20u) |
      (left_forward ? 0xc0u : 0x80u));
  return {direction,
          static_cast<std::uint8_t>(left_pwm < 0 ? -left_pwm : left_pwm),
          static_cast<std::uint8_t>(right_pwm < 0 ? -right_pwm : right_pwm)};
}

} // namespace mowgli_motor_safety
