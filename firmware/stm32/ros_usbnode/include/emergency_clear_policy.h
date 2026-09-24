#ifndef EMERGENCY_CLEAR_POLICY_H
#define EMERGENCY_CLEAR_POLICY_H

#include <stdint.h>

typedef struct {
  uint8_t stop_yellow;
  uint8_t stop_white;
  uint8_t wheel_lift_blue;
  uint8_t wheel_lift_red;
  uint8_t mechanical_tilt;
  uint8_t accelerometer_tilt;
} EmergencyPhysicalInputs;

/* A reset is permitted only after every physical safety input has cleared. */
static inline uint8_t emergency_physical_inputs_clear(
    EmergencyPhysicalInputs inputs) {
  return !inputs.stop_yellow && !inputs.stop_white &&
         !inputs.wheel_lift_blue && !inputs.wheel_lift_red &&
         !inputs.mechanical_tilt && !inputs.accelerometer_tilt;
}

/* Returns non-zero only after a complete PLAY hold that began and elapsed
 * while the emergency is latched and every physical input is clear. */
static inline uint8_t emergency_play_clear_hold_step(
    uint32_t now, uint32_t hold_ms, uint8_t play_held,
    uint8_t emergency_latched, EmergencyPhysicalInputs inputs,
    uint32_t *hold_started) {
  if (!play_held || !emergency_latched ||
      !emergency_physical_inputs_clear(inputs)) {
    *hold_started = 0;
    return 0;
  }

  if (*hold_started == 0) {
    *hold_started = now;
    return 0;
  }

  return (uint32_t)(now - *hold_started) >= hold_ms;
}

#endif  /* EMERGENCY_CLEAR_POLICY_H */
