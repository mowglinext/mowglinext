#ifndef __EMERGENCY_H
#define __EMERGENCY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t Emergency_State(void);
void Emergency_SetState(uint8_t new_emergency_state);
int Emergency_Tilt(void);
int Emergency_StopButtonYellow(void);
int Emergency_StopButtonWhite(void);
int Emergency_WheelLiftBlue(void);
int Emergency_WheelLiftRed(void);
int Emergency_LowZAccelerometer(void);
void EmergencyController(void);
void Emergency_Init(void);
/* Runtime emergency-sensor timeouts (PKT_ID_SET_SAFETY_LIMITS). The four trip
 * timeouts clamp LOWER-ONLY to their compiled board_defaults value (faster
 * e-stop); play_clear clamps HIGHER-ONLY (harder to un-latch). Compile-time
 * values stay the power-on fallback. */
void emergency_set_timeouts(uint32_t one_wheel_lift_ms,
                            uint32_t both_wheels_lift_ms, uint32_t tilt_ms,
                            uint32_t stop_button_ms, uint32_t play_clear_ms);

#ifdef __cplusplus
}
#endif

#endif  /* __EMERGENCY_H */