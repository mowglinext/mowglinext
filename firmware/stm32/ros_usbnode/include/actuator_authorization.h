#ifndef ACTUATOR_AUTHORIZATION_H
#define ACTUATOR_AUTHORIZATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Shared authorization state for all actuator outputs. A safety boundary
 * advances epoch; a fresh, accepted zero CMD_VEL is required before a drive
 * request in that epoch can be authorized. */
typedef struct {
  uint32_t epoch;
  uint32_t drive_zero_epoch;
} ActuatorAuthorizationState;

static inline void actuator_authorization_invalidate(
    ActuatorAuthorizationState *state) {
  ++state->epoch;
  if (state->epoch == 0u) ++state->epoch;
}

static inline bool actuator_authorization_accept_drive(
    ActuatorAuthorizationState *state, bool zero_motion,
    bool safety_boundary_active) {
  if (safety_boundary_active) return false;
  if (zero_motion) {
    state->drive_zero_epoch = state->epoch;
    return true;
  }
  return state->drive_zero_epoch == state->epoch;
}

static inline bool actuator_authorization_drive_request_is_current(
    const ActuatorAuthorizationState *state, uint32_t request_epoch) {
  return request_epoch == state->epoch &&
         state->drive_zero_epoch == state->epoch;
}

uint32_t ActuatorAuthorization_Epoch(void);
void ActuatorAuthorization_Invalidate(void);
bool ActuatorAuthorization_AcceptDriveCommand(bool zero_motion,
                                              bool safety_boundary_active,
                                              uint32_t *accepted_epoch);
bool ActuatorAuthorization_DriveRequestIsCurrent(uint32_t request_epoch);

#ifdef __cplusplus
}
#endif

#endif
