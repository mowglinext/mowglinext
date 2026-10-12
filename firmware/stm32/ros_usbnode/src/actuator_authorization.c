#include "actuator_authorization.h"

#include <stddef.h>

#include "stm32f_board_hal.h"

/* Start unarmed: a fresh zero-motion command is also the boot re-arm phase. */
static ActuatorAuthorizationState actuator_authorization_state = {1u, 0u};

static uint32_t actuator_authorization_lock(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void actuator_authorization_unlock(uint32_t primask) {
  __set_PRIMASK(primask);
}

uint32_t ActuatorAuthorization_Epoch(void) {
  const uint32_t primask = actuator_authorization_lock();
  const uint32_t epoch = actuator_authorization_state.epoch;
  actuator_authorization_unlock(primask);
  return epoch;
}

void ActuatorAuthorization_Invalidate(void) {
  const uint32_t primask = actuator_authorization_lock();
  actuator_authorization_invalidate(&actuator_authorization_state);
  actuator_authorization_unlock(primask);
}

bool ActuatorAuthorization_AcceptDriveCommand(
    bool zero_motion, bool safety_boundary_active,
    uint32_t *accepted_epoch) {
  const uint32_t primask = actuator_authorization_lock();
  const bool accepted = actuator_authorization_accept_drive(
      &actuator_authorization_state, zero_motion, safety_boundary_active);
  if (accepted && accepted_epoch != NULL) {
    *accepted_epoch = actuator_authorization_state.epoch;
  }
  actuator_authorization_unlock(primask);
  return accepted;
}

bool ActuatorAuthorization_DriveRequestIsCurrent(uint32_t request_epoch) {
  const uint32_t primask = actuator_authorization_lock();
  const bool current = actuator_authorization_drive_request_is_current(
      &actuator_authorization_state, request_epoch);
  actuator_authorization_unlock(primask);
  return current;
}
