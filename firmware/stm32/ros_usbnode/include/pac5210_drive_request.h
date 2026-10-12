#ifndef PAC5210_DRIVE_REQUEST_H
#define PAC5210_DRIVE_REQUEST_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t direction;
  uint8_t left_speed;
  uint8_t right_speed;
} Pac5210DriveRequest;

static inline Pac5210DriveRequest pac5210_request_from_signed_pwm(
    int16_t left_pwm, int16_t right_pwm) {
  if (left_pwm > 255) left_pwm = 255;
  if (left_pwm < -255) left_pwm = -255;
  if (right_pwm > 255) right_pwm = 255;
  if (right_pwm < -255) right_pwm = -255;
  const uint8_t direction = (uint8_t)((right_pwm > 0 ? 0x30u : 0x20u) |
                                      (left_pwm > 0 ? 0xc0u : 0x80u));
  Pac5210DriveRequest request = {
      direction,
      (uint8_t)(left_pwm < 0 ? -left_pwm : left_pwm),
      (uint8_t)(right_pwm < 0 ? -right_pwm : right_pwm)};
  return request;
}

static inline bool pac5210_should_stop_output(
    bool emergency_active, bool idle, bool link_inhibited,
    bool feedback_healthy, bool host_zero_intent,
    bool authorization_current) {
  return emergency_active || idle || link_inhibited || !feedback_healthy ||
         host_zero_intent || !authorization_current;
}

static inline Pac5210DriveRequest pac5210_apply_final_output_gate(
    Pac5210DriveRequest request, bool stop) {
  if (stop) {
    request.direction = 0xa0u;
    request.left_speed = 0u;
    request.right_speed = 0u;
  }
  return request;
}

static inline void pac5210_encode_drive_packet(
    uint8_t packet[12], const Pac5210DriveRequest request) {
  packet[0] = 0x55u;
  packet[1] = 0xaau;
  packet[2] = 0x08u;
  packet[3] = 0x10u;
  packet[4] = 0x80u;
  packet[5] = request.direction;
  packet[6] = request.left_speed;
  packet[7] = request.right_speed;
  packet[8] = 0u;
  packet[9] = 0u;
  packet[10] = 0u;
  uint8_t checksum = 0u;
  for (uint8_t i = 0u; i < 11u; ++i) checksum = (uint8_t)(checksum + packet[i]);
  packet[11] = checksum;
}

#endif
