#include <limits>
#include <cstdint>

#include <unity.h>

#include "cmd_vel_safety.hpp"
#include "imu/imu_mount_transform.h"
#include "blade_emergency_policy.hpp"
#include "heartbeat_emergency_policy.hpp"
#include "motor_output_safety.hpp"
#include "pac5210_drive_request.h"
#include "actuator_authorization.h"

using mowgli_cmd_vel::SafetyState;
using mowgli_cmd_vel::apply_safety;

static void test_blade_authorization_is_discarded_at_emergency_boundary()
{
  auto decision = decide_blade_intent(1u, 7u, false, 0u, 7u, false, true,
                                      false, 8u);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.retained_request);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.effective_output);
  decision = decide_blade_intent(decision.retained_request,
                                 decision.request_generation, false, 0u, 8u,
                                 false, false, false, 8u);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.retained_request);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.effective_output);
}

static void test_blade_intent_is_cleared_after_cmd_vel_timeout()
{
  auto decision = decide_blade_intent(0u, 7u, true, 1u, 7u, false, false,
                                      false, 7u);
  TEST_ASSERT_EQUAL_UINT8(1u, decision.effective_output);
  decision = stop_blade_intent(7u);
  decision = decide_blade_intent(decision.retained_request,
                                 decision.request_generation, false, 0u, 7u,
                                 false, false, false, 7u);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.retained_request);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.effective_output);
}

static void test_blade_on_requires_fresh_cmd_vel_and_heartbeat()
{
  TEST_ASSERT_FALSE(blade_on_command_is_fresh(100u, 100u, false, 200u,
                                               100u, true, 2000u));
  TEST_ASSERT_FALSE(blade_on_command_is_fresh(401u, 100u, true, 200u,
                                               401u, true, 2000u));
  TEST_ASSERT_FALSE(blade_on_command_is_fresh(2201u, 2201u, true, 200u,
                                               100u, true, 2000u));
  TEST_ASSERT_TRUE(blade_on_command_is_fresh(500u, 450u, true, 200u,
                                              400u, true, 2000u));

  const auto rejected = decide_blade_intent(
      0u, 9u, true, 0u, 9u, false, false, false, 9u);
  TEST_ASSERT_EQUAL_UINT8(0u, rejected.retained_request);
  TEST_ASSERT_EQUAL_UINT8(0u, rejected.effective_output);
}

static void test_heartbeat_watchdog_starts_without_received_packet()
{
  TEST_ASSERT_TRUE(heartbeat_timed_out(30001u, 0u, 30000u));
  TEST_ASSERT_FALSE(heartbeat_timed_out(30000u, 0u, 30000u));
  TEST_ASSERT_FALSE(heartbeat_timed_out(30001u, 30000u, 30000u));
}

static void test_blade_on_received_during_latch_is_not_deferred()
{
  auto decision = decide_blade_intent(0u, 12u, true, 1u, 12u, false, true,
                                      false, 12u);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.retained_request);
  decision = decide_blade_intent(decision.retained_request,
                                 decision.request_generation, false, 0u, 12u,
                                 false, false, false, 12u);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.effective_output);
}

static void test_blade_on_during_link_rearm_is_not_deferred()
{
  const auto decision = decide_blade_intent(0u, 4u, true, 1u, 4u, false,
                                            false, true, 4u);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.retained_request);
  TEST_ASSERT_EQUAL_UINT8(0u, decision.effective_output);
}

static void test_motor_link_requires_fresh_zero_after_recovery()
{
  mowgli_motor_safety::LinkRearmState state{};
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, false, 3u, true, true, 0u, 0u, true));
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, true, 3u, true, true, 0u, 0u, true));
  TEST_ASSERT_FALSE(mowgli_motor_safety::update_link_rearm(
      state, true, 4u, true, true, 0u, 0u, true));
}

static void test_idle_zero_can_complete_safe_motor_link_rearm()
{
  mowgli_motor_safety::LinkRearmState state{};
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, false, 0u, true, true, 0u, 0u, true));
  /* A verified zero/off phase may arrive while IDLE. It can clear the inhibit
   * because IDLE independently hard-gates both physical actuator outputs. */
  TEST_ASSERT_FALSE(mowgli_motor_safety::update_link_rearm(
      state, true, 1u, true, true, 0u, 0u, true));
}

static void test_motor_link_fault_requires_new_zero_and_blade_off()
{
  mowgli_motor_safety::LinkRearmState state{false, 5u, 2u, 9u};
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, true, 5u, false, true, 3u, 9u, false));
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, true, 6u, true, false, 3u, 9u, true));
  TEST_ASSERT_FALSE(mowgli_motor_safety::update_link_rearm(
      state, true, 7u, true, true, 3u, 9u, true));
}

static void test_motor_link_zero_rearm_does_not_require_zero_during_motion()
{
  mowgli_motor_safety::LinkRearmState state{};
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, false, 4u, true, true, 0u, 0u, true));
  TEST_ASSERT_FALSE(state.rearm_completed);
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, true, 4u, true, true, 0u, 0u, true));
  TEST_ASSERT_FALSE(state.rearm_completed);
  TEST_ASSERT_FALSE(mowgli_motor_safety::update_link_rearm(
      state, true, 5u, true, true, 0u, 0u, true));
  TEST_ASSERT_TRUE(state.rearm_completed);

  // After the zero/off revalidation clears the physical inhibit, ordinary
  // motion (including blade ON) must not repeat that zero/off-only check.
  for (int cycle = 0; cycle < 10; ++cycle) {
    TEST_ASSERT_FALSE(mowgli_motor_safety::update_link_rearm(
        state, true, 5u, false, false, 0u, 0u, false));
    TEST_ASSERT_FALSE(state.rearm_completed);
  }

  // A new fault still inhibits immediately and requires another fresh zero.
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, true, 5u, false, false, 1u, 0u, true));
  TEST_ASSERT_FALSE(state.rearm_completed);
  TEST_ASSERT_TRUE(mowgli_motor_safety::update_link_rearm(
      state, true, 5u, true, true, 1u, 0u, true));
  TEST_ASSERT_FALSE(state.rearm_completed);
  TEST_ASSERT_FALSE(mowgli_motor_safety::update_link_rearm(
      state, true, 6u, true, true, 1u, 0u, true));
  TEST_ASSERT_TRUE(state.rearm_completed);
  TEST_ASSERT_FALSE(mowgli_motor_safety::update_link_rearm(
      state, true, 6u, true, true, 1u, 0u, false));
  TEST_ASSERT_FALSE(state.rearm_completed);
}

static void test_zero_host_intent_suppresses_yaw_invented_targets()
{
  // These candidate trims represent gyro offset, retained integral, and turn
  // exit respectively. The production target shaper discards each at host zero.
  const float trims[] = {-0.12f, 0.08f, 0.03f};
  for (float trim : trims) {
    const auto targets = mowgli_motor_safety::apply_yaw_trim(
        0.0f, 0.0f, trim, true, false);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, targets.left_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, targets.right_mps);
  }
  TEST_ASSERT_FALSE(mowgli_motor_safety::yaw_loop_active(true, false, true));
  TEST_ASSERT_FALSE(mowgli_motor_safety::yaw_loop_active(false, false, false));
  TEST_ASSERT_TRUE(mowgli_motor_safety::yaw_loop_active(true, false, false));
  const auto disabled = mowgli_motor_safety::apply_yaw_trim(
      0.0f, 0.0f, 0.0f, true, false);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, disabled.left_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, disabled.right_mps);
}

static void test_zero_host_intent_preserves_braking_but_not_stationary_pwm()
{
  TEST_ASSERT_EQUAL_INT16(
      0, mowgli_motor_safety::suppress_stationary_zero_output(0.0f, 0.0f, 37));
  TEST_ASSERT_EQUAL_INT16(
      0, mowgli_motor_safety::suppress_stationary_zero_output(0.0f, -0.01f, -42));
  TEST_ASSERT_EQUAL_INT16(
      -24, mowgli_motor_safety::suppress_stationary_zero_output(0.0f, 0.03f, -24));
  TEST_ASSERT_EQUAL_INT16(
      18, mowgli_motor_safety::suppress_stationary_zero_output(0.0f, -0.04f, 18));
}

static void test_signed_zero_and_braking_reach_pac5210_frame()
{
  std::uint8_t packet[12]{};
  const Pac5210DriveRequest zero = pac5210_request_from_signed_pwm(0, 0);
  pac5210_encode_drive_packet(packet, zero);
  TEST_ASSERT_EQUAL_HEX8(0xa0u, packet[5]);
  TEST_ASSERT_EQUAL_UINT8(0u, packet[6]);
  TEST_ASSERT_EQUAL_UINT8(0u, packet[7]);
  std::uint8_t zero_checksum = 0u;
  for (std::uint8_t i = 0u; i < 11u; ++i) {
    zero_checksum = static_cast<std::uint8_t>(zero_checksum + packet[i]);
  }
  TEST_ASSERT_EQUAL_UINT8(zero_checksum, packet[11]);

  const Pac5210DriveRequest braking =
      pac5210_request_from_signed_pwm(-24, 18);
  pac5210_encode_drive_packet(packet, braking);
  TEST_ASSERT_EQUAL_HEX8(0xb0u, packet[5]);
  TEST_ASSERT_EQUAL_UINT8(24u, packet[6]);
  TEST_ASSERT_EQUAL_UINT8(18u, packet[7]);
  std::uint8_t checksum = 0u;
  for (std::uint8_t i = 0u; i < 11u; ++i) {
    checksum = static_cast<std::uint8_t>(checksum + packet[i]);
  }
  TEST_ASSERT_EQUAL_UINT8(checksum, packet[11]);
}

static void test_final_drive_gate_keeps_packet_zero_for_safety_events()
{
  const Pac5210DriveRequest reverse{0xb0u, 100u, 100u};
  const bool stop_cases[][6] = {
      {true, false, false, true, false, true},
      {false, true, false, true, false, true},
      {false, false, true, true, false, true},
      {false, false, false, false, false, true},
      {false, false, false, true, true, true},
      {false, false, false, true, false, false},
  };
  std::uint8_t packet[12]{};
  for (const auto &stop_case : stop_cases) {
    TEST_ASSERT_TRUE(pac5210_should_stop_output(
        stop_case[0], stop_case[1], stop_case[2], stop_case[3],
        stop_case[4], stop_case[5]));
  }
  TEST_ASSERT_FALSE(
      pac5210_should_stop_output(false, false, false, true, false, true));
  const Pac5210DriveRequest gated =
      pac5210_apply_final_output_gate(reverse, true);
  pac5210_encode_drive_packet(packet, gated);
  TEST_ASSERT_EQUAL_HEX8(0xa0u, packet[5]);
  TEST_ASSERT_EQUAL_UINT8(0u, packet[6]);
  TEST_ASSERT_EQUAL_UINT8(0u, packet[7]);
  std::uint8_t checksum = 0u;
  for (std::uint8_t i = 0u; i < 11u; ++i) {
    checksum = static_cast<std::uint8_t>(checksum + packet[i]);
  }
  TEST_ASSERT_EQUAL_UINT8(checksum, packet[11]);

  const Pac5210DriveRequest allowed =
      pac5210_apply_final_output_gate(reverse, false);
  TEST_ASSERT_EQUAL_UINT8(100u, allowed.left_speed);
  TEST_ASSERT_EQUAL_UINT8(100u, allowed.right_speed);
}

static void test_authorization_epoch_requires_fresh_zero_after_boundary()
{
  ActuatorAuthorizationState auth{1u, 1u};
  const Pac5210DriveRequest moving =
      pac5210_request_from_signed_pwm(100, 100);
  std::uint8_t packet[12]{};

  /* An emergency assert and release between motor ticks still invalidates
   * the cached request epoch. */
  actuator_authorization_invalidate(&auth);
  TEST_ASSERT_FALSE(
      actuator_authorization_drive_request_is_current(&auth, 1u));
  TEST_ASSERT_FALSE(actuator_authorization_accept_drive(&auth, false, false));
  Pac5210DriveRequest gated = pac5210_apply_final_output_gate(
      moving,
      pac5210_should_stop_output(false, false, false, true, false,
                                 actuator_authorization_drive_request_is_current(
                                     &auth, 1u)));
  pac5210_encode_drive_packet(packet, gated);
  TEST_ASSERT_EQUAL_HEX8(0xa0u, packet[5]);
  TEST_ASSERT_EQUAL_UINT8(0u, packet[6]);
  TEST_ASSERT_EQUAL_UINT8(0u, packet[7]);

  uint32_t accepted_epoch = 0u;
  TEST_ASSERT_FALSE(
      actuator_authorization_accept_drive(&auth, true, true));
  TEST_ASSERT_TRUE(
      actuator_authorization_accept_drive(&auth, true, false));
  accepted_epoch = auth.epoch;
  TEST_ASSERT_TRUE(
      actuator_authorization_drive_request_is_current(&auth,
                                                       accepted_epoch));
  TEST_ASSERT_TRUE(
      actuator_authorization_accept_drive(&auth, false, false));
  gated = pac5210_apply_final_output_gate(
      moving,
      pac5210_should_stop_output(false, false, false, true, false,
                                 actuator_authorization_drive_request_is_current(
                                     &auth, accepted_epoch)));
  pac5210_encode_drive_packet(packet, gated);
  TEST_ASSERT_EQUAL_UINT8(100u, packet[6]);
  TEST_ASSERT_EQUAL_UINT8(100u, packet[7]);

  actuator_authorization_invalidate(&auth); /* IDLE or link inhibition */
  TEST_ASSERT_FALSE(
      actuator_authorization_drive_request_is_current(&auth,
                                                       accepted_epoch));
}

static void test_finite_and_zero_are_accepted()
{
  SafetyState state{3.0f, 1.0f, -1.0f, 10u};
  TEST_ASSERT_TRUE(apply_safety(0.25f, -0.8f, 20u, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -0.8f, state.cmd_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, state.left_target_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, state.right_target_mps);
  TEST_ASSERT_EQUAL_UINT32(20u, state.last_valid_tick);
  TEST_ASSERT_FALSE(state.zero_motion_intent);
  TEST_ASSERT_FALSE(state.yaw_inhibited);
  TEST_ASSERT_TRUE(apply_safety(0.0f, 0.0f, 30u, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, state.left_target_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, state.right_target_mps);
  TEST_ASSERT_EQUAL_UINT32(30u, state.last_valid_tick);
  TEST_ASSERT_TRUE(state.zero_motion_intent);
  TEST_ASSERT_TRUE(state.yaw_inhibited);
  TEST_ASSERT_TRUE(apply_safety(0.0f, 0.0f, 40u, state));
  TEST_ASSERT_TRUE(state.zero_motion_intent);
  TEST_ASSERT_TRUE(state.yaw_inhibited);
}

static void test_cmd_vel_in_idle_cannot_reopen_motion_or_yaw_after_mowing()
{
  SafetyState state{0.4f, 0.35f, 0.25f, 77u, false, false};
  TEST_ASSERT_FALSE(
      mowgli_cmd_vel::apply_safety_for_mode(0.6f, 0.2f, 99u, true, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.left_target_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.right_target_mps);
  TEST_ASSERT_EQUAL_UINT32(77u, state.last_valid_tick);
  TEST_ASSERT_TRUE(state.zero_motion_intent);
  TEST_ASSERT_TRUE(state.yaw_inhibited);

  // Returning to MOWING does not change that state. The driver gate stays
  // closed and yaw cannot invent a differential until a fresh command arrives.
  TEST_ASSERT_FALSE(mowgli_motor_safety::yaw_loop_active(
      true, false, state.zero_motion_intent));
  const Pac5210DriveRequest cached_old_request =
      pac5210_request_from_signed_pwm(80, 60);
  const Pac5210DriveRequest gated = pac5210_apply_final_output_gate(
      cached_old_request, state.zero_motion_intent);
  TEST_ASSERT_EQUAL_UINT8(0xa0u, gated.direction);
  TEST_ASSERT_EQUAL_UINT8(0u, gated.left_speed);
  TEST_ASSERT_EQUAL_UINT8(0u, gated.right_speed);

  TEST_ASSERT_TRUE(
      mowgli_cmd_vel::apply_safety_for_mode(0.3f, 0.0f, 120u, false, state));
  TEST_ASSERT_FALSE(state.zero_motion_intent);
  TEST_ASSERT_FALSE(state.yaw_inhibited);
  TEST_ASSERT_EQUAL_UINT32(120u, state.last_valid_tick);
}

static void test_idle_zero_is_rearm_only_and_final_drive_gate_stays_closed()
{
  SafetyState state{0.4f, 0.35f, 0.25f, 77u, false, false};
  TEST_ASSERT_TRUE(mowgli_cmd_vel::apply_safety_for_mode(
      0.0f, 0.0f, 99u, true, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.left_target_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.right_target_mps);
  TEST_ASSERT_EQUAL_UINT32(77u, state.last_valid_tick);
  TEST_ASSERT_TRUE(state.zero_motion_intent);
  TEST_ASSERT_TRUE(state.yaw_inhibited);

  ActuatorAuthorizationState auth{1u, 0u};
  const bool boundary_active = mowgli_cmd_vel::authorization_boundary_active(
      true, state.zero_motion_intent, false);
  TEST_ASSERT_FALSE(boundary_active);
  TEST_ASSERT_TRUE(
      actuator_authorization_accept_drive(&auth, true, boundary_active));
  const uint32_t accepted_epoch = auth.epoch;
  TEST_ASSERT_EQUAL_UINT32(1u, accepted_epoch);
  const bool authorized =
      actuator_authorization_drive_request_is_current(&auth, accepted_epoch);
  TEST_ASSERT_TRUE(authorized);
  const Pac5210DriveRequest candidate =
      pac5210_request_from_signed_pwm(100, 100);
  const bool stop = pac5210_should_stop_output(
      false, true, false, true, true, authorized);
  TEST_ASSERT_TRUE(stop);
  const Pac5210DriveRequest final_request =
      pac5210_apply_final_output_gate(candidate, stop);
  TEST_ASSERT_EQUAL_UINT8(0u, final_request.left_speed);
  TEST_ASSERT_EQUAL_UINT8(0u, final_request.right_speed);

  /* An IDLE zero does not make blade ON look like a fresh host motion session. */
  TEST_ASSERT_FALSE(blade_on_command_is_fresh(
      100u, 99u, false, 200u, 100u, true, 2000u));
  TEST_ASSERT_FALSE(mowgli_cmd_vel::apply_safety_for_mode(
      0.1f, 0.0f, 120u, true, state));
  TEST_ASSERT_TRUE(mowgli_cmd_vel::authorization_boundary_active(
      true, false, false));
  TEST_ASSERT_TRUE(mowgli_cmd_vel::authorization_boundary_active(
      true, true, true));
}

static void test_nonfinite_commands_clear_targets_without_refresh()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const float values[] = {nan, inf, -inf};
  for (float vx : values) {
    SafetyState state{0.7f, 0.4f, -0.4f, 77u};
    TEST_ASSERT_FALSE(apply_safety(vx, 0.0f, 99u, state));
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.left_target_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.right_target_mps);
    TEST_ASSERT_EQUAL_UINT32(77u, state.last_valid_tick);
    TEST_ASSERT_FALSE(state.zero_motion_intent);
    TEST_ASSERT_TRUE(state.yaw_inhibited);
  }
  for (float wz : values) {
    SafetyState state{0.7f, 0.4f, -0.4f, 77u};
    TEST_ASSERT_FALSE(apply_safety(0.0f, wz, 99u, state));
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.left_target_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.right_target_mps);
    TEST_ASSERT_EQUAL_UINT32(77u, state.last_valid_tick);
  }
  for (float vx : values) {
    for (float wz : values) {
      SafetyState state{0.7f, 0.4f, -0.4f, 77u};
      TEST_ASSERT_FALSE(apply_safety(vx, wz, 99u, state));
      TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
      TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.left_target_mps);
      TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.right_target_mps);
      TEST_ASSERT_EQUAL_UINT32(77u, state.last_valid_tick);
    }
  }
}

static void test_valid_command_after_invalid_is_normal()
{
  SafetyState state{0.0f, 0.0f, 0.0f, 11u};
  TEST_ASSERT_FALSE(apply_safety(
      std::numeric_limits<float>::quiet_NaN(), 0.0f, 12u, state));
  TEST_ASSERT_TRUE(apply_safety(0.3f, -0.2f, 13u, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -0.2f, state.cmd_wz);
  TEST_ASSERT_EQUAL_UINT32(13u, state.last_valid_tick);
}

static void test_imu_mount_rotation_identity_preserves_all_axes()
{
  float x = 1.0f;
  float y = -2.0f;
  float z = 3.0f;
  IMU_ApplyMountRotation(IMU_MOUNT_ROTATION_IDENTITY, &x, &y, &z);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -2.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.0f, z);
}

static void test_imu_mount_rotation_yaw_180_negates_x_y_only()
{
  float x = 1.0f;
  float y = -2.0f;
  float z = 3.0f;
  IMU_ApplyMountRotation(IMU_MOUNT_ROTATION_YAW_180, &x, &y, &z);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 2.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.0f, z);
}

static void test_configured_mount_maps_accel_and_gyro_basis_vectors()
{
#if IMU_MOUNT_ROTATION == IMU_MOUNT_ROTATION_YAW_180
  float x = 1.0f, y = 0.0f, z = 0.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // +X acceleration
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, z);

  x = 0.0f; y = 1.0f; z = 0.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // +Y acceleration
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, z);

  x = 1.0f; y = 0.0f; z = 0.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // gyro +X
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, z);

  x = 0.0f; y = 1.0f; z = 0.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // gyro +Y
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, z);

  x = 0.0f; y = 0.0f; z = 1.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // gyro +Z
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, z);

  x = 1.0f; y = 2.0f; z = 3.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // magnetometer path
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -2.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.0f, z);
#else
  float x = 1.0f, y = 2.0f, z = 3.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 2.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.0f, z);
#endif
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_blade_authorization_is_discarded_at_emergency_boundary);
  RUN_TEST(test_blade_intent_is_cleared_after_cmd_vel_timeout);
  RUN_TEST(test_blade_on_requires_fresh_cmd_vel_and_heartbeat);
  RUN_TEST(test_heartbeat_watchdog_starts_without_received_packet);
  RUN_TEST(test_blade_on_received_during_latch_is_not_deferred);
  RUN_TEST(test_blade_on_during_link_rearm_is_not_deferred);
  RUN_TEST(test_motor_link_requires_fresh_zero_after_recovery);
  RUN_TEST(test_idle_zero_can_complete_safe_motor_link_rearm);
  RUN_TEST(test_motor_link_zero_rearm_does_not_require_zero_during_motion);
  RUN_TEST(test_motor_link_fault_requires_new_zero_and_blade_off);
  RUN_TEST(test_zero_host_intent_suppresses_yaw_invented_targets);
  RUN_TEST(test_zero_host_intent_preserves_braking_but_not_stationary_pwm);
  RUN_TEST(test_signed_zero_and_braking_reach_pac5210_frame);
  RUN_TEST(test_final_drive_gate_keeps_packet_zero_for_safety_events);
  RUN_TEST(test_authorization_epoch_requires_fresh_zero_after_boundary);
  RUN_TEST(test_finite_and_zero_are_accepted);
  RUN_TEST(test_nonfinite_commands_clear_targets_without_refresh);
  RUN_TEST(test_valid_command_after_invalid_is_normal);
  RUN_TEST(test_cmd_vel_in_idle_cannot_reopen_motion_or_yaw_after_mowing);
  RUN_TEST(test_idle_zero_is_rearm_only_and_final_drive_gate_stays_closed);
  RUN_TEST(test_imu_mount_rotation_identity_preserves_all_axes);
  RUN_TEST(test_imu_mount_rotation_yaw_180_negates_x_y_only);
  RUN_TEST(test_configured_mount_maps_accel_and_gyro_basis_vectors);
  return UNITY_END();
}
