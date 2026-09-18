#include <limits>

#include <unity.h>

#include "cmd_vel_safety.hpp"
#include "dfu_transition.hpp"

using mowgli_cmd_vel::SafetyState;
using mowgli_cmd_vel::apply_safety;

static void test_finite_and_zero_are_accepted()
{
  SafetyState state{3.0f, 1.0f, -1.0f, 10u};
  TEST_ASSERT_TRUE(apply_safety(0.25f, -0.8f, 20u, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -0.8f, state.cmd_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, state.left_target_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, state.right_target_mps);
  TEST_ASSERT_EQUAL_UINT32(20u, state.last_valid_tick);
  TEST_ASSERT_TRUE(apply_safety(0.0f, 0.0f, 30u, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, state.left_target_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, state.right_target_mps);
  TEST_ASSERT_EQUAL_UINT32(30u, state.last_valid_tick);
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

static void test_dfu_request_requires_f401_idle_magic_and_idle_phase()
{
  using mowgli_dfu::Phase;
  TEST_ASSERT_TRUE(mowgli_dfu::request_is_accepted(true, true, 0xD3u, 0xD3u,
                                                    Phase::Idle));
  TEST_ASSERT_FALSE(mowgli_dfu::request_is_accepted(false, true, 0xD3u, 0xD3u,
                                                     Phase::Idle));
  TEST_ASSERT_FALSE(mowgli_dfu::request_is_accepted(true, false, 0xD3u, 0xD3u,
                                                     Phase::Idle));
  TEST_ASSERT_FALSE(mowgli_dfu::request_is_accepted(true, true, 0x00u, 0xD3u,
                                                     Phase::Idle));
  TEST_ASSERT_FALSE(mowgli_dfu::request_is_accepted(true, true, 0xD3u, 0xD3u,
                                                     Phase::Stopping));
}

static void test_dfu_transition_locks_later_commands()
{
  TEST_ASSERT_FALSE(mowgli_dfu::inputs_are_locked(mowgli_dfu::Phase::Idle));
  TEST_ASSERT_TRUE(mowgli_dfu::inputs_are_locked(mowgli_dfu::Phase::Requested));
  TEST_ASSERT_TRUE(mowgli_dfu::inputs_are_locked(mowgli_dfu::Phase::Stopping));
}

static void test_dfu_stop_window_is_bounded_and_tick_wrap_safe()
{
  TEST_ASSERT_FALSE(mowgli_dfu::ready_to_reset(100u, 599u));
  TEST_ASSERT_TRUE(mowgli_dfu::ready_to_reset(100u, 600u));
  TEST_ASSERT_FALSE(mowgli_dfu::ready_to_reset(0xFFFFFF00u, 0x000000F3u));
  TEST_ASSERT_TRUE(mowgli_dfu::ready_to_reset(0xFFFFFF00u, 0x000000F4u));
}

static void test_dfu_marker_requires_magic_inverse_and_software_reset()
{
  constexpr uint32_t magic = mowgli_dfu::kMarkerMagic;
  TEST_ASSERT_TRUE(mowgli_dfu::marker_is_valid(magic, ~magic, true));
  TEST_ASSERT_FALSE(mowgli_dfu::marker_is_valid(magic, ~magic, true, true));
  TEST_ASSERT_FALSE(mowgli_dfu::marker_is_valid(magic, ~magic, false));
  TEST_ASSERT_FALSE(mowgli_dfu::marker_is_valid(0u, ~magic, true));
  TEST_ASSERT_FALSE(mowgli_dfu::marker_is_valid(magic, 0u, true));
}

static void test_dfu_reset_cause_accepts_sftrst_plus_pin_but_not_pin_only()
{
  constexpr uint32_t sftrst = 1u << 0u;
  constexpr uint32_t pinrst = 1u << 1u;
  constexpr uint32_t wwdgrst = 1u << 2u;
  constexpr uint32_t rejected = wwdgrst;
  TEST_ASSERT_TRUE(
      DFU_ResetCauseAllowsEntry(sftrst | pinrst, sftrst, rejected));
  TEST_ASSERT_FALSE(DFU_ResetCauseAllowsEntry(pinrst, sftrst, rejected));
  TEST_ASSERT_FALSE(
      DFU_ResetCauseAllowsEntry(sftrst | wwdgrst, sftrst, rejected));
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_finite_and_zero_are_accepted);
  RUN_TEST(test_nonfinite_commands_clear_targets_without_refresh);
  RUN_TEST(test_valid_command_after_invalid_is_normal);
  RUN_TEST(test_dfu_request_requires_f401_idle_magic_and_idle_phase);
  RUN_TEST(test_dfu_transition_locks_later_commands);
  RUN_TEST(test_dfu_stop_window_is_bounded_and_tick_wrap_safe);
  RUN_TEST(test_dfu_marker_requires_magic_inverse_and_software_reset);
  RUN_TEST(test_dfu_reset_cause_accepts_sftrst_plus_pin_but_not_pin_only);
  return UNITY_END();
}
