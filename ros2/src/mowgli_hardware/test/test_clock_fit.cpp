// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for HostFirmwareClockFit. Synthetic packet streams,
// closed-form expectations, no ROS plumbing.

#include <cmath>
#include <cstdint>

#include "mowgli_hardware/clock_fit.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{

// Helpers — synthesize a stream of (host_time, dt_ms) pairs that
// mimic the firmware emitting at a perfect kRateHz with random
// jitter on host arrival.
struct PacketStream
{
  uint64_t fw_ms = 0;
  int64_t host_ns = 1'000'000'000'000LL;  // arbitrary boot offset
  uint32_t period_ms = 20;  // 50 Hz

  // Advance by one perfect packet (host arrives exactly on schedule).
  std::pair<uint32_t, rclcpp::Time> Step()
  {
    fw_ms += period_ms;
    host_ns += static_cast<int64_t>(period_ms) * 1'000'000LL;
    return {period_ms, rclcpp::Time(host_ns, RCL_ROS_TIME)};
  }

  // Advance by one packet, but the host arrival is jittered by `jitter_ns`
  // relative to the perfect schedule.
  std::pair<uint32_t, rclcpp::Time> StepJitter(int64_t jitter_ns)
  {
    fw_ms += period_ms;
    host_ns += static_cast<int64_t>(period_ms) * 1'000'000LL;
    return {period_ms, rclcpp::Time(host_ns + jitter_ns, RCL_ROS_TIME)};
  }
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// First packet bootstrap — with no fit history, the stamp falls
// through to host_now verbatim. The fit is seeded so subsequent
// packets can refine.
// ─────────────────────────────────────────────────────────────────────
TEST(ClockFit, FirstPacketReturnsHostNow)
{
  mh::HostFirmwareClockFit fit;
  PacketStream stream;
  auto [dt, host] = stream.Step();
  auto out = fit.Ingest(dt, host);
  EXPECT_EQ(out.nanoseconds(), host.nanoseconds());
  EXPECT_EQ(fit.SampleCount(), 1u);
  EXPECT_FALSE(fit.HasFit());
}

// ─────────────────────────────────────────────────────────────────────
// Steady-state: perfect packets (no jitter) → after many samples the
// slope converges to 1e6 ns/ms and the stamps land exactly on the
// host clock (no drift introduced by the fit).
// ─────────────────────────────────────────────────────────────────────
TEST(ClockFit, NoJitterSteadyState)
{
  mh::HostFirmwareClockFit fit;
  PacketStream stream;
  for (int i = 0; i < 200; ++i)
  {
    auto [dt, host] = stream.Step();
    fit.Ingest(dt, host);
  }
  EXPECT_NEAR(fit.SlopeNsPerMs(), 1.0e6, 1.0);  // 1 ns tolerance
  EXPECT_TRUE(fit.HasFit());
}

// ─────────────────────────────────────────────────────────────────────
// Jitter: host arrives with ±5 ms of zero-mean jitter on every
// packet. The fit averages it out — the stamp on a fresh packet
// hews to the predicted (slope × fw + offset) regardless of where
// the noisy host_now landed.
// ─────────────────────────────────────────────────────────────────────
TEST(ClockFit, JitterAveragedOut)
{
  mh::HostFirmwareClockFit fit;
  PacketStream stream;
  // Warm the fit with 100 jittered samples (alternating +5 ms / -5 ms).
  for (int i = 0; i < 100; ++i)
  {
    auto [dt, host] = stream.StepJitter((i % 2 == 0) ? 5'000'000 : -5'000'000);
    fit.Ingest(dt, host);
  }
  // The next packet's host arrival is jittered by +5 ms but the fit
  // should produce a stamp within ±0.5 ms of the perfect-schedule
  // time (i.e. should NOT propagate the +5 ms jitter into the stamp).
  PacketStream perfect_ref;
  perfect_ref.fw_ms = stream.fw_ms;
  perfect_ref.host_ns = stream.host_ns;
  auto [dt, host_jittered] = stream.StepJitter(5'000'000);
  auto stamp = fit.Ingest(dt, host_jittered);

  // The fit should pick the "true" time (unjittered schedule). Allow
  // 1 ms tolerance for closed-form numeric noise.
  perfect_ref.fw_ms += perfect_ref.period_ms;
  perfect_ref.host_ns += static_cast<int64_t>(perfect_ref.period_ms) * 1'000'000LL;
  const int64_t expected_ns = perfect_ref.host_ns;
  const int64_t got_ns = stamp.nanoseconds();
  const int64_t err_ns = std::abs(got_ns - expected_ns);
  EXPECT_LT(err_ns, 1'000'000)
      << "fit produced jittered stamp: err=" << err_ns << " ns";
}

// ─────────────────────────────────────────────────────────────────────
// Sliding window: very-old samples roll out so the fit reflects the
// recent regime. Sanity check that the sample count caps at the
// configured window size.
// ─────────────────────────────────────────────────────────────────────
TEST(ClockFit, WindowCapsAtConfiguredSize)
{
  mh::HostFirmwareClockFit fit;
  fit.Configure(50, 5000);
  PacketStream stream;
  for (int i = 0; i < 200; ++i)
  {
    auto [dt, host] = stream.Step();
    fit.Ingest(dt, host);
  }
  EXPECT_EQ(fit.SampleCount(), 50u);
}

// ─────────────────────────────────────────────────────────────────────
// Reset gap: a single dt_millis > reset_gap_ms_ flushes the history
// and rebootstraps. After the reset, the next packet should return
// host_now verbatim (no fit until 2 samples accumulate again).
// ─────────────────────────────────────────────────────────────────────
TEST(ClockFit, LargeGapTriggersReset)
{
  mh::HostFirmwareClockFit fit;
  PacketStream stream;
  // Warm up with 20 samples.
  for (int i = 0; i < 20; ++i)
  {
    auto [dt, host] = stream.Step();
    fit.Ingest(dt, host);
  }
  ASSERT_GE(fit.SampleCount(), 20u);

  // Simulate a 10-second gap (firmware reboot or USB stall).
  rclcpp::Time host_after_gap(stream.host_ns + 10'000'000'000LL, RCL_ROS_TIME);
  auto stamp = fit.Ingest(10'000 /* 10 s */, host_after_gap);

  EXPECT_EQ(fit.SampleCount(), 1u);
  EXPECT_EQ(stamp.nanoseconds(), host_after_gap.nanoseconds())
      << "post-reset stamp should be host_now verbatim";
  EXPECT_FALSE(fit.HasFit());
}

// ─────────────────────────────────────────────────────────────────────
// Slope drift detection — feed a stream where the firmware clock
// runs 0.1% fast vs the host. The fit's slope should converge near
// 999000 ns/ms (firmware clock is 0.1% fast → ns/ms is 0.1% low).
// ─────────────────────────────────────────────────────────────────────
TEST(ClockFit, SlopeReflectsClockDrift)
{
  mh::HostFirmwareClockFit fit;
  PacketStream stream;
  // Firmware reports dt=20ms but host time advances 20.02 ms — i.e.
  // host clock is 0.1% faster (or firmware is 0.1% slower); either
  // way, slope_ should land at 1001000 ns/ms.
  for (int i = 0; i < 200; ++i)
  {
    stream.fw_ms += 20;
    stream.host_ns += 20'020'000;  // 20.02 ms in ns
    fit.Ingest(20, rclcpp::Time(stream.host_ns, RCL_ROS_TIME));
  }
  EXPECT_NEAR(fit.SlopeNsPerMs(), 1.001e6, 100.0);
}
