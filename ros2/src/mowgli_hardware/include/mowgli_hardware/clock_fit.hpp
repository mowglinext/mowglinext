// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// HostFirmwareClockFit — host-side fitter that maps cumulative
// firmware milliseconds to host nanoseconds (rclcpp::Time::nanoseconds()).
//
// Why: the STM32 sends packet timing as `dt_millis` since the last
// emit. The host's first instinct is to stamp the message with
// `rclcpp::Node::now()`, but that bakes in 5-20 ms of USB / executor
// jitter on every packet — visible downstream as IMU samples appearing
// to arrive in bursts, wheel-odom rate varying by ±30 %, and the
// fusion graph absorbing the jitter as motion noise.
//
// This fitter accumulates a virtual firmware clock by summing
// dt_millis across packets, then fits a piecewise-linear model
// `host_ns ≈ a · fw_ms + b` over a sliding window of recent samples.
// Each new packet stamps at `a · fw_ms + b` — the systematic latency
// (a stays close to 1e6 ns/ms, b stays close to host_ns - fw_ms*1e6
// at startup) absorbed into the constants, the random jitter
// averaged out across the window.
//
// Reset semantics: a single dt_millis larger than `reset_gap_ms_`
// (default 5000 ms = 5 s) is interpreted as a firmware reboot or
// USB stall recovery. The fitter discards the prior samples and
// rebootstraps on the next packet.
//
// Thread safety: single-threaded by contract. The hardware_bridge
// node's serial-RX path is single-threaded; if a future caller adds
// a second writer, wrap with a mutex externally.

#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include <rclcpp/time.hpp>

namespace mowgli_hardware
{

class HostFirmwareClockFit
{
public:
  HostFirmwareClockFit() = default;

  // Configure the sliding window size + the dt_millis above which we
  // consider the firmware to have rebooted. Defaults are reasonable
  // for a 50 Hz IMU stream: 100 samples ≈ 2 s of recent history.
  void Configure(size_t window_samples, uint32_t reset_gap_ms)
  {
    window_samples_ = window_samples;
    reset_gap_ms_ = reset_gap_ms;
  }

  // Ingest a new packet. `pkt_dt_millis` is the firmware-reported
  // inter-packet interval (the only timing field we currently have
  // on the wire). `host_now` is the host's monotonic time at the
  // moment the packet was decoded.
  //
  // Returns the smoothed host stamp to use on the published message.
  // On the very first call (no prior reference), returns host_now
  // verbatim — there is no fit history to smooth against. By the
  // time the window fills (~2 s), the fit has converged and
  // subsequent stamps are jitter-free.
  rclcpp::Time Ingest(uint32_t pkt_dt_millis, const rclcpp::Time& host_now);

  // Reset the fitter. Called automatically on a reset_gap_ms gap;
  // callers can also invoke manually (e.g. after a known firmware
  // reflash sequence).
  void Reset();

  // Diagnostic accessors.
  size_t SampleCount() const
  {
    return samples_.size();
  }
  uint64_t FirmwareClockMs() const
  {
    return fw_clock_ms_;
  }
  bool HasFit() const
  {
    return samples_.size() >= 2;
  }
  // Slope (ns per firmware-ms). Should hover near 1e6 in steady state;
  // values far from that indicate the firmware clock is running at a
  // significantly different rate than the host clock (worth investigating).
  double SlopeNsPerMs() const
  {
    return slope_;
  }
  // Offset (ns). `host_ns ≈ slope · fw_ms + offset`. Captures the boot-
  // time delta between host and firmware clocks.
  double OffsetNs() const
  {
    return offset_;
  }

private:
  struct Sample
  {
    uint64_t fw_ms;
    int64_t host_ns;
  };

  // Refit the linear regression over the current window.
  void Refit();

  size_t window_samples_ = 100;
  uint32_t reset_gap_ms_ = 5000;
  uint64_t fw_clock_ms_ = 0;
  std::deque<Sample> samples_;
  double slope_ = 1.0e6;  // ns per firmware-ms — initial unit slope
  double offset_ = 0.0;
  // True once we've ingested at least one sample. Distinct from
  // SampleCount() == 0 in case of future use of seeded initial state.
  bool has_any_ = false;
};

}  // namespace mowgli_hardware
