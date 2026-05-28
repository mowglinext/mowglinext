// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_hardware/clock_fit.hpp"

namespace mowgli_hardware
{

rclcpp::Time HostFirmwareClockFit::Ingest(uint32_t pkt_dt_millis, const rclcpp::Time& host_now)
{
  // Reset gap detection — a single large dt_millis is firmware reboot
  // or USB stall recovery. Discard accumulated history and rebootstrap
  // on this sample (next Ingest will append the second point and we
  // get a 2-sample linear fit). The reset path stamps with host_now
  // verbatim since we have no reference.
  if (has_any_ && pkt_dt_millis > reset_gap_ms_)
  {
    Reset();
  }

  // Accumulate the virtual firmware clock.
  fw_clock_ms_ += pkt_dt_millis;
  has_any_ = true;

  const int64_t host_ns = host_now.nanoseconds();

  // Append sample, trim oldest if window is full.
  samples_.push_back(Sample{fw_clock_ms_, host_ns});
  while (samples_.size() > window_samples_)
  {
    samples_.pop_front();
  }

  // Bootstrap: with a single sample we have no fit; stamp at host_now
  // and seed the slope/offset for the next iteration.
  if (samples_.size() < 2)
  {
    slope_ = 1.0e6;  // 1 ms = 1e6 ns
    offset_ = static_cast<double>(host_ns) - slope_ * static_cast<double>(fw_clock_ms_);
    return host_now;
  }

  Refit();

  // Stamp using the fit. Cast to int64_t after offset application so
  // a slope drift of e.g. 1.0001e6 ns/ms over 1 hour of firmware time
  // (3.6e6 ms) shifts the stamp by ~360 ms — still well below typical
  // tf transform_tolerance and absorbed gracefully downstream.
  const int64_t fitted_ns =
      static_cast<int64_t>(slope_ * static_cast<double>(fw_clock_ms_) + offset_);
  return rclcpp::Time(fitted_ns, host_now.get_clock_type());
}

void HostFirmwareClockFit::Refit()
{
  // Closed-form least-squares: y = a x + b
  // a = (n·Σxy − Σx·Σy) / (n·Σx² − (Σx)²)
  // b = (Σy − a·Σx) / n
  //
  // Work in double precision because fw_ms is bounded (≤ uint32_t →
  // ~4.3e9), host_ns is ~1.7e18 (since 1970), and the products in
  // Σxy reach 1.7e27 — outside int64_t range. To keep the sums in
  // double's 15-16 decimal-digit precision regardless of session
  // length, subtract a reference (the first sample's values) from
  // every term. Slope is invariant under this shift; offset is
  // recovered by adding back the reference contribution.
  const Sample& ref = samples_.front();
  double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
  const double n = static_cast<double>(samples_.size());
  for (const auto& s : samples_)
  {
    const double x = static_cast<double>(s.fw_ms - ref.fw_ms);
    const double y = static_cast<double>(s.host_ns - ref.host_ns);
    sum_x += x;
    sum_y += y;
    sum_xy += x * y;
    sum_x2 += x * x;
  }
  const double denom = n * sum_x2 - sum_x * sum_x;
  if (denom <= 0.0)
  {
    // Degenerate: all samples at the same fw_ms (shouldn't happen
    // after the first packet because dt_millis > 0). Fall back to
    // the previous fit values to avoid a divide-by-zero.
    return;
  }
  slope_ = (n * sum_xy - sum_x * sum_y) / denom;
  const double mean_x_in_shift = sum_x / n;
  const double mean_y_in_shift = sum_y / n;
  // Recover the un-shifted offset: y_orig = slope*x_orig + offset
  // where y_orig = y_shift + ref.host_ns and x_orig = x_shift + ref.fw_ms.
  offset_ = (mean_y_in_shift + static_cast<double>(ref.host_ns)) -
            slope_ * (mean_x_in_shift + static_cast<double>(ref.fw_ms));
}

void HostFirmwareClockFit::Reset()
{
  samples_.clear();
  fw_clock_ms_ = 0;
  slope_ = 1.0e6;
  offset_ = 0.0;
  has_any_ = false;
}

}  // namespace mowgli_hardware
