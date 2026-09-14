// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// Correction-diagnostics projection, extracted from the bridge's inline
// helpers. Mirrors CorrectionDiagnosticTracker in the reference Python bridge
// (sensors/gps/universal_gnss_topic_bridge.py) exactly.

#ifndef MOWGLI_GNSS_BRIDGE__CORRECTION_DIAGNOSTIC_TRACKER_HPP_
#define MOWGLI_GNSS_BRIDGE__CORRECTION_DIAGNOSTIC_TRACKER_HPP_

#include <chrono>
#include <memory>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"

namespace mowgli_gnss_bridge
{

/// Bounded, source-owned projection of Universal GNSS correction diagnostics.
///
/// Each Universal GNSS producer contribution is treated as a complete snapshot:
/// unrelated arrays are OMIT, while a represented producer replaces its prior
/// snapshot so omitted entries become CLEAR. A steady clock owns liveness; the
/// ROS diagnostic stamp is used only to reject older snapshots.
class CorrectionDiagnosticTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit CorrectionDiagnosticTracker(std::chrono::duration<double> timeout);
  ~CorrectionDiagnosticTracker();

  CorrectionDiagnosticTracker(CorrectionDiagnosticTracker &&) noexcept;
  CorrectionDiagnosticTracker & operator=(CorrectionDiagnosticTracker &&) noexcept;

  CorrectionDiagnosticTracker(const CorrectionDiagnosticTracker &) = delete;
  CorrectionDiagnosticTracker & operator=(const CorrectionDiagnosticTracker &) = delete;

  void update(
    const diagnostic_msgs::msg::DiagnosticArray & diagnostics,
    TimePoint received_at = Clock::now());

  void apply(mowgli_interfaces::msg::GnssStatus & status, TimePoint now = Clock::now()) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mowgli_gnss_bridge

#endif  // MOWGLI_GNSS_BRIDGE__CORRECTION_DIAGNOSTIC_TRACKER_HPP_
