// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Windowed wheel-vs-gyro slip veto (issue #516). Pure — no ROS / GTSAM — so it
// is unit-testable, same shape as dr_slip_veto.hpp / rtk_wrongfix_gate.hpp.
//
// CreateNodeLocked used to evaluate the slip veto on ONE node's accumulators
// (node_period_s = 0.04 s). At that scale the gate cannot separate slip from
// the encoder quantum: one tick of left/right asymmetry in one 40 ms frame is
// ≈ 0.011 rad of wheel dθ — already above slip_wheel_min_rad (0.005) and one
// tick short of slip_residual_thresh_rad (0.01); two ticks trip it. Field
// 2026-09-02 (73 min LiDAR mow, RTK Fixed): the veto fired 1.33 times/s on
// STRAIGHT swaths (0.27/s turning, 0.18/s stopped); at the increments the
// wheel yaw rate was p50 0.00 rad/s vs gyro 0.05 rad/s — ~5 % of nodes had
// their wheel translation zeroed on pure quantization jitter.
//
// The genuine signature the veto exists for (2026-05-27: wheels ~0.3 rad/s,
// gyro < 0.02 rad/s, sustained for seconds) is ≈ 0.012 rad per frame —
// indistinguishable per frame. The signal is the PERSISTENCE: over 0.5 s
// genuine slip accumulates ~0.15 rad of wheel dθ while tick jitter averages to
// ~0. So the SAME three-way rule is applied to the WINDOW SUMS with the
// per-node thresholds scaled by the number of nodes in the window — the yaml
// values keep their per-node meaning and the window just integrates.
//
// A window of one node (slip_window_s: 0) reproduces the old per-node gate
// EXACTLY: the sums are that node's deltas, the scale factor is 1 and
// min_nodes = 1, so the very first sample is evaluated. Do not raise the
// per-frame thresholds instead — that also blinds the veto to the real case.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <utility>

namespace fusion_graph
{

// Window length in nodes for a wall-clock window. Floors at 1 (the old
// per-node gate) for a disabled (<= 0) or degenerate configuration. Ties
// round DOWN (0.5 s / 0.04 s is 12.5 — 12 nodes, the figure in #516; the raw
// double quotient is 12.500000000000002, which llround alone would take to 13).
inline std::size_t SlipWindowNodes(double window_s, double node_period_s)
{
  if (window_s <= 0.0 || node_period_s <= 0.0)
    return 1;
  constexpr double kTieBreakDown = 1e-6;
  const long long nodes = std::llround(window_s / node_period_s - kTieBreakDown);
  return static_cast<std::size_t>(std::max<long long>(1, nodes));
}

// The per-node rule — |wheel − gyro| > residual AND |gyro| < gyro_max AND
// |wheel| > wheel_min — applied to window sums, thresholds scaled by the
// node count so each keeps its per-node (rad / node) meaning.
inline bool SlipDetectedOverWindow(double sum_dtheta_wheel,
                                   double sum_dtheta_gyro,
                                   std::size_t nodes_in_window,
                                   double residual_thresh_rad,
                                   double gyro_max_rad,
                                   double wheel_min_rad)
{
  if (nodes_in_window == 0)
    return false;
  const double n = static_cast<double>(nodes_in_window);
  return std::abs(sum_dtheta_wheel - sum_dtheta_gyro) > residual_thresh_rad * n &&
         std::abs(sum_dtheta_gyro) < gyro_max_rad * n &&
         std::abs(sum_dtheta_wheel) > wheel_min_rad * n;
}

// Bounded ring of per-node (dθ_wheel, dθ_gyro) samples. Sums are recomputed
// over the (short) ring on demand rather than kept as running totals, so hours
// of add/subtract cannot accumulate floating-point drift in the veto.
class SlipWindow
{
public:
  // min_nodes = 0 means "the full window" — a cold window never vetoes.
  explicit SlipWindow(std::size_t window_nodes = 1, std::size_t min_nodes = 0)
      : capacity_(std::max<std::size_t>(1, window_nodes)),
        min_nodes_(min_nodes == 0 ? capacity_ : std::min(min_nodes, capacity_))
  {
  }

  void Push(double dtheta_wheel, double dtheta_gyro)
  {
    samples_.emplace_back(dtheta_wheel, dtheta_gyro);
    if (samples_.size() > capacity_)
      samples_.pop_front();
  }
  void Clear()
  {
    samples_.clear();
  }
  std::size_t Size() const
  {
    return samples_.size();
  }
  std::size_t Capacity() const
  {
    return capacity_;
  }

  bool Detected(double residual_thresh_rad, double gyro_max_rad, double wheel_min_rad) const
  {
    if (samples_.size() < min_nodes_)
      return false;
    double sum_wheel = 0.0;
    double sum_gyro = 0.0;
    for (const auto& [wheel, gyro] : samples_)
    {
      sum_wheel += wheel;
      sum_gyro += gyro;
    }
    return SlipDetectedOverWindow(
        sum_wheel, sum_gyro, samples_.size(), residual_thresh_rad, gyro_max_rad, wheel_min_rad);
  }

private:
  std::size_t capacity_;
  std::size_t min_nodes_;
  std::deque<std::pair<double, double>> samples_;
};

}  // namespace fusion_graph
