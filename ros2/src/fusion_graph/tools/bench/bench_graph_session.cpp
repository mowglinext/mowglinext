// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Deterministic synthetic mowing-session benchmark for fusion_graph's
// GraphManager (no ROS graph, no robot, no simulator).
//
// Drives GraphManager exactly the way FusionGraphNode does on the deployed
// cadence (node_period_s 0.04, max_graph_nodes 6000, rebase budget 2000,
// cov_update_every_n 10, relinearize skip 5) with a seeded synthetic stream:
//   * serpentine coverage (0.4 m/s straights, 0.16 m swath spacing) with
//     pivot U-turns and periodic stops,
//   * wheel odometry 50 Hz, gyro 91 Hz (bias + noise),
//   * GNSS 10 Hz with an 80 ms measurement-stamp lag (QueueGnss targets
//     FindNodeAtOrBefore(stamp), as OnGnss does), RTK-Fixed with periodic
//     RTK-Float windows and complete outages,
//   * COG yaw at 5 Hz while RTK-Fixed and driving straight,
//   * the 1 Hz diagnostics/markers consumers (Stats, LiveNodeCount,
//     GetAllPoses) and the 30 s maintenance rebase (run synchronously here;
//     production runs it on a detached thread, its CPU still counts).
//
// Every Tick output and a 1 Hz snapshot are written to --dump so two builds
// can be compared bit-for-bit or within a tolerance (--compare A B).
//
//   bench_graph_session --minutes 120 --dump out.bin
//   bench_graph_session --compare a.bin b.bin

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "fusion_graph/graph_manager.hpp"
#include <sys/resource.h>

namespace fg = fusion_graph;

namespace
{

// Mirrors fusion_graph.yaml + the navigation.launch.py overrides.
fg::GraphParams ProductionParams(double lever_arm_x)
{
  fg::GraphParams gp;
  gp.node_period_s = 0.04;
  gp.wheel_sigma_x_per_sqrt_m = 0.05;
  gp.wheel_sigma_y_per_sqrt_m = 0.005;
  gp.wheel_creep_speed_mps = 0.04;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.gps_sigma_floor = 0.003;
  gp.prior_sigma_xy = 0.05;
  gp.prior_sigma_theta = 0.05;
  gp.lever_arm_x = lever_arm_x;
  gp.lever_arm_y = 0.0;
  gp.cov_update_every_n = 10;
  gp.isam2_relinearize_skip = 5;
  gp.max_graph_nodes = 6000;
  gp.stationary_motion_thresh_m = 0.02;
  gp.stationary_motion_thresh_theta = 0.01;
  gp.stationary_node_period_s = 5.0;
  gp.stationary_thresh_xy_m = 0.001;
  gp.stationary_thresh_theta = 0.002;
  gp.stationary_sigma_theta = 0.01;
  gp.pivot_gate_dtheta_rad = 0.012;
  gp.pivot_wheel_sigma_x = 0.5;
  gp.stationary_gyro_thresh_rad_per_s = 0.10;
  gp.slip_residual_thresh_rad = 0.01;
  gp.slip_gyro_max_rad = 0.005;
  gp.slip_wheel_min_rad = 0.005;
  gp.slip_window_s = 0.5;
  gp.gyro_bias_estimation_enabled = true;
  gp.gyro_bias_ema_tau_s = 30.0;
  gp.gyro_bias_max_sample_rad_per_s = 0.10;
  gp.use_imu_preint = false;
  gp.adaptive_noise_enabled_gain = 10.0;
  gp.adaptive_noise_ema_tau_s = 0.5;
  gp.adaptive_noise_residual_floor_rad = 0.005;
  return gp;
}

// Ground-truth motion: piecewise (v, w) script, serpentine with pivot turns.
struct Segment
{
  double duration;
  double v;
  double w;
};

std::vector<Segment> CoverageScript(double minutes, double park_end_minutes)
{
  std::vector<Segment> s;
  s.push_back({30.0, 0.0, 0.0});  // parked: stationary throttle + dock-like idle
  const double kRow = 20.0, kV = 0.4, kSpacing = 0.16, kPivotW = 0.6;
  double t = 30.0;
  int row = 0;
  while (t < minutes * 60.0)
  {
    s.push_back({kRow / kV, kV, 0.0});
    const double dir = (row % 2 == 0) ? 1.0 : -1.0;
    s.push_back({M_PI_2 / kPivotW, 0.0, dir * kPivotW});
    s.push_back({kSpacing / 0.2, 0.2, 0.0});
    s.push_back({M_PI_2 / kPivotW, 0.0, dir * kPivotW});
    t += kRow / kV + M_PI / kPivotW + kSpacing / 0.2;
    if (++row % 6 == 0)
    {
      s.push_back({8.0, 0.0, 0.0});  // operator / obstacle stop
      t += 8.0;
    }
  }
  // Return to the dock and sit there (charging): stationary throttle + 10 Hz
  // GNSS-only updates against an unchanged window.
  s.push_back({park_end_minutes * 60.0 + 1e6, 0.0, 0.0});
  return s;
}

enum class GnssMode
{
  kFixed,
  kFloat,
  kOutage
};

GnssMode ModeAt(double t)
{
  // 30-min cycle: 3 min Float starting at 10 min, 60 s outage at 22 min.
  const double c = std::fmod(t, 1800.0);
  if (c >= 600.0 && c < 780.0)
    return GnssMode::kFloat;
  if (c >= 1320.0 && c < 1380.0)
    return GnssMode::kOutage;
  return GnssMode::kFixed;
}

struct Record
{
  // kind 0 = Tick output, 1 = 1 Hz snapshot (+ stats), 2 = rebase marker
  double kind, t, node, revision, x, y, th;
  double cov[9];
  double slip_veto, hand_push, live_nodes, successful_updates, rebases;
};

long RssKb()
{
  std::ifstream f("/proc/self/statm");
  long pages = 0, rss = 0;
  f >> pages >> rss;
  return rss * (sysconf(_SC_PAGESIZE) / 1024);
}

int Compare(const char* a_path, const char* b_path)
{
  auto load = [](const char* p)
  {
    std::vector<Record> v;
    std::ifstream f(p, std::ios::binary);
    Record r;
    while (f.read(reinterpret_cast<char*>(&r), sizeof(r)))
      v.push_back(r);
    return v;
  };
  const auto a = load(a_path), b = load(b_path);
  if (a.size() != b.size())
  {
    std::printf("RECORD COUNT MISMATCH %zu vs %zu\n", a.size(), b.size());
    return 2;
  }
  double max_pos = 0, max_th = 0, max_cov_rel = 0, max_cov_abs = 0;
  size_t int_mismatch = 0, identical = 0;
  for (size_t i = 0; i < a.size(); ++i)
  {
    const auto &p = a[i], &q = b[i];
    if (p.kind != q.kind || p.node != q.node || p.revision != q.revision ||
        p.slip_veto != q.slip_veto || p.hand_push != q.hand_push || p.live_nodes != q.live_nodes ||
        p.rebases != q.rebases)
      ++int_mismatch;
    max_pos = std::max({max_pos, std::abs(p.x - q.x), std::abs(p.y - q.y)});
    max_th = std::max(max_th, std::abs(std::remainder(p.th - q.th, 2 * M_PI)));
    bool same = p.x == q.x && p.y == q.y && p.th == q.th;
    for (int k = 0; k < 9; ++k)
    {
      const double d = std::abs(p.cov[k] - q.cov[k]);
      same = same && d == 0.0;
      max_cov_abs = std::max(max_cov_abs, d);
      const double scale = std::sqrt(std::abs(p.cov[k / 3 * 4]) * std::abs(p.cov[k % 3 * 4]));
      if (scale > 0)
        max_cov_rel = std::max(max_cov_rel, d / scale);
    }
    identical += same;
  }
  std::printf(
      "records=%zu bit_identical=%zu integer_mismatches=%zu max|dpos|=%.3e m max|dth|=%.3e rad "
      "max|dcov|=%.3e max_rel_dcov=%.3e\n",
      a.size(),
      identical,
      int_mismatch,
      max_pos,
      max_th,
      max_cov_abs,
      max_cov_rel);
  return int_mismatch == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  double minutes = 30.0;
  double park_end_minutes = 0.0;
  double tick_jitter_us = 300.0;
  double lever_arm_x = 0.3;
  unsigned seed = 1;
  std::string dump;
  bool rebase = true;
  bool viz = true;
  for (int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    if (a == "--compare" && i + 2 < argc)
      return Compare(argv[i + 1], argv[i + 2]);
    if (a == "--minutes" && i + 1 < argc)
      minutes = std::atof(argv[++i]);
    else if (a == "--park-end-minutes" && i + 1 < argc)
      park_end_minutes = std::atof(argv[++i]);
    else if (a == "--tick-jitter-us" && i + 1 < argc)
      tick_jitter_us = std::atof(argv[++i]);
    else if (a == "--seed" && i + 1 < argc)
      seed = static_cast<unsigned>(std::atoi(argv[++i]));
    else if (a == "--dump" && i + 1 < argc)
      dump = argv[++i];
    else if (a == "--no-rebase")
      rebase = false;
    else if (a == "--no-viz")
      viz = false;
    else if (a == "--lever-arm-x" && i + 1 < argc)
      lever_arm_x = std::atof(argv[++i]);
  }

  const auto gp = ProductionParams(lever_arm_x);
  fg::GraphManager gm(gp);
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::mt19937_64 jitter_rng(seed + 1000);
  std::uniform_real_distribution<double> u01(0.0, 1.0);

  const auto script = CoverageScript(minutes, park_end_minutes);
  minutes += park_end_minutes;
  // Records stream straight to the dump file and timing goes into fixed-size
  // tables allocated up front, so RSS growth reflects GraphManager alone.
  std::ofstream dump_f;
  if (!dump.empty())
    dump_f.open(dump, std::ios::binary);
  auto emit = [&](const Record& r)
  {
    if (dump_f)
      dump_f.write(reinterpret_cast<const char*>(&r), sizeof(r));
  };

  // Integer microsecond clock so event cadences are exact.
  const int64_t kEnd = static_cast<int64_t>(minutes * 60e6);
  const int64_t kImuUs = 10989, kWheelUs = 20000, kGnssUs = 100000, kCogUs = 200000,
                kTickUs = 40000, kVizUs = 1000000, kMaintUs = 30000000, kGnssLagUs = 80000;
  const double kGyroBias = 0.004;

  // Ground-truth state.
  double gx = 2.0, gy = 3.0, gth = 0.3;
  size_t seg = 0;
  double seg_t = 0.0;
  // History of truth for delayed GNSS stamps (antenna at stamp).
  struct Truth
  {
    int64_t us;
    double x, y, th, v, w;
  };
  // Ring buffer of 1 ms truth samples (covers the GNSS stamp lag).
  constexpr size_t kHist = 1024;
  std::vector<Truth> hist(kHist);

  double t_tick = 0, t_rebase = 0, t_viz = 0, t_inputs = 0;
  // Node-tick latency histogram (1 us bins up to 200 ms) and per-second sums.
  constexpr size_t kHistBins = 200000;
  std::vector<uint32_t> tick_hist(kHistBins + 1, 0);
  double tick_max_ms = 0.0;
  const size_t seconds = static_cast<size_t>(minutes * 60.0) + 2;
  std::vector<double> sec_sum(seconds, 0.0);
  std::vector<uint32_t> sec_n(seconds, 0);
  std::vector<double> rebase_times;
  size_t nodes = 0, rebases = 0;
  long rss_start = 0, rss_peak = 0;
  std::vector<std::pair<double, long>> rss_trace;
  rss_trace.reserve(static_cast<size_t>(minutes) + 2);
  uint64_t token = 0;
  bool initialized = false;

  using clk = std::chrono::steady_clock;
  auto timed = [](double& acc, auto&& fn)
  {
    const auto t0 = clk::now();
    fn();
    acc += std::chrono::duration<double>(clk::now() - t0).count();
  };

  int64_t last_wheel = -1, last_imu = -1;
  for (int64_t us = 0; us <= kEnd; us += 1000)
  {
    const double t = us * 1e-6;
    // Advance truth by 1 ms.
    double v = 0, w = 0;
    while (seg < script.size() && seg_t >= script[seg].duration)
    {
      seg_t -= script[seg].duration;
      ++seg;
    }
    if (seg < script.size())
    {
      v = script[seg].v;
      w = script[seg].w;
    }
    gx += v * std::cos(gth) * 1e-3;
    gy += v * std::sin(gth) * 1e-3;
    gth += w * 1e-3;
    seg_t += 1e-3;
    hist[static_cast<size_t>(us / 1000) % kHist] = {us, gx, gy, gth, v, w};

    if (us % kWheelUs == 0)
    {
      if (last_wheel >= 0)
      {
        // Encoder noise; pivots leak phantom forward speed (field-observed).
        // Encoders read exactly zero at rest; draw noise unconditionally so the
        // random stream does not depend on the motion script.
        const double nv1 = n01(rng), nv2 = n01(rng), nw = n01(rng);
        const bool still = v == 0.0 && w == 0.0;
        const double vm =
            still ? 0.0 : v * (1.0 + 0.01 * nv1) + 0.002 * nv2 + (std::abs(w) > 0.3 ? 0.02 : 0.0);
        const double wm = still ? 0.0 : w * 1.03 + 0.01 * nw;
        timed(t_inputs,
              [&]
              {
                gm.AddWheelTwist(vm, 0.0, wm, (us - last_wheel) * 1e-6);
              });
      }
      last_wheel = us;
    }
    if (us % kImuUs == 0)
    {
      if (last_imu >= 0)
      {
        const double gz = w + kGyroBias + 0.005 * n01(rng);
        timed(t_inputs,
              [&]
              {
                gm.AddGyroDelta(gz, (us - last_imu) * 1e-6);
              });
      }
      last_imu = us;
    }
    if (us % kGnssUs == 0 && us >= kGnssLagUs)
    {
      const auto mode = ModeAt(t);
      if (mode != GnssMode::kOutage)
      {
        const Truth& s = hist[static_cast<size_t>((us - kGnssLagUs) / 1000) % kHist];
        const double noise = mode == GnssMode::kFixed ? 0.008 : 0.15;
        const double sigma = mode == GnssMode::kFixed ? 0.014 : 0.30;
        const double ax = s.x + lever_arm_x * std::cos(s.th) + noise * n01(rng);
        const double ay = s.y + lever_arm_x * std::sin(s.th) + noise * n01(rng);
        const double stamp = s.us * 1e-6;
        timed(t_inputs,
              [&]
              {
                if (!initialized)
                {
                  gm.Initialize(gtsam::Pose2(ax, ay, s.th), t, 0.02);
                  initialized = true;
                  return;
                }
                const auto node = gm.FindNodeAtOrBefore(stamp);
                if (node)
                  gm.QueueGnss(ax, ay, sigma, /*robust=*/true, node, ++token);
              });
      }
    }
    if (us % kCogUs == 0 && initialized && ModeAt(t) == GnssMode::kFixed && v > 0.08 &&
        std::abs(w) < 1e-6)
    {
      const double yaw = gth + 0.05 * n01(rng);
      timed(t_inputs,
            [&]
            {
              gm.QueueYaw(yaw, 0.15);
            });
    }
    if (us % kTickUs == 0)
    {
      // rclcpp wall timers fire on a drift-free schedule plus a small, varying
      // callback latency; the node passes now() to Tick.
      const double now_s = t + tick_jitter_us * 1e-6 * u01(jitter_rng);
      const auto t0 = clk::now();
      const auto r = gm.Tick(now_s);
      const double dt = std::chrono::duration<double>(clk::now() - t0).count();
      t_tick += dt;
      if (r)
      {
        ++nodes;
        ++tick_hist[std::min(kHistBins, static_cast<size_t>(dt * 1e6))];
        tick_max_ms = std::max(tick_max_ms, dt * 1e3);
        sec_sum[static_cast<size_t>(t)] += dt * 1e3;
        ++sec_n[static_cast<size_t>(t)];
        Record rec{};
        rec.kind = 0;
        rec.t = now_s;
        rec.node = static_cast<double>(r->node_index);
        rec.revision = static_cast<double>(r->estimate_revision);
        rec.x = r->pose.x();
        rec.y = r->pose.y();
        rec.th = r->pose.theta();
        std::memcpy(rec.cov, r->covariance.data(), sizeof(rec.cov));
        emit(rec);
      }
    }
    if (viz && us % kVizUs == 0 && initialized)
    {
      Record rec{};
      timed(t_viz,
            [&]
            {
              const auto st = gm.Stats();
              const auto snap = gm.LatestSnapshot();
              rec.live_nodes = static_cast<double>(gm.LiveNodeCount());
              const auto poses = gm.GetAllPoses();
              rec.kind = 1;
              rec.t = t;
              rec.slip_veto = static_cast<double>(st.slip_veto);
              rec.hand_push = static_cast<double>(st.stationary_hand_push);
              rec.successful_updates = static_cast<double>(st.successful_updates);
              rec.rebases = static_cast<double>(st.successful_rebases);
              if (snap)
              {
                rec.node = static_cast<double>(snap->node_index);
                rec.revision = static_cast<double>(snap->estimate_revision);
                rec.x = snap->pose.x();
                rec.y = snap->pose.y();
                rec.th = snap->pose.theta();
                std::memcpy(rec.cov, snap->covariance.data(), sizeof(rec.cov));
              }
              // Fold a cheap checksum of the full trajectory into the record.
              double sx = 0, sy = 0;
              for (const auto& [idx, p] : poses)
              {
                sx += p.x();
                sy += p.y();
              }
              rec.cov[1] = sx;
              rec.cov[3] = sy;
            });
      emit(rec);
      if (us % 60000000 == 0)
      {
        const long rss = RssKb();
        if (rss_start == 0)
          rss_start = rss;
        rss_trace.emplace_back(t / 60.0, rss);
      }
    }
    if (rebase && us % kMaintUs == 0 && us > 0 && gm.RebaseDue(2000))
    {
      timed(t_rebase,
            [&]
            {
              gm.RebaseISAM2();
            });
      ++rebases;
      rebase_times.push_back(t);
      Record rec{};
      rec.kind = 2;
      rec.t = t;
      emit(rec);
    }
  }

  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  rss_peak = ru.ru_maxrss;
  auto pct = [&](double p)
  {
    if (p >= 1.0)
      return tick_max_ms;
    const double target = p * static_cast<double>(nodes);
    double acc = 0;
    for (size_t i = 0; i <= kHistBins; ++i)
    {
      acc += tick_hist[i];
      if (acc >= target)
        return (static_cast<double>(i) + 0.5) * 1e-3;
    }
    return tick_max_ms;
  };
  const double session_s = minutes * 60.0;
  std::printf(
      "[bench] minutes=%.1f nodes=%zu rebases=%zu | tick %.3f s (%.3f%% of 1 core) "
      "node p50=%.3f p95=%.3f p99=%.3f max=%.3f ms | rebase %.3f s | viz %.3f s | inputs %.3f s | "
      "total %.3f s = %.3f%% core\n",
      minutes,
      nodes,
      rebases,
      t_tick,
      100.0 * t_tick / session_s,
      pct(0.5),
      pct(0.95),
      pct(0.99),
      pct(1.0),
      t_rebase,
      t_viz,
      t_inputs,
      t_tick + t_rebase + t_viz + t_inputs,
      100.0 * (t_tick + t_rebase + t_viz + t_inputs) / session_s);
  std::printf("[bench] rss_start=%ld kB end=%ld kB peak=%ld kB", rss_start, RssKb(), rss_peak);
  if (rss_trace.size() >= 2)
  {
    const auto& a = rss_trace[rss_trace.size() / 2];
    const auto& b = rss_trace.back();
    std::printf(" growth(2nd half)=%.1f kB/h",
                (b.second - a.second) / ((b.first - a.first) / 60.0));
  }
  std::printf("\n");
  // Mean node-tick cost in 5-minute blocks, and in the 60 s windows either side
  // of the first rebase (isolates the effect of the rebuilt graph's shape).
  {
    auto mean_in = [&](double a, double b)
    {
      double s = 0;
      size_t n = 0;
      for (size_t i = static_cast<size_t>(std::max(0.0, a)); i < std::min<size_t>(seconds, b); ++i)
      {
        s += sec_sum[i];
        n += sec_n[i];
      }
      return n ? s / n : 0.0;
    };
    std::printf("[bench] mean node tick ms per 5 min:");
    for (double a = 0; a < session_s; a += 300)
      std::printf(" %.3f", mean_in(a, a + 300));
    std::printf("\n");
    if (!rebase_times.empty())
      std::printf(
          "[bench] first rebase at %.0f s: mean tick 60 s before=%.3f ms, 60 s after=%.3f ms\n",
          rebase_times[0],
          mean_in(rebase_times[0] - 60, rebase_times[0]),
          mean_in(rebase_times[0], rebase_times[0] + 60));
  }
  return 0;
}
