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
//   * wheel odometry 50 Hz, gyro ~91 Hz (11 ms; bias + noise),
//   * GNSS 10 Hz with an 80 ms measurement-stamp lag (QueueGnss targets
//     FindNodeAtOrBefore(stamp), as OnGnss does), RTK-Fixed with periodic
//     RTK-Float windows and complete outages,
//   * COG yaw at 5 Hz while RTK-Fixed and driving straight,
//   * the 1 Hz diagnostics/markers consumers (Stats, LiveNodeCount,
//     GetAllPoses) and the 30 s maintenance rebase (run synchronously here;
//     production runs it on a detached thread, its CPU still counts).
//
// Every Tick output and a 1 Hz snapshot are written to --dump so two builds
// can be compared (--compare A B): it prints the largest pose / covariance
// differences and exits 0 only when every record is byte-identical.
//
// --faults SEED adds a seeded fault stream on top (off by default, so the
// clean stream and its dumps are unchanged). GNSS then goes through the same
// gates OnGnss applies, in the same order (value-change tracking, wrong-fix
// jump, unknown/zero sigma, receipt node, stuck value, future stamp), and the
// stream carries: wrong fixes, a stuck receiver, extra outages, zero/NaN/huge
// sigma, stamps minutes in the past (factors on deep window poses) or in the
// future, wheel slip, hand pushes, wheel/IMU dropouts, gyro spikes, timer
// stalls, backward and repeated Tick stamps, LiDAR map XY and mag yaw factors,
// and the lifecycle calls (extra and back-to-back rebases, dock re-pin
// RigidTransformAll, ForceAnchor on the newest and on deep nodes,
// Save -> Reset -> Load, clear_graph Reset, InvalidateGnssSupport). --abuse
// additionally feeds NaN/Inf straight into GraphManager past the node's
// filters. --async-rebase runs the maintenance rebase on a thread as
// production does (timing-dependent, so its dumps are not comparable).
// --pose-hash adds an exact 1 Hz fingerprint of every live pose to the dump.
//
//   bench_graph_session --minutes 120 --dump out.bin
//   bench_graph_session --minutes 20 --window 1500 --faults 7 --dump f.bin
//   bench_graph_session --compare a.bin b.bin

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "fusion_graph/gps_stuck_gate.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/rtk_wrongfix_gate.hpp"
#include <sys/resource.h>
#include <unistd.h>

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
  // kind 0 = Tick output, 1 = 1 Hz snapshot (+ stats), 2 = rebase marker,
  // 3 = 1 Hz counters (fault runs), 4 = injected lifecycle call (fault runs),
  // 5 = 1 Hz hash of every pose's bits (--pose-hash, implied by --faults)
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
  // An unreadable or truncated dump is an error, never an empty match.
  auto load = [](const char* p, std::vector<Record>& v)
  {
    std::ifstream f(p, std::ios::binary);
    if (!f)
    {
      std::printf("CANNOT OPEN %s\n", p);
      return false;
    }
    Record r;
    while (f.read(reinterpret_cast<char*>(&r), sizeof(r)))
      v.push_back(r);
    if (f.gcount() != 0 || !f.eof())
    {
      std::printf("TRUNCATED OR UNREADABLE %s\n", p);
      return false;
    }
    return true;
  };
  std::vector<Record> a, b;
  if (!load(a_path, a) || !load(b_path, b))
    return 3;
  if (a.size() != b.size())
  {
    std::printf("RECORD COUNT MISMATCH %zu vs %zu\n", a.size(), b.size());
    return 2;
  }
  double max_pos = 0, max_th = 0, max_cov_rel = 0, max_cov_abs = 0;
  size_t int_mismatch = 0, identical = 0, bitwise = 0, first_diff = a.size();
  for (size_t i = 0; i < a.size(); ++i)
  {
    const auto &p = a[i], &q = b[i];
    // Byte comparison of the whole record: also equal for identical NaNs.
    if (std::memcmp(&p, &q, sizeof(Record)) == 0)
      ++bitwise;
    else if (first_diff == a.size())
      first_diff = i;
    // Bitwise: kind-5 records carry a hash in `node`, which may be a NaN pattern.
    auto differs = [](double u, double w)
    {
      return std::memcmp(&u, &w, sizeof(u)) != 0;
    };
    if (differs(p.kind, q.kind) || differs(p.node, q.node) || differs(p.revision, q.revision) ||
        differs(p.slip_veto, q.slip_veto) || differs(p.hand_push, q.hand_push) ||
        differs(p.live_nodes, q.live_nodes) || differs(p.rebases, q.rebases))
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
  std::printf("records_bitwise_identical=%zu/%zu", bitwise, a.size());
  if (first_diff < a.size())
    std::printf(" first_difference=record %zu (kind %.0f, t=%.3f)",
                first_diff,
                a[first_diff].kind,
                a[first_diff].t);
  std::printf("\n");
  return bitwise == a.size() ? 0 : 1;
}

// Seeded fault stream. It owns its own generator and every draw happens
// unconditionally at a fixed point of the event schedule, so the faults are
// the same for any two builds that produce the same outputs.
class Faults
{
public:
  explicit Faults(unsigned seed) : rng_(seed * 7919ULL + 13)
  {
  }
  bool Chance(double p)
  {
    return u_(rng_) < p;
  }
  double Uniform(double a, double b)
  {
    return a + (b - a) * u_(rng_);
  }

private:
  std::mt19937_64 rng_;
  std::uniform_real_distribution<double> u_{0.0, 1.0};
};

// A fault that stays active until a deadline on the microsecond clock.
struct Window
{
  int64_t until_us = -1;
  bool Active(int64_t us) const
  {
    return us < until_us;
  }
  void Start(int64_t us, double seconds)
  {
    until_us = us + static_cast<int64_t>(seconds * 1e6);
  }
};

// OnGnss's per-fix state for the wrong-fix and stuck-value gates
// (fusion_graph_node_callbacks_a.cpp), fed from the same wheel/gyro inputs.
struct GnssGateState
{
  bool have_last_xy = false;
  double last_x = 0, last_y = 0;
  double last_reported_x = std::numeric_limits<double>::quiet_NaN();
  double last_reported_y = std::numeric_limits<double>::quiet_NaN();
  double wheel_dist_since_last_gps_m = 0, abs_dtheta_since_last_gps_rad = 0;
  double wheel_dist_since_value_changed_m = 0, abs_dtheta_since_last_sample_rad = 0;
  uint64_t dropped_no_receipt_node = 0, dropped_future = 0, dropped_no_node = 0, queue_refused = 0,
           queued = 0;
};

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
  bool faults_on = false, abuse = false, async_rebase = false, pose_hash = false;
  unsigned fault_seed = 0;
  long window = -1, rebase_budget = 2000;
  std::string scratch = std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp";
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
    else if (a == "--faults" && i + 1 < argc)
    {
      faults_on = true;
      fault_seed = static_cast<unsigned>(std::atoi(argv[++i]));
    }
    else if (a == "--abuse")
      abuse = true;
    else if (a == "--pose-hash")
      pose_hash = true;
    else if (a == "--async-rebase")
      async_rebase = true;
    else if (a == "--window" && i + 1 < argc)
      window = std::atol(argv[++i]);
    else if (a == "--rebase-budget" && i + 1 < argc)
      rebase_budget = std::atol(argv[++i]);
    else if (a == "--scratch" && i + 1 < argc)
      scratch = argv[++i];
  }

  pose_hash = pose_hash || faults_on;
  auto gp = ProductionParams(lever_arm_x);
  if (window >= 0)
    gp.max_graph_nodes = static_cast<decltype(gp.max_graph_nodes)>(window);
  fg::GraphManager gm(gp);
  Faults fx(fault_seed);
  GnssGateState gate;
  // fusion_graph.yaml values of the OnGnss gates.
  constexpr double kWrongFixMaxJumpM = 0.08, kStuckMinWheelDistM = 1.0, kStuckMaxYawRad = 1.047,
                   kFutureStampToleranceS = 0.5, kMotionIntegrationGapS = 1.0;
  const double lever_arm_radius = std::hypot(gp.lever_arm_x, gp.lever_arm_y);
  Window stuck, outage, slip, hand_push, wheel_drop, imu_drop, stall;
  double slip_rate = 0.0;
  const std::string ckpt = scratch + "/bench_graph_session_" + std::to_string(getpid());
  std::thread rebase_thread;
  // Pose error against truth while RTK-Fixed (robustness summary; not in the dump).
  double max_fixed_err = 0.0, sum_fixed_err = 0.0;
  size_t n_fixed_err = 0, unrequested_resets = 0;
  bool requested_reset = false;
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

  // Integer microsecond clock so event cadences are exact. It steps 1 ms, so
  // every period must be a whole number of milliseconds or it never fires.
  const int64_t kEnd = static_cast<int64_t>(minutes * 60e6);
  constexpr int64_t kStepUs = 1000;
  constexpr int64_t kImuUs = 11000, kWheelUs = 20000, kGnssUs = 100000, kCogUs = 200000,
                    kTickUs = 40000, kVizUs = 1000000, kMaintUs = 30000000, kGnssLagUs = 80000;
  static_assert(kImuUs % kStepUs == 0 && kWheelUs % kStepUs == 0 && kGnssUs % kStepUs == 0 &&
                kCogUs % kStepUs == 0 && kTickUs % kStepUs == 0 && kVizUs % kStepUs == 0 &&
                kMaintUs % kStepUs == 0 && kGnssLagUs % kStepUs == 0);
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
  // Coarser 100 ms ring (~13.6 min) for fault stamps deep in the window.
  constexpr size_t kDeepHist = 8192;
  std::vector<Truth> deep_hist(faults_on ? kDeepHist : 0);
  auto truth_at = [&](int64_t at_us) -> const Truth&
  {
    const Truth& fine = hist[static_cast<size_t>(at_us / 1000) % kHist];
    if (fine.us == at_us || deep_hist.empty())
      return fine;
    const Truth& coarse = deep_hist[static_cast<size_t>(at_us / 100000) % kDeepHist];
    return coarse.us == at_us / 100000 * 100000 ? coarse : fine;
  };

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
  for (int64_t us = 0; us <= kEnd; us += kStepUs)
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
    if (faults_on && us % 100000 == 0)
      deep_hist[static_cast<size_t>(us / 100000) % kDeepHist] = {us, gx, gy, gth, v, w};

    if (faults_on && us % 1000000 == 0)
    {
      // Start new fault windows (draws are unconditional; a window already
      // running is simply extended or restarted).
      const double d_stuck = fx.Uniform(5, 40), d_out = fx.Uniform(1, 120),
                   d_slip = fx.Uniform(0.5, 3), r_slip = fx.Uniform(-1.0, 1.0),
                   d_push = fx.Uniform(1, 5), d_wdrop = fx.Uniform(0.1, 2.5),
                   d_idrop = fx.Uniform(0.1, 2.5), d_stall = fx.Uniform(0.2, 5);
      if (fx.Chance(0.004))
        stuck.Start(us, d_stuck);
      if (fx.Chance(0.002))
        outage.Start(us, d_out);
      if (fx.Chance(0.004))
      {
        slip.Start(us, d_slip);
        slip_rate = r_slip < 0 ? r_slip - 0.3 : r_slip + 0.3;
      }
      if (fx.Chance(0.002))
        hand_push.Start(us, d_push);
      if (fx.Chance(0.003))
        wheel_drop.Start(us, d_wdrop);
      if (fx.Chance(0.003))
        imu_drop.Start(us, d_idrop);
      if (fx.Chance(0.003))
        stall.Start(us, d_stall);
    }

    if (us % kWheelUs == 0)
    {
      // Fault draws first, unconditionally, so they never depend on state.
      const bool w_nan = faults_on && fx.Chance(abuse ? 0.0005 : 0.0);
      const bool dropped = faults_on && wheel_drop.Active(us);
      if (last_wheel >= 0 && !dropped)
      {
        // Encoder noise; pivots leak phantom forward speed (field-observed).
        // Encoders read exactly zero at rest; draw noise unconditionally so the
        // random stream does not depend on the motion script.
        const double nv1 = n01(rng), nv2 = n01(rng), nw = n01(rng);
        const bool still = v == 0.0 && w == 0.0;
        double vm =
            still ? 0.0 : v * (1.0 + 0.01 * nv1) + 0.002 * nv2 + (std::abs(w) > 0.3 ? 0.02 : 0.0);
        double wm = still ? 0.0 : w * 1.03 + 0.01 * nw;
        const double dt = (us - last_wheel) * 1e-6;
        if (faults_on)
        {
          if (slip.Active(us))
            wm = slip_rate;  // wheels claim a turn the gyro does not see
          if (hand_push.Active(us))
            vm = wm = 0.0;  // chassis moved by hand, encoders still
          if (w_nan)
            vm = std::numeric_limits<double>::quiet_NaN();
        }
        // OnWheelOdom drops non-finite twists and integrates only gaps < 1 s.
        const bool node_accepts =
            !faults_on || abuse || (std::isfinite(vm) && dt < kMotionIntegrationGapS);
        if (node_accepts)
        {
          timed(t_inputs,
                [&]
                {
                  gm.AddWheelTwist(vm, 0.0, wm, dt);
                });
          gate.wheel_dist_since_last_gps_m += std::abs(vm) * dt;
          gate.wheel_dist_since_value_changed_m += std::abs(vm) * dt;
        }
      }
      if (!dropped)
        last_wheel = us;
    }
    if (us % kImuUs == 0)
    {
      const bool spike = faults_on && fx.Chance(0.001);
      const double spike_rate = faults_on ? fx.Uniform(-3, 3) : 0.0;
      const bool g_inf = faults_on && fx.Chance(abuse ? 0.0005 : 0.0);
      const bool dropped = faults_on && imu_drop.Active(us);
      if (last_imu >= 0 && !dropped)
      {
        double gz = w + kGyroBias + 0.005 * n01(rng);
        const double dt = (us - last_imu) * 1e-6;
        if (spike)
          gz += spike_rate;
        if (g_inf)
          gz = std::numeric_limits<double>::infinity();
        const bool node_accepts =
            !faults_on || abuse || (std::isfinite(gz) && dt < kMotionIntegrationGapS);
        if (node_accepts)
        {
          timed(t_inputs,
                [&]
                {
                  gm.AddGyroDelta(gz, dt);
                });
          gate.abs_dtheta_since_last_gps_rad += std::abs(gz) * dt;
          gate.abs_dtheta_since_last_sample_rad += std::abs(gz) * dt;
        }
      }
      if (!dropped)
        last_imu = us;
    }
    if (faults_on && us % kGnssUs == 0 && us >= kGnssLagUs)
    {
      // Draw everything first, then run OnGnss's gate sequence.
      const bool deep = fx.Chance(0.02), future = fx.Chance(0.002), wrong = fx.Chance(0.01),
                 sig_zero = fx.Chance(0.003), sig_nan = fx.Chance(0.002),
                 sig_huge = fx.Chance(0.01), raw_nan = fx.Chance(abuse ? 0.002 : 0.0);
      const double deep_lag_s = fx.Uniform(0.1, 600), future_s = fx.Uniform(0.6, 3),
                   wrong_m = fx.Uniform(0.05, 3), wrong_dir = fx.Uniform(-M_PI, M_PI),
                   huge_sigma = fx.Uniform(0.5, 5);
      const double nx = n01(rng), ny = n01(rng);
      auto mode = ModeAt(t);
      if (outage.Active(us))
        mode = GnssMode::kOutage;
      int64_t stamp_us = us - kGnssLagUs;
      if (deep)
        stamp_us =
            std::max<int64_t>(0, us - static_cast<int64_t>(deep_lag_s * 1e6)) / 100000 * 100000;
      if (future)
        stamp_us = us + static_cast<int64_t>(future_s * 1e6);
      if (mode != GnssMode::kOutage)
      {
        const Truth& s = truth_at(std::min(stamp_us, us));
        const double noise = mode == GnssMode::kFixed ? 0.008 : 0.15;
        double sigma = mode == GnssMode::kFixed ? 0.014 : 0.30;
        double ax = s.x + lever_arm_x * std::cos(s.th) + noise * nx;
        double ay = s.y + lever_arm_x * std::sin(s.th) + noise * ny;
        if (wrong)
        {
          ax += wrong_m * std::cos(wrong_dir);
          ay += wrong_m * std::sin(wrong_dir);
        }
        if (stuck.Active(us) && std::isfinite(gate.last_reported_x))
        {
          ax = gate.last_reported_x;  // receiver repeats its last solution
          ay = gate.last_reported_y;
        }
        if (sig_zero)
          sigma = 0.0;
        if (sig_nan)
          sigma = std::numeric_limits<double>::quiet_NaN();
        if (sig_huge)
          sigma = huge_sigma;
        const double stamp = stamp_us * 1e-6;
        timed(t_inputs,
              [&]
              {
                if (fg::GpsValueChanged(ax, ay, gate.last_reported_x, gate.last_reported_y))
                  gate.wheel_dist_since_value_changed_m = 0.0;
                gate.last_reported_x = ax;
                gate.last_reported_y = ay;
                if (!initialized)
                {
                  if (std::isfinite(sigma) && sigma > 0.0 && !deep && !future)
                  {
                    gm.Initialize(gtsam::Pose2(ax, ay, s.th), t, 0.02);
                    initialized = true;
                    requested_reset = false;
                  }
                  return;
                }
                if (raw_nan)
                {
                  // Past every node-side filter: a NaN position reaches the graph.
                  gm.QueueGnss(std::numeric_limits<double>::quiet_NaN(), ay, 0.014, true);
                  return;
                }
                if (gate.have_last_xy &&
                    fg::GpsJumpImplausible(std::hypot(ax - gate.last_x, ay - gate.last_y),
                                           kWrongFixMaxJumpM,
                                           lever_arm_radius,
                                           gate.abs_dtheta_since_last_gps_rad,
                                           gate.wheel_dist_since_last_gps_m))
                {
                  gm.RecordGpsRejectWrongFix();
                  gate.last_x = ax;
                  gate.last_y = ay;
                  fg::ResetRtkWrongFixAccumulators(gate.wheel_dist_since_last_gps_m,
                                                   gate.abs_dtheta_since_last_gps_rad);
                  return;
                }
                gate.have_last_xy = true;
                gate.last_x = ax;
                gate.last_y = ay;
                fg::ResetRtkWrongFixAccumulators(gate.wheel_dist_since_last_gps_m,
                                                 gate.abs_dtheta_since_last_gps_rad);
                if (!std::isfinite(sigma) || sigma <= 0.0)
                {
                  gm.RecordGpsRejectWrongFix();
                  return;
                }
                if (!gm.FindNodeAtOrBefore(stamp))
                {
                  ++gate.dropped_no_receipt_node;
                  return;
                }
                const bool stuck_value =
                    fg::GpsStuckImplausible(gate.wheel_dist_since_value_changed_m,
                                            gate.abs_dtheta_since_last_sample_rad,
                                            kStuckMinWheelDistM,
                                            kStuckMaxYawRad);
                gate.abs_dtheta_since_last_sample_rad = 0.0;
                if (stuck_value)
                {
                  gm.RecordGpsRejectStuckValue();
                  return;
                }
                if (stamp - t > kFutureStampToleranceS)
                {
                  ++gate.dropped_future;
                  return;
                }
                const auto node = gm.FindNodeAtOrBefore(stamp);
                if (!node)
                {
                  ++gate.dropped_no_node;
                  return;
                }
                if (gm.QueueGnss(ax, ay, sigma, /*robust=*/true, node, ++token))
                  ++gate.queued;
                else
                  ++gate.queue_refused;
              });
      }
    }
    if (!faults_on && us % kGnssUs == 0 && us >= kGnssLagUs)
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
        std::abs(w) < 1e-6 && !(faults_on && outage.Active(us)))
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
      double now_s = t + tick_jitter_us * 1e-6 * u01(jitter_rng);
      const bool backward = faults_on && fx.Chance(0.001);
      const double back_s = faults_on ? fx.Uniform(0, 1) : 0.0;
      const bool repeat = faults_on && fx.Chance(0.002);
      if (backward)
        now_s -= back_s;
      auto tick = [&]
      {
        const auto t0 = clk::now();
        const auto r = gm.Tick(now_s);
        const double dt = std::chrono::duration<double>(clk::now() - t0).count();
        t_tick += dt;
        if (initialized && !gm.IsInitialized())
        {
          // iSAM2 failure recovery (ResetLocked): re-seed from the next fix.
          unrequested_resets += requested_reset ? 0 : 1;
          initialized = false;
        }
        if (!r)
          return;
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
        if (ModeAt(t) == GnssMode::kFixed && !(faults_on && outage.Active(us)))
        {
          const double err = std::hypot(r->pose.x() - gx, r->pose.y() - gy);
          max_fixed_err = std::max(max_fixed_err, err);
          sum_fixed_err += err;
          ++n_fixed_err;
        }
      };
      if (!(faults_on && stall.Active(us)))
      {
        tick();
        if (repeat)
          tick();
      }
    }
    if (viz && us % kVizUs == 0 && initialized)
    {
      Record rec{}, hash_rec{};
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
              if (pose_hash)
              {
                // Exact fingerprint of every pose's bits (a sum would absorb
                // a last-bit change in one deep pose).
                uint64_t h = 1469598103934665603ULL;
                auto mix = [&h](const void* p, size_t n)
                {
                  const auto* b = static_cast<const unsigned char*>(p);
                  for (size_t i = 0; i < n; ++i)
                    h = (h ^ b[i]) * 1099511628211ULL;
                };
                for (const auto& [idx, p] : poses)
                {
                  const double xyz[] = {p.x(), p.y(), p.theta()};
                  mix(&idx, sizeof(idx));
                  mix(xyz, sizeof(xyz));
                }
                hash_rec.kind = 5;
                hash_rec.t = t;
                std::memcpy(&hash_rec.node, &h, sizeof(h));
                hash_rec.live_nodes = static_cast<double>(poses.size());
              }
            });
      emit(rec);
      if (pose_hash)
        emit(hash_rec);
      if (faults_on)
      {
        // Every counter the diagnostics topic and the gates expose.
        const auto st = gm.Stats();
        Record c{};
        c.kind = 3;
        c.t = t;
        const double v3[] = {static_cast<double>(st.total_nodes),
                             static_cast<double>(st.successful_updates),
                             static_cast<double>(st.updates_since_rebase),
                             static_cast<double>(st.successful_rebases),
                             static_cast<double>(st.factor_slots),
                             static_cast<double>(st.live_factors),
                             static_cast<double>(st.gps_rejects_wrongfix),
                             static_cast<double>(st.gps_rejects_stuck_value),
                             static_cast<double>(st.stationary_hand_push),
                             static_cast<double>(st.slip_veto),
                             st.residual_ema_rad,
                             st.wheel_sigma_x_eff,
                             st.gyro_bias_z,
                             static_cast<double>(st.gyro_bias_updates),
                             static_cast<double>(gm.LidarAnchorFactorCount()),
                             static_cast<double>(gm.GraphEpoch()),
                             static_cast<double>(gm.IsInitialized()),
                             static_cast<double>(gate.queued + 1000 * gate.queue_refused),
                             static_cast<double>(gate.dropped_no_receipt_node +
                                                 1000 * gate.dropped_future +
                                                 1000000 * gate.dropped_no_node)};
        static_assert(sizeof(v3) == sizeof(double) * 19);
        c.node = v3[0];
        c.revision = v3[1];
        c.x = v3[2];
        c.y = v3[3];
        c.th = v3[4];
        std::memcpy(c.cov, v3 + 5, sizeof(c.cov));
        c.slip_veto = v3[14];
        c.hand_push = v3[15];
        c.live_nodes = v3[16];
        c.successful_updates = v3[17];
        c.rebases = v3[18];
        emit(c);
      }
      if (us % 60000000 == 0)
      {
        const long rss = RssKb();
        if (rss_start == 0)
          rss_start = rss;
        rss_trace.emplace_back(t / 60.0, rss);
      }
    }
    if (rebase && us % kMaintUs == 0 && us > 0 &&
        gm.RebaseDue(static_cast<uint64_t>(rebase_budget)))
    {
      if (async_rebase)
      {
        // Production runs the rebase on a detached maintenance thread while
        // Tick keeps going (phase-3 replay); join the previous one first.
        if (rebase_thread.joinable())
          rebase_thread.join();
        rebase_thread = std::thread(
            [&gm]
            {
              gm.RebaseISAM2();
            });
      }
      else
      {
        timed(t_rebase,
              [&]
              {
                gm.RebaseISAM2();
              });
      }
      ++rebases;
      rebase_times.push_back(t);
      Record rec{};
      rec.kind = 2;
      rec.t = t;
      emit(rec);
    }
    if (faults_on && us % 1000000 == 500000)
    {
      // Lifecycle calls the node makes outside the steady stream. Every draw
      // is made each second; the op is applied only while initialized.
      const bool op_rebase = fx.Chance(0.01), op_double = fx.Chance(0.002),
                 op_rigid = fx.Chance(0.003), op_anchor = fx.Chance(0.005),
                 op_deep_anchor = fx.Chance(0.002), op_reload = fx.Chance(0.002),
                 op_reset = fx.Chance(0.001), op_invalidate = fx.Chance(0.005),
                 op_lidar = fx.Chance(0.1), op_mag = fx.Chance(0.05);
      const double rx = fx.Uniform(-0.5, 0.5), ry = fx.Uniform(-0.5, 0.5),
                   rth = fx.Uniform(-0.1, 0.1), deep_s = fx.Uniform(1, 200),
                   lidar_s = fx.Uniform(0, 1), lx = fx.Uniform(-0.05, 0.05),
                   ly = fx.Uniform(-0.05, 0.05), mag_bias = fx.Uniform(-0.2, 0.2);
      auto mark = [&](int op)
      {
        Record rec{};
        rec.kind = 4;
        rec.t = t;
        rec.node = op;
        rec.revision = gm.IsInitialized() ? 1 : 0;
        emit(rec);
      };
      if (initialized && !(async_rebase && rebase_thread.joinable()))
      {
        if (op_rebase || op_double)
        {
          gm.RebaseISAM2();
          if (op_double)
            gm.RebaseISAM2();
          mark(op_double ? 2 : 1);
        }
        if (op_rigid)
        {
          gm.RigidTransformAll(gtsam::Pose2(rx, ry, rth), 0.005, 0.005);
          mark(3);
        }
        if (op_anchor)
        {
          if (const auto snap = gm.LatestSnapshot())
            gm.ForceAnchor(snap->node_index, gtsam::Pose2(gx, gy, gth), 0.03, 0.035);
          mark(4);
        }
        if (op_deep_anchor)
        {
          if (const auto node = gm.FindNodeAtOrBefore(t - deep_s))
            if (const auto pose = gm.GetPose(*node))
              gm.ForceAnchor(*node,
                             gtsam::Pose2(pose->x() + lx, pose->y() + ly, pose->theta()),
                             0.05,
                             0.05);
          mark(5);
        }
        if (op_reload)
        {
          // Restart with autoload: checkpoint, wipe, load it back.
          const bool saved = gm.Save(ckpt);
          gm.Reset();
          const bool loaded = saved && gm.Load(ckpt);
          initialized = loaded;
          requested_reset = !loaded;
          mark(loaded ? 6 : 7);
        }
        if (op_reset)
        {
          gm.Reset();  // ~/clear_graph; re-seeded by the next usable fix
          initialized = false;
          requested_reset = true;
          mark(8);
        }
        if (op_invalidate)
          gm.InvalidateGnssSupport();
        if (op_lidar && initialized)
        {
          if (const auto node = gm.FindNodeAtOrBefore(t - lidar_s))
          {
            const Truth& at = truth_at((us - static_cast<int64_t>(lidar_s * 1e6)) / 1000 * 1000);
            const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() * (0.03 * 0.03);
            gm.QueueLidarMapXy(gtsam::Vector2(at.x + lx, at.y + ly), cov, true, node);
          }
        }
        if (op_mag && initialized)
          gm.QueueYaw(gth + mag_bias, 0.2, /*robust=*/true);
      }
    }
  }
  if (rebase_thread.joinable())
    rebase_thread.join();
  if (faults_on)
  {
    std::remove((ckpt + ".checkpoint").c_str());
    std::remove((ckpt + ".checkpoint.previous").c_str());
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
  if (n_fixed_err > 0)
    std::printf("[bench] RTK-Fixed pose error vs truth: mean=%.4f m max=%.4f m (%zu ticks)\n",
                sum_fixed_err / static_cast<double>(n_fixed_err),
                max_fixed_err,
                n_fixed_err);
  if (faults_on)
  {
    const auto st = gm.Stats();
    std::printf(
        "[bench] faults: unrequested_isam_resets=%zu wrongfix_rejects=%lu stuck_rejects=%lu "
        "gnss_queued=%lu refused=%lu dropped(no receipt node=%lu future=%lu no node=%lu) "
        "slip_veto=%lu hand_push=%lu rebases=%lu\n",
        unrequested_resets,
        static_cast<unsigned long>(st.gps_rejects_wrongfix),
        static_cast<unsigned long>(st.gps_rejects_stuck_value),
        static_cast<unsigned long>(gate.queued),
        static_cast<unsigned long>(gate.queue_refused),
        static_cast<unsigned long>(gate.dropped_no_receipt_node),
        static_cast<unsigned long>(gate.dropped_future),
        static_cast<unsigned long>(gate.dropped_no_node),
        static_cast<unsigned long>(st.slip_veto),
        static_cast<unsigned long>(st.stationary_hand_push),
        static_cast<unsigned long>(st.successful_rebases));
  }
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
