// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// Unit tests for the LocalizationGuard's input signal
// (mowgli_behavior/localization_health.hpp).
//
// The headline regression is PivotCovarianceSpikeDoesNotDegrade: replaying the
// 2026-08-20 field trace — RTK-Fixed at hacc 0.014 m throughout while the
// fused marginal swung 0.05 m parked -> 1.86 m pivoting — must NOT pause
// mowing. Under the old σ-only guard that trace was a livelock: ten
// degrade/recover cycles in eight minutes with FollowStrip pinned at 0 %.

#include <vector>

#include "mowgli_behavior/localization_health.hpp"
#include "mowgli_interfaces/gnss_observation_freshness.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_behavior::LocalizationFault;
using mowgli_behavior::LocalizationHealthCfg;
using mowgli_behavior::LocalizationHealthMonitor;
using mowgli_behavior::LocalizationObservation;
using mowgli_behavior::PersistentLatch;
using mowgli_behavior::RtkMode;
namespace freshness = mowgli_interfaces::gnss_observation_freshness;

constexpr std::int64_t kSecond = 1000000000LL;

LocalizationHealthCfg ImmediateHealthCfg()
{
  LocalizationHealthCfg cfg;
  cfg.gnss_pause_persist_s = 0.0;
  cfg.gnss_resume_persist_s = 0.0;
  return cfg;
}

// A healthy RTK-Fixed epoch, as the live robot reports it.
LocalizationObservation HealthyFix(double sigma_xy = 0.05)
{
  LocalizationObservation obs;
  obs.gnss_seen = true;
  obs.gnss_fresh = true;
  obs.rtk_mode = RtkMode::kFixed;
  obs.gnss_accuracy_m = 0.014;
  obs.fused_sigma_xy_m = sigma_xy;
  return obs;
}

// Drive the monitor for `duration_s` at 5 Hz (the GNSS rate) with a
// per-sample observation builder. Returns true if it EVER latched.
template <typename MakeObs>
bool RunFor(LocalizationHealthMonitor* mon, double start_s, double duration_s, MakeObs make)
{
  bool ever = false;
  for (double t = start_s; t <= start_s + duration_s; t += 0.2)
  {
    if (mon->Update(t, make()))
      ever = true;
  }
  return ever;
}

// ── The regression this change exists for ──────────────────────────────────

TEST(LocalizationHealthTest, PivotCovarianceSpikeDoesNotDegrade)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  // Field 2026-08-20: every degrade fired at one of these σ values while the
  // receiver held RTK-Fixed; every recovery read exactly 0.05 m (parked).
  const std::vector<double> field_sigmas = {
      0.05, 0.57, 0.05, 0.74, 0.81, 0.05, 0.92, 0.99, 1.07, 0.05, 1.38, 1.86, 0.05};

  double t = 100.0;
  bool ever_degraded = false;
  for (double sigma : field_sigmas)
  {
    // Hold each value for 15 s — far longer than any persistence window.
    if (RunFor(&mon,
               t,
               15.0,
               [sigma]()
               {
                 return HealthyFix(sigma);
               }))
      ever_degraded = true;
    t += 15.2;
  }

  EXPECT_FALSE(ever_degraded) << "pivot/slip covariance inflation must not pause mowing "
                                 "while the receiver reports RTK-Fixed at 1.4 cm";
  EXPECT_FALSE(mon.degraded());
  EXPECT_EQ(mon.fault(), LocalizationFault::kNone);
}

// ── The incident the guard exists for must still be caught ─────────────────

TEST(LocalizationHealthTest, PlainGpsFallbackStillDegrades)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  ASSERT_FALSE(RunFor(&mon,
                      0.0,
                      10.0,
                      []()
                      {
                        return HealthyFix();
                      }));

  // 2026-08-02: RTK -> plain GPS, sigma ~1.5 m, for >60 s.
  const bool degraded = RunFor(&mon,
                               20.0,
                               60.0,
                               []()
                               {
                                 LocalizationObservation obs;
                                 obs.gnss_seen = true;
                                 obs.gnss_fresh = true;
                                 obs.rtk_mode = RtkMode::kNone;
                                 obs.gnss_accuracy_m = 1.5;
                                 obs.fused_sigma_xy_m = 1.5;
                                 return obs;
                               });

  EXPECT_TRUE(degraded);
  EXPECT_TRUE(mon.degraded());
  EXPECT_EQ(mon.fault(), LocalizationFault::kGnssAccuracy);
}

TEST(LocalizationHealthTest, RecoveryReleasesTheLatch)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  ASSERT_TRUE(RunFor(&mon,
                     0.0,
                     30.0,
                     []()
                     {
                       LocalizationObservation obs;
                       obs.gnss_seen = true;
                       obs.gnss_fresh = true;
                       obs.rtk_mode = RtkMode::kNone;
                       obs.gnss_accuracy_m = 1.5;
                       return obs;
                     }));

  // RTK-Fixed comes back; the resume persistence is 2 s.
  RunFor(&mon,
         40.0,
         10.0,
         []()
         {
           return HealthyFix();
         });
  EXPECT_FALSE(mon.degraded());
  EXPECT_EQ(mon.fault(), LocalizationFault::kNone);
}

// ── Debounce ───────────────────────────────────────────────────────────────

TEST(LocalizationHealthTest, BriefAccuracyExcursionDoesNotFlipTheLatch)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  // 2 s of bad accuracy, under the 3 s pause persistence.
  const bool degraded = RunFor(&mon,
                               0.0,
                               2.0,
                               []()
                               {
                                 LocalizationObservation obs;
                                 obs.gnss_seen = true;
                                 obs.gnss_fresh = true;
                                 obs.rtk_mode = RtkMode::kFloat;
                                 obs.gnss_accuracy_m = 0.9;
                                 return obs;
                               });

  EXPECT_FALSE(degraded);
}

TEST(LocalizationHealthTest, DeadBandHoldsTheLatchUntilAccuracyClearsResume)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  ASSERT_TRUE(RunFor(&mon,
                     0.0,
                     30.0,
                     []()
                     {
                       LocalizationObservation obs;
                       obs.gnss_seen = true;
                       obs.gnss_fresh = true;
                       obs.rtk_mode = RtkMode::kFloat;
                       obs.gnss_accuracy_m = 1.2;
                       return obs;
                     }));

  // 0.20 m sits between resume (0.15) and pause (0.30): still latched.
  RunFor(&mon,
         40.0,
         30.0,
         []()
         {
           LocalizationObservation obs;
           obs.gnss_seen = true;
           obs.gnss_fresh = true;
           obs.rtk_mode = RtkMode::kFloat;
           obs.gnss_accuracy_m = 0.20;
           return obs;
         });
  EXPECT_TRUE(mon.degraded());
}

// ── Feed health ────────────────────────────────────────────────────────────

TEST(LocalizationHealthTest, StaleGnssFeedDegrades)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};
  ASSERT_FALSE(RunFor(&mon,
                      0.0,
                      10.0,
                      []()
                      {
                        return HealthyFix();
                      }));

  // /gps/status stops at t=10; clock keeps running.
  LocalizationObservation frozen = HealthyFix();
  bool degraded = false;
  for (double t = 10.0; t <= 30.0; t += 0.2)
  {
    frozen.gnss_fresh = t <= 15.0;
    degraded = mon.Update(t, frozen);
  }

  EXPECT_TRUE(degraded);
  EXPECT_EQ(mon.fault(), LocalizationFault::kGnssStale);
}

TEST(LocalizationHealthFreshnessTest, GenuineFixedObservationAuthorizesHealth)
{
  freshness::PhysicalObservationTracker tracker;
  LocalizationHealthMonitor mon{ImmediateHealthCfg()};
  auto obs = HealthyFix();

  ASSERT_EQ(tracker.Observe(1, 100 * kSecond, 100 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  obs.gnss_fresh = tracker.ObservationIsFresh(100 * kSecond, 1 * kSecond, 5 * kSecond);

  EXPECT_FALSE(mon.Update(100.0, obs));
  EXPECT_EQ(mon.fault(), LocalizationFault::kNone);
}

TEST(LocalizationHealthFreshnessTest, CachedFixedCallbacksCannotExtendDeadline)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(1, 100 * kSecond, 100 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);

  for (std::int64_t second = 101; second <= 106; ++second)
  {
    EXPECT_EQ(tracker.Observe(1, 100 * kSecond, second * kSecond, (second - 99) * kSecond),
              freshness::ObservationUpdate::kCachedPublication);
  }

  EXPECT_FALSE(tracker.ObservationIsFresh(106 * kSecond, 7 * kSecond, 5 * kSecond));
  EXPECT_TRUE(tracker.DeliveryIsLive(7 * kSecond, 1 * kSecond));
}

TEST(LocalizationHealthFreshnessTest, CachedFixedBecomesGnssStaleAfterTimeout)
{
  freshness::PhysicalObservationTracker tracker;
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};
  auto obs = HealthyFix();
  ASSERT_EQ(tracker.Observe(4, 100 * kSecond, 100 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_EQ(tracker.Observe(4, 100 * kSecond, 106 * kSecond, 7 * kSecond),
            freshness::ObservationUpdate::kCachedPublication);

  obs.gnss_fresh = tracker.ObservationIsFresh(106 * kSecond, 7 * kSecond, 5 * kSecond);
  EXPECT_TRUE(mon.Update(106.0, obs));
  EXPECT_EQ(mon.fault(), LocalizationFault::kGnssStale);
}

TEST(LocalizationHealthFreshnessTest, GenuineFixedAfterTimeoutRestoresHealth)
{
  freshness::PhysicalObservationTracker tracker;
  LocalizationHealthMonitor mon{ImmediateHealthCfg()};
  auto obs = HealthyFix();
  ASSERT_EQ(tracker.Observe(8, 100 * kSecond, 100 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  obs.gnss_fresh = tracker.ObservationIsFresh(106 * kSecond, 7 * kSecond, 5 * kSecond);
  ASSERT_TRUE(mon.Update(106.0, obs));

  ASSERT_EQ(tracker.Observe(9, 106 * kSecond, 106 * kSecond, 7 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  obs.gnss_fresh = tracker.ObservationIsFresh(106 * kSecond, 7 * kSecond, 5 * kSecond);
  EXPECT_FALSE(mon.Update(106.0, obs));
  EXPECT_EQ(mon.fault(), LocalizationFault::kNone);
}

TEST(LocalizationHealthTest, MonitorIsInertBeforeAnyGnssStatus)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  // Legacy bring-up: no /gps/status publisher, huge fused sigma. The guard
  // must not block — BoundaryGuard still covers leaving the area.
  LocalizationObservation obs;
  obs.gnss_seen = false;
  obs.fused_sigma_xy_m = 40.0;

  bool degraded = false;
  for (double t = 0.0; t <= 60.0; t += 0.2)
    degraded = mon.Update(t, obs);

  EXPECT_FALSE(degraded);
}

TEST(LocalizationHealthTest, MissingAccuracyFallsBackToRtkMode)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  // Receiver reports no accuracy but a valid Float solution -> trusted.
  ASSERT_FALSE(RunFor(&mon,
                      0.0,
                      20.0,
                      []()
                      {
                        LocalizationObservation obs;
                        obs.gnss_seen = true;
                        obs.gnss_fresh = true;
                        obs.rtk_mode = RtkMode::kFloat;
                        obs.gnss_accuracy_m = -1.0;
                        return obs;
                      }));

  // Solution lost and still no accuracy -> degrade.
  const bool degraded = RunFor(&mon,
                               30.0,
                               20.0,
                               []()
                               {
                                 LocalizationObservation obs;
                                 obs.gnss_seen = true;
                                 obs.gnss_fresh = true;
                                 obs.rtk_mode = RtkMode::kNone;
                                 obs.gnss_accuracy_m = -1.0;
                                 return obs;
                               });
  EXPECT_TRUE(degraded);
  EXPECT_EQ(mon.fault(), LocalizationFault::kGnssFixLost);
}

TEST(LocalizationHealthTest, GoodAccuracyOutranksUnknownRtkMode)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  // A receiver that never fills rtk_mode but reports 1.4 cm is healthy.
  const bool degraded = RunFor(&mon,
                               0.0,
                               60.0,
                               []()
                               {
                                 LocalizationObservation obs;
                                 obs.gnss_seen = true;
                                 obs.gnss_fresh = true;
                                 obs.rtk_mode = RtkMode::kUnknown;
                                 obs.gnss_accuracy_m = 0.014;
                                 return obs;
                               });
  EXPECT_FALSE(degraded);
}

// ── Divergence backstop ────────────────────────────────────────────────────

TEST(LocalizationHealthTest, SigmaBackstopCatchesLocalizerDivergence)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  // GnssMobileGate-style runaway: receiver perfect, localizer rejecting every
  // fix, sigma parked far above the pivot ceiling.
  const bool degraded = RunFor(&mon,
                               0.0,
                               30.0,
                               []()
                               {
                                 return HealthyFix(8.0);
                               });

  EXPECT_TRUE(degraded);
  EXPECT_EQ(mon.fault(), LocalizationFault::kSigmaBackstop);
}

TEST(LocalizationHealthTest, SigmaBackstopSitsAboveThePivotInflationCeiling)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  // Worst-case deliberate inflation is sqrt(5)*1.05 ~= 2.35 m (adaptive gain
  // fully engaged). Held indefinitely, that must never reach the backstop.
  const bool degraded = RunFor(&mon,
                               0.0,
                               120.0,
                               []()
                               {
                                 return HealthyFix(2.35);
                               });

  EXPECT_FALSE(degraded);
}

TEST(LocalizationHealthTest, SigmaBackstopDisabledAtZero)
{
  LocalizationHealthCfg cfg;
  cfg.sigma_backstop_pause_m = 0.0;
  LocalizationHealthMonitor mon{cfg};

  const bool degraded = RunFor(&mon,
                               0.0,
                               120.0,
                               []()
                               {
                                 return HealthyFix(40.0);
                               });
  EXPECT_FALSE(degraded);
}

TEST(LocalizationHealthTest, MissingSigmaNeverLatchesTheBackstop)
{
  LocalizationHealthMonitor mon{LocalizationHealthCfg{}};

  const bool degraded = RunFor(&mon,
                               0.0,
                               120.0,
                               []()
                               {
                                 return HealthyFix(-1.0);
                               });
  EXPECT_FALSE(degraded);
}

// ── PersistentLatch itself ─────────────────────────────────────────────────

TEST(PersistentLatchTest, RequiresSustainedConditionInBothDirections)
{
  PersistentLatch latch;

  EXPECT_FALSE(latch.Update(0.0, /*bad=*/true, /*good=*/false, 3.0, 2.0));
  EXPECT_FALSE(latch.Update(2.9, true, false, 3.0, 2.0));
  EXPECT_TRUE(latch.Update(3.0, true, false, 3.0, 2.0));

  EXPECT_TRUE(latch.Update(4.0, false, true, 3.0, 2.0));
  EXPECT_TRUE(latch.Update(5.9, false, true, 3.0, 2.0));
  EXPECT_FALSE(latch.Update(6.0, false, true, 3.0, 2.0));
}

TEST(PersistentLatchTest, InterruptedConditionRestartsTheTimer)
{
  PersistentLatch latch;

  EXPECT_FALSE(latch.Update(0.0, true, false, 3.0, 2.0));
  EXPECT_FALSE(latch.Update(2.0, false, true, 3.0, 2.0));  // resets
  EXPECT_FALSE(latch.Update(4.0, true, false, 3.0, 2.0));  // starts over
  EXPECT_FALSE(latch.Update(6.9, true, false, 3.0, 2.0));
  EXPECT_TRUE(latch.Update(7.0, true, false, 3.0, 2.0));
}

TEST(PersistentLatchTest, DeadBandHoldsCurrentState)
{
  PersistentLatch latch;

  // Neither bad nor good (inside the hysteresis dead-band).
  for (double t = 0.0; t < 100.0; t += 1.0)
    EXPECT_FALSE(latch.Update(t, false, false, 3.0, 2.0));

  ASSERT_TRUE(latch.Update(100.0, true, false, 0.0, 2.0));
  for (double t = 101.0; t < 200.0; t += 1.0)
    EXPECT_TRUE(latch.Update(t, false, false, 3.0, 2.0));
}

}  // namespace
