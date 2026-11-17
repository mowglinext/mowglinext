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
#include <functional>
#include <limits>

#include "mowgli_behavior/precision_coverage_support.hpp"
#include <gtest/gtest.h>
using mowgli_behavior::PrecisionCoverageSupport;
using Status = PrecisionCoverageSupport::Status;
using Fix = PrecisionCoverageSupport::Fix;
using Report = PrecisionCoverageSupport::Report;
namespace
{
constexpr int64_t S = 1000000000LL;
auto Stamp(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = ns / S;
  t.nanosec = ns % S;
  return t;
}
Status Fixed(int64_t t, uint64_t seq)
{
  Status s;
  s.header.stamp = Stamp(t);
  s.fix_valid = true;
  s.fix_type = Status::FIX_TYPE_RTK_FIXED;
  s.position_observation_sequence = seq;
  return s;
}
Fix Gps(int64_t t)
{
  Fix f;
  f.header.stamp = Stamp(t);
  f.header.frame_id = "gps_link";
  f.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  f.latitude = 48.0;
  f.longitude = 11.0;
  f.altitude = std::numeric_limits<double>::quiet_NaN();
  f.position_covariance[0] = .000009;
  f.position_covariance_type = Fix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  return f;
}
Report Commit(const Fix& f, int64_t support, uint64_t rev = 1)
{
  Report r;
  r.producer_incarnation = "producer-a";
  r.graph_epoch = 1;
  r.estimate_revision = r.applied_revision = rev;
  r.pose_published = r.has_applied_fix = true;
  r.applied_fix = f;
  r.tf_support_stamp = Stamp(support);
  return r;
}
}  // namespace
TEST(PrecisionCoverage, RawPairClosesBeforeDelayedCommitAndNeedsTf)
{
  PrecisionCoverageSupport p;
  auto s = Fixed(S, 1);
  auto f = Gps(S);
  p.AddStatus(s, S, S);
  p.AddFix(f, S + 5000000, S + 5000000);
  auto r = Commit(f, S + 80000000);
  p.AddReport(r, S + 70000000, S + 70000000);
  EXPECT_FALSE(p.Ready(3 * S,
                       3 * S,
                       [](int64_t)
                       {
                         return false;
                       }));
  EXPECT_TRUE(p.Ready(3 * S,
                      3 * S,
                      [](int64_t)
                      {
                        return true;
                      }));
}
TEST(PrecisionCoverage, LossRequiresNewAppliedFixDespiteStableRawFixed)
{
  PrecisionCoverageSupport p;
  auto s = Fixed(S, 1);
  auto f = Gps(S);
  p.AddFix(f, S, S);
  p.AddStatus(s, S, S);
  p.AddReport(Commit(f, S), S, S);
  EXPECT_TRUE(p.Ready(3 * S,
                      3 * S,
                      [](int64_t)
                      {
                        return true;
                      }));
  auto lost = Fixed(3 * S, 1);
  lost.fix_type = Status::FIX_TYPE_RTK_FLOAT;
  p.AddStatus(lost, 3 * S, 3 * S);
  EXPECT_FALSE(p.Ready(3 * S,
                       3 * S,
                       [](int64_t)
                       {
                         return true;
                       }));
  auto fresh = Fixed(4 * S, 2);
  auto nf = Gps(4 * S);
  p.AddStatus(fresh, 4 * S, 4 * S);
  p.AddFix(nf, 4 * S + 10000000, 4 * S + 10000000);
  p.AddReport(Commit(f, S, 2), 4 * S + 20000000, 4 * S + 20000000);
  EXPECT_FALSE(p.Ready(6 * S,
                       6 * S,
                       [](int64_t)
                       {
                         return true;
                       }));
  p.AddReport(Commit(nf, 6 * S + 100000000, 3), 6 * S + 1000000, 6 * S + 1000000);
  EXPECT_FALSE(p.Ready(6 * S + 2000000,
                       6 * S + 2000000,
                       [](int64_t)
                       {
                         return true;
                       }));
  EXPECT_TRUE(p.Ready(6 * S + 100000000,
                      6 * S + 100000000,
                      [](int64_t)
                      {
                        return true;
                      }));
}

TEST(PrecisionCoverage, AmbiguousPairAndClockRewindDeny)
{
  PrecisionCoverageSupport p;
  auto s = Fixed(S, 1);
  auto f = Gps(S);
  p.AddStatus(s, S, S);
  p.AddFix(f, S + 1000000, S + 1000000);
  auto conflict = s;
  conflict.position_observation_sequence = 2;
  p.AddStatus(conflict, S + 2000000, S + 2000000);
  p.AddReport(Commit(f, S), S + 3000000, S + 3000000);
  EXPECT_FALSE(p.Ready(3 * S,
                       3 * S,
                       [](int64_t)
                       {
                         return true;
                       }));
  EXPECT_FALSE(p.Ready(S / 2,
                       4 * S,
                       [](int64_t)
                       {
                         return true;
                       }));
}

TEST(PrecisionCoverage, FutureNewerRevisionDoesNotEvictEarlierUsableSupport)
{
  PrecisionCoverageSupport p;
  auto s = Fixed(S, 1);
  auto f = Gps(S);
  p.AddStatus(s, S, S);
  p.AddFix(f, S + 1000000, S + 1000000);
  p.AddReport(Commit(f, S), S + 2000000, S + 2000000);
  p.AddReport(Commit(f, 4 * S, 2), S + 3000000, S + 3000000);
  EXPECT_TRUE(p.Ready(3 * S,
                      3 * S,
                      [](int64_t)
                      {
                        return true;
                      }));
}

TEST(PrecisionCoverage, InvalidationAndOldProducerCannotResurrectOldFix)
{
  PrecisionCoverageSupport p;
  auto status = Fixed(S, 1);
  auto fix = Gps(S);
  auto old = Commit(fix, S);
  p.AddStatus(status, S, S);
  p.AddFix(fix, S + 1000000, S + 1000000);
  p.AddReport(old, S + 2000000, S + 2000000);
  EXPECT_TRUE(p.Ready(3 * S,
                      3 * S,
                      [](int64_t)
                      {
                        return true;
                      }));
  auto invalid = old;
  invalid.pose_published = false;
  invalid.has_applied_fix = false;
  invalid.graph_epoch = 2;
  p.AddReport(invalid, 3 * S, 3 * S);
  old.graph_epoch = 2;
  p.AddReport(old, 3 * S + 1000000, 3 * S + 1000000);
  EXPECT_FALSE(p.Ready(3 * S + 2000000,
                       3 * S + 2000000,
                       [](int64_t)
                       {
                         return true;
                       }));
  auto restart = invalid;
  restart.producer_incarnation = "producer-b";
  restart.graph_epoch = 1;
  p.AddReport(restart, 3 * S + 3000000, 3 * S + 3000000);
  p.AddReport(old, 3 * S + 4000000, 3 * S + 4000000);
  EXPECT_FALSE(p.Ready(3 * S + 5000000,
                       3 * S + 5000000,
                       [](int64_t)
                       {
                         return true;
                       }));
}

TEST(PrecisionCoverage, NewerFixedStatusDoesNotRequireNewestAppliedSequence)
{
  PrecisionCoverageSupport p;
  auto s = Fixed(S, 1);
  auto f = Gps(S);
  p.AddStatus(s, S, S);
  p.AddFix(f, S + 1000000, S + 1000000);
  p.AddReport(Commit(f, S), S + 2000000, S + 2000000);
  auto next = Fixed(3 * S, 2);
  p.AddStatus(next, 3 * S, 3 * S);
  EXPECT_TRUE(p.Ready(3 * S + 1000000,
                      3 * S + 1000000,
                      [](int64_t)
                      {
                        return true;
                      }));
}

TEST(PrecisionCoverage, InvalidProvenanceAndChangedSourceDenyOldEvidence)
{
  PrecisionCoverageSupport p;
  auto status = Fixed(S, 1);
  status.source_id = "receiver";
  status.source_incarnation = "one";
  auto fix = Gps(S);
  p.AddStatus(status, S, S);
  p.AddFix(fix, S + 1000000, S + 1000000);
  p.AddReport(Commit(fix, S), S + 2000000, S + 2000000);
  ASSERT_TRUE(p.Ready(3 * S,
                      3 * S,
                      [](int64_t)
                      {
                        return true;
                      }));
  auto invalid = status;
  invalid.header.stamp = Stamp(0);
  p.AddStatus(invalid, 3 * S, 3 * S);
  EXPECT_FALSE(p.Ready(3 * S,
                       3 * S,
                       [](int64_t)
                       {
                         return true;
                       }));
  auto changed = Fixed(4 * S, 2);
  changed.source_id = "receiver";
  changed.source_incarnation = "two";
  p.AddStatus(changed, 4 * S, 4 * S);
  auto newest = Gps(4 * S);
  p.AddFix(newest, 4 * S + 1000000, 4 * S + 1000000);
  p.AddReport(Commit(fix, S), 4 * S + 2000000, 4 * S + 2000000);
  EXPECT_FALSE(p.Ready(6 * S,
                       6 * S,
                       [](int64_t)
                       {
                         return true;
                       }));
}

TEST(PrecisionCoverage, FreshRawFixedCannotRefreshOldAppliedFixSteadyAge)
{
  PrecisionCoverageSupport p;
  auto status = Fixed(S, 1);
  auto fix = Gps(S);
  p.AddFix(fix, S, S);
  p.AddStatus(status, S + 1000000, S + 1000000);
  p.AddReport(Commit(fix, S + 100000000), S + 2000000, S + 2000000);
  const auto tf = [](int64_t)
  {
    return true;
  };
  EXPECT_FALSE(p.Ready(S + 60000000, S + 60000000, tf));  // close raw association
  ASSERT_TRUE(p.Ready(3 * S + 10000000, 3 * S + 10000000, tf));
  // Raw receiver observations remain fresh while graph rejects all new fixes.
  // Slow simulated ROS time must not extend the old applied fix's steady age.
  for (int i = 0; i < 8; ++i)
  {
    const int64_t ros = 3 * S + (i + 1) * 20000000LL;
    const int64_t steady = (4 + i) * S;
    p.AddFix(Gps(ros), ros, steady);
    p.AddStatus(Fixed(ros, i + 2), ros + 1000000, steady + 1000000);
  }
  EXPECT_FALSE(p.Ready(3 * S + 200000000, 11 * S + 60000000, tf));
}

TEST(PrecisionCoverage, TfCurrentAcceptsTheNewestBroadcastJustBeforeNow)
{
  // The newest transform is always a little in the past: that is current.
  EXPECT_TRUE(PrecisionCoverageSupport::TfCurrent(10 * S - 80000000, 10 * S));
  EXPECT_TRUE(PrecisionCoverageSupport::TfCurrent(10 * S, 10 * S));
  // fusion_graph future-dates its TF slightly.
  EXPECT_TRUE(PrecisionCoverageSupport::TfCurrent(10 * S + 50000000, 10 * S));
  // A localizer that stopped publishing is not.
  EXPECT_FALSE(
      PrecisionCoverageSupport::TfCurrent(10 * S - PrecisionCoverageSupport::kMaxTfLagNs - 1,
                                          10 * S));
  EXPECT_FALSE(PrecisionCoverageSupport::TfCurrent(0, 10 * S));
}

TEST(PrecisionCoverage, StaysReadyAcrossTicksBetweenTfBroadcasts)
{
  // Field 2026-11-09: RTK-Fixed, robot still, reports every 0.2 s, TF newest
  // stamp 4-80 ms behind each BT tick. The gate must not flap.
  PrecisionCoverageSupport p;
  for (int64_t k = 0; k < 20; ++k)
  {
    const int64_t t = S + k * 200000000LL;
    p.AddStatus(Fixed(t, static_cast<uint64_t>(k + 1)), t, t);
    p.AddFix(Gps(t), t + 5000000, t + 5000000);
    p.AddReport(Commit(Gps(t), t + 40000000, static_cast<uint64_t>(k + 1)),
                t + 30000000,
                t + 30000000);
  }
  const int64_t newest_tf = S + 19 * 200000000LL + 40000000;
  for (int64_t tick = 0; tick < 5; ++tick)
  {
    const int64_t now = newest_tf + 4000000 + tick * 15000000;  // 4-64 ms after it
    EXPECT_TRUE(p.Ready(now,
                        now,
                        [&](int64_t support)
                        {
                          return support <= newest_tf &&
                                 PrecisionCoverageSupport::TfCurrent(newest_tf, now);
                        }))
        << "tick " << tick;
  }
}

namespace
{
// Field 2026-11-09 (bag 113449Z): during a "Float" episode the receiver reported
// fix_type 3D with 1.4 cm horizontal accuracy and /gps/fix status STATUS_FIX.
Status Degraded(int64_t t, uint64_t seq, float horizontal_accuracy_m)
{
  Status s = Fixed(t, seq);
  s.fix_type = Status::FIX_TYPE_3D_FIX;
  s.horizontal_accuracy_m = horizontal_accuracy_m;
  s.capability_flags |= Status::CAP_HORIZONTAL_ACCURACY;
  s.value_flags |= Status::CAP_HORIZONTAL_ACCURACY;
  return s;
}
Fix NonRtk(int64_t t)
{
  Fix f = Gps(t);
  f.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  return f;
}
// Feed `n` 0.2 s epochs of (status, fix, commit) starting at t0.
void Feed(PrecisionCoverageSupport& p,
          int64_t t0,
          int n,
          uint64_t first_seq,
          const std::function<Status(int64_t, uint64_t)>& status,
          const std::function<Fix(int64_t)>& fix)
{
  for (int k = 0; k < n; ++k)
  {
    const int64_t t = t0 + k * 200000000LL;
    const uint64_t seq = first_seq + static_cast<uint64_t>(k);
    p.AddStatus(status(t, seq), t, t);
    p.AddFix(fix(t), t + 5000000, t + 5000000);
    p.AddReport(Commit(fix(t), t + 40000000, seq), t + 30000000, t + 30000000);
  }
}
const auto kTfOk = [](int64_t)
{
  return true;
};
}  // namespace

TEST(PrecisionCoverage, StrictModeStillPausesOnAShortDegradedEpisode)
{
  PrecisionCoverageSupport p;
  Feed(p, S, 20, 1, Fixed, Gps);
  ASSERT_TRUE(p.Ready(S + 19 * 200000000LL + 50000000, S + 19 * 200000000LL + 50000000, kTfOk));
  Feed(
      p,
      5 * S,
      5,
      21,
      [](int64_t t, uint64_t seq)
      {
        return Degraded(t, seq, 0.014f);
      },
      NonRtk);
  EXPECT_FALSE(p.Ready(5 * S + 900000000, 5 * S + 900000000, kTfOk));
}

TEST(PrecisionCoverage, LidarMapModeRidesThroughAFloatEpisode)
{
  PrecisionCoverageSupport p;
  p.SetLidarMapMode(true);
  Feed(p, S, 20, 1, Fixed, Gps);
  ASSERT_TRUE(p.Ready(S + 19 * 200000000LL + 50000000, S + 19 * 200000000LL + 50000000, kTfOk));
  // The field episode: 3D fix reporting 1.4 cm, NavSatFix STATUS_FIX.
  for (int64_t k = 0; k < 10; ++k)
  {
    const int64_t t = 5 * S + k * 200000000LL;
    Feed(
        p,
        t,
        1,
        21 + static_cast<uint64_t>(k),
        [](int64_t at, uint64_t seq)
        {
          return Degraded(at, seq, 0.014f);
        },
        NonRtk);
    EXPECT_TRUE(p.Ready(t + 50000000, t + 50000000, kTfOk)) << "epoch " << k;
  }
  // Genuine RTK Float (no accuracy reported) is accepted too.
  Feed(
      p,
      8 * S,
      5,
      31,
      [](int64_t t, uint64_t seq)
      {
        Status s = Fixed(t, seq);
        s.fix_type = Status::FIX_TYPE_RTK_FLOAT;
        return s;
      },
      NonRtk);
  EXPECT_TRUE(p.Ready(8 * S + 850000000, 8 * S + 850000000, kTfOk));
}

TEST(PrecisionCoverage, LidarMapModeStillPausesOnAPoorOrMissingFix)
{
  PrecisionCoverageSupport p;
  p.SetLidarMapMode(true);
  Feed(p, S, 20, 1, Fixed, Gps);
  // A plain 3D fix reporting 2 m is not usable.
  Feed(
      p,
      5 * S,
      5,
      21,
      [](int64_t t, uint64_t seq)
      {
        return Degraded(t, seq, 2.0f);
      },
      NonRtk);
  EXPECT_FALSE(p.Ready(5 * S + 900000000, 5 * S + 900000000, kTfOk));
  // No fix at all.
  PrecisionCoverageSupport q;
  q.SetLidarMapMode(true);
  Feed(q, S, 20, 1, Fixed, Gps);
  Status lost = Fixed(5 * S, 21);
  lost.fix_valid = false;
  lost.fix_type = Status::FIX_TYPE_NO_FIX;
  q.AddStatus(lost, 5 * S, 5 * S);
  EXPECT_FALSE(q.Ready(5 * S + 100000000, 5 * S + 100000000, kTfOk));
}
