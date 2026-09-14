// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Map-learning state and insertion cadence. Pure: no ROS / GTSAM / Beluga.
// Fresh RTK permits map updates; stale RTK freezes learning. After an outage,
// require continuously fresh Fixed for the dwell before learning resumes.
// LidarComputeGate separately owns PF scheduling and reseeding. These states
// describe mapping availability, not permission to compute or apply a factor.

#pragma once

#include <cstdint>

namespace fusion_graph
{

enum class LidarAnchorState : uint8_t
{
  kDisabled,
  kWaitingForMap,  // enabled, but the grid has no occupied cells yet
  kMapping,
  kAnchoring,
};

struct LidarAnchorDecision
{
  LidarAnchorState state = LidarAnchorState::kDisabled;
  bool insert_scan = false;  // add this scan to the grid
};

class LidarMapAnchorGate
{
public:
  LidarMapAnchorGate(bool enabled,
                     double engage_age_s,
                     double insert_period_s,
                     double disengage_dwell_s = 1.0)
      : enabled_(enabled),
        engage_age_s_(engage_age_s),
        insert_period_s_(insert_period_s),
        disengage_dwell_s_(disengage_dwell_s)
  {
  }

  // rtk_fixed_age_s: seconds since the last accepted RTK-Fixed receipt
  // (a large value when none was ever received). map_has_structure: the grid
  // exports at least one occupied cell. now_s: ROS time; reconstruct this gate
  // when its clock epoch changes.
  LidarAnchorDecision Step(double rtk_fixed_age_s, bool map_has_structure, double now_s)
  {
    LidarAnchorDecision d;
    if (!enabled_)
    {
      state_ = LidarAnchorState::kDisabled;
      d.state = state_;
      return d;
    }
    const bool fixed_fresh = rtk_fixed_age_s <= engage_age_s_;
    if (fixed_fresh)
    {
      if (state_ == LidarAnchorState::kAnchoring)
      {
        // Hysteresis: stay anchored until Fixed has been fresh for the dwell.
        if (fresh_since_s_ < 0.0)
          fresh_since_s_ = now_s;
        if ((now_s - fresh_since_s_) < disengage_dwell_s_)
        {
          d.state = state_;
          return d;
        }
      }
      fresh_since_s_ = -1.0;
      state_ = LidarAnchorState::kMapping;
      d.insert_scan = (now_s - last_insert_s_) >= insert_period_s_;
      if (d.insert_scan)
        last_insert_s_ = now_s;
    }
    else if (!map_has_structure)
    {
      // Nothing to localise against yet — stay honest, produce no factor.
      state_ = LidarAnchorState::kWaitingForMap;
    }
    else
    {
      fresh_since_s_ = -1.0;  // any stale sample resets the disengage dwell
      state_ = LidarAnchorState::kAnchoring;
    }
    d.state = state_;
    return d;
  }

  LidarAnchorState state() const
  {
    return state_;
  }

private:
  bool enabled_;
  double engage_age_s_;
  double insert_period_s_;
  double disengage_dwell_s_;
  LidarAnchorState state_ = LidarAnchorState::kDisabled;
  double last_insert_s_ = -1e9;
  double fresh_since_s_ = -1.0;  // when Fixed became fresh again while anchoring; <0 = not yet
};

}  // namespace fusion_graph
