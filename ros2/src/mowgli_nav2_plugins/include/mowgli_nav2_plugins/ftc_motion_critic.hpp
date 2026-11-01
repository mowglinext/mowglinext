// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <memory>

#include "mowgli_interfaces/motion_geometry_types.hpp"
#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"

namespace mowgli_interfaces::motion
{
struct Snapshot;
}

namespace mowgli_nav2_plugins
{
class MotionRegionIndex;
// Candidate-space geometry critic, separate from the final executed-Twist
// certificate. References follow complete chords, body yaw interpolates by
// the shortest angle. Body overhang is legal only at the outer boundary;
// the complete chassis sweep must avoid every raw hole.
LatticeGeometryBlocked MakeMotionLatticeCritic(
    std::shared_ptr<const mowgli_interfaces::motion::Snapshot> snapshot,
    mowgli_interfaces::motion::Ring footprint,
    mowgli_interfaces::motion::Pose costmap_in_map,
    std::shared_ptr<const MotionRegionIndex> prepared = {});
}  // namespace mowgli_nav2_plugins
