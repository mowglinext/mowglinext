// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef MOWGLI_INTERFACES__COVERAGE_PATH_INVARIANTS_HPP_
#define MOWGLI_INTERFACES__COVERAGE_PATH_INVARIANTS_HPP_

#include <cstddef>

namespace mowgli_interfaces
{

// PathProgressGoalChecker permits proximity-only completion for paths no longer
// than this. Keep resume replay longer: an interruption is never completion.
inline constexpr std::size_t kCoverageShortPathPoses = 10;

// Two poses beyond the short-path threshold also keep the initial, bounded
// progress search from reaching 95% of a replay before the robot has moved.
inline constexpr std::size_t kCoverageResumeReplayPoses = kCoverageShortPathPoses + 2;

}  // namespace mowgli_interfaces

#endif  // MOWGLI_INTERFACES__COVERAGE_PATH_INVARIANTS_HPP_
