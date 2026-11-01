// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "mowgli_coverage/coverage_planning.hpp"
#include <nlohmann/json.hpp>

namespace mowgli_coverage::replay_contract
{
// Defined only by the standalone adapter around byte-unchanged baseline source.
// No fallback implementation: incompatible private baseline helpers fail to build.
void applyCertifiedRings(BoustrophedonPlan& plan,
                         const nlohmann::json& snapshot,
                         std::size_t field,
                         const nlohmann::json& state,
                         const nlohmann::json& parameters,
                         const std::vector<std::pair<double, double>>& raw_boundary);
void verifyReferenceStages(const nlohmann::json& snapshot,
                           const nlohmann::json& baseline_stages,
                           std::size_t field,
                           std::size_t ring_count);
}  // namespace mowgli_coverage::replay_contract
