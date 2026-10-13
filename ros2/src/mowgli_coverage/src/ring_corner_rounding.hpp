// Copyright 2026 Mowgli Project
#pragma once
#include <functional>
#include <utility>
#include <vector>
namespace mowgli_coverage::detail
{
// Internal sparse headland-ring seam; not installed as a public API.
std::vector<std::pair<double, double>> roundBeveledCorners(
    const std::vector<std::pair<double, double>>& points,
    const std::vector<std::pair<double, double>>& boundary,
    const std::vector<std::vector<std::pair<double, double>>>& holes,
    double max_turn_rad,
    double fillet_radius,
    double min_radius,
    double step,
    const std::function<bool(const std::pair<double, double>&, double)>& keep_sharp = {});
}  // namespace mowgli_coverage::detail
