// Copyright 2026 Mowgli Project
#pragma once
#include <functional>
#include <utility>
#include <vector>
namespace mowgli_coverage::detail
{
// Passive offline evidence: indices refer to source/intermediate, never a
// permission verdict. The replay oracle independently checks every replacement.
struct BevelTrace
{
  using Path = std::vector<std::pair<double, double>>;
  struct Replacement
  {
    std::size_t source_vertex;
    std::size_t output_begin;
    Path local;
  };
  Path source, intermediate, rounded;
  std::vector<Replacement> replacements;
};
// Internal sparse headland-ring seam; not installed as a public API.
std::vector<std::pair<double, double>> roundBeveledCorners(
    const std::vector<std::pair<double, double>>& points,
    const std::vector<std::pair<double, double>>& boundary,
    const std::vector<std::vector<std::pair<double, double>>>& holes,
    double max_turn_rad,
    double fillet_radius,
    double min_radius,
    double step,
    const std::function<bool(const std::pair<double, double>&, double)>& keep_sharp = {},
    BevelTrace* trace = nullptr);
}  // namespace mowgli_coverage::detail
