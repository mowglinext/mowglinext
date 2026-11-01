// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace mowgli_coverage::replay_contract
{
struct DiagnosticNotes
{
  std::vector<std::string> semantic;
  std::vector<std::string> elapsed;
};

// The producer mixes clocks and semantic geometry into one known note. Keep
// every non-clock character in identity and retain the original as telemetry.
inline DiagnosticNotes splitDiagnosticNotes(const std::vector<std::string>& notes)
{
  static const std::regex timing(
      R"(^timing: headland=([0-9]+)ms mainland=([0-9]+)ms swaths=([0-9]+)ms (\(angles=[0-9]+, mainland_area=[0-9]+\.[0-9] m²\))$)");
  DiagnosticNotes out;
  for (const auto& note : notes)
  {
    if (!note.starts_with("timing:"))
    {
      out.semantic.push_back(note);
      continue;
    }
    std::smatch match;
    if (!std::regex_match(note, match, timing))
      throw std::runtime_error("unsupported planner timing-note schema");
    out.elapsed.push_back(note);
    out.semantic.push_back("timing: headland=<elapsed>ms mainland=<elapsed>ms swaths=<elapsed>ms " +
                           match[4].str());
  }
  return out;
}
}  // namespace mowgli_coverage::replay_contract
