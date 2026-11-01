// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "semantic_diagnostics.hpp"
#include <gtest/gtest.h>

namespace
{
using mowgli_coverage::replay_contract::splitDiagnosticNotes;
const std::string first =
    "timing: headland=0ms mainland=0ms swaths=76ms (angles=72, mainland_area=20.7 m²)";
const std::string second =
    "timing: headland=12ms mainland=345ms swaths=39ms (angles=72, mainland_area=20.7 m²)";
TEST(SemanticDiagnostics, OnlyElapsedValuesLeaveIdentityAndRawTelemetryIsRetained)
{
  const auto a = splitDiagnosticNotes({first}), b = splitDiagnosticNotes({second});
  EXPECT_EQ(a.semantic, b.semantic);
  EXPECT_EQ(a.semantic,
            (std::vector<std::string>{"timing: headland=<elapsed>ms mainland=<elapsed>ms "
                                      "swaths=<elapsed>ms (angles=72, mainland_area=20.7 m²)"}));
  EXPECT_EQ(a.elapsed, (std::vector<std::string>{first}));
  EXPECT_EQ(b.elapsed, (std::vector<std::string>{second}));
}
TEST(SemanticDiagnostics, ChangedSemanticSuffixRemainsDifferent)
{
  auto angle = first, area = first;
  angle.replace(angle.find("angles=72"), 9, "angles=73");
  area.replace(area.find("20.7"), 4, "20.8");
  EXPECT_NE(splitDiagnosticNotes({first}).semantic, splitDiagnosticNotes({angle}).semantic);
  EXPECT_NE(splitDiagnosticNotes({first}).semantic, splitDiagnosticNotes({area}).semantic);
}
TEST(SemanticDiagnostics, AllOtherNotesAndTheirOrderAreExact)
{
  const std::vector<std::string> notes{"AUTO fallback: area exceeds search limit",
                                       first,
                                       "rings disabled; raw field retained"};
  const auto result = splitDiagnosticNotes(notes);
  ASSERT_EQ(result.semantic.size(), 3u);
  EXPECT_EQ(result.semantic.front(), notes.front());
  EXPECT_EQ(result.semantic.back(), notes.back());
  auto changed = notes;
  changed.front() += "!";
  EXPECT_NE(splitDiagnosticNotes(changed).semantic, result.semantic);
  std::swap(changed.front(), changed.back());
  EXPECT_NE(splitDiagnosticNotes(changed).semantic, result.semantic);
}
TEST(SemanticDiagnostics, MalformedRecognizedTimingNotesFailClosed)
{
  for (const auto& note :
       {"timing: unknown producer format",
        "timing: headland=0ms mainland=0ms swaths=NaNms (angles=72, mainland_area=20.7 m²)",
        "timing: headland=-1ms mainland=0ms swaths=76ms (angles=72, mainland_area=20.7 m²)",
        "timing: headland=0ms mainland=0ms swaths=76ms (angles=72, mainland_area=20.70 m²)",
        "timing: headland=0ms mainland=0ms swaths=76ms (angles=72, mainland_area=20.7 m²) extra",
        "timing: headland=0ms mainland=0ms swaths=76ms (angles=72, mainland_area=20.7 m²)\n"})
    EXPECT_THROW(splitDiagnosticNotes({note}), std::runtime_error) << note;
}
}  // namespace
