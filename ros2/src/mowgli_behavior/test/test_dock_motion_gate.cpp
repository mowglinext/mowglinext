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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_dock_motion_gate.cpp
 * @brief Structural regression test: no dock transit may be entered while the
 *        published high-level state is IDLE.
 *
 * Field incident 2026-09-11. A completed mow published
 * HighLevelStatus state=1 (HIGH_LEVEL_STATE_IDLE, state_name
 * "MOWING_COMPLETE") and only then ran DockRobot. hardware_bridge forwards
 * that state verbatim to the STM32 (current_mode_ = msg->state), where
 * HL_MODE_IDLE becomes OPENMOWER_STATUS_IDLE and motors_handler() turns it
 * into a per-cycle hard stop:
 *
 *     } else if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
 *       hard_stop = true;
 *
 * So the tree declared the robot idle and then asked a hard-stopped robot to
 * drive 6.4 m to the dock. Nav2 planned normally and twist_mux carried
 * cmd_vel at 10 Hz, but every wheel setpoint was clamped to zero: the
 * progress checker tripped, recovery ran spin then backup (both equally
 * zeroed), and DockRobot never returned — so the tree accepted no further
 * command either. Encoder totals sat frozen for 474 consecutive samples.
 *
 * The IDLE publish long predated the firmware gate and was latent until the
 * wheel hard stop was added for blade safety, which is why it surfaced as a
 * sudden "stuck after mowing complete" with no tree change to blame.
 *
 * The contract: the LAST state published before a DockRobot must not be
 * IDLE, because the firmware refuses to move the wheels in that mode. Every
 * other dock path in the tree already honours it — CRITICAL_BATTERY_DOCKING,
 * RAIN_DETECTED_DOCKING, LOW_BATTERY_DOCKING, COVERAGE_FAILED_DOCKING and
 * RETURNING_HOME all publish state=2 (AUTONOMOUS) before docking. Only the
 * two MOWING_COMPLETE sites did not.
 *
 * This is deliberately a STRUCTURAL test against the real main_tree.xml
 * rather than a behavioural one: the failure is a mismatch between a literal
 * in the tree and a gate in firmware, so it cannot be caught by ticking
 * mocked nodes. It fails on the pre-fix tree and passes on the fixed one.
 */

#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{

/// HIGH_LEVEL_STATE_IDLE — the value the firmware maps to a wheel hard stop.
constexpr const char* kStateIdle = "1";

std::string readMainTree()
{
  std::ifstream file(MOWGLI_MAIN_TREE_PATH);
  EXPECT_TRUE(file.is_open()) << "cannot open " << MOWGLI_MAIN_TREE_PATH;
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

struct Publish
{
  std::string state;
  std::string state_name;
  std::size_t line;
};

}  // namespace

// Walk the tree in document order. Track the most recent
// PublishHighLevelStatus; every time a DockRobot is reached, that publish is
// the state the firmware will be sitting in when the dock drive starts.
TEST(DockMotionGate, NoDockRobotIsEnteredWhilePublishingIdle)
{
  const std::string tree = readMainTree();
  const std::regex publish_re(
      R"RE(<PublishHighLevelStatus\s+state="(\d+)"\s+state_name="([^"]+)")RE");
  const std::regex dock_re(R"RE(<DockRobot\b)RE");

  std::istringstream stream(tree);
  std::string line;
  std::size_t line_no = 0;
  std::size_t dock_sites = 0;
  bool have_publish = false;
  Publish last{};
  std::vector<std::string> violations;

  while (std::getline(stream, line))
  {
    ++line_no;

    std::smatch match;
    if (std::regex_search(line, match, publish_re))
    {
      last = Publish{match[1].str(), match[2].str(), line_no};
      have_publish = true;
    }

    if (std::regex_search(line, dock_re))
    {
      ++dock_sites;
      if (!have_publish)
      {
        violations.push_back("DockRobot at line " + std::to_string(line_no) +
                             " is not preceded by any PublishHighLevelStatus");
      }
      else if (last.state == kStateIdle)
      {
        violations.push_back(
            "DockRobot at line " + std::to_string(line_no) +
            " is entered while publishing state=1 (IDLE) as '" + last.state_name + "' at line " +
            std::to_string(last.line) +
            "; the firmware hard-stops the wheels in IDLE so this dock can never drive");
      }
    }
  }

  // Guard the guard: if the tree is refactored so DockRobot disappears or the
  // publish attribute order changes, this test must fail loudly rather than
  // silently passing over nothing.
  EXPECT_GT(dock_sites, 0u) << "found no <DockRobot> sites — scan is not matching the tree";
  EXPECT_TRUE(have_publish) << "found no <PublishHighLevelStatus> — scan is not matching the tree";

  std::string report;
  for (const auto& v : violations)
  {
    report += "\n  " + v;
  }
  EXPECT_TRUE(violations.empty()) << "dock transit(s) entered in IDLE:" << report;
}
