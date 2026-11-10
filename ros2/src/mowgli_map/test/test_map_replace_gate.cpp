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
// Unit tests for map_replace_gate.hpp: a whole-map replace (clear_map,
// load_areas) must wait while a mission is running, and only then.

#include "mowgli_map/map_replace_gate.hpp"
#include <gtest/gtest.h>

namespace
{
using mowgli_map::kHighLevelStateAutonomous;
using mowgli_map::kHighLevelStatusMaxAgeS;
using mowgli_map::MapReplaceRejection;

constexpr uint8_t kNull = 0;
constexpr uint8_t kIdle = 1;
constexpr uint8_t kRecording = 3;
constexpr uint8_t kManualMowing = 4;
}  // namespace

TEST(MapReplaceGate, RefusesWhileAMissionIsRunning)
{
  const auto why = MapReplaceRejection(true, 0.5, kHighLevelStateAutonomous);
  ASSERT_TRUE(why.has_value());
  // The operator has to be told what to do about it.
  EXPECT_NE(why->find("Stop"), std::string::npos);
}

TEST(MapReplaceGate, AllowsEveryStateThatIsNotAMission)
{
  // IDLE covers docked, a STOP hold and a mid-mission charge hold — "Stop,
  // edit the map, Start" must work.
  EXPECT_FALSE(MapReplaceRejection(true, 0.5, kIdle).has_value());
  EXPECT_FALSE(MapReplaceRejection(true, 0.5, kNull).has_value());
  EXPECT_FALSE(MapReplaceRejection(true, 0.5, kRecording).has_value());
  EXPECT_FALSE(MapReplaceRejection(true, 0.5, kManualMowing).has_value());
}

TEST(MapReplaceGate, FailsOpenWithoutAFreshStatus)
{
  // No BT, or a BT that stopped publishing: nothing says a mission runs, and
  // refusing would make the map uneditable on a bench or after a BT crash.
  EXPECT_FALSE(MapReplaceRejection(false, 0.0, kHighLevelStateAutonomous).has_value());
  EXPECT_FALSE(MapReplaceRejection(true, kHighLevelStatusMaxAgeS + 0.1, kHighLevelStateAutonomous)
                   .has_value());
  // Exactly at the limit is still fresh.
  EXPECT_TRUE(
      MapReplaceRejection(true, kHighLevelStatusMaxAgeS, kHighLevelStateAutonomous).has_value());
}

TEST(MapReplaceGate, AutonomousConstantMatchesTheMessageEnum)
{
  // area_manager.cpp static_asserts this against HighLevelStatus.msg; the
  // literal here keeps the pure test honest if someone renumbers the header.
  EXPECT_EQ(kHighLevelStateAutonomous, 2);
}
