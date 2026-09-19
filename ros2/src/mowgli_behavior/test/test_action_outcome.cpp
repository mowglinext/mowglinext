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
//
// Unit tests for the action-goal terminal verdict (action_outcome.hpp).
// ROS-free by construction.

#include <optional>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_behavior/action_outcome.hpp"

namespace
{

using mowgli_behavior::ActionOutcomeSlot;
using mowgli_behavior::GoalOutcome;
using mowgli_behavior::kGoalStatusAborted;
using mowgli_behavior::kGoalStatusAccepted;
using mowgli_behavior::kGoalStatusCanceled;
using mowgli_behavior::kGoalStatusCanceling;
using mowgli_behavior::kGoalStatusExecuting;
using mowgli_behavior::kGoalStatusSucceeded;
using mowgli_behavior::kGoalStatusUnknown;
using mowgli_behavior::ResolveGoalStatus;

TEST(ActionOutcome, AStuckPollIsResolvedByTheRecordedResult)
{
  // The 2026-09-17 dock hang: the server succeeded in the same instant it
  // accepted, the status message was dropped, and the poll stayed on ACCEPTED.
  EXPECT_EQ(ResolveGoalStatus(kGoalStatusAccepted, GoalOutcome::kSucceeded), kGoalStatusSucceeded);
  EXPECT_EQ(ResolveGoalStatus(kGoalStatusExecuting, GoalOutcome::kAborted), kGoalStatusAborted);
  EXPECT_EQ(ResolveGoalStatus(kGoalStatusCanceling, GoalOutcome::kCanceled), kGoalStatusCanceled);
  EXPECT_EQ(ResolveGoalStatus(kGoalStatusUnknown, GoalOutcome::kSucceeded), kGoalStatusSucceeded);
}

TEST(ActionOutcome, WithoutARecordedResultThePolledStatusIsUntouched)
{
  for (const std::int8_t status : {kGoalStatusUnknown,
                                   kGoalStatusAccepted,
                                   kGoalStatusExecuting,
                                   kGoalStatusCanceling,
                                   kGoalStatusSucceeded,
                                   kGoalStatusAborted,
                                   kGoalStatusCanceled})
  {
    EXPECT_EQ(ResolveGoalStatus(status, std::nullopt), status);
  }
}

TEST(ActionOutcome, APolledTerminalStatusWinsOverTheRecordedOne)
{
  // They cannot legitimately disagree; if they ever do, the value the rest of
  // the node already acted on is the one to keep.
  EXPECT_EQ(ResolveGoalStatus(kGoalStatusCanceled, GoalOutcome::kSucceeded), kGoalStatusCanceled);
  EXPECT_EQ(ResolveGoalStatus(kGoalStatusAborted, GoalOutcome::kSucceeded), kGoalStatusAborted);
}

TEST(ActionOutcome, TheSlotStartsEmptyAndResetClearsIt)
{
  ActionOutcomeSlot slot;
  EXPECT_FALSE(slot.Get().has_value());
  slot.Record(GoalOutcome::kAborted);
  ASSERT_TRUE(slot.Get().has_value());
  slot.Reset();
  EXPECT_FALSE(slot.Get().has_value());
}

TEST(ActionOutcome, TheFirstVerdictWinsSoALateDuplicateCannotReopenAGoal)
{
  ActionOutcomeSlot slot;
  slot.Record(GoalOutcome::kSucceeded);
  slot.Record(GoalOutcome::kAborted);
  EXPECT_EQ(slot.Get(), GoalOutcome::kSucceeded);
}

TEST(ActionOutcome, RecordingFromAnotherThreadWhileTicking)
{
  // The result callback runs on an executor thread, the poll on the BT tick.
  ActionOutcomeSlot slot;
  std::thread writer(
      [&slot]()
      {
        slot.Record(GoalOutcome::kSucceeded);
      });
  for (int i = 0; i < 1000; ++i)
  {
    (void)ResolveGoalStatus(kGoalStatusAccepted, slot.Get());
  }
  writer.join();
  EXPECT_EQ(ResolveGoalStatus(kGoalStatusAccepted, slot.Get()), kGoalStatusSucceeded);
}

}  // namespace
