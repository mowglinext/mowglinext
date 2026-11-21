// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <chrono>
#include <string>
#include <vector>

#include "mowgli_behavior/boot_hold.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BootHold;
using namespace std::chrono_literals;

namespace
{
const BootHold::Clock::time_point t0{};
}

TEST(BootHold, RunsEveryStepInOrderAndFinishesWhenAllSucceeded)
{
  std::vector<std::string> order;
  BootHold hold(t0, 3s);
  hold.add_step("blade",
                [&]
                {
                  order.push_back("blade");
                  return true;
                });
  hold.add_step("follow_path",
                [&]
                {
                  order.push_back("follow_path");
                  return true;
                });
  EXPECT_FALSE(hold.done());
  EXPECT_TRUE(hold.poll(t0 + 100ms));
  EXPECT_TRUE(hold.done());
  EXPECT_FALSE(hold.timed_out());
  EXPECT_EQ(order, (std::vector<std::string>{"blade", "follow_path"}));
}

TEST(BootHold, RetriesAStepThatIsNotReadyAndNeverRepeatsOneThatSucceeded)
{
  int blade = 0;
  int nav = 0;
  BootHold hold(t0, 3s);
  hold.add_step("blade",
                [&]
                {
                  ++blade;
                  return true;
                });
  hold.add_step("navigate_to_pose",
                [&]
                {
                  return ++nav >= 3;  // the action server shows up on the third poll
                });
  EXPECT_FALSE(hold.poll(t0 + 100ms));
  EXPECT_FALSE(hold.poll(t0 + 200ms));
  EXPECT_TRUE(hold.poll(t0 + 300ms));
  EXPECT_EQ(blade, 1);  // a second blade-off or goal cancel would be a repeat
  EXPECT_EQ(nav, 3);
  EXPECT_TRUE(hold.poll(t0 + 400ms));
  EXPECT_EQ(nav, 3);
}

TEST(BootHold, GivesUpAtTheDeadlineAndNamesWhatNeverRan)
{
  BootHold hold(t0, 3s);
  hold.add_step("blade",
                []
                {
                  return true;
                });
  hold.add_step("follow_path",
                []
                {
                  return false;
                });  // no Nav2 yet: a cold boot
  EXPECT_FALSE(hold.poll(t0 + 2900ms));
  EXPECT_TRUE(hold.poll(t0 + 3s));
  EXPECT_TRUE(hold.done());
  EXPECT_TRUE(hold.timed_out());
  EXPECT_EQ(hold.pending(), (std::vector<std::string>{"follow_path"}));
}

TEST(BootHold, AGroupOnlyRunsOnceEveryEarlierStepHasSucceeded)
{
  std::vector<std::string> order;
  int parent_polls = 0;
  BootHold hold(t0, 3s);
  hold.add_step("parent",
                [&]
                {
                  order.push_back("parent");
                  return ++parent_polls >= 3;  // acknowledged on the third poll
                });
  hold.next_group();
  hold.add_step("controller",
                [&]
                {
                  order.push_back("controller");
                  return true;
                });
  EXPECT_FALSE(hold.poll(t0 + 100ms));
  EXPECT_FALSE(hold.poll(t0 + 200ms));
  EXPECT_EQ(order, (std::vector<std::string>{"parent", "parent"}));  // the controller waits
  EXPECT_TRUE(hold.poll(t0 + 300ms));
  EXPECT_EQ(order, (std::vector<std::string>{"parent", "parent", "parent", "controller"}));
}

TEST(BootHold, AStepOfALaterGroupIsGivenUpOnWithTheOthersAtTheDeadline)
{
  BootHold hold(t0, 3s);
  hold.add_step("parent",
                []
                {
                  return false;
                });
  hold.next_group();
  hold.add_step("controller",
                []
                {
                  return true;
                });
  EXPECT_TRUE(hold.poll(t0 + 3s));
  EXPECT_TRUE(hold.timed_out());
  EXPECT_EQ(hold.pending(), (std::vector<std::string>{"parent", "controller"}));
}

TEST(BootHold, ARunningHoldIsNotDoneBeforeItsFirstPoll)
{
  BootHold hold(t0, 3s);
  hold.add_step("blade",
                []
                {
                  return true;
                });
  EXPECT_FALSE(hold.done());
}

TEST(BootHold, ZeroTimeoutDisablesTheHold)
{
  int runs = 0;
  BootHold hold(t0, 0s);
  hold.add_step("blade",
                [&]
                {
                  ++runs;
                  return true;
                });
  EXPECT_TRUE(hold.done());
  EXPECT_TRUE(hold.poll(t0));
  EXPECT_EQ(runs, 0);
}
