// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the same-unit resume decision (unit_resume.hpp). ROS-free.

#include "gtest/gtest.h"
#include "mowgli_behavior/unit_resume.hpp"

namespace
{

using mowgli_behavior::DecideUnitResume;
using mowgli_behavior::UnitResumeCfg;

TEST(UnitResume, ResumesTheSerpentineUnitCancelledNearItsStart)
{
  // Field 2026-09-17: 30 127-pose unit cancelled at pose 280, cursor stepped to 308.
  const auto d = DecideUnitResume(30127, 280, 308, 0);
  EXPECT_TRUE(d.resume);
  EXPECT_EQ(d.consecutive, 0u + 1u)
      << "280 poses is real progress: counter cleared, then one spent";
}

TEST(UnitResume, GivesUpAfterConsecutiveResumesWithoutProgress)
{
  const UnitResumeCfg cfg;
  std::size_t spent = 0;
  for (std::size_t i = 0; i < cfg.max_consecutive; ++i)
  {
    const auto d = DecideUnitResume(5000, 3, 30, spent, cfg);
    ASSERT_TRUE(d.resume) << "resume " << i;
    spent = d.consecutive;
  }
  EXPECT_FALSE(DecideUnitResume(5000, 3, 30, spent, cfg).resume)
      << "a deterministic re-abort loop must end";
}

TEST(UnitResume, RealProgressClearsTheCounter)
{
  const UnitResumeCfg cfg;
  const auto d = DecideUnitResume(5000, cfg.progress_reset_poses, 200, cfg.max_consecutive, cfg);
  EXPECT_TRUE(d.resume);
  EXPECT_EQ(d.consecutive, 1u);
}

TEST(UnitResume, ATinyRemainderIsNotWorthATransit)
{
  EXPECT_FALSE(DecideUnitResume(100, 70, 90, 0).resume);
}

TEST(UnitResume, CursorAtOrPastTheEndNeverResumes)
{
  EXPECT_FALSE(DecideUnitResume(100, 99, 100, 0).resume);
  EXPECT_FALSE(DecideUnitResume(100, 99, 250, 0).resume);
}

}  // namespace
