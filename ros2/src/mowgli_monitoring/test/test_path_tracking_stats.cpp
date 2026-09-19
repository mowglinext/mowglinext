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
// Unit tests for the shared path-tracking reduction
// (mowgli_interfaces/path_tracking_stats.hpp). ROS-free by construction.

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "mowgli_interfaces/path_tracking_stats.hpp"

namespace
{

using mowgli_interfaces::path_tracking::kGoalRestartIndexDrop;
using mowgli_interfaces::path_tracking::Sample;
using mowgli_interfaces::path_tracking::Summary;

constexpr double kEps = 1e-9;

TEST(PathTrackingStats, ReportsNoSamplesBeforeAnythingArrives)
{
  // Arrange
  const Summary summary;

  // Act / Assert
  EXPECT_FALSE(summary.HasSamples());
  EXPECT_EQ(summary.Count(), 0u);
  EXPECT_NEAR(summary.MaxAbsPositionErrorM(), 0.0, kEps);
  EXPECT_NEAR(summary.MeanAbsPositionErrorM(), 0.0, kEps);
  EXPECT_NEAR(summary.RmsPositionErrorM(), 0.0, kEps);
}

TEST(PathTrackingStats, AggregatesAbsoluteErrorRegardlessOfSide)
{
  // Arrange — same magnitude on both sides of the path.
  Summary summary;

  // Act
  summary.Add({0.02, 0.0, 0});
  summary.Add({-0.02, 0.0, 1});

  // Assert — a left/right pair must NOT average itself away to zero.
  EXPECT_EQ(summary.Count(), 2u);
  EXPECT_NEAR(summary.MeanAbsPositionErrorM(), 0.02, kEps);
  EXPECT_NEAR(summary.MaxAbsPositionErrorM(), 0.02, kEps);
  EXPECT_NEAR(summary.RmsPositionErrorM(), 0.02, kEps);
}

TEST(PathTrackingStats, KeepsTheLastSignedErrorSoTheSideStaysKnown)
{
  // Arrange
  Summary summary;

  // Act
  summary.Add({0.05, 0.0, 0});
  summary.Add({-0.03, 0.0, 1});

  // Assert
  EXPECT_NEAR(summary.LastPositionErrorM(), -0.03, kEps);
  EXPECT_EQ(summary.LastPathIndex(), 1u);
}

TEST(PathTrackingStats, RmsExceedsTheMeanWhenOneExcursionDominates)
{
  // Arrange — nine tight samples and one wide swing, the shape that leaves an
  // uncut band while the mean still looks healthy.
  Summary summary;

  // Act
  for (std::uint32_t i = 0; i < 9; ++i)
  {
    summary.Add({0.01, 0.0, i});
  }
  summary.Add({0.30, 0.0, 9});

  // Assert
  EXPECT_NEAR(summary.MaxAbsPositionErrorM(), 0.30, kEps);
  EXPECT_GT(summary.RmsPositionErrorM(), summary.MeanAbsPositionErrorM());
}

TEST(PathTrackingStats, TracksTheWorstHeadingErrorInEitherDirection)
{
  // Arrange
  Summary summary;

  // Act
  summary.Add({0.0, 0.1, 0});
  summary.Add({0.0, -0.4, 1});
  summary.Add({0.0, 0.2, 2});

  // Assert
  EXPECT_NEAR(summary.MaxAbsHeadingErrorRad(), 0.4, kEps);
}

TEST(PathTrackingStats, StartsANewEpisodeWhenThePathIndexRestarts)
{
  // Arrange — a goal that tracked badly, then a new goal starting back at 0.
  Summary summary;
  summary.Add({0.50, 0.0, 100});
  summary.Add({0.50, 0.0, 101});

  // Act
  summary.Add({0.01, 0.0, 0});

  // Assert — the previous goal's excursion must not haunt the new one.
  EXPECT_EQ(summary.Count(), 1u);
  EXPECT_NEAR(summary.MaxAbsPositionErrorM(), 0.01, kEps);
}

TEST(PathTrackingStats, SmallIndexRetreatIsAReplanNotANewGoal)
{
  // Arrange
  Summary summary;
  summary.Add({0.40, 0.0, 20});

  // Act — a retreat within the slack: same goal, path updated under the robot.
  summary.Add({0.01, 0.0, 20 - kGoalRestartIndexDrop});

  // Assert
  EXPECT_EQ(summary.Count(), 2u);
  EXPECT_NEAR(summary.MaxAbsPositionErrorM(), 0.40, kEps);
}

TEST(PathTrackingStats, DropsNonFiniteSamplesInsteadOfPoisoningTheSummary)
{
  // Arrange
  Summary summary;
  summary.Add({0.02, 0.0, 0});

  // Act
  summary.Add({std::numeric_limits<double>::quiet_NaN(), 0.0, 1});
  summary.Add({0.03, std::numeric_limits<double>::infinity(), 2});

  // Assert
  EXPECT_EQ(summary.Count(), 1u);
  EXPECT_TRUE(std::isfinite(summary.MeanAbsPositionErrorM()));
  EXPECT_NEAR(summary.MaxAbsPositionErrorM(), 0.02, kEps);
}

TEST(PathTrackingStats, ResetClearsEverything)
{
  // Arrange
  Summary summary;
  summary.Add({0.40, 0.3, 42});

  // Act
  summary.Reset();

  // Assert
  EXPECT_FALSE(summary.HasSamples());
  EXPECT_NEAR(summary.MaxAbsPositionErrorM(), 0.0, kEps);
  EXPECT_NEAR(summary.MaxAbsHeadingErrorRad(), 0.0, kEps);
  EXPECT_EQ(summary.LastPathIndex(), 0u);
}

TEST(PathTrackingStats, FirstSampleAtAHighIndexIsNotTreatedAsARestart)
{
  // Arrange — a resumed sub-path legitimately starts mid-plan.
  Summary summary;

  // Act
  summary.Add({0.02, 0.0, 250});

  // Assert
  EXPECT_EQ(summary.Count(), 1u);
  EXPECT_EQ(summary.LastPathIndex(), 250u);
}

}  // namespace
