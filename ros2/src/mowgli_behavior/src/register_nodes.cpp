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

#include "mowgli_behavior/calibration_nodes.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_behavior/docking_nodes.hpp"
#include "mowgli_behavior/navigation_nodes.hpp"
#include "mowgli_behavior/recording_nodes.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include "mowgli_behavior/utility_nodes.hpp"

namespace mowgli_behavior
{

void registerAllNodes(BT::BehaviorTreeFactory& factory)
{
  // Condition nodes
  factory.registerNodeType<IsEmergency>("IsEmergency");
  factory.registerNodeType<IsCharging>("IsCharging");
  factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
  factory.registerNodeType<IsRainDetected>("IsRainDetected");
  factory.registerNodeType<NeedsDocking>("NeedsDocking");
  factory.registerNodeType<IsBatteryAbove>("IsBatteryAbove");
  factory.registerNodeType<IsCommand>("IsCommand");

  factory.registerNodeType<IsGPSFixed>("IsGPSFixed");
  factory.registerNodeType<ReplanNeeded>("ReplanNeeded");
  factory.registerNodeType<IsBoundaryViolation>("IsBoundaryViolation");
  factory.registerNodeType<IsLethalBoundaryViolation>("IsLethalBoundaryViolation");
  factory.registerNodeType<IsNewRain>("IsNewRain");
  factory.registerNodeType<IsRainModeAtLeast>("IsRainModeAtLeast");
  factory.registerNodeType<IsResumeUndockAllowed>("IsResumeUndockAllowed");
  factory.registerNodeType<IsChargingProgressing>("IsChargingProgressing");
  factory.registerNodeType<PreFlightCheck>("PreFlightCheck");
  factory.registerNodeType<Nav2Active>("Nav2Active");
  factory.registerNodeType<IsObstacleStuck>("IsObstacleStuck");
  factory.registerNodeType<WasRecentlyInCollisionStop>("WasRecentlyInCollisionStop");

  // Action nodes
  factory.registerNodeType<SetMowerEnabled>("SetMowerEnabled");
  factory.registerNodeType<StopMoving>("StopMoving");
  factory.registerNodeType<ClearCostmap>("ClearCostmap");
  factory.registerNodeType<PublishHighLevelStatus>("PublishHighLevelStatus");
  factory.registerNodeType<WaitForDuration>("WaitForDuration");
  factory.registerNodeType<WaitForGpsFix>("WaitForGpsFix");
  factory.registerNodeType<NavigateToPose>("NavigateToPose");
  factory.registerNodeType<NavigateInsideBoundary>("NavigateInsideBoundary");
  factory.registerNodeType<BackUp>("BackUp");
  factory.registerNodeType<ClearCommand>("ClearCommand");
  factory.registerNodeType<EndSession>("EndSession");
  factory.registerNodeType<IncrementSkippedSwaths>("IncrementSkippedSwaths");
  factory.registerNodeType<SaveObstacles>("SaveObstacles");
  factory.registerNodeType<SetNavMode>("SetNavMode");
  factory.registerNodeType<WasRainingAtStart>("WasRainingAtStart");
  factory.registerNodeType<RecordUndockStart>("RecordUndockStart");
  factory.registerNodeType<CalibrateHeadingFromUndock>("CalibrateHeadingFromUndock");
  factory.registerNodeType<SeedYawFromMotion>("SeedYawFromMotion");
  factory.registerNodeType<DockRobot>("DockRobot");
  factory.registerNodeType<UndockRobot>("UndockRobot");
  factory.registerNodeType<RecordResumeUndockFailure>("RecordResumeUndockFailure");
  factory.registerNodeType<ResetEmergency>("ResetEmergency");

  // Swath-segmented coverage nodes. GetNextUnmowedArea iterates areas,
  // PlanCoverageArea plans the full area (F2C, DISCONTINUOUS), FollowStrip
  // follows it one swath at a time.
  factory.registerNodeType<GetNextUnmowedArea>("GetNextUnmowedArea");
  factory.registerNodeType<FollowStrip>("FollowStrip");
  factory.registerNodeType<TransitToStrip>("TransitToStrip");
  factory.registerNodeType<DetourAroundObstacle>("DetourAroundObstacle");

  // F2C-backed full-area planner (mowgli_coverage plan_coverage segments).
  // Output goes into ctx->current_strip_path; FollowStrip consumes it.
  factory.registerNodeType<PlanCoverageArea>("PlanCoverageArea");

  // Area recording node
  factory.registerNodeType<RecordArea>("RecordArea");
}

}  // namespace mowgli_behavior
