#!/usr/bin/env python3
"""In-container patch: swap FollowCoveragePath MPPI/RotationShim -> DWB for an
A/B test. Backs up the original first. Loses comments in the install copy
(runtime-only; the committed source is untouched)."""
import shutil
import sys

import yaml

P = "/ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/nav2_params.yaml"
BAK = P + ".predwb.bak"

if len(sys.argv) > 1 and sys.argv[1] == "restore":
    shutil.copy(BAK, P)
    print("restored original nav2_params.yaml")
    sys.exit(0)

shutil.copy(P, BAK)
with open(P) as f:
    doc = yaml.safe_load(f)

cs = doc["controller_server"]["ros__parameters"]
cs["FollowCoveragePath"] = {
    "plugin": "dwb_core::DWBLocalPlanner",
    # diff-drive velocity envelope (match MPPI); min_vel_x 0 => allow in-place pivot
    "min_vel_x": 0.0, "max_vel_x": 0.20,
    "min_vel_y": 0.0, "max_vel_y": 0.0,
    "max_vel_theta": 1.9,
    "min_speed_xy": 0.0, "max_speed_xy": 0.20, "min_speed_theta": 0.0,
    "acc_lim_x": 1.0, "acc_lim_y": 0.0, "acc_lim_theta": 2.5,
    "decel_lim_x": -1.0, "decel_lim_y": 0.0, "decel_lim_theta": -2.5,
    "vx_samples": 20, "vy_samples": 0, "vtheta_samples": 40,
    "sim_time": 1.7,
    "linear_granularity": 0.05, "angular_granularity": 0.025,
    "transform_tolerance": 0.3, "xy_goal_tolerance": 0.15,
    "trans_stopped_velocity": 0.10,
    "short_circuit_trajectory_evaluation": True, "stateful": True,
    "prune_plan": True, "prune_distance": 1.0, "forward_prune_distance": 1.0,
    # Path-following heavy (follow exactly + return to path), modest obstacle.
    "critics": ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
                "PathAlign", "PathDist", "GoalDist"],
    "BaseObstacle.scale": 0.02,
    "PathAlign.scale": 32.0, "PathAlign.forward_point_distance": 0.1,
    "GoalAlign.scale": 24.0, "GoalAlign.forward_point_distance": 0.1,
    "PathDist.scale": 32.0,
    "GoalDist.scale": 24.0,
    "RotateToGoal.scale": 32.0,
    "RotateToGoal.slowing_factor": 5.0,
    "RotateToGoal.lookahead_time": -1.0,
}
with open(P, "w") as f:
    yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)

# sanity: key sections still present
d2 = yaml.safe_load(open(P))
need = ["controller_server", "coverage_server", "collision_monitor", "global_costmap"]
ok = all(k in d2 for k in need)
print("patched FollowCoveragePath -> DWB; sections intact:", ok,
      "| plugin =", d2["controller_server"]["ros__parameters"]["FollowCoveragePath"]["plugin"])
