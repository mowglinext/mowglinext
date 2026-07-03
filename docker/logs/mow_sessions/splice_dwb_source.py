#!/usr/bin/env python3
"""Replace the FollowCoveragePath block (+ its '# -- Coverage controller' header)
in a source nav2_params*.yaml with the DWB (DISCONTINUOUS) block, preserving the
rest of the file (comments included)."""
import sys

DWB = '''    # ── Coverage controller: DWB on the DISCONTINUOUS F2C path ──────────────
    # The F2C path is DISCONTINUOUS: straight swaths joined by sharp heading-
    # jumps. DWB tracks the straights with heavy PathDist/PathAlign and PIVOTS
    # IN PLACE at each jump (min_vel_x 0 lets it sample vx=0 + vθ). This is DWB's
    # native strength (straight-line tracking + in-place rotation) and gives the
    # diff-drive in-place turns we want — no RotationShim wrapper needed.
    #
    # Replaces the RotationShim+MPPI block (recoverable from git history) which
    # tracked straights well but CUT the CC-Dubins U-turn arcs (0.4–0.6 m) — a
    # consequence of MPPI on CONTINUOUS arcs. On DISCONTINUOUS, DWB pivoting at
    # the jump is the correct motion, not a spin.
    #
    # Deadband is handled downstream (hardware_bridge min_linear_vel=0.05 clamp +
    # vx compensator + wz duty-cycling; controller thresholds 0.0), so DWB can
    # emit natural slow commands and the firmware clears the breakaway.
    #
    # SPIN CONTROL: min_vel_x 0 is required for the pivots but invites a runaway
    # rotate on CONTINUOUS; here the path EXPECTS the pivot, and RotateToGoal is
    # weakened + Oscillation tuned so it can't lock into an endless spin.
    FollowCoveragePath:
      plugin: "dwb_core::DWBLocalPlanner"
      # diff-drive envelope; min_vel_x 0 => can pivot in place at heading-jumps
      min_vel_x: 0.0
      max_vel_x: 0.20                 # coverage mow speed (< 0.25 transit ceiling)
      min_vel_y: 0.0
      max_vel_y: 0.0
      max_vel_theta: 1.5              # pivot authority, capped vs runaway spin
      min_speed_xy: 0.0
      max_speed_xy: 0.20
      min_speed_theta: 0.0
      acc_lim_x: 1.0
      acc_lim_y: 0.0
      acc_lim_theta: 2.5
      decel_lim_x: -1.0
      decel_lim_y: 0.0
      decel_lim_theta: -2.5
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40             # dense θ grid => finer pivot/arc resolution
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.3       # ARM TF jitter envelope
      xy_goal_tolerance: 0.15
      trans_stopped_velocity: 0.10
      short_circuit_trajectory_evaluation: true
      stateful: true
      prune_plan: true
      prune_distance: 1.0
      forward_prune_distance: 1.0
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathDist.scale: 32.0           # follow the path exactly + return to it
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalDist.scale: 24.0
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      RotateToGoal.scale: 5.0        # weak: terminal heading only, no mid-path spin drive
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
      Oscillation.scale: 1.0         # breaks a lock-in spin without blocking legit pivots
      Oscillation.oscillation_reset_dist: 0.05
      Oscillation.oscillation_reset_angle: 0.2
      Oscillation.oscillation_reset_time: -1.0
'''


def splice(path):
    lines = open(path).read().splitlines(keepends=True)
    h = next(i for i, l in enumerate(lines)
             if l.lstrip().startswith("# ── Coverage controller"))
    f = next(i for i in range(h, len(lines)) if lines[i].startswith("    FollowCoveragePath:"))
    e = f + 1
    while e < len(lines):
        l = lines[e]
        if l.strip() == "":
            e += 1
            continue
        indent = len(l) - len(l.lstrip())
        if indent <= 4:
            break
        e += 1
    new = lines[:h] + [DWB] + lines[e:]
    open(path, "w").write("".join(new))
    print(f"{path}: replaced lines {h+1}..{e} with DWB(DISCONTINUOUS) block")


for p in sys.argv[1:]:
    splice(p)
