# Research / Decision: obstacle handling during coverage with full-coverage guarantees

> **Historical document (2026-05)** — describes the state at that time. Current reference: [`docs/claude/codemaps/mowgli_coverage.md`](claude/codemaps/mowgli_coverage.md) and [`docs/claude/codemaps/mowgli_behavior.md`](claude/codemaps/mowgli_behavior.md).

**Status:** research only — nothing implemented. Decision document.
**Date:** 2026-05-30
**Author:** investigation + multi-source web research (deep-research harness: 22 sources, 77 claims extracted, 25 adversarially verified 2/3-vote, 0 refuted).

## Problem

A large obstacle — sometimes **dynamic** (a person standing then walking away, a temporary object) — blocks one or more coverage swaths. Today:

- F2C plans the area **once, no replan**, and does **not** know about real-time LiDAR obstacles (`get_remaining_area_polygon` only subtracts operator-**promoted** obstacles + dock + `mow_progress`). It lays swaths straight through the obstacle and relies on the follower's local deviation.
- For a **big** obstacle, FTC can't deviate far enough (`max_lateral_deviation ≈ 1.5 m`), the swath is abandoned, and — with no replan — **the region behind the obstacle is never mowed** → permanent un-mowed patch.

Naive replanning is undesirable because:
1. **Dynamic obstacles** appear/disappear, so a replan freezes a *transient* state into the persistent plan.
2. **No determinism guarantee**: a re-plan over the not-yet-mowed zones need not re-cover them consistently → gaps.

**Desired behaviour:** when the mower reaches an obstacle mid-swath, **circumnavigate** it (drive around its perimeter) and **resume coverage on the far side**, with a guarantee that all reachable un-mowed area is eventually covered.

## TL;DR recommendation

Implement **option (b): a behavior-tree recovery** that, on a blocked swath, (optionally) boundary-follows the obstacle, then **re-queries F2C on the remaining region defined by the `mow_progress` grid** — *not* a replan that ingests the transient obstacle as geometry.

The `mow_progress` grid + the (now multi-lobe, hole-aware) `get_remaining_area_polygon` we already have **is the completeness primitive** the whole literature relies on. The stack is already ~70% of the way to option (b).

> **Central honest caveat:** *none* of the surveyed full-coverage guarantees hold under genuinely **dynamic** appear/disappear obstacles — they all assume static, or "discovered-online but permanent" obstacles. The robust answer for the dynamic case is not a proof but a **defer-and-retry** heuristic (below).

## 1. Landscape of methods (verified)

### Coverage-path-planning with obstacles
- **Morse / boustrophedon cellular decomposition** (Choset): split free space into cells where back-and-forth sweep is complete; obstacles become cell boundaries. — *Choset, CMU.*
- **Sensor-based incremental coverage, provably complete ONLINE** — guarantees covering all reachable cells **while detouring around obstacles discovered at runtime**, building map and coverage simultaneously. — *Acar & Choset, IJRR 21(4), 2002.* `https://journals.sagepub.com/doi/10.1177/027836402320556368`
- **Space-filling curves (SFC) over a grid** — completeness proved; detours around obstacles blocking cells. — *Wakode & Sinha.* `https://arxiv.org/abs/2209.01426`
- **OARP-Replan (anytime replanning)** — explicitly argues *against* full online recompute (too costly / unstable), in favour of anytime incremental replanning. Validates the instinct to avoid naive replans. — *Ramesh et al., RA-L 2024.* `https://arxiv.org/abs/2311.17837`

### Obstacle-aware swath-following controllers ("FTC fidelity but obstacle-aware")
- **Guiding Vector Fields (GVF)** — composite guiding vector fields **detour around an obstacle and return to the same path with minimal deviation**, with formally proven convergence/completeness. This is the "circumnavigate then resume the swath" behaviour at the *control* layer. — `https://arc.aiaa.org/doi/10.2514/1.G004053`, `https://arxiv.org/pdf/2303.15869`, `https://arxiv.org/pdf/2003.10012`
- ⚠️ Sources are control-theory / UAV; **not validated on a ground mower**. Treat as a future upgrade to FTC's deviation, not a ready brick.

### ROS2 coverage ecosystem (alternatives to F2C)
- **opennav_coverage** = Fields2Cover **v1.2.1**, **no interior-void/hole support** → *behind* our F2C v2. `https://github.com/open-navigation/opennav_coverage`
- **nobleo/full_coverage_path_planner** = Boustrophedon + Backtracking (BA*), **static-map only**, no online re-coverage. `https://github.com/nobleo/full_coverage_path_planner`
- **Conclusion:** no ROS2 alternative beats the current stack for dynamic obstacles. Do **not** switch planners.

### Commercial mowers (Husqvarna, Worx, Navimow, iRobot/Roborock)
- **No verifiable claims** — only marketing pages. Nothing usable on their internal re-coverage logic. (Navimow/Yarbo confirm "avoidance" exists but say nothing about completeness guarantees.)

## 2. The three architectural options, evaluated

| Option | Verdict | Rationale |
|---|---|---|
| **(a)** Local controller behaviour (GVF-style: detour + return to swath) | **Complement, not sufficient alone** | Elegant, keeps <10 mm swath fidelity, but does not guarantee the region *behind* the obstacle gets covered if the detour can't complete. Not ground-validated. |
| **(b)** BT recovery: detect blocked swath → boundary-follow around obstacle → **re-query F2C on remaining region (holes = `mow_progress`)** | ✅ **RECOMMENDED** | Reuses the `mow_progress` grid (= the literature's completeness primitive). No double-mow (F2C only plans the un-mowed remainder). **Deterministic**: the remaining region is defined by what is *actually* mowed, not by a transient plan. |
| **(c)** Planner ingests the live obstacle as a hole + replan | ❌ **Bad for dynamic** | Exactly the identified trap: a pedestrian becomes a *permanent* hole in the plan; the transient obstacle is frozen into geometry. |

## 3. The hard part — dynamic obstacles (the honest answer)

No method proves completeness under appear/disappear obstacles. The only robust pattern in the literature is **incremental cell coverage with deferred RETRY**:

1. **Never freeze a transient obstacle into the persistent plan.** A blocked swath → mark cells **"deferred" (neither mowed nor DEAD)**, not a permanent hole.
2. **Continue the rest of the plan**, then at area end **re-query `get_remaining_area_polygon`** (which already returns the un-mowed region as holes/lobes — recently hardened) and do a **second pass** over the deferred region. If the pedestrian has left, the cell is reachable again → mowed.
3. **Distinguish transient vs permanent with a counter**: blocked once = deferred + retry; blocked across N consecutive passes = promote to a persistent obstacle (DEAD) and *only then* ingest it into geometry. That is the transient/permanent boundary.

The current architecture (F2C plans the holed `remaining` + FTC follows + `mow_progress` grid) is already ~70% of option (b). Missing pieces:
- (i) detect "swath abandoned → mark deferred" instead of silently abandoning,
- (ii) the optional circumnavigation / boundary-follow loop,
- (iii) the second pass over the deferred region,
- (iv) the transient→permanent counter.

## Suggested implementation order (when we leave research phase)

1. **Deferred-cells + second pass** — lowest risk, reuses existing `mow_progress` + `get_remaining_area_polygon`. Gives most of the win.
2. **Boundary-following circumnavigation** — drive around the obstacle perimeter to cover up to its edge before deferring the rest.
3. **GVF** — far-future upgrade to FTC's local deviation for cleaner detour-and-return.

## Open architectural dependency

This builds on the deeper gap tracked in memory `project_coverage_obstacle_gap`: F2C currently plans through un-promoted obstacles with no replan. Option (b) is the path to closing it **without** violating CLAUDE.md invariant 5 (costmap obstacles disabled in coverage; collision_monitor handles real-time avoidance) — because the obstacle is handled by a BT recovery + the `mow_progress`/deferred grid, not by feeding the live costmap into the coverage planner.

## Sources

- Acar & Choset, *Sensor-based coverage with provable completeness*, IJRR 21(4), 2002 — `https://journals.sagepub.com/doi/10.1177/027836402320556368`
- Wakode & Sinha, *SFC complete coverage* — `https://arxiv.org/abs/2209.01426`
- Ramesh et al., *OARP-Replan* (RA-L 2024) — `https://arxiv.org/abs/2311.17837`
- GVF path-following + obstacle avoidance — `https://arc.aiaa.org/doi/10.2514/1.G004053`, `https://arxiv.org/pdf/2303.15869`, `https://arxiv.org/pdf/2003.10012`
- Choset, coverage survey (CMU) — `https://www.ri.cmu.edu/pub_files/pub4/choset_howie_2000_3/choset_howie_2000_3.pdf`
- `https://github.com/open-navigation/opennav_coverage` (F2C 1.2.1, no holes)
- `https://github.com/nobleo/full_coverage_path_planner` (BA*, static-map only)
- Survey: coverage path planning in dynamic environments — `https://www.researchgate.net/publication/390360515`

**Verification note:** all factual claims above were adversarially verified (2/3-vote) by the research harness. Control-theory/GVF results are peer-reviewed (IEEE TAC/TCST, AIAA) but UAV-domain, unvalidated on ground mowers. Commercial-mower internals produced **no** verifiable claims. No surveyed guarantee covers genuinely dynamic obstacles.
