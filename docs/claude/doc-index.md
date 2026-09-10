# Documentation Index

> Index generated 2026-09-03 at f21729e9.
> Every human-written `.md` in the repo, classified so a future Claude knows which document is
> authoritative and which is a historical record. Excludes `ros2/src/opennav_coverage/**` and
> `ros2/src/external/**` (vendored submodules) and `node_modules/`.

## How to read this index

- **status** — `current` (trust it), `historical` (a dated record of *what was decided/found then*),
  `superseded` (a newer doc replaced it), `generated` (produced from code, regenerate rather than edit).
- **audience** — `claude` (agent context), `contributor` (works on the code), `operator` (runs a mower),
  `maintainer` (release/repo chores).
- **Precedence when two docs disagree:** root [`CLAUDE.md`](../../CLAUDE.md) wins over everything, then
  the codemaps (regenerated from code at f21729e9), then `wiki/`, then anything dated.
- Several codemaps say "Loaded on demand from `ros2/CLAUDE.md`" (or `gui/`, `firmware/`, `install/`,
  `docker/`, `sensors/`). Those nested `CLAUDE.md` files are written in the **last phase** of this
  documentation pass — a reference to one is correct, not a broken link.
- A doc listed as `current` can still contain a stale *section*; where that is known it is called out
  inline. When in doubt, check the code, not the prose.

## Claude-facing references (load on demand)

Always-on (read first, not on demand):

| Doc | What it is |
|-----|-----------|
| [`CLAUDE.md`](../../CLAUDE.md) | **Authoritative.** Safety, monorepo layout, the 16 Architecture Invariants, What NOT to Do. Everything else defers to it. |
| [`.claude/rules/ros2.md`](../../.claude/rules/ros2.md) | ROS2 node/QoS/topic-naming/launch/testing/ARM conventions for this stack. |
| [`AGENTS.md`](../../AGENTS.md) | Repo-local agent rules: `clang-format` cleanliness for `ros2/` C++, non-root build/tool notes. |

On-demand reference material (`docs/claude/`):

| Doc | Read it when… |
|-----|---------------|
| [`high-level-api.md`](high-level-api.md) | Touching `HighLevelControl.srv` / `HighLevelStatus.msg`, BT states, area recording, stop-hold/pause, manual mowing, or the GUI scheduler + IrriSense soil gate. |
| [`ros2-specifics.md`](ros2-specifics.md) | Working on the localizer, TF chain, navigation, coverage, GPS fusion, IMU calibration, or Nav2 tuning — the detail behind the invariants. |
| [`ros-interfaces.md`](ros-interfaces.md) | Looking up a topic, service, action, twist_mux lane or TF frame — every endpoint the stack creates, fully resolved, with the `node · file:line` that creates it. **Generated** at f21729e9; regenerate rather than hand-patch. |
| [`parameters.md`](parameters.md) | Chasing a config knob — where its default lives, which node consumes it, whether a launch file injects it (a key nothing injects is `INERT`), and whether the GUI can edit it. Read alongside CLAUDE.md Invariant 15. **Generated** at f21729e9; regenerate rather than hand-patch. |
| [`commands.md`](commands.md) | Building/testing, or after changing `.msg`/`.srv` files (the three code-generation targets). |
| [`testing-ci.md`](testing-ci.md) | Running or adding tests — every suite, the exact command, and the workflow that gates it (several are gated by nothing). **Generated** at f21729e9; regenerate rather than hand-patch. |
| [`session-monitoring.md`](session-monitoring.md) | Running a mowing/tuning test — record the JSONL session timeline in parallel. |
| [`contributing.md`](contributing.md) | Code style, commit conventions, git/branch workflow, recommended skills and agents. |

Codemaps (`docs/claude/codemaps/`, 18 files) — per-area "where to look" maps, **generated** from the
tree at f21729e9; regenerate when files are added or removed rather than hand-patching:

| Codemap | Read it when… |
|---------|---------------|
| [`fusion_graph.md`](codemaps/fusion_graph.md) | GTSAM iSAM2 localizer: factors, gates, keyframe map, graph persistence, diagnostics. |
| [`mowgli_behavior.md`](codemaps/mowgli_behavior.md) | BehaviorTree.CPP v4 mission executor, guards, mow/home/record/manual/stop branches, coverage-resume state. |
| [`mowgli_coverage.md`](codemaps/mowgli_coverage.md) | Fields2Cover v3 planner: rings, swaths, connectors, hole-free sub-paths, `plan_coverage` action. |
| [`mowgli_map.md`](codemaps/mowgli_map.md) | Area polygons, `areas.dat` + datum stamp, keepout mask, `mow_progress`, promoted obstacles, dock pose. |
| [`mowgli_nav2_plugins.md`](codemaps/mowgli_nav2_plugins.md) | FTCController (FSM, PID, stall crawl, obstacle deviation, cul-de-sac guard, reverse escape) and `PathProgressGoalChecker`. |
| [`mowgli_hardware.md`](codemaps/mowgli_hardware.md) | The ROS2↔STM32 boundary: COBS/CRC link, odometry, blade/e-stop, runtime tuning, dig detector. |
| [`mowgli_bringup.md`](codemaps/mowgli_bringup.md) | Launch files, the `mowgli_robot.yaml` template, Nav2 base+overlay params, twist_mux, URDF, param injection. |
| [`mowgli_localization.md`](codemaps/mowgli_localization.md) | GPS→ENU projection, COG/mag yaw, LiDAR scan pipeline, dock detection, calibration node. |
| [`mowgli_interfaces.md`](codemaps/mowgli_interfaces.md) | `.msg`/`.srv`/`.action` definitions and the header-only helpers (WGS84 projection, YAML splicing). |
| [`mowgli_monitoring.md`](codemaps/mowgli_monitoring.md) | `/diagnostics` aggregation and the optional MQTT bridge. |
| [`mowgli_simulation.md`](codemaps/mowgli_simulation.md) | Webots world/PROTO/URDF, `KinematicDrive`, sim-only helper nodes. |
| [`mowgli_leds.md`](codemaps/mowgli_leds.md) | Optional WS2812 status ring over SPI. |
| [`gui_backend.md`](codemaps/gui_backend.md) | Go server: foxglove bridge, settings backend, DB, scheduler, tools, MQTT/HomeKit. |
| [`gui_frontend.md`](codemaps/gui_frontend.md) | React 19 SPA: pages, hooks, the single multiplexed WebSocket. |
| [`firmware.md`](codemaps/firmware.md) | STM32 `ros_usbnode`: wheel/yaw loops, `ANTIDIG_*`, blade authority, wire protocol. |
| [`deploy.md`](codemaps/deploy.md) | Installer, compose fragments, config seeds, dockerised sensor drivers, `stack.sh`. |
| [`ros2_workspace_tooling.md`](codemaps/ros2_workspace_tooling.md) | `ros2/Makefile`, scripts, Dockerfile, systemd unit, E2E harnesses, submodules. |
| [`ci_repo_tooling.md`](codemaps/ci_repo_tooling.md) | GitHub Actions, drift gates, GHCR pipelines, wiki/Pages publishers, repo metadata. |

## Contributor docs (current)

| Doc | Audience | What it is |
|-----|----------|-----------|
| [`README.md`](../../README.md) | contributor | Project front page: what it does, quick start, monorepo table, doc links, license. |
| [`CONTRIBUTING.md`](../../CONTRIBUTING.md) | contributor | Fork→branch→PR flow, dev setup, code style, bug/feature reporting. |
| [`CODE_OF_CONDUCT.md`](../../CODE_OF_CONDUCT.md) | contributor | Contributor Covenant pledge, standards, enforcement contact. |
| [`SECURITY.md`](../../SECURITY.md) | contributor | Private vulnerability reporting via GitHub Security Advisories; scope; blade-safety notice. |
| [`.github/PULL_REQUEST_TEMPLATE.md`](../../.github/PULL_REQUEST_TEMPLATE.md) | contributor | PR body skeleton (summary / changes / component / testing / checklist). |
| [`ros2/README.md`](../../ros2/README.md) | contributor | The long-form ROS2 stack reference: architecture, packages, TF tree, topics, config, build, Docker, Webots sim. |
| [`gui/README.md`](../../gui/README.md) | contributor/operator | GUI install (Docker / MowgliNextOS podman unit), usage, dev workflow. |
| [`gui/web/tests/e2e/README.md`](../../gui/web/tests/e2e/README.md) | contributor | Playwright E2E: fully mocked data, scenario matrix, what each spec asserts. |
| [`docs/WEBOTS_SIM.md`](../WEBOTS_SIM.md) | contributor | **Read before editing the sim.** Five load-bearing ODE workarounds, each as RULE → ANCHOR → SYMPTOM. |
| [`docs/IMPORT_OPENMOWER_MAP.md`](../IMPORT_OPENMOWER_MAP.md) | contributor | OpenMower `map.json` import — live and wired (`openmower_import.go` + `ImportOpenMowerModal.tsx`). §6 (`.bag`) is design-only. |
| [`ros2/scripts/compute_nav2_params.md`](../../ros2/scripts/compute_nav2_params.md) | contributor | Design note for the physics-derived Nav2 param calculator. Its § *Future: launch injection* is still aspirational — the script has no caller in the tree. |
| [`ros2/scripts/diagnostics/README.md`](../../ros2/scripts/diagnostics/README.md) | contributor | Single-shot drivetrain/localization characterisation scripts; all five files still present. |
| [`ros2/src/mowgli_hardware/firmware/README.md`](../../ros2/src/mowgli_hardware/firmware/README.md) | contributor | COBS+CRC-16 wire protocol reference and the firmware-side drop-in files. |
| [`ros2/src/mowgli_leds/README.md`](../../ros2/src/mowgli_leds/README.md) | contributor/operator | WS2812 ring: wiring, SPI overlay, display semantics, encoding. Self-flagged **NOT hardware-verified**. |
| [`tools/motor/README.md`](../../tools/motor/README.md) | contributor/operator | `mowgli_tools` sidecar — the `tune_drive_pid` drive-PID assistant. |
| [`sensors/README.md`](../../sensors/README.md) | contributor | Supported sensor drivers, the Universal GNSS contract, how to add a sensor. |
| [`firmware/README.md`](../../firmware/README.md) | contributor | Firmware tree layout, safety warning (remove blades), supported hardware. |
| [`TODO-runtime-backups.md`](../../TODO-runtime-backups.md) | maintainer | **Still open**, not historical: `migrate_runtime_paths()` still backs up `.env` + `docker-compose.yaml` on every run and `install/lib/deploy.sh:276` points back at this file. None of its "future improvements" (rotation, `--no-backup`) exist yet. |

## Operator docs (current)

| Doc | What it is |
|-----|-----------|
| [`docs/FIRST_BOOT.md`](../FIRST_BOOT.md) | The post-install checklist: GUI up → RTK Fixed → IMU cal → yaw cal → dock pose → drive tuning → record area → first mow, plus troubleshooting. |
| [`wiki/User-Guide.md`](../../wiki/User-Guide.md) | Operator walkthrough of the live GUI, built from a real-robot session (also synced to the wiki). |
| [`docker/README.md`](../../docker/README.md) | Manual (non-installer) Docker Compose deployment: hardware requirements, quick start, config reference, container architecture. **Partially stale** — still documents SLAM Toolbox, `slam_mode`, `slam_toolbox.yaml`, which were removed (see stale claims below). |
| [`docker/config/mowgli/README.md`](../../docker/config/mowgli/README.md) | What the read-only `/ros2_ws/config/` bind mount is, which files are git-ignored, how parameter override works. |
| [`install/config/mowgli/README.md`](../../install/config/mowgli/README.md) | The installer's seed copy of the same note (shipped into `docker/config/mowgli/`). Near-duplicate by design. |

## Wiki pages

`wiki/*.md` is the **source of truth for the GitHub wiki**: `.github/workflows/wiki-sync.yml` copies
`wiki/*.md` into the `mowglinext.wiki` repo on every push to `main` that touches `wiki/**` (plus
`workflow_dispatch`). Edit here, never in the wiki UI — the sync overwrites it.

| Page | One line | Status |
|------|----------|--------|
| [`Home.md`](../../wiki/Home.md) | Wiki hub: quick links, project links, monorepo structure, key design decisions. | current |
| [`_Sidebar.md`](../../wiki/_Sidebar.md) | Wiki navigation sidebar. | current |
| [`Getting-Started.md`](../../wiki/Getting-Started.md) | Hardware requirements, Codespaces/devcontainer, automated + manual install, GUI features. | current, **stale localizer paragraph** (L55 still calls robot_localization the default dual-EKF localizer) |
| [`User-Guide.md`](../../wiki/User-Guide.md) | Operator GUI walkthrough (quickstart, page tour, configuring, troubleshooting). | current |
| [`Architecture.md`](../../wiki/Architecture.md) | The long technical design doc: packages, dependency graph, data flow, TF tree. Cited by CLAUDE.md as the steady-state fusion_graph reference. | current, **stale simulation sections** (describes Gazebo Harmonic / ros_gz_bridge; the sim is Webots) |
| [`Configuration.md`](../../wiki/Configuration.md) | Per-file YAML parameter reference. | current, **two stale sections** (§2 `robot_localization.yaml`, §4 Fields2Cover v2.0) |
| [`Behavior-Trees.md`](../../wiki/Behavior-Trees.md) | BT overview, states, commands, tree structure, node types, how to add a node. | current, **stale node list** (mentions `SaveSlamMap`, EKF-based heading calibration) |
| [`Deployment.md`](../../wiki/Deployment.md) | Compose generation, runtime services, GNSS deployment shape, troubleshooting. | **stale GNSS sections** — see historical table |
| [`Sensors.md`](../../wiki/Sensors.md) | GNSS contract (`/gps/fix`, `/gps/status`, …) and LiDAR integration. | current |
| [`Firmware.md`](../../wiki/Firmware.md) | Framed as a ROS1-rosserial→ROS2-COBS *migration guide*, but its packet-structure reference and per-wheel velocity-PI section are the live protocol. | current (migration framing is historical) |
| [`GUI.md`](../../wiki/GUI.md) | GUI access, dashboard, pages, design system, architecture, dev + Docker. | current |
| [`Contributing.md`](../../wiki/Contributing.md) | Wiki-side contribution summary, dev environment, style, testing, AI assistance. | current |
| [`AI-Assisted-Contributing.md`](../../wiki/AI-Assisted-Contributing.md) | How Claude review / `@claude` work here, guidelines and pitfalls for AI-assisted PRs. | current |
| [`FAQ.md`](../../wiki/FAQ.md) | General / deployment / navigation / development Q&A. | **stale localizer + sim answers** — see historical table |
| [`Simulation.md`](../../wiki/Simulation.md) | Operator-facing sim guide — still Gazebo-era. | superseded — see historical table |

## Historical / superseded — do NOT treat as current

| Doc | Last touched | Why it is not current | Superseded by |
|-----|--------------|-----------------------|---------------|
| [`docs/HANDOFF_FUSION_GRAPH.md`](../HANDOFF_FUSION_GRAPH.md) | 2026-07-03 | Self-declared historical (FR banner): the FusionCore→iSAM2 migration record, kept for git archaeology. CLAUDE.md says the same. | [`wiki/Architecture.md`](../../wiki/Architecture.md) § fusion_graph + [`codemaps/fusion_graph.md`](codemaps/fusion_graph.md) |
| [`SESSION-2026-04-18.md`](../../SESSION-2026-04-18.md) | 2026-04-18 | Dated field-debug log from the `feat/slam-toolbox-tuned` / `gps_slam_corrector` era. Every component it names (slam_toolbox, the Umeyama corrector) has since been removed. | [`codemaps/fusion_graph.md`](codemaps/fusion_graph.md), CLAUDE.md Invariant 1 |
| [`docker/logs/mow_sessions/fusion_graph_float_review_2026-06-11.md`](../../docker/logs/mow_sessions/fusion_graph_float_review_2026-06-11.md) | 2026-06-28 | Dated multi-agent review of RTK-Float behaviour, verified against 2026-06 code. Its central blocker ("keyframe/scan-matching disabled in deployed config") no longer holds — `use_keyframe_map: true`, `kf_min_inliers: 16` in `fusion_graph.yaml`. | [`codemaps/fusion_graph.md`](codemaps/fusion_graph.md) |
| [`docker/logs/mow_sessions/fusion_graph_keyframe_blueprint_2026-06-11.md`](../../docker/logs/mow_sessions/fusion_graph_keyframe_blueprint_2026-06-11.md) | 2026-06-28 | Implementation blueprint for the RTK-anchored keyframe layer — **shipped** (`use_keyframe_map`, `ScanToKeyframe` path live). Its "Phase 2: migrate to IncrementalFixedLagSmoother" is unverified/likely still open. | [`codemaps/fusion_graph.md`](codemaps/fusion_graph.md) |
| [`OBSTACLE_WEDGE_RECOVERY_SPEC.md`](../../OBSTACLE_WEDGE_RECOVERY_SPEC.md) | 2026-07-22 | Spec for a since-shipped effort: Part A's cul-de-sac guard (`obstacle_deviation.hpp` `hasClearExit`) and Part B's "stalled BESIDE an obstacle" gate (`detour_resume.hpp`) both exist, and `obstacle_reverse_enabled: true` in `nav2_params_base.yaml:544`. Its "Deployed state on the robot NOW" section is stale. `use_footprint_clearance` is still `false`, so the Part A footprint item remains open. | [`codemaps/mowgli_nav2_plugins.md`](codemaps/mowgli_nav2_plugins.md), [`codemaps/mowgli_behavior.md`](codemaps/mowgli_behavior.md) |
| [`SAFETY_REVIEW_2026-07-23.md`](../../SAFETY_REVIEW_2026-07-23.md) | 2026-07-23 | Dated 3-agent safety review against `dev@76a6862c`. Its whole **P0** block shipped (`FootprintApproach` + `PolygonStopNarrow`, `odom_rebase_dist_m: 6.0`, `kf_match_max_divergence_xy_m: 0.10`). P1–P3 shipped only in part — **unverified item by item**. | CLAUDE.md invariants + the nav2/fusion_graph codemaps |
| [`docs/RESEARCH_obstacle_coverage.md`](../RESEARCH_obstacle_coverage.md) | 2026-05-30 | Self-declared "research only — nothing implemented" decision doc. The stack since shipped detour-and-continue (`detour_resume.hpp`), a partial take on its recommended option (b); the full re-query-remaining-region loop is **unverified**. | [`codemaps/mowgli_coverage.md`](codemaps/mowgli_coverage.md), [`codemaps/mowgli_behavior.md`](codemaps/mowgli_behavior.md) |
| [`docs/gnss_gui_design.md`](../gnss_gui_design.md) | 2026-06-14 | Design doc whose closing § *Backend Gap* ("plan apply and factory reset are not wired yet") is closed: `gui/pkg/api/gnss.go` implements plan / apply / factory-reset / restart against `gnss_config_plan` + `gnss_config_apply`. | [`codemaps/gui_backend.md`](codemaps/gui_backend.md) |
| [`docs/GNSS_INTEGRATION_AUDIT.md`](../GNSS_INTEGRATION_AUDIT.md) | 2026-06-12 | Self-declared "Archived note" — content removed, link-catcher stub. | Universal GNSS contract in [`sensors/README.md`](../../sensors/README.md) + [`codemaps/deploy.md`](codemaps/deploy.md) |
| [`docs/UNIVERSAL_GNSS_SIDECAR_MIGRATION.md`](../UNIVERSAL_GNSS_SIDECAR_MIGRATION.md) | 2026-06-12 | Self-declared "Archived note" — migration plan removed once Universal GNSS became the only runtime. | same as above |
| [`docs/unicore_live_validate.md`](../unicore_live_validate.md) | 2026-06-12 | Self-declared "Archived Receiver Validation Note", kept only to catch old links. | same as above |
| [`docs/unicore_profiles.md`](../unicore_profiles.md) | 2026-07-13 | Self-declared "Archived Receiver Profiles", kept only to catch old links. | same as above |
| [`wiki/Simulation.md`](../../wiki/Simulation.md) | 2026-08-24 | Carries a maintainer banner saying it still describes the Gazebo-era stack; the sim is Webots R2025a (`ros2/src/mowgli_simulation/` has no Gazebo/SDF assets at all). | [`docs/WEBOTS_SIM.md`](../WEBOTS_SIM.md), [`ros2/README.md`](../../ros2/README.md) § Running Webots Simulation, [`codemaps/mowgli_simulation.md`](codemaps/mowgli_simulation.md) |
| [`wiki/FAQ.md`](../../wiki/FAQ.md) § localizer | 2026-04-29 | Presents `ekf_map_node` as the default with `use_fusion_graph:=false` and fusion_graph as opt-in. Both are gone: no `use_fusion_graph` arg exists, and the EKFs were removed. | CLAUDE.md Invariant 1, [`docs/claude/ros2-specifics.md`](ros2-specifics.md) |
| [`wiki/Deployment.md`](../../wiki/Deployment.md) § GNSS | 2026-06-05 | Describes `GNSS_STACK=legacy` and a "Legacy GNSS Removal Plan"; the installer now accepts only `universal|disabled` and the legacy fragments it lists no longer exist. | [`codemaps/deploy.md`](codemaps/deploy.md), [`sensors/README.md`](../../sensors/README.md) |
| [`firmware/stm32/test_code/README.md`](../../firmware/stm32/test_code/README.md) | 2026-04-03 | UART-proxy bring-up code; the file itself says to use `ros_usbnode` instead. | [`firmware/stm32/ros_usbnode/README.md`](../../firmware/stm32/ros_usbnode/README.md), [`codemaps/firmware.md`](codemaps/firmware.md) |
| [`.claude/memory/MEMORY.md`](../../.claude/memory/MEMORY.md) + `feedback_sim_cleanup.md`, `project_e2e_status.md`, `user_cedric.md` | 2026-04-04 | Frozen agent memory from the Gazebo/SLAM era (`ExecuteFullCoveragePath`, "SLAM map grows", `make sim-stop` against Gazebo). Not maintained. | CLAUDE.md + this index + the codemaps |

## Machine-generated or vendored docs

| Doc | Kind | Note |
|-----|------|------|
| [`docs/claude/codemaps/*.md`](codemaps/) (18 files) | generated | Produced from the tree at f21729e9; each carries its own "regenerate when files are added/removed" line. Regenerate rather than hand-edit. Currently untracked in git (created this pass). |
| [`docs/screenshots/README.md`](../screenshots/README.md) | maintainer/asset | How to regenerate the dashboard screenshots referenced by README, wiki, and GitHub Pages. |
| [`firmware/stm32/ros_usbnode/README.md`](../../firmware/stm32/ros_usbnode/README.md) | vendored | Inherited verbatim from upstream [cloudn1ne/Mowgli](https://github.com/cloudn1ne/Mowgli); untouched since the 2026-04-03 import. PlatformIO flashing + UART debugging howto. |
| [`firmware/stm32/mainboard_firmware/README.md`](../../firmware/stm32/mainboard_firmware/README.md) | vendored | Upstream stock-firmware backup/restore via ST-Link/JLink + OpenOCD, with SHA256 hashes. Still the procedure to follow before flashing. |
| [`firmware/stm32/panel_firmware/README.md`](../../firmware/stm32/panel_firmware/README.md) | vendored | Upstream panel-firmware backup/restore procedure (JP3 SWD header). |
| [`.github/PULL_REQUEST_TEMPLATE.md`](../../.github/PULL_REQUEST_TEMPLATE.md) | template | Rendered into every PR body; not prose to read. |
| `docs/index.html`, `docs/style.css`, `docs/install.sh` | hand-authored site | The mowgli.garden landing page + one-line install composer published by `.github/workflows/pages.yml` (guarded by `docs/test_install.sh` + `docs/test_web_composer.sh`) — not Markdown, listed here so `docs/` is not mistaken for a pure Markdown tree. |
