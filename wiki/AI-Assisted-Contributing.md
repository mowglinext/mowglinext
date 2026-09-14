# AI-Assisted Contributing

MowgliNext embraces AI-assisted development: the repository ships Claude Code configuration so that an assistant running on your machine loads the full project context. This page explains how to use AI tools effectively — and how to avoid common pitfalls.

## How AI Works in This Project

### Automated (no setup needed)

| Feature | What happens |
|---------|-------------|
| **Welcome Bot** | First-time contributors get a welcome message with guidance (`.github/workflows/welcome.yml`) |

Until 2026-07 the repository also ran Claude GitHub Actions — automatic PR review, an `@claude` mention bot, and a weekly improvement proposer. Those workflows were removed; AI assistance here is now **local**, driven by the checked-in configuration described below, and PRs are reviewed by maintainers.

### For Contributors Using Claude Code

If you use [Claude Code](https://claude.ai/claude-code) locally, the project includes configuration that makes Claude understand the full project context:

1. **`CLAUDE.md`** — Project instructions loaded automatically (safety rules, architecture invariants, code style)
2. **`.claude/settings.json`** — Pre-configured permissions for colcon, docker, ros2, pio commands
3. **`.claude/rules/ros2.md`** — ROS2-specific coding rules (QoS, node patterns, launch files)
4. **A per-area `CLAUDE.md`** in `ros2/`, `gui/`, `firmware/`, `install/`, `docker/` and `sensors/` — a short orientation file for work inside that directory, pointing at the detailed reference
5. **`docs/claude/`** — that detailed reference: `codemaps/` (one "where to look" map per package), `ros-interfaces.md` (every topic, service, action and TF frame, and the node and `file:line` that creates it), `parameters.md` (every config key, its default and its consumers), `testing-ci.md` (every test suite and the CI job that gates it), and `doc-index.md` (which document is authoritative and which is historical)

**No extra setup needed** — just clone the repo and run `claude` in the project directory. The configuration loads automatically.

**Tip:** The [DevContainer / GitHub Codespaces](Getting-Started#development-with-github-codespaces--devcontainer) environment comes with Claude Code CLI and GitHub CLI pre-installed. Open a Codespace and start using `claude` immediately — no local setup at all.

### Recommended Skills

If you have the [Everything Claude Code](https://github.com/anthropics/claude-code) skills installed:

| Skill | When to use |
|-------|-------------|
| `/ros2-engineering` | Any work in `ros2/` — node patterns, QoS, launch files, Nav2 |
| `/cpp-coding-standards` | C++ code reviews and new C++ code |
| `/docker-patterns` | Dockerfile and compose changes |
| `/tdd` | Implementing new features (write tests first) |
| `/code-review` | After writing any code |

## Guidelines for Using AI Tools

### Any AI Tool (Copilot, Cursor, ChatGPT, Claude, etc.)

**DO:**
- Use AI to generate boilerplate (CMakeLists.txt, package.xml, launch files)
- Ask AI to explain existing code before modifying it
- Use AI for test generation
- Let AI help with documentation

**DON'T:**
- Blindly accept AI-generated code without reviewing it
- Let AI add dependencies without checking they exist in ROS2 Kilted
- Trust AI with safety-critical blade control logic
- Submit AI-generated code that you don't understand

### Common AI Mistakes to Watch For

These are real problems we've seen from AI-generated contributions:

#### 1. Wrong ROS2 Distro

AI models often generate code for ROS2 Humble or Foxy instead of Kilted:

```cpp
// WRONG — Humble-era pattern
auto node = rclcpp::Node::make_shared("my_node");

// RIGHT — Kilted pattern (same API, but check package availability)
// Verify the package exists: apt list ros-kilted-*
```

**Check:** If AI suggests a ROS2 package, verify it exists for Kilted: `apt list ros-kilted-<package>`

#### 2. FastRTPS Instead of Cyclone DDS

AI defaults to FastRTPS since it's the "default" DDS. We use Cyclone DDS because FastRTPS has stale shared memory issues on ARM.

```yaml
# WRONG
RMW_IMPLEMENTATION: rmw_fastrtps_cpp

# RIGHT
RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
```

#### 3. TF Authority

AI may suggest having multiple nodes publish the same TF transforms. **Don't.** `fusion_graph_node` — the GTSAM factor-graph localizer — is the sole, unconditional localizer and owns **both** `map→odom` and `odom→base_footprint`. No other node may publish either. The robot_localization dual EKF (`ekf_map_node` / `ekf_odom_node`) and the `use_fusion_graph` launch argument were removed, so any suggestion built around them no longer applies. There is no SLAM back-end either: the navigable world comes from user-recorded area polygons that `map_server_node` publishes as Nav2 costmap filters (`/keepout_mask` + `/costmap_filter_info`).

#### 4. MPPI Controller for Coverage

AI often suggests MPPI as "more advanced". It was tried on the coverage slot and reverted on 2026-06-19: as a *sampling* controller it cut corners and made omega-loops at swath U-turns, and every attempt to sharpen its corners made it weave on the straights.

The two controller slots are different plugins, and neither is MPPI:

- `FollowCoveragePath` (coverage swaths) → **`mowgli_nav2_plugins/FTCController`**, a deterministic Follow-the-Carrot controller with a decoupled longitudinal/lateral/angular PID, which also skirts obstacles by deviating the path laterally (`enable_obstacle_deviation`).
- `FollowPath` (transit) → **RotationShim wrapping `nav2_regulated_pure_pursuit_controller`** (RPP).

Don't swap either one. `ros2/src/mowgli_bringup/test/test_nav2_params.py::test_coverage_is_ftc_transit_is_not` fails the build if coverage stops being FTC or if FTC leaks onto the transit slot.

#### 5. Hallucinated ROS2 APIs

AI may generate service/topic names that don't exist:

```cpp
// WRONG — AI hallucinated this service
client = create_client<std_srvs::srv::SetBool>("/mowgli/enable_blade");

// RIGHT — actual blade control
client = create_client<mowgli_interfaces::srv::MowerControl>("/hardware_bridge/mower_control");
```

**Check:** Verify against `docs/claude/ros-interfaces.md`, which lists every real topic, service, action and TF frame together with the node and `file:line` that creates it.

#### 6. Missing Firmware Safety

AI may implement blade control as a direct motor command:

```cpp
// DANGEROUS — bypasses firmware safety
publish_raw_motor_command(BLADE_MOTOR, speed);

// CORRECT — fire-and-forget service call (firmware decides if safe)
auto request = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
request->mow_enabled = 1u;
request->mow_direction = 0u;
client_->async_send_request(request);   // response deliberately ignored
// Firmware gates the blade on the emergency latch (tilt, wheel lift, stop
// button), the IDLE/docked state, and the cmd_vel watchdog.
```

#### 7. ROS1 Patterns

AI trained on older data may use ROS1 patterns:

```python
# WRONG — ROS1
import rospy
rospy.init_node('my_node')

# RIGHT — ROS2
import rclpy
rclpy.init()
node = rclpy.create_node('my_node')
```

## Quality Checks on PRs

Depending on which paths it touches, a PR goes through these automated checks:

| Check | What it catches |
|-------|----------------|
| **Build & Test (ROS2 kilted)** | Compilation errors and failing `colcon test` suites — the required status check on `dev` |
| **Formatting (clang-format)** | C++ formatting violations, checked on changed lines with clang-format **18** |
| **Static Analysis (cppcheck)** | Warnings, performance and portability findings — informational only (`continue-on-error`), never blocking |
| **Codegen Drift (Go / TS / firmware msg types)** | A changed `.msg`/`.srv` without the regenerated Go, TypeScript and firmware bindings |
| **Config Drift (mowgli_robot.yaml)** | Defaults padded into the sparse installed config instead of the in-package template |
| **Unit Tests (vitest + tsc)** | `gui/web/` typecheck, lint and unit-test failures |

There is no automated AI reviewer on this repository — a maintainer reads the PR. Fixing the red checks before asking for review is the fastest path to a merge.

## Contributing Workflow with AI

### Recommended approach:

1. **Read first** — Let your AI tool read the root `CLAUDE.md`, the `CLAUDE.md` of the directory you are working in, and the matching `docs/claude/codemaps/*.md` before generating code
2. **Ask, then code** — Ask the AI to explain the relevant existing code before writing new code
3. **Generate tests first** — Use `/tdd` or ask the AI to write tests before implementation
4. **Review the diff** — Read every line the AI generates. If you don't understand it, don't commit it
5. **Run checks locally** — `colcon build && colcon test` before pushing
6. **Get CI green, then ask for review** — no bot will catch issues for you; a red check just delays the maintainer

### If CI or a reviewer flags issues:

1. Read the feedback carefully, and re-check the claim against the code before acting on it
2. Fix anything the required `Build & Test (ROS2 kilted)` check is unhappy about — that one is blocking
3. Address the rest where reasonable
4. Push fixes — the workflows re-run automatically on the new commit
5. If you disagree with a suggestion, explain why in a comment — the maintainer will decide

## Need Help?

- Open an [issue](https://github.com/mowglinext/mowglinext/issues) describing what you tried
- Ask in [Discussions](https://github.com/mowglinext/mowglinext/discussions)
- Check the [FAQ](FAQ) for common questions
