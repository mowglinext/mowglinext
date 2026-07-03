# AI-Assisted Contributing

MowgliNext embraces AI-assisted development. Claude reviews every PR, proposes improvements, and helps contributors via `@claude` mentions. This page explains how to use AI tools effectively — and how to avoid common pitfalls.

## How AI Works in This Project

### Automated (no setup needed)

| Feature | What happens |
|---------|-------------|
| **PR Review** | Claude automatically reviews every PR for safety, correctness, ROS2 best practices, and breaking changes |
| **@claude Bot** | Mention `@claude` in any issue or PR comment — it reads the codebase and responds |
| **Weekly Improvements** | Claude analyzes the codebase every Monday and proposes 2-3 improvements as new issues |
| **Welcome Bot** | First-time contributors get a welcome message with guidance |

### For Contributors Using Claude Code

If you use [Claude Code](https://claude.ai/claude-code) locally, the project includes configuration that makes Claude understand the full project context:

1. **`CLAUDE.md`** — Project instructions loaded automatically (safety rules, architecture invariants, code style)
2. **`.claude/settings.json`** — Pre-configured permissions for colcon, docker, ros2, pio commands
3. **`.claude/rules/ros2.md`** — ROS2-specific coding rules (QoS, node patterns, launch files)
4. **`ros2/CLAUDE.md`** — Detailed ROS2 stack reference (packages, topics, architecture, TODOs)

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

AI may suggest having multiple nodes publish the same TF transforms. **Don't.** `map→odom` is owned by either `ekf_map_node` (default) or `fusion_graph_node` (when `use_fusion_graph:=true`) — never both. `odom→base_footprint` is owned by `ekf_odom_node` (local dead reckoning). There is no SLAM back-end; the `/map` is built from user-recorded area polygons.

#### 4. MPPI Controller for Coverage

AI often suggests MPPI as "more advanced". For boustrophedon coverage paths with 0.18 m swath spacing, MPPI's Euclidean nearest-point matching jumps between adjacent parallel swaths. Mowgli uses **FTCController** (Follow-the-Carrot, 3-axis PID + native obstacle deviation) for BOTH `FollowPath` (transit) and `FollowCoveragePath` (coverage swaths) — single plugin, single tuning, sub-10 mm lateral error along a strip. RPP/RotationShim are not in the active stack.

#### 5. Hallucinated ROS2 APIs

AI may generate service/topic names that don't exist:

```cpp
// WRONG — AI hallucinated this service
client = create_client<std_srvs::srv::SetBool>("/mowgli/enable_blade");

// RIGHT — actual blade control
client = create_client<mowgli_interfaces::srv::MowerControl>("/mowgli/hardware/mower_control");
```

**Check:** Verify against `ros2/CLAUDE.md` which lists all real topics and services.

#### 6. Missing Firmware Safety

AI may implement blade control as a direct motor command:

```cpp
// DANGEROUS — bypasses firmware safety
publish_raw_motor_command(BLADE_MOTOR, speed);

// CORRECT — fire-and-forget to firmware (firmware decides if safe)
auto msg = mowgli_interfaces::msg::MowerControl();
msg.blade_on = true;
publisher_->publish(msg);
// Firmware checks: tilt, emergency, battery, rain before activating
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

Every PR goes through these automated checks:

| Check | What it catches |
|-------|----------------|
| **Claude Code Review** | Safety issues, logic errors, ROS2 anti-patterns, breaking changes |
| **clang-format** | C++ formatting violations |
| **cppcheck** | Static analysis (warnings, performance, portability) |
| **colcon build** | Compilation errors |
| **colcon test** | Test failures |

If Claude flags something as 🔴 **Critical**, it must be fixed before merge. 🟡 **Suggestion** items are recommended but not blocking.

## Contributing Workflow with AI

### Recommended approach:

1. **Read first** — Let your AI tool read `CLAUDE.md` and `ros2/CLAUDE.md` before generating code
2. **Ask, then code** — Ask the AI to explain the relevant existing code before writing new code
3. **Generate tests first** — Use `/tdd` or ask the AI to write tests before implementation
4. **Review the diff** — Read every line the AI generates. If you don't understand it, don't commit it
5. **Run checks locally** — `colcon build && colcon test` before pushing
6. **Let Claude review** — The PR review will catch issues, but fixing them before submission is faster

### If Claude's PR review flags issues:

1. Read the feedback carefully — it has full project context
2. Fix 🔴 Critical items (blocking)
3. Address 🟡 Suggestions where reasonable
4. Push fixes — Claude will re-review automatically
5. If you disagree with a suggestion, explain why in a comment — the maintainer will decide

## Need Help?

- Mention `@claude` in your issue or PR for instant AI help
- Ask in [Discussions](https://github.com/mowglinext/mowglinext/discussions)
- Check the [FAQ](FAQ) for common questions
