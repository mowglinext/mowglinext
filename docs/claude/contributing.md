# Contributing: Style, Commits, Git Workflow, Tooling

> Code style, commit conventions, branch/PR workflow, and recommended skills/agents. Loaded on demand from [`../../CLAUDE.md`](../../CLAUDE.md). The ROS2-specific rules also live in [`../../.claude/rules/ros2.md`](../../.claude/rules/ros2.md).

## Code Style

| Component | Style | Tool |
|-----------|-------|------|
| C++ (ros2/) | 2-space indent, `snake_case` files/params, `CamelCase` classes | `clang-format` (config in `ros2/.clang-format`) |
| Go (gui/) | Standard Go | `gofmt` |
| TypeScript (gui/web/) | Prettier + ESLint | `yarn lint` |
| Python (launch files) | PEP 8 | — |
| YAML (config) | 2-space indent, `snake_case` keys | — |

## Commit Conventions

```
<type>: <description>

Types: feat, fix, refactor, docs, test, chore, perf, ci
```

No Co-Authored-By lines. Keep messages concise and focused on "why".

## Git Workflow

**NEVER commit directly to main.** Always use feature branches and PRs:
```bash
git checkout main && git pull
git checkout -b feat/my-feature    # or fix/, refactor/, test/, chore/, docs/
# ... make changes ...
git add <files> && git commit -m "feat: description"
gh pr create --title "feat: my feature" --body "..."
```

### Dev Branch Workflow

Docker builds trigger on both `main` and `dev` branches. Images are tagged `:main` and `:dev` respectively. Use `mowgli-dev` / `mowgli-main` commands to switch between environments. Iterate on `dev`, merge to `main` when stable.

## Recommended Skills and Agents

When using Claude Code on this project:

### Skills to Use
- `/ros2-engineering` — ROS2 node patterns, QoS, launch files, Nav2 (use for any ros2/ work)
- `/cpp-coding-standards` — C++ Core Guidelines (use for C++ reviews)
- `/docker-patterns` — Dockerfile and compose patterns (use for docker/ and sensors/ work)
- `/tdd` — Test-driven development (use when adding new features)

### Agents to Invoke
- **code-reviewer** — after any code changes
- **cpp-reviewer** — after C++ changes in ros2/
- **security-reviewer** — before commits touching auth, configs, or firmware commands
- **build-error-resolver** — when colcon or Docker builds fail
- **tdd-guide** — when implementing new features
- **architect** — for design decisions spanning multiple packages
