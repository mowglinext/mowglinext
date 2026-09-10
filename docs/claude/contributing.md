# Contributing: Style, Commits, Git Workflow, Tooling

> Code style, commit conventions, branch/PR workflow, and recommended skills/agents. Loaded on demand from [`../../CLAUDE.md`](../../CLAUDE.md). The ROS2-specific rules also live in [`../../.claude/rules/ros2.md`](../../.claude/rules/ros2.md).
>
> **Where to look things up:** per-area file maps in [`codemaps/`](codemaps/); every topic/service/action/TF and its publisher in [`ros-interfaces.md`](ros-interfaces.md); every config key, default and consumer in [`parameters.md`](parameters.md); every test, its CI gate and the pre-PR checklist in [`testing-ci.md`](testing-ci.md); which document is authoritative vs historical in [`doc-index.md`](doc-index.md). Each top-level component also has its own `CLAUDE.md` (`ros2/`, `gui/`, `firmware/`, `install/`, `docker/`, `sensors/`).

## Code Style

| Component | Style | Tool |
|-----------|-------|------|
| C++ (ros2/) | 2-space indent, Allman braces, 100-col, `snake_case` files/params, `CamelCase` classes | `clang-format` **18** (config in `ros2/.clang-format`; CI installs `clang-format-18`, pre-commit pins `v18.1.8` — another major reformats files CI never flagged) |
| Go (gui/) | Standard Go | `gofmt` (nothing in CI enforces it) |
| TypeScript (gui/web/) | ESLint 9 flat config (`gui/web/eslint.config.js`) — **no Prettier is installed or configured** | `yarn lint` |
| Python (launch files) | PEP 8 | `ruff` via `ros2/.pre-commit-config.yaml` (opt-in; not in CI) |
| YAML (config) | 2-space indent, `snake_case` keys | — |

## Commit Conventions

```
<type>: <description>

Types: feat, fix, refactor, docs, test, chore, perf, ci
```

No Co-Authored-By lines. Keep messages concise and focused on "why".

## Git Workflow

**NEVER commit directly to `main` or `dev`** — both are protected and require a PR with one approving review. **PRs land on `dev`**; `main` is the release branch:
```bash
git checkout dev && git pull
git checkout -b feat/my-feature    # or fix/, refactor/, chore/, perf/ — CI push
                                   # triggers match exactly these prefixes
# ... make changes ...
git add <files> && git commit -m "feat: description"
gh pr create --base dev --title "feat: my feature" --body "..."
```

The required status check on `dev` is named `Build & Test (ROS2 kilted)` (`main` additionally requires `Formatting (clang-format)`). The full "what will fail, in what order" checklist is in [`testing-ci.md`](testing-ci.md) § *Before opening a PR*.

### Dev Branch Workflow

Docker builds trigger on `main`, `dev` and the `feat/**` / `fix/**` / `refactor/**` / `chore/**` / `perf/**` branch prefixes, plus `v*.*.*` tags. Images are tagged with the branch name (`:main`, `:dev`, `:feat-my-feature`), so pushing a feature branch is how you get CI to build an image for field testing. Which tag a robot runs is `IMAGE_TAG` in `docker/.env` — set by the installer (menu: `main` / `dev` / custom tag, or `bash install/mowglinext.sh --image-tag=dev`), then `mowgli-pull && mowgli-up`. There is no `mowgli-dev` / `mowgli-main` command. Iterate on `dev`, merge to `main` when stable.

## Recommended Skills and Agents

When using Claude Code on this project. The repo carries no `.claude/skills/` or `.claude/agents/` — the following come from your own install (see [wiki AI-Assisted-Contributing](https://github.com/mowglinext/mowglinext/wiki/AI-Assisted-Contributing#recommended-skills)), so skip any you do not have.

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
