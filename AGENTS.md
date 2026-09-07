# AGENTS.md

Repository-local working rules for code agents.

## Where the repository index lives

These files are the maintained map of the tree — read the relevant one before changing code, instead of grepping blind. They apply to every agent, not only Claude.

- [`CLAUDE.md`](CLAUDE.md) — safety rules, monorepo layout, architecture invariants, "what NOT to do".
- Per-tree notes: [`ros2/CLAUDE.md`](ros2/CLAUDE.md), [`gui/CLAUDE.md`](gui/CLAUDE.md), [`firmware/CLAUDE.md`](firmware/CLAUDE.md), [`install/CLAUDE.md`](install/CLAUDE.md), [`docker/CLAUDE.md`](docker/CLAUDE.md), [`sensors/CLAUDE.md`](sensors/CLAUDE.md).
- [`docs/claude/doc-index.md`](docs/claude/doc-index.md) — which document is authoritative and which is a historical record.
- [`docs/claude/codemaps/`](docs/claude/codemaps/) (per-package file maps), [`ros-interfaces.md`](docs/claude/ros-interfaces.md) (every topic/service/action/TF), [`parameters.md`](docs/claude/parameters.md) (every config key and its default), [`testing-ci.md`](docs/claude/testing-ci.md) (every test and the CI job that gates it).

## ROS2 formatting

- Any change under `ros2/` that touches C++ source or headers (`.cpp`, `.hpp`, `.h`) must be `clang-format` clean before finishing.
- **`ros2/` is the whole scope — do not apply this to C++ elsewhere.** `sensors/gps/mowgli_gnss_bridge` is C++ that `ros2/scripts/format.sh` has never walked (its `find` starts at `ros2/src/`), so it follows ament's uncrustify profile instead and its `CMakeLists.txt` keeps that linter enabled. Running clang-format there restyles ~180 lines and then fails CI. See [`sensors/CLAUDE.md`](sensors/CLAUDE.md).
- Use **clang-format 18** and the repository style file. CI installs `clang-format-18`, `ros2/scripts/format.sh` expects major 18 (it only *warns* on a mismatch) and `ros2/.pre-commit-config.yaml` pins `v18.1.8` — any other major reformats lines CI never asked about, and the opt-in `.githooks/pre-push` hook will amend them into a `chore: auto-format C++ files` commit.

```bash
clang-format -i -style=file:ros2/.clang-format <touched-files>
```

- Before concluding work on `ros2/` files, verify formatting explicitly. Preferred check — this mirrors the CI job `Formatting (clang-format)`, which gates only the **changed lines** against the merge-base with `origin/main` (yes, `main`, even though PRs target `dev`):

```bash
git clang-format --style=file:ros2/.clang-format \
  --diff "$(git merge-base origin/main HEAD)" -- <touched-files>
```

- If `origin/main` is unavailable locally, at minimum run:

```bash
clang-format -n -style=file:ros2/.clang-format <touched-files>
```

## Non-root builds and tools

- Never build as `root`.
- Never run `pio`, `platformio`, `colcon`, `cmake`, `make`, `ninja`, `go`, `yarn`, `npm`, formatters, or generated-file workflows as `root`.
- Always use the normal project user so the repository does not accumulate root-owned files.
- If a previous run created root-owned artifacts, fix ownership or remove those artifacts before continuing.

## Evidence and hardware claims

This robot has spinning blades and most of its hard bugs were only ever settled
by a field observation. Two rules follow from that.

**Never generalise hardware evidence past its exact baseline.** A measurement is
valid for the repo commit, the firmware image, the submodule gitlink, the robot
unit, and the receiver/driver revision it was taken on — and nothing else. Say
which of those the number came from. This is not pedantry here: a GNSS receiver
reporting 14 mm accuracy while sitting 2.5 m off after a cold power cycle, and a
graph marginal reporting metre-level sigma while the receiver reported
centimetres, are both real MowgliNext incidents where a number was trusted
outside the conditions that produced it.

**When the remaining question is physical, stop and say so** instead of doing
more repository searching. Record it in one of two explicit states:

- `HARDWARE_REQUIRED` — the remaining acceptance evidence intrinsically needs
  the robot or the field, and has not been acquired.
- `HARDWARE_PENDING` — the exact setup and procedure are identified and every
  software-side prerequisite is done; only the run is outstanding.

Either way write down the exact baseline, the procedure, the observable
pass/fail criterion, and the safety prerequisites. "Field test owed" on its own
is not a handover — the next person cannot run it.

## Diagnosing freshness and provenance

**Never paper over an ownership or provenance defect with a timeout.** If the
real defect is that a consumer cannot tell a genuinely new observation from a
cached republication, or cannot tell which producer a value came from, then a
deadline only changes how long the wrong answer survives. Fix the identity
first, then apply a deadline to it.

This is the finding behind the GNSS observation-identity work
(`mowgli_interfaces/gnss_observation_freshness.hpp`): several consumers treated
callback arrival as physical acquisition, so a frozen receiver kept looking
healthy. Receipt provenance and delivery liveness are two different questions in
two different clock domains — the ROS clock for "when was this observed", a
monotonic clock for "is the stream alive". Do not substitute one for the other,
and do not mix the domains.
