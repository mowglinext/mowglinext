# Fake blade-service executor lifecycle hotfix

Investigation date: 2026-10-14. Baseline: `48e571034a96135ddd597119abc537afa38d30ba`,
branch `fix/ros2-blade-service-lifecycle`, [PR #996](https://github.com/mowglinext/mowglinext/pull/996).
This is a test-fixture investigation, with no robot or hardware acceptance claim.

## Evidence and uncertainty

The reported CI results on the same commit were a 43.99-second passing PR run
and a 180.10-second push timeout, last printing
`MissingServiceDoesNotRenewTheReadinessTimeout`. That last test name alone does
not identify the blocking instruction.

Temporary, flushed stderr checkpoints covered creation, spin entry/exit,
cancellation, spinner join, executor destruction, and service destruction.
The instrumented baseline passed 50 repetitions of both original scenarios on
each middleware locally. The spontaneous CI hang was **not reproduced**.

A separate diagnostic derivative used a promise gate to hold the initial
spinner immediately before `spin()`, then released it after `cancel()` returned.
Both Fast DDS and Cyclone DDS stalled at the join until the 12-second process
watchdog shut ROS down. A separate three-second observation followed by SIGKILL
on Fast DDS ended with these flushed checkpoints:

```text
LIFECYCLE start create
LIFECYCLE cancel begin spinning=0
LIFECYCLE cancel end
LIFECYCLE join begin
LIFECYCLE spin enter
```

There was no spin-exit, join-end, executor-destruction or service-destruction
checkpoint before termination. Thus **joining the spinner** is the operation
blocked in this controlled ordering; cancellation returned, and destruction
and restart had not been reached. A debugger attach was unavailable in this
environment, so there is no captured native stack trace.

This ordering exposes a concrete startup race: [`Executor::cancel()`](https://github.com/ros2/rclcpp/blob/lyrical/rclcpp/src/rclcpp/executor.cpp)
sets `spinning` false, but a subsequent [`MultiThreadedExecutor::spin()`](https://github.com/ros2/rclcpp/blob/lyrical/rclcpp/src/rclcpp/executors/multi_threaded_executor.cpp)
sets it true again. Starting a thread does not synchronize with its entry into
`spin()`. DDS discovery can make the fake service visible before that entry.
This proves a fixture defect, **not the exact cause of the uninstrumented CI
timeout**; another shutdown/interleaving defect in that run cannot be excluded.

## Fix and regression coverage

- The fake mower-control service has its own node and single-threaded executor.
  Neither fixture node is added to two executors.
- Its worker uses a persistent atomic stop flag around `spin_once(20 ms)`.
  This is a bounded ROS work wait, not a sleep or readiness/discovery timeout.
  Stopping before worker entry remains effective; no cancellation is needed.
- Removal joins only this worker, then destroys its executor, service and node.
  Weak-pointer assertions verify actual service/node destruction. The existing
  five-second discovery budget checks both client readiness and zero service
  servers in the ROS graph. The global service-name list includes clients and
  is therefore unsuitable for checking server disappearance.
- The main FollowStrip/Nav2 executor stays alive during removal/recreation.
  Its initial startup observes `is_spinning()` before returning, preventing an
  early teardown cancellation from preceding `spin()` entry.
- A regression performs 20 cycles per invocation, each with immediate removal
  after creation, recreation, an acknowledged mower-control request and an
  accepted/completed fake Nav2 goal, while retaining the main executor identity.
  A condition variable waits for the server's accepted callback before completing
  the goal: a client acceptance response alone does not prove that callback ran.
- The original missing-service deadline, no-dispatch assertions and
  service-reappearance readiness scenario are preserved. Production safety
  logic and the 180-second CTest timeout are unchanged.

## Local validation

Environment: Ubuntu 26.04, ROS 2 Lyrical, `rclcpp` 32.0.3,
`rmw_fastrtps_cpp` 9.4.10 and `rmw_cyclonedds_cpp` 4.1.5. Interface and behavior
build artifacts, diagnostic binaries and logs are outside version control under
`/tmp/mowgli-ci-hotfix` and `/tmp/mowgli-ci-followup`. Stress processes use distinct
DDS domains; package tests use the existing isolated ROS test runner.

Final results on the revised fixture:

| Check | Fast DDS | Cyclone DDS |
| --- | --- | --- |
| Missing-service readiness deadline | 100/100 passed | 100/100 passed |
| Service reappearance during readiness wait | 100/100 passed | 100/100 passed |
| Lifecycle/Nav2 regression | 50/50 invocations passed | 50/50 invocations passed |
| Controlled removal before blade-worker entry | 5/5 passed | 5/5 passed |
| Complete `mowgli_behavior` package | 54/54 CTest targets passed, 45.03 s | 54/54 CTest targets passed, 46.65 s |

Each lifecycle invocation performs 20 cycles: **1,000 cycles and 2,000 asserted
service removals per middleware**, with real mower-control responses and Nav2
goal acceptance/completion. The corrected diagnostic derivative gates the blade
worker before its first stop-flag check and releases it only after setting the
flag. Its flushed trace reaches `worker released stopping=1`, `join end`, and
`destruction end`; the missing-service deadline and no-dispatch assertions pass.
All temporary gates/checkpoints were removed from the repository source.

Evidence files under `/tmp/mowgli-ci-followup`: `diagnostic-{fastdds,cyclone}.log`,
`gated-{fastdds,cyclone}.log`, `gated-backtrace.log`,
`final-{fastdds,cyclone}-100.log`, `final-lifecycle-{fastdds,cyclone}-50.log`,
`final-gated-{fastdds,cyclone}.log`, and `suite-final-{fastdds,cyclone}.log`.
The final stress binaries use `--gtest_repeat=100` for the two original scenarios
and `--gtest_repeat=50` for the lifecycle regression. Package validation uses
`colcon test --packages-select mowgli_behavior --return-code-on-test-failure`
with four CTest workers and the `/tmp` build/install/result directories.

`clang-format-18` 18.1.8 passed both the full-file check and the CI-equivalent
changed-lines check against `git merge-base origin/main HEAD`; `git diff --check`
passed. Only the fixture test and this report changed; the pre-existing MAVROS
submodule difference was left untouched. No commit or push was performed.

Kilted is not installed locally. A passing GitHub Actions run on the revised
fixture is still outstanding; local stress results do not establish that the
unobserved CI interleaving cannot recur.
