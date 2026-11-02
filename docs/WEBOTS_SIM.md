# Webots Simulation — ODE Quirks and Load-Bearing Workarounds

## 1. Scope

**Audience:** anyone editing `ros2/src/mowgli_simulation/worlds_webots/`,
`protos/`, `urdf_webots/`, or `mowgli_simulation/kinematic_drive.py`.

**This document is NOT:**

- how to *run* the sim — see [`ros2/README.md` § Running Webots Simulation](../ros2/README.md#running-webots-simulation);
- the operator guide — see [`wiki/Simulation.md`](../wiki/Simulation.md);
- the package index (files, nodes, topics, parameters, tests) — see
  [`docs/claude/codemaps/mowgli_simulation.md`](claude/codemaps/mowgli_simulation.md).

It exists because the Webots/ODE sim carries five non-obvious workarounds that
each cost days to find and each look like dead code or a mistake to a reader
who does not know the history. Removing any one of them silently breaks the
sim in a way that presents as a *Nav2 bug*, not a sim bug. Every quirk below
is stated as **RULE → ANCHOR → SYMPTOM IF REINTRODUCED**.

**On commit references:** issue #201 quotes `a5a4ff6e`, `c25f22b7`, `57324f54`
and `940d9d9f`. **None of those SHAs resolve in this repository** — the branch
was rewritten/squashed before it landed. The reachable history is:

| Commit | What it landed |
|---|---|
| `364cad30` | `feat(sim): migrate from Gazebo Ignition to Webots` — the whole Webots stack in ONE commit (`kinematic_drive.py` already at revision 3, `MowgliMower.proto`, `mowgli_garden.wbt`, `webots_minimal.launch.py`). All three quirk fixes arrived together, not as three commits. |
| `2e181873` | firmware PI + deadband motor model inside the plugin |
| `8cba7d36` | Webots Lidar `noise 0.002` / `resolution 0.01`; disabled `synthesize_from_cmd_vel` (the motor model made the Webots gyro trustworthy again) |
| `46042015` | `sim_actuation_node` (firmware deadband + angular-rate PI on the wheel command). The angular-rate PI stage was **removed on 2026-07-17** — Option C moved the yaw-rate loop into the STM32 firmware, so `wz` now passes straight through to the per-wheel model (`sim_actuation_node.cpp:20`–`:30`, `sim_full_system.launch.py:477`–`:481`). |

Everything below is therefore anchored to **file:line plus a searchable
token**, not to a commit.

## 2. Three-layer architecture

The sim robot is split across three files, and it is easy to look in the wrong
one:

| Layer | File | Owns | Does NOT own |
|---|---|---|---|
| **A — world + body** | `worlds_webots/mowgli_garden.wbt` | The world, and the physical robot instance. `EXTERNPROTO "../protos/MowgliMower.proto"` (`:11`), the `MowgliMower { … }` instance (`:71`–`:82`), `WorldInfo.basicTimeStep 20` (`:16`, matching controller_manager 50 Hz), sensor nodes in `extensionSlot` (`:82`+) | ROS topic names |
| **B — device wiring** | `urdf_webots/mowgli_webots.urdf` | Which Webots device maps to which ROS topic, and which plugins load. `<webots>` block `:161`–`:204`, `<ros2_control>` joints `:206`–`:220` | Any geometry, mass, or joint physics |
| **C — body motion** | `mowgli_simulation/kinematic_drive.py` | native wheel-position increments → Supervisor teleport of the chassis, plus the chassis ODE velocity for sensor sampling | Wheel motor commands (owned by `diff_drive_controller`); the firmware motor model itself |

**Geometry lives in A only.** The URDF is *device + plugin wiring* — it does
not spawn a body (see § 7).

The garden ground uses native `Solid`/`Plane` nodes so it also loads in the
stripped offline SDK. It preserves [R2025a Floor's collision semantics](https://github.com/cyberbotics/webots/blob/R2025a/projects/objects/floors/protos/Floor.proto):
a static, locked solid at z=0 with the default contact material. The visible
lawn is 20×20 m; the collision plane is **infinite**, as in the upstream
Floor (Webots ignores `Plane.size` for collision). Its plain green appearance
needs no texture tiling. `protos/webots/` bundles the unmodified R2025a
TireRubber appearance and texture maps, with provenance and license, so wheel
appearance resolution needs neither SDK assets nor a runtime download.

### Signal chain (current topic names)

`sim_actuation_node` owns the firmware motor model. It sends `/cmd_vel_wheels`
to `diffdrive_controller`, the sole motor writer, which retains its velocity,
acceleration and command-timeout limits. After the native physics step,
`KinematicDrive` reads the same wheel PositionSensors as ros2_control and
integrates their measured differential-drive increment. Sharing a command
alone did not account for those limits or the wheel joints' actual motion.

```
/cmd_vel → sim_actuation_node → /cmd_vel_wheels → diffdrive_controller → motors
                                                        ↑                  ↓
                                                  PositionSensors ← physics step
                                                        ↓
                                                 KinematicDrive
                                                        ↓
                                            planar chassis pose + ODE velocity
```

`/sim/ground_truth_pose` and `/sim/applied_twist` describe one completed native
wheel interval and carry its ending Webots epoch. Initial offsets and rewinds
establish a baseline; nonfinite acquisitions hold the body without publishing
until a new finite baseline and positive interval exist. Repeated epochs cannot
publish another acquisition. Residual measured wheel motion during braking is
still body motion; this model provides no physical stopping evidence.

The synthetic IMU consumes the applied observation directly, including its
stamp. It waits for asynchronous `/clock` delivery with a bounded ten-sample
queue matching its DDS history and a ROS timer that can drain a final zero
without another input packet. Clock rewind clears accepted and pending epochs.
Raw IMU noise mode remains available independently.

The GNSS relay uses raw GPS as a cadence trigger. Generated coordinates and
fix/status stamps belong to the selected new truth pose; the raw antenna frame
is preserved. Each generated observation has a nonzero lifetime sequence, even
at rest. Future truth waits for clock eligibility without advancing authority;
clock rewind clears the pending/accepted stamps but preserves sequence identity.
Raw vendor wheel/IMU header phase and arbitrary cross-generation replay remain
separate contracts.

`diffdrive_controller` uses `velocity_rolling_window_size: 1` so its reported
twist describes the pose interval instead of a trailing mean during transitions.
`/wheel_odom`, `/imu/data` and `/gps/fix` feed `fusion_graph_node`, the sole
localizer. This repair does not alter that localizer or its tolerances.

## 3. Quirk 1 — a Webots `Cylinder`'s axis is +Z, not +Y

**RULE.** `Cylinder { radius R height H }` (VRML97, see
`/opt/webots/resources/nodes/Cylinder.wrl`) has its symmetry axis on **Z**. A
wheel must roll about body **+Y**, so every wheel cylinder — visual *and*
collision — must sit inside `Pose { rotation 1 0 0 -1.5708 children [ … ] }`,
which sends Z → +Y.

**ANCHORS** (`ros2/src/mowgli_simulation/protos/MowgliMower.proto`):

| Line | Token | Why it is separate |
|---|---|---|
| `:168`–`:184` | `DEF WHEEL_VISUAL Pose` (with the explanatory comment at `:169`–`:173`, geometry `DEF WHEEL_CYL` at `:181`) | The visual. `USE WHEEL_VISUAL` at `:223` reuses the *rotated* Pose, so the right wheel's visual is covered. |
| `:187`–`:192` | left `boundingObject Pose` | `USE WHEEL_CYL` alone is the **unrotated** geometry — the rotation must be repeated here. |
| `:226`–`:231` | right `boundingObject Pose` | Same. |

**The trap is the `boundingObject`.** `WHEEL_CYL` is a DEF of the bare
`Cylinder`, so `USE WHEEL_CYL` outside a rotated `Pose` gives you a collision
cylinder standing on end while the *visual* still looks perfect. This is the
silent failure mode: nothing looks wrong in the Webots viewport.

**SYMPTOM IF REINTRODUCED.** Phase 2.1's pathology: ~90 % wheel-floor slip and
a chassis settling at 12–13° forward pitch. The full historical record — and
the list of knobs that did *not* fix it — is preserved at
`protos/MowgliMower.proto:19`–`:49`.

Because `kinematic_drive` now teleports the body (§ 5), a broken
`boundingObject` no longer stops the robot from moving; it just makes the
chassis sit wrong and the wheels contact the floor incorrectly. That makes it
*harder* to notice, not easier.

## 4. Quirk 2 — the HingeJoint axis is deliberately `(0, -1, 0)`

**RULE.** `diff_drive_controller` commands **positive** ω for **forward**
motion. Under the right-hand rule about **+Y**, positive ω drives the wheel
bottom toward −X — i.e. the body *backward*. The PROTO therefore uses
`axis 0 -1 0`, which is intentional and must not be "corrected".

**ANCHORS** (`protos/MowgliMower.proto`):

- rationale comment `:78`–`:85`, including the empirical check: with
  `(0, +1, 0)` and `vx=+0.10`, the GPS reading moved **westward** (backward);
  flipping to `(0, -1, 0)` reversed it;
- `DEF LEFT_JOINT` → `HingeJointParameters` `axis 0 -1 0` at `:143`–`:146`;
- `DEF RIGHT_JOINT` → `axis 0 -1 0` at `:202`–`:204`.

**SYMPTOM IF REINTRODUCED.** Flipping to the ROS-looking `(0, 1, 0)` makes the
robot drive *away* from every Nav2 goal while `/wheel_odom_raw` still looks
entirely plausible (the encoders count the same magnitude). Every Nav2
behaviour then diverges instead of converging, and the first suspicion falls on
the controller, not on the PROTO.

## 5. Quirk 3 — a field-set teleport does not update ODE velocity

This is the core of the plugin and the easiest thing to "simplify" into
breakage.

### 5a Keep planar teleport and ODE velocity together

`KinematicDrive.__place_body` sets `translation` and `rotation` to the integrated
planar pose, then calls `setVelocity([vx_world, vy_world, 0, 0, 0, wz])` using
the measured interval twist. A teleport alone leaves ODE velocity unchanged.
Without zero vertical velocity, gravity accumulates between steps and attached
sensors can sink below the floor, causing `/scan` to become all `inf`.
Without angular velocity, the native gyro reports zero during a visible turn.
Velocity alone also fails: historical trials retained the chassis pitch because
wheel contact forces still act during ODE integration. Both writes are required.

### 5b Reset physics exactly once, and never write motors

`resetPhysics()` recurses into wheel HingeJoints and zeroes their velocities.
Calling it each step defeats motor commands and freezes wheel odometry. The
plugin calls it only at initialization to clear startup impulses.
`setVelocity()` affects only the chassis Solid, so it can run each step.

The plugin reads `left_wheel_sensor` and `right_wheel_sensor`; it never writes
either motor. `diffdrive_controller` owns motor initialization and setpoints.
A second writer races controller initialization and can leave the wheels at zero.
`supervisor TRUE` remains required for `getSelf()` and pose-field writes.

## 6. Quirk 4 — `mode:=realtime` is the default on purpose

**RULE.** Under `mode:=fast` Webots advances sim time at roughly 5× wall on
typical dev hardware. Nav2's `controller_server` is a **wall-rate** loop, so it
gets CPU-starved: the controller dt clamps to ~0.5 s and FTC's PID cannot close
a large heading error before `goal_timeout`. `realtime` is therefore the
default in both launch files.

**ANCHORS:**

- `ros2/src/mowgli_simulation/launch/webots_minimal.launch.py:45`–`:48`
  (`default_value='realtime'`);
- `ros2/src/mowgli_bringup/launch/sim_full_system.launch.py:101`–`:116`
  (the full rationale comment plus the arg).

The actuator and controller own command expiry; `KinematicDrive` has no
independent command cache or timeout. It follows measured wheel motion through
braking until the wheels actually stop. Their clock/timer behavior is a separate
actuation contract.

Fast mode remains legitimate for E2E runs whose timing budget allows it — the
plugin paces fine either way; the bottleneck is the Nav2 controller loop.

**SYMPTOM IF REINTRODUCED** (defaulting to fast): `PRE_ROTATE` never converges
and `FollowStrip` reports `goal_timeout` on every dispatch. Switching
`CMD_VEL_TIMEOUT_S` to sim time instead produces a robot that stutters and
stops repeatedly under fast mode.

## 7. Quirk 5 — an `<extern>` controller needs a body that already exists

**RULE.** `WebotsController` does **not** spawn geometry. It attaches to a
`Robot` node **already present in the `.wbt`** and wires that node's devices to
ROS. Custom geometry must therefore live in the world — inline, or in a PROTO
shipped inside the package and pulled by a *relative* `EXTERNPROTO`. A URDF
alone spawns nothing.

**ANCHORS:**

- `worlds_webots/mowgli_garden.wbt:11` — `EXTERNPROTO "../protos/MowgliMower.proto"`;
- `mowgli_garden.wbt:71`–`:76` — the `MowgliMower { … controller "<extern>" }` instance;
- `launch/webots_minimal.launch.py:59`–`:64` — `WebotsLauncher(..., ros2_supervisor=True)`;
- `launch/webots_minimal.launch.py:88`–`:99` — `WebotsController(robot_name='MowgliMower', …)`
  with `'set_robot_state_publisher': False`. The launch file runs its **own**
  `robot_state_publisher` with the same URDF content (`:79`–`:86`); leaving the
  driver-internal republish on gives two publishers on `/robot_description` and
  `controller_manager`'s transient_local subscriber can latch the wrong one.

**Boot-race sub-point** (reads like a quirk in the field): `webots-controller`
has a hardcoded **30 s** connect timeout, while Webots takes **20–40 s** to
load the world and only opens the `<extern>` slot at the end. If the timeout
fires first the controller prints "Gives up" and dies, taking the whole stack
down because `WaitForControllerConnection` then never starts the ros2_control
spawners. `respawn=True` re-attempts the connect and the second attempt
succeeds. See `webots_minimal.launch.py:114`–`:125`.

## 8. Troubleshooting — if X, check Y

| Symptom | Check |
|---|---|
| `/scan` is all `inf` | zero z velocity in `KinematicDrive.__place_body` |
| Wheels never spin | a second motor writer or per-step `resetPhysics()` |
| Chassis pitches forward | wheel Cylinder collision rotations (§ 3) |
| Positive wheel increments drive backward | HingeJoint axis (§ 4) |
| Body holds without truth packets | plugin warning for nonfinite native acquisitions |
| Synthetic IMU pauses | pending native samples waiting for `/clock` |
| Controller prints "Gives up" | external-controller boot retry (§ 7) |
| Body passes through obstacles | accepted kinematic collision-response limitation |

## 9. Smoke test

Minimal world, no Nav2:

```bash
ros2 launch mowgli_simulation webots_minimal.launch.py mode:=realtime
```

Command a forward creep. Note the type is **`TwistStamped`**, not `Twist`
(`kinematic_drive.py:241`–`:242`, matching twist_mux and
`diff_drive_controller`'s `use_stamped_vel: true`):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.15}, angular: {z: 0.3}}}'
```

The minimal launch starts `sim_actuation_node`, so `/cmd_vel` drives the wheels
and measured chassis together. Inspect `/wheel_odom_raw`, `/joint_states`,
`/sim/ground_truth_pose`, `/sim/applied_twist`, `/imu/data_sim` and `/scan`.
The full launch additionally starts the synthetic IMU and GNSS helpers.
Expect straight, pivot, arc and braking increments to agree with the measured
wheels; a new stationary interval must carry zero applied twist and unchanged
pose. The native body workaround retains finite scan returns above the floor.

## 10. Known drift / follow-ups

Flagged, **deliberately not fixed here** — each deserves its own issue.

**(a) Stale `robot_localization` / dual-EKF references remain in other sim
sources.** `fusion_graph_node` owns both localization transforms; controller
TF ownership and older launch comments may still name the removed EKF.

**(b) Numeric drift on the extension-slot lift.** `mowgli_garden.wbt:65` and
`:69`–`:70` say the slot is lifted **+0.10 m**; the PROTO actually lifts it
**0.283 m** (`protos/MowgliMower.proto:134`–`:139`). A sensor-placement trap:
anyone sizing a new sensor's slot z from the world comment lands 18 cm low.

**(c) `sim_imu_noise.py` still defaults to the Gazebo-era topic.**
`scripts/sim_imu_noise.py:32`–`:33` documents `gazebo_bridge.yaml`, and `:68`
defaults `input_topic` to `/imu/data_gz`. Only the launch override
(`sim_full_system.launch.py:437`, `"input_topic": "/imu/data_sim"`) makes it
correct — running the node standalone silently produces nothing.

**(d) Proposed CI guard.** Quirks 1 and 2 are pure prose today. A pytest in
`ros2/src/mowgli_simulation/test/` that parses `MowgliMower.proto` and asserts
(i) every wheel `Cylinder` geometry/`boundingObject` sits under a `Pose` with
`rotation 1 0 0 -1.5708`, and (ii) both `HingeJointParameters` carry
`axis 0 -1 0`, would turn them into enforced checks — the way
`ros2/scripts/check_config_drift.py` enforces Invariant 15. Not part of this
docs change.
