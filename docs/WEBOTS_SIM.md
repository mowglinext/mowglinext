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
| **C — body motion** | `mowgli_simulation/kinematic_drive.py` | `/cmd_vel` → firmware motor model → Supervisor teleport of the chassis, plus the chassis ODE velocity for sensor sampling | Wheel motor commands (owned by `diff_drive_controller`) |

**Geometry lives in A only.** The URDF is *device + plugin wiring* — it does
not spawn a body (see § 7).

### Signal chain (current topic names)

Note the **split**: `kinematic_drive` reads raw `/cmd_vel`, while the wheels
are driven from `/cmd_vel_wheels` via `sim_actuation_node`. The body pose and
the wheel encoders are therefore fed by two different points in the chain.

```
/cmd_vel ─┬─> kinematic_drive plugin  ──> Supervisor teleport of the chassis
          │      (urdf:202 <cmdVelTopic>/cmd_vel</cmdVelTopic>)
          │
          └─> sim_actuation_node ──> /cmd_vel_wheels ──> diffdrive_controller
                 (sim_actuation_node.cpp:80-83)   (webots_minimal.launch.py:111)
                                                          │
                                    /wheel_odom_raw <─────┘ (launch:112)
                                          │
                        sim_wheel_slip (sim_full_system.launch.py:396-415)
                                          │
                                     /wheel_odom
```

Sensors:

| Raw topic | Source | Post-processor | Final topic |
|---|---|---|---|
| `/imu/data_sim` | Ros2IMU plugin, `mowgli_webots.urdf:185` | `sim_imu_noise` (`sim_full_system.launch.py:424-437`) | `/imu/data` |
| `/gps/fix_raw` | GPS device, `mowgli_webots.urdf:176` | `sim_navsat_rtk_fix` (`sim_full_system.launch.py:313-330`) | `/gps/fix` |
| `/scan` | Lidar device, `mowgli_webots.urdf:166` | — (world-level `noise 0.002` / `resolution 0.01`, `mowgli_garden.wbt:122-123`) | `/scan` |

`/wheel_odom`, `/imu/data` and `/gps/fix` then feed `fusion_graph_node`, which
is the sole localizer and owns **both** `map→odom` and `odom→base_footprint`
(CLAUDE.md Invariant 1). There is no EKF in this stack — see § 10(a) for the
stale comments that still claim otherwise.

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

### 5.0 The three revisions (`kinematic_drive.py:49`–`:76`, `EVOLUTION`)

| Rev | Approach | Result |
|---|---|---|
| 1 | `Supervisor.setVelocity()` **only**, no teleport | cmd_vel honesty 10 % → 70 % (0.36 m over 5 s vs 0.50 m expected), but the 13° chassis pitch **remained** — gravity and wheel-friction normals still act during ODE integration. |
| 2 | field-set teleport + **per-tick** `resetPhysics()` | honesty 95 %, pitch 0°, but `resetPhysics()` recursed into the wheel HingeJoints and zeroed `diff_drive_controller`'s motor setpoints — wheels never spun. |
| 3 (current) | field-set teleport + per-tick `setVelocity()` of the **commanded** twist, with `resetPhysics()` called **exactly once at init** (`:273`–`:275`) | Both problems solved. |

> Issue #201 paraphrases quirk 3 as "the bug was the absence of
> `setVelocity()` each tick". That is **wrong in mechanism**, and documenting
> it that way would teach a future maintainer to drop the teleport. Revision 1
> *was* setVelocity-only and still pitched 13°. The teleport and the
> `setVelocity` are a **pair**: the teleport owns the pose, the `setVelocity`
> owns the ODE velocity that gravity and the sensors read. And the gyro bug
> came from setting all six DOF to **zero**, not from omitting the call.

### 5a Gravity / `/scan` all-`inf`

**RULE.** The teleport rewrites `translation`
(`kinematic_drive.py:456`, `setSFVec3f`) but leaves the ODE velocity alone, so
gravity accumulates a downward velocity between ticks. Sensors sample **after**
the physics step but **before** the plugin teleport, so the chassis — and the
rigidly attached Lidar — is sampled *sunken*. Once the implied terminal
velocity puts the Lidar below the floor, every ray clips the floor's underside
and returns `+inf`.

**Fix:** the literal `0.0` in the third slot of the `setVelocity` vector.

**ANCHOR:** `kinematic_drive.py:475`–`:489` (`WHY THIS IS LOAD-BEARING
(gravity / lidar)`) and the call at `:512`–`:515`.

**SYMPTOM IF REINTRODUCED.** `/scan` returns all `+inf` after a few hundred
ticks; the costmap goes blank and obstacle deviation never triggers.

### 5b Gyro / IMU reads zero

**RULE.** The `InertialUnit` + `Gyro` + `Accelerometer` trio reports the rigid
body's **ODE velocity**, never the field-set teleport rate. A six-DOF-zero
`setVelocity` therefore makes the gyro read 0 rad/s while the chassis visibly
rotates.

**Fix:** feed the **commanded** twist —
`[vx·cos(yaw), vx·sin(yaw), 0.0, 0.0, 0.0, wz]`.

**ANCHOR:** `kinematic_drive.py:490`–`:501` (`WHY THIS IS LOAD-BEARING
(sensor angular velocity)`) and the call at `:512`–`:515`.

**SYMPTOM IF REINTRODUCED.** `fusion_graph`'s gyro between-factor concludes the
robot is static, so the fused yaw never advances; FTC's `PRE_ROTATE` never sees
the heading error close and the 10 s `goal_timeout` fires on every
`FollowStrip` dispatch.

### 5c NEVER call `resetPhysics()` per tick

**RULE.** `resetPhysics()` **recurses into children**, zeroing the wheel
HingeJoint angular velocities and silently overwriting
`diff_drive_controller`'s motor setpoints every tick.
`Supervisor.setVelocity()` does **not** recurse — it touches only this Solid's
ODE state — which is exactly why it is safe to call every step and
`resetPhysics()` is not.

**ANCHORS:** the one legitimate call is at `kinematic_drive.py:273`–`:275`
("Reset physics ONCE at boot"); the warning block is `:517`–`:541`.

**SYMPTOM IF REINTRODUCED.** `/joint_states` frozen at position 0 →
`/wheel_odom_raw` integrates to 0 → Nav2's pose-tracking behaviours (BackUp,
DriveOnHeading, Spin) all time out believing the robot never moved.

### 5d Two more contracts in the same function

- **The plugin must never write the wheel motors.** `diff_drive_controller`
  owns them; an "idempotent re-init" from the plugin race-conditions with the
  controller's own init and leaves the motors stuck at zero. Rationale at
  `kinematic_drive.py:542`–`:565` and, for the init-time case, `:214`–`:225`.
- **`supervisor TRUE` is mandatory.** `Supervisor.getSelf()` and the
  `translation`/`rotation` field writes require it. Declared in the world at
  `mowgli_garden.wbt:77`–`:81`; the plugin that needs it is declared at
  `mowgli_webots.urdf:196`–`:203`. The plugin raises a `RuntimeError` naming
  this explicitly if `getSelf()` returns NULL (`kinematic_drive.py:183`–`:190`).

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

**The non-obvious corollary:** the plugin's `cmd_vel` timeout is **wall-clock,
not sim-time**, precisely so fast mode does not stutter the robot. Publishers
(controller_server, behavior_server, collision_monitor, twist_mux) cycle at a
wall rate; a sim-time timeout would trip every cycle under fast mode and stop
the robot repeatedly. See `CMD_VEL_TIMEOUT_S` and its docstring at
`kinematic_drive.py:160`–`:170`, and the use site at `:416`–`:428`
(`time.monotonic()`).

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
| `/scan` is all `+inf` | the `0.0` z component of `setVelocity` — `kinematic_drive.py:512`–`:515`, rationale `:475`–`:489` |
| `/imu/data` gyro_z stays 0 while the robot visibly turns | the `wz` component of the same call — `kinematic_drive.py:490`–`:501` |
| Wheels never spin, `/joint_states` frozen, Nav2 BackUp times out | a per-tick `resetPhysics()` crept back in (`:517`–`:541`), or the plugin started writing the motors (`:542`–`:565`) |
| Chassis pitches forward / robot barely moves | a wheel `Cylinder` or `boundingObject` lost its `Pose { rotation 1 0 0 -1.5708 }` — `MowgliMower.proto:168`, `:187`, `:226` |
| Robot drives backward on a positive `vx` | HingeJoint axis flipped to `(0, 1, 0)` — `MowgliMower.proto:143`–`:146`, `:202`–`:204` |
| `PRE_ROTATE` never converges / `FollowStrip` `goal_timeout` | running `mode:=fast` — `sim_full_system.launch.py:101`–`:116` |
| Robot stutters and stops repeatedly under fast mode | a sim-time (rather than wall-clock) cmd_vel timeout — `kinematic_drive.py:160`–`:170` |
| Controller prints "Gives up" at boot; spawners never start | `respawn` disabled — `webots_minimal.launch.py:114`–`:125` |
| `Supervisor.getSelf()` returns NULL (plugin raises at init) | `supervisor FALSE` in the world — `mowgli_garden.wbt:81` |
| The body passes straight through obstacles | known and accepted: we teleport every step, so collision **response** is out of scope. Nav2's collision_monitor + costmap own stopping. `kinematic_drive.py:91`–`:98` |
| Wheels spin but the body does not move as commanded | the firmware motor model is deliberately filtering a sub-deadband command — `kinematic_drive.py:132`–`:157` (`FIRMWARE_DEADBAND_PWM_*`) |

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

**`/cmd_vel` alone moves the body but not the wheels here.** The minimal launch
boots Webots + the driver + the ros2_control spawners only; it does **not**
start `sim_actuation_node` (that node is added by
`sim_full_system.launch.py:483`). `diffdrive_controller`'s command topic is
remapped to `/cmd_vel_wheels` (`webots_minimal.launch.py:111`), so in the
minimal world nothing publishes it and `/wheel_odom_raw` stays at zero. Feed
the same twist to the wheels as well:

```bash
ros2 topic pub -r 10 /cmd_vel_wheels geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.15}, angular: {z: 0.3}}}'
```

Expectations:

| Check | Command | Expect |
|---|---|---|
| Wheels turn | `ros2 topic echo /wheel_odom_raw --field twist.twist.linear.x` | advances, **not** zero |
| Gyro is honest | `ros2 topic echo /imu/data_sim --field angular_velocity.z` | tracks the commanded `wz` |
| Lidar is above the floor | `ros2 topic echo /scan --field ranges` | finite values, not all `inf` |
| Body actually moved | `ros2 topic echo /sim/ground_truth_pose` | tracks the integrated command (`kinematic_drive.py:459`–`:469`) |

Two gotchas:

- Check **`/imu/data_sim`**, the raw topic — `/imu/data` carries
  `sim_imu_noise`'s injected white noise and bias walk, which is easy to
  mistake for the bug you are hunting.
- **`/cmd_vel_wheels` is the actuation-model output, not the nav command.** A
  discrepancy between `/cmd_vel` and `/cmd_vel_wheels` is `sim_actuation_node`
  reproducing the firmware deadband, working as intended.

## 10. Known drift / follow-ups

Flagged, **deliberately not fixed here** — each deserves its own issue.

**(a) Stale `robot_localization` / dual-EKF references in sim sources.** The
EKF was removed; `fusion_graph_node` is the sole localizer and owns both TFs
(CLAUDE.md Invariant 1). These comments predate that and are wrong:
`kinematic_drive.py:43`, `:85`, `:89`, `:223`, `:495`, `:498`, `:527`–`:528`,
`:555`; `config_webots/ros2_control.yaml:12` and `:44`–`:46`;
`urdf_webots/mowgli_webots.urdf:56` and `:62`; `protos/MowgliMower.proto:11`. The
mechanism they describe (wheel encoders → `/wheel_odom_raw` → dead-reckoning →
Nav2 behaviours) is still correct; only the consumer's name is stale. **This
document describes `fusion_graph` and deliberately does not repeat them.**

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
