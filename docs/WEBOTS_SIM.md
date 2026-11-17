# Webots Simulation — Native Plant and Observation Contracts

## 1. Scope

**Audience:** anyone editing `ros2/src/mowgli_simulation/worlds_webots/`,
`protos/`, `urdf_webots/`, or `mowgli_simulation/kinematic_drive.py`.

**This document is NOT:**

- how to *run* the sim — see [`ros2/README.md` § Running Webots Simulation](../ros2/README.md#running-webots-simulation);
- the operator guide — see [`wiki/Simulation.md`](../wiki/Simulation.md);
- the package index (files, nodes, topics, parameters, tests) — see
  [`docs/claude/codemaps/mowgli_simulation.md`](claude/codemaps/mowgli_simulation.md).

The current plant uses native motor/contact physics. KinematicDrive retains
its name for URDF compatibility but only observes native body pose intervals.
The old per-step teleport/velocity workaround is retired: grounded native
controls identified the teleport path as necessary for accumulating child-joint
deformation in that fixture. Historical observations below remain specific to
their baseline; they are not requirements for the current plant.

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
| **C — body observations** | `mowgli_simulation/kinematic_drive.py` | native Solid-origin pose + full orientation → paired native-epoch truth/planar interval twist | Wheel motor commands (owned by `diff_drive_controller`); the firmware motor model itself |

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

`sim_actuation_node` owns the firmware motor model. ros2_control is the sole
native motor writer, retaining its limits and command timeout. Native physics
determines body motion; KinematicDrive reads the completed Solid pose. Wheel
odometry independently reads PositionSensors and can disagree with body truth
due to native slip/contact response. Do not restore wheel-derived truth or
another command-to-body model.

```text
/cmd_vel -> sim_actuation_node -> /cmd_vel_wheels -> diffdrive_controller
                                                        |
                                                 native motors
                                                   /         \
                                     native body pose    PositionSensors
                                            |                  |
                                     KinematicDrive      joint odometry
                                     GT + interval       /wheel_odom_raw
                                     applied twist             |
                                                         sim_wheel_slip
                                                               |
                                                          /wheel_odom
```

The observation plugin has no command subscription, motor/device access, body
field writes, velocity setter, or physics reset. Braking, collisions and slip
remain native motion; expired commands do not authorize reporting false zero.
A first finite pose establishes an interval baseline. Invalid acquisitions emit
no packet and clear that baseline; recovery needs a finite baseline followed by
a new positive interval. A repeated native epoch cannot replace a payload, and
rewind starts a new baseline at the actual native location.

`/sim/ground_truth_pose` is the completed native Solid-origin XYZ and full
world-from-body quaternion at `Supervisor.getTime()`, in the garden ENU/map
frame with forward/left/up body axes. When level, the origin coincides with
`base_footprint`. `/sim/applied_twist` has the same ending epoch and describes
the constant SE(2) interval twist reconstructing the consecutive native XY and
projected body-X yaw observations. Linear x/y are in the moving yaw-only
`base_footprint` frame, including lateral slip; angular.z is the shortest signed
yaw increment divided by interval duration. Turns beyond pi within one interval
are not identifiable from these endpoints. Roll/pitch are preserved in truth,
but this planar twist and synthetic IMU are not a full 3D dynamics model.
`getVelocity()` is deliberately unused: it reports ODE **COM** velocity, while
`getPosition()` reports the **Solid origin**. With the current offset COM,
the two linear velocities differ during rotation.

The synthetic IMU consumes the applied observation directly, including its
stamp. It waits for asynchronous `/clock` delivery with a bounded ten-sample
queue matching its DDS history and a ROS timer that can drain a final zero
without another input packet. Clock rewind clears accepted and pending epochs.
Raw IMU noise mode remains available independently.

The GNSS relay uses raw GPS as a cadence trigger. Its planar antenna lever arm
uses yaw extracted from the full native quaternion; this is not a 3D antenna
model. Generated coordinates and fix/status stamps belong to the selected new truth pose; the raw antenna frame
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

**Historical symptom.** Phase 2.1 reported: ~90 % wheel-floor slip and
a chassis settling at 12–13° forward pitch. The full historical record — and
the list of knobs that did *not* fix it — is preserved at
`protos/MowgliMower.proto:19`–`:49`.

Native physics makes correct wheel geometry and an actual collision floor
necessary for motion. A visually present floor without a boundingObject is not
a physical contact surface.

## 4. Physical hinge axes match the positive-forward ROS convention

Both wheel HingeJointParameters use **`axis 0 1 0`**, matching the two ROS URDF
joint axes. Positive angular velocity about +Y moves the wheel bottom toward
-X **relative to the axle**; rolling on stationary ground therefore moves the
body toward +X. The old -Y rationale reversed contact-point and body velocity.

Grounded R2025a motor-only controls at #1063 `cc4e40c8` plus ground `91f130ff`
showed the distinction: removing body writes stabilized native mechanics, but
with -Y axes the body moved +0.552875 m at 15 s while wheel integration claimed
-0.554931 m. Changing only both physical axes to +Y produced -0.552739 m body
motion with the same wheel-model value and the correct pivot sign. These are
native mechanical controls, not a full-driver mission or hardware validation.
Do not invert encoder feedback to mask a physical axis error.

## 5. No Supervisor body actuation in the observation plugin

The native motor/contact plant owns body translation, rotation and velocity.
The plugin must never call field setters, setVelocity or resetPhysics, nor
write motors or add a second firmware loop. `supervisor TRUE` remains required
for `getSelf()` and native pose observations.

Grounded controls of the unchanged #1063 plugin, using the same recorded motor
input trace, reached 2326 m maximum child-anchor error and 177 degrees tilt.
Omitting only the two per-step translation/rotation writes held maximum anchor
error to 0.000100008 m and tilt to 0.7935 degrees over the same 200 sim seconds.
Removing all body writes and correcting both axes also remained stable. These
measurements isolate the teleport path for that native fixture; they do not
claim full-system, navigation or physical acceptance.

In R2025a, a Supervisor teleport recursively moves children and refreshes joint
anchors. ODE recomputes local anchors from the current bodies, which can retain
deformation. `setVelocity()` changes only the root ODE body; it cannot repair
child geometry. Per-step `resetPhysics()` recursively disables motors and is
not a repair either. Publish native body observations, leaving wheel odometry
independent; never hide abnormal wheel increments with clamps or fake truth.

### Historical workaround record (superseded)

The earlier workarounds below predate the grounded controls and must not be
restored as current rules. An incomplete world without physical ground can
invalidate conclusions previously attributed to ODE quirks.

| Revision | Historical approach | Historical observation |
|---|---|---|
| 1 | Root setVelocity only | Claimed 70% commanded travel and 13-degree pitch. |
| 2 | Teleport plus per-step resetPhysics | Claimed 95% travel and level body; wheel motors stopped. |
| 3 | Teleport plus per-step planar setVelocity and one init reset | Claimed to solve gravity/sunken lidar and zero gyro sampling. Later native controls expose accumulating joint deformation. |
| #1063 | Derive teleport and velocity from actual wheel increments | Fixed command/wheel-model disagreement, source epochs and helper clock eligibility; could still faithfully integrate corrupt native wheels while missing native body failure. |

The historical two-plant and wheel-interval evidence remains in
[`MOTION_AUTHORIZATION_IMPLEMENTATION.md`](MOTION_AUTHORIZATION_IMPLEMENTATION.md).
It establishes the behavior of those exact models, not native body truth or
current mission acceptance. Helper identity, clock eligibility and wheel
odometry rolling window of one remain in force.

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
independent command cache or timeout. It observes native body motion through
braking until the body actually stops. Their clock/timer behavior is a separate
actuation contract.

Fast mode remains legitimate for E2E runs whose timing budget allows it — the
plugin paces fine either way; the bottleneck is the Nav2 controller loop.

**SYMPTOM IF REINTRODUCED** (defaulting to fast): `PRE_ROTATE` never converges
and `FollowStrip` reports `goal_timeout` on every dispatch. Mechanical
controls in fast mode do not establish full-stack realtime behavior.

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
| `/scan` is all `inf` | native sensor height/orientation, physical ground and scene assets |
| Wheels never spin | a second motor writer or per-step `resetPhysics()` |
| Chassis pitches forward | wheel Cylinder collision rotations (§ 3), physical ground, native child geometry |
| Positive wheel increments drive backward | HingeJoint axis (§ 4) |
| No truth packets | plugin warning for invalid native body acquisitions; finite baseline and new interval required |
| Synthetic IMU pauses | pending native samples waiting for `/clock` |
| Controller prints "Gives up" | external-controller boot retry (§ 7) |
| Wheel odom differs from native truth | native slip/contact response; wheel integration is not body truth |

## 9. Smoke test

Minimal world, no Nav2:

```bash
ros2 launch mowgli_simulation webots_minimal.launch.py mode:=realtime
```

Command a forward creep. Note the type is **`TwistStamped`**, not `Twist`
(matching twist_mux and
`diff_drive_controller`'s `use_stamped_vel: true`):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.15}, angular: {z: 0.3}}}'
```

The minimal launch starts `sim_actuation_node`, so `/cmd_vel` drives the wheels
and measured chassis together. Inspect `/wheel_odom_raw`, `/joint_states`,
`/sim/ground_truth_pose`, `/sim/applied_twist`, `/imu/data_sim` and `/scan`.
The full launch additionally starts the synthetic IMU and GNSS helpers.
Expect signed straight/pivot/arc and braking motion from native physics.
Truth must equal the native Solid pose; applied planar twist reconstructs its
completed interval. Wheel odom may differ due to slip. A stationary body interval
must carry zero applied twist and unchanged pose. Check native sensor height and
scan returns directly; no body workaround enforces them.

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

**(d) Focused native-observation regression guards.**
`test_kinematic_wheel_feedback.py` retains its existing CI registration but
now supplies actual native pose/orientation fixtures: truth independence from
wheel/control history, signed straight/pivot/curved intervals, lateral slip,
full orientation, no body/motor writes and invalid/duplicate/rewind recovery.
It also pins both physical and ROS URDF joint axes to +Y. The separate real
upstream wheel-odom interval test still checks encoder pose/velocity coherence;
no equality to native body truth is assumed. The wheel-cylinder rotation guard
proposed earlier remains a possible follow-up.
