# External MAVROS Backend Migration Audit

> **Active migration checkpoint (2026-09-08).** This is an audit and implementation plan, not a
> claim that the MAVROS backend is safe to operate. The native STM32 backend remains the only
> operationally validated backend. Actuation, blade, power, failsafe, reconnect, and Raspberry Pi
> deployment acceptance remain `HARDWARE_PENDING`.

## 1. Scope and audited baselines

This audit answers the minimum work required for current MowgliNext to consume the standalone
MowgliMAVROS image without reviving the historical in-tree implementation.

| Repository / evidence | Ref audited | Role |
|---|---|---|
| `mowglinext/mowglinext` | `feat/mavros`, `dev`, and `origin/dev` = `3b0974809809ace589a567b6c32e5bed6e599489` | authoritative MowgliNext baseline |
| `mavros-archive/mavrosdev` | `241de50630f6889fb485b0e80f9946c91ddad8ae` | historical evidence only |
| `mavrosdev-history` | `241de50630f6889fb485b0e80f9946c91ddad8ae` | identical local historical ref |
| merge base | `a201c3f0bbefb7416c56a9c180854565167b321d` | boundary for history-aware comparison |
| `Pepeuch/mowglimavros` (redirects to `mowglifrenchtouch/mowglimavros`) | code baseline `6b4f1975ad1ece9e44c7f681e56ccdc0d90f3972`; audit-doc HEAD `4bb0b2c` | intended external backend |
| upstream MAVROS | 2.15.1, commit `22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e` | pinned dependency used by the external image |

Commands used were history-aware and path-limited: `git merge-base`, `git log
<merge-base>..mavros-archive/mavrosdev`, `git diff dev...mavros-archive/mavrosdev`, per-file history,
and direct blob inspection. The historical branch was never checked out, built, merged, rebased, or
cherry-picked. No build, formatter, generated-file workflow, commit, or push was run.

Pre-existing untracked `.agent/` and `.devcontainer/devcontainer-lock.json` were not touched.

## 2. Executive result

The installer already retains much of the old *selection* scaffolding, but the runtime is not a
working backend swap. The stable finding IDs below must not be reassigned:

1. **MN-MAV-001 — duplicate backend ownership.** `mowgli.launch.py` always launches the native `mowgli_hardware` node. Selecting the MAVROS
   compose fragment therefore starts two command-consuming nodes named `hardware_bridge`.
2. **MN-MAV-002 — obsolete compose topology.** The compose fragment points at an image MowgliNext does not build and starts a separate NTRIP
   container whose package is absent from the main image. The external image already owns MAVROS,
   the compatibility bridge, and NTRIP.
3. **MN-MAV-003 — incompatible interface copy.** The external image embeds stale `mowgli_interfaces`. Its `Status.msg` lacks current reset,
   firmware, debug, and blade-freshness fields; its `HighLevelStatus.msg` lacks
   `coverage_percent`. DDS type compatibility with current MowgliNext is therefore not established.
4. **MN-MAV-004 — missing public GNSS contract.** The external backend does not publish current MowgliNext's `/gps/fix` plus observation-identity
   `/gps/status` contract. A raw topic remap is insufficient because receiver selection, fix/status
   pairing, acquisition identity, and delivery liveness must remain explicit.
5. **MN-MAV-005 — invalid wheel-odometry assumption.** It republishes `/mavros/local_position/odom` as `/wheel_odom`. That is not proven wheel-only
   odometry, was not observed in the passive Pixhawk session, and may already contain autopilot GPS
   fusion. Feeding it to `fusion_graph` as a wheel between-factor risks missing feedback or
   double-counting absolute position.
6. **MN-MAV-006 — wrong power-instance model.** Its single last-message-wins `/mavros/battery` path cannot represent this installation's
   `POWER1=dock/charger` and `POWER2=traction` semantics. MAVROS 2.15.1 publishes all
   `BATTERY_STATUS` instances on `/mavros/battery` and identifies them with
   `BatteryState.location="idN"`; the adapter must map instances by configuration.
7. **MN-MAV-007 — ambiguous composite USB selection.** The selected Pixhawk exposes two USB serial interfaces. The current udev rule matches only
   VID/PID/serial, so both `if00` and `if02` can claim `/dev/mavros`. The validated ArduRover link is
   the stable `...-if00` endpoint.
8. **MN-MAV-008 — cached data presented as fresh.** The external bridge refreshes `Status`/`Power` stamps from a timer even when the underlying
   MAVROS observation is cached. Connection, observation freshness, and stream liveness therefore
   need separate semantics before readiness can be trusted.

Accordingly, a safe first MowgliNext patch may integrate an **inert** external sidecar and prove
exclusive launch/topology, but it must not enable driving or mowing. Operational acceptance is
blocked on the external contract work and the hardware gates below.

## 3. Historical MAVROS introduction

The following commits are the relevant source-bearing history. Earlier duplicate lines
(`ebe84d27`/`89ec6acb`/`381d29cc` and `ecf4e336`/`edfab521`/`0898ae3b`) arose on converging branch
lines; they are evidence, not a set to replay.

| Commit(s) | Historical contribution |
|---|---|
| `ebe84d27`, `ecf4e336` | first MAVROS compatibility bridge implementation |
| `89ec6acb`, `edfab521` | initial safety/frame handling and package metadata |
| `381d29cc`, `0898ae3b` | installer selection, Pixhawk detection, env/udev, bundled image entrypoint |
| `892ff5ae`, `04b3df57`, `92e0cb84` | installed MAVROS build dependencies and visibility diagnostics |
| `40961908`, `06c56c2c` | hardware detection and installer hardening |
| `6819518c`, `a0813e9f` | ArduPilot/PX4 launch selection |
| `2c4c2356` | consolidated bridge plus provisional-safe configuration |
| `518382f7` | bringup selection and backend topic launch tests |
| `7b2a8430` | runtime/installer alignment |
| `6b4669cc` | runtime GUI backend card |
| `a4d3fc0b` | separate NTRIP compose service |
| `73c33ba2` | configurable GCS forwarding |
| `1efbb228` | NTRIP/frontend follow-up |
| `11b3abf2`, `4d82eff1` | installer/runtime configuration convergence |
| `00e99991`, `bba6a137`, `5f74f2d0`, `11d9fbea` | MAVROS-aware checks, datum behavior, and build guards |
| `7022a28e`, `584a26db`, `a708040f` | formatting/build cleanup and `TwistStamped` correction |

The final historical MAVROS-specific file set was:

- `sensors/mavros/{Dockerfile,ros2_entrypoint.sh}`;
- `ros2/src/mowgli_mavros_bridge/**`;
- `ros2/src/mowgli_bringup/config/hardware_bridge_mavros.yaml`;
- backend conditions and tests under `ros2/src/mowgli_bringup/{launch,test,CMakeLists.txt,package.xml}`;
- `install/compose/docker-compose.mavros.yml`, `install/lib/backend_choice.sh`, and MAVROS paths in
  `checks.sh`, `compose.sh`, `config.sh`, `deploy.sh`, `env.sh`, `udev.sh`, and `mowglinext.sh`;
- `gui/web/src/{components/BackendSettingsCard.tsx,hooks/useBackendSettings.ts}` plus page wiring;
- NTRIP/GCS and then-obsolete GNSS configuration and tests.

## 4. Migration classification

Each item has exactly one requested migration classification.

| Historical concept/change | Class | Decision |
|---|---|---|
| `HARDWARE_BACKEND=mowgli|mavros` as the single selector | `ALREADY_PRESENT` | Retained in current installer/env/compose logic. It must become effective in ROS bringup. |
| Installer CLI/preset backend choice | `ALREADY_PRESENT` | Current `backend_choice.sh`, `config.sh`, state allowlist, and tests already carry it. |
| Mutual exclusion of direct GNSS container in MAVROS mode | `ADAPT` | Current installer does this, but the external adapter must provide the canonical public GNSS contract before direct GNSS is disabled operationally. |
| Conditional native bridge launch | `ADAPT` | Restore only the selection condition, updated for current launch/config architecture. Do not launch an in-tree MAVROS bridge. |
| In-tree `ros2/src/mowgli_mavros_bridge` | `DO_NOT_PORT` | The bridge now belongs exclusively to the external repository/image. |
| Bundled `sensors/mavros` image | `OBSOLETE` | Replaced by the pinned standalone image. |
| Current `docker-compose.mavros.yml` service shape | `ADAPT` | One external service must replace the current MAVROS + broken main-image NTRIP pair. |
| `MAVROS_IMAGE` configuration | `ADAPT` | Point at the standalone image and pin a validated Kilted multi-arch tag/digest independently of MowgliNext's image channel. |
| Pixhawk detection and `/dev/mavros` udev alias | `ADAPT` | Preserve explicit selection but include USB interface identity (`if00`), not only VID/PID/serial. |
| `ROS_DOMAIN_ID`, Cyclone DDS, host networking, Cyclone config mount | `ALREADY_PRESENT` | Current fragments already carry the correct common DDS environment and mount. |
| GCS URL, target system/component, autopilot selection, 921600 baud | `KEEP` | These remain required external-image inputs; validate values and URL syntax. |
| Historical topic-name compatibility goal | `KEEP` | The rest of MowgliNext should keep consuming the native public contract. |
| Historical `/mavros/local_position/odom` to `/wheel_odom` mapping | `DO_NOT_PORT` | It is not evidence of wheel-only odometry and can double-count autopilot fusion. |
| Historical single `/mavros/battery` semantic mapping | `DO_NOT_PORT` | Replace with configurable dock/traction instance mapping. |
| Historical periodic cached status/power republication | `DO_NOT_PORT` | Preserve observation identity and distinguish it from monotonic delivery liveness. |
| Historical separate `mowgli-ntrip` service in the main image | `OBSOLETE` | The package is absent from current MowgliNext and NTRIP belongs to the external sidecar. |
| External NTRIP path as currently implemented | `ADAPT` | Segment every publication to `<=720` bytes and test byte-perfect concatenation. |
| Historical u-blox/Unicore/NMEA/NTRIP stack and launch files | `DO_NOT_PORT` | Universal GNSS public semantics are the architecture of record. |
| Historical copied install-time Nav2/SLAM/localization configs | `DO_NOT_PORT` | They conflict with the sparse config, `fusion_graph`, current Nav2 overlays, and package-share ownership. |
| Historical launch topic tests | `ADAPT` | Keep their exclusivity/contract intent, but test an external sidecar boundary and the expanded current contract. |
| Historical runtime GUI backend card | `DO_NOT_PORT` | Saving two keys and restarting ROS2 cannot safely regenerate/recreate a compose topology. Installer/web-composer selection is sufficient for the minimum migration. |
| Steering/throttle/manual-control mapping | `HARDWARE_PENDING` | Defaults must remain disabled until bounded hardware validation. |
| Blade command/feedback | `HARDWARE_PENDING` | Defaults must remain disabled; firmware/autopilot safety authority needs explicit acceptance. |
| HOLD, arm/disarm, emergency/failsafe behavior | `HARDWARE_PENDING` | An asynchronous request being queued is not physical confirmation. |
| Charging and two-power-instance behavior | `HARDWARE_PENDING` | Software mapping is required first, then physical validation. |
| USB reconnect, FCU reboot, power-cycle recovery, ARM64/RPi4 | `HARDWARE_PENDING` | Workstation passive reconnect is evidence only for its recorded baseline. |

## 5. Hardware contract matrix

| Surface | Current native `mowgli_hardware` | Historical `mavrosdev` | External MowgliMAVROS at `6b4f197` | Required migration disposition |
|---|---|---|---|---|
| Ownership | Main container node `hardware_bridge`; sole STM32 serial owner | Main-container compatibility bridge plus a separate MAVROS-only container | One sidecar contains MAVROS, bridge, and optional NTRIP | Main container must suppress only native bridge; sidecar owns backend; never both |
| Velocity input | `/cmd_vel`, `geometry_msgs/TwistStamped`; m/s and rad/s sent to firmware | Scaled to `mavros_msgs/ManualControl.x/r` | Same provisional scaling; disabled by default | Keep disabled; add bounds, zero/watchdog/freshness semantics, then hardware-test |
| Wheel feedback | `/wheel_odom`, wheel-encoder-derived, reliable QoS(10), zero while charging | Relayed `/mavros/local_position/odom` | Same | Replace with proven wheel-only source; do not label fused local position as wheel odom |
| Wheel ticks | `/wheel_ticks`, `mowgli_interfaces/WheelTick` | absent | absent | Document intentional absence or add equivalent only if a consumer requires it; dig detection cannot silently pretend parity |
| IMU | `/imu/data` reliable, calibrated STM32 IMU; `/imu/mag_raw` also supplied | relayed `/mavros/imu/data` | same relay | Verify frame, convention, covariance, timestamp, calibration, and reliable publisher QoS |
| Public GNSS | `/gps/fix` + observation-identified `/gps/status`; `/rtcm` in Universal GNSS mode | old direct/MAVROS GNSS assumptions | exposes MAVROS GPS1/GPS2 surfaces only | External adapter must publish current public contract with explicit selected receiver/source incarnation |
| Status | Current `mowgli_interfaces/Status`, live hardware semantics, firmware handshake | old interface and mostly synthetic/default fields | stale interface; timer republishes cached fields | Synchronize exact IDL; define backend readiness and stale/disconnected behavior; never forge fresh observations |
| Power | `v_charge=dock`, `v_battery=traction`; charger bit authoritative | last `/mavros/battery` message becomes both traction and charge state | same | Map `BatteryState.location=idN` through configurable dock/traction instance IDs |
| `/battery_state` | traction voltage; current=`abs(charge_current)` only while charging, else zero; reliable QoS(10) | republishes MAVROS battery message | same, SensorDataQoS | Publish traction SoC/voltage plus Mowgli docking-current convention from mapped sources |
| Blade service | `/hardware_bridge/mower_control`; firmware makes final safety decision | service exists but reports failure; no actuator mapping | same, disabled by default | Keep disabled until command, feedback, stale-stop, and safety ownership pass hardware acceptance |
| Emergency service | Firmware latch/request path; physical trigger remains authoritative | local latch plus async HOLD/disarm requests | same; reset clears active but not the local latched flag | Define FCU-confirmed state and reset semantics; validate physically before use |
| Arming/mode | Firmware high-level state and heartbeat; no MAVROS arming | emergency path can set HOLD/disarm; no normal arming policy | same | Define explicit startup/operational arming policy; keep inert meanwhile |
| Dig safety | Wheel-vs-GNSS detector, bounded reverse, pending keepout, escalation latch | absent | absent | Record as an intentional backend capability gap or implement equivalent with independent signals before mowing |
| Reboot/debug services | `reboot_board`, `set_firmware_debug` | absent | absent | May be backend-specific, but GUI/checks must not promise unsupported services |
| Startup/reconnect | Serial reopen, firmware handshake, RX watchdog, tuning resend | MAVROS handles transport; bridge has no readiness/freshness state machine | same | Gate readiness on live FCU + required streams; invalidate observations across reconnect/incarnation |
| Device | `/dev/mowgli` | selected `/dev/mavros` | default `serial:///dev/mavros:921600` | Pin stable Pixhawk `if00`; reject ambiguity/missing device |
| Safety authority | STM32 is sole blade/e-stop authority | intended autopilot replacement, unvalidated | provisional HOLD/disarm and disabled blade/manual control | `HARDWARE_PENDING`; no claim of equivalence yet |

### Status compatibility details

Current consumers make the status gap blocking, not cosmetic. `PreFlightCheck` rejects
`firmware_compatible=false`; `fusion_graph` uses charging transitions; behavior, map, calibration,
monitoring, MQTT, and GUI consume `/hardware_bridge/status`. The migration must either generalize
the field into a documented backend-readiness contract or make preflight explicitly backend-aware.
Publishing `true` merely to bypass the guard is forbidden.

## 6. Power and battery target contract

For this installation:

```text
Pixhawk POWER1 -> dock / charger side
Pixhawk POWER2 -> robot traction battery
```

Do not infer `BATT1=traction` or `BATT2=backup`. Add explicit external-backend configuration for:

- dock/charger battery instance ID;
- traction battery instance ID;
- the charging-present/charger-enabled signal and its freshness;
- current sign convention;
- behavior when either instance is absent or stale.

The adapter must route the traction instance to `Power.v_battery`, battery percentage, and battery
failsafe inputs. The dock instance and charging signal must drive `Power.v_charge`,
`Power.charger_enabled`, `Status.is_charging`, and the docking-compatible `/battery_state.current`
convention. Loss of dock `POWER1` while undocked must not be interpreted as traction-battery loss or
trigger a battery failsafe. Do not change ArduPilot failsafe parameters in this migration.

## 7. External image integration requirements

The compose target is one service, conceptually:

```text
mowgli-mavros (external image)
  = pinned MAVROS 2.15.1 + compatibility bridge + optional bounded NTRIP client
```

Requirements:

- use the external repository's actual image name
  `ghcr.io/pepeuch/mowglimavros/mowgli-mavros-sidecar`;
- consume a Kilted multi-architecture tag and pin the validated production deployment by digest;
- do not derive this image from MowgliNext's `REPO_URL` or assume its `main/dev` tag lifecycle;
- keep `network_mode: host`, `ipc: host`, the shared `ROS_DOMAIN_ID`,
  `rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=file:///cyclonedds.xml`, and the tracked Cyclone config mount;
- map the selected, interface-specific `/dev/mavros` endpoint; broad `privileged: true` and `/dev:/dev`
  are existing behavior, not a least-privilege target;
- pass autopilot, port, baud, GCS URL, target system/component, and canonical `GNSS_NTRIP_*` values
  to the names the external launch consumes (or change the external launch to consume the canonical
  names directly);
- remove the separate `mowgli-ntrip` service;
- make the sidecar depend on no main-container package;
- keep the main ROS2 container running for RSP, twist mux, localization, Nav2, behavior, and GUI
  bridges, while conditionally suppressing only `mowgli_hardware`;
- fail configuration generation on any unsupported backend, contradictory enable alias, duplicate
  backend service, missing selected device, non-`if00` selection for this validated Pixhawk baseline,
  or MAVROS mode without a compatible public GNSS adapter;
- define readiness from FCU connection plus required fresh output streams. A running container is
  only liveness, not readiness.

`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` is compatible with the intended host-network topology:
all ROS containers share the host network namespace and loopback. Changing networking would require
revalidating discovery.

## 8. Exact expected file changes on `feat/mavros`

### Minimum MowgliNext integration patch

| File | Expected change |
|---|---|
| `ros2/src/mowgli_bringup/launch/mowgli.launch.py` | declare/validate `hardware_backend`; condition only the native bridge; retain RSP and twist mux |
| `ros2/src/mowgli_bringup/launch/full_system.launch.py` | expose and forward the backend argument so CLI and env behavior are deterministic |
| `ros2/src/mowgli_bringup/test/test_backend_selection.py` *(new, final name may follow package convention)* | static/launch contract and invalid-value coverage |
| `ros2/src/mowgli_bringup/test/test_mowgli_launch_topics_{mowgli,mavros}.launch.py` *(new or equivalent)* | prove exactly one backend owner and required public surfaces |
| `ros2/src/mowgli_bringup/CMakeLists.txt` | register focused tests |
| `ros2/src/mowgli_bringup/package.xml` | add a test dependency only if the selected harness needs one |
| `install/compose/docker-compose.mavros.yml` | one external service, canonical env mapping, Cyclone/config/device mounts, no separate NTRIP service |
| `install/lib/config.sh` | external image default independent of MowgliNext image prefix/tag; restart service list |
| `install/lib/compose.sh` | deterministic single-fragment selection and validation |
| `install/lib/env.sh` | normalize aliases; preserve canonical NTRIP and external image values |
| `install/lib/checks.sh` | expect one sidecar; check graph/type/readiness; stop expecting `mowgli-ntrip` |
| `install/lib/backend_choice.sh` | select the correct composite USB interface, not just the first Pixhawk product match |
| `install/lib/udev.sh` | include USB interface identity in the stable rule |
| `install/mowglinext.sh` | only if orchestration/order must pass a newly normalized selection value |
| `docker/.env.example` | document the external image and backend variables |
| `install/tests/test_hardware_presets.sh` | update the mowgli/mavros service matrix and reject duplicates |
| `install/tests/test_compose_validity.sh` | validate both backend topologies and removal of separate NTRIP |
| `install/tests/test_env_output.sh` | pin new image/default/alias semantics |
| `install/tests/test_udev_install.sh` | pin `if00`-specific rule and ambiguous-interface rejection |
| `install/tests/test_check_mode.sh` | pin one-sidecar readiness diagnostics |
| `docs/MAVROS_EXTERNAL_BACKEND_MIGRATION.md` | update finding statuses as work lands |
| `docs/claude/doc-index.md` | keep this checkpoint discoverable and classify it when completed/superseded |

No GUI file is required for the minimum migration. A future runtime backend switch must be designed
as an authenticated compose regeneration/recreation workflow with rollback; the historical settings
card is not sufficient.

### External MowgliMAVROS prerequisites (not changes to this branch)

- synchronize or reproducibly generate its minimal `mowgli_interfaces` from the exact MowgliNext
  interface revision and add an IDL fingerprint compatibility test;
- add the `/gps/fix` + `/gps/status` adapter with explicit GPS1/GPS2 selection and observation
  incarnation/sequence semantics;
- replace local-position-as-wheel-odom with a proven wheel-only source;
- implement configurable dock/traction battery-instance mapping;
- add disconnected/stale/freshness behavior and a backend readiness diagnostic;
- implement and test RTCM chunking to `<=720` bytes;
- keep manual control and blade control disabled until their hardware checkpoints pass.

## 9. Test migration and software acceptance

Historical tests only checked topic discovery and could pass with both backends or semantically wrong
publishers. Current installer tests already cover part of the selection matrix, but encode the broken
separate NTRIP service. Use the following disposition:

| Test area | Disposition |
|---|---|
| Historical `backend_topics_test_common.py` and two launch files | replace with expanded ownership, type, QoS, and invalid-selection tests |
| Current `test_hardware_presets.sh` | adapt; retain env/fragment assertions |
| Current `test_compose_validity.sh` | adapt; add a full MAVROS case and forbid native GPS/separate NTRIP/duplicate backend |
| Current `test_env_output.sh`, `test_state_parsing.sh` | adapt for the external image and canonical aliases |
| Current `test_udev_install.sh` | add composite-interface cases |
| Current `test_check_mode.sh` | replace container-only success with FCU/contract readiness checks |
| External RTCM tests | add 1, 179, 180, 181, 719, 720, 721, and 4096-byte cases; concatenated outputs must equal input |
| External interface test | add exact IDL/type-hash fingerprint against the pinned MowgliNext contract |
| External GNSS adapter tests | add receiver selection, new observation vs cached delivery, reconnect incarnation, RTK mapping, invalid/stale inputs |
| External power tests | add interleaved `id0`/`id1`, missing dock, missing traction, current sign, stale source, and failsafe-source cases |

Minimum software acceptance:

1. `backend=mowgli` composes and launches the native bridge only.
2. `backend=mavros` composes the one external sidecar and the main container launches no native
   bridge.
3. An invalid/empty backend and contradictory aliases fail closed.
4. The required current command/feedback/status/services exist with exact types and expected QoS.
5. Publisher/node ownership proves no duplicate backend, not merely that a topic name exists.
6. Compose selection is deterministic across fresh install, preset, rerun, and `stack.sh regen`.
7. Missing/ambiguous/non-selected Pixhawk device fails before command capability is enabled.
8. Interface fingerprint, GNSS identity/freshness, battery-instance mapping, and RTCM bounds tests pass.
9. The inert sidecar stays unable to command motion or blade by default.

ROS2 builds/tests and any formatting must be run as the normal project user, never root.

## 10. Ordered minimal implementation plan

1. **External contract prerequisite:** synchronize exact interfaces and add a fingerprint gate.
2. **External data prerequisite:** implement current GNSS identity/freshness, a wheel-only odometry
   source, battery-instance mapping, readiness/staleness, and RTCM chunking. Keep actuators disabled.
3. Publish a tested Kilted amd64+arm64 image and record its digest. The existing amd64 workstation
   image is not ARM64/RPi4 evidence.
4. In MowgliNext, add a validated backend launch argument and suppress only the native bridge when
   `mavros`; add ownership tests before changing compose.
5. Replace the compose fragment with the single pinned external sidecar; remove separate NTRIP and
   align checks/restart expectations.
6. Make Pixhawk selection/rules interface-specific (`if00`) and add deterministic installer tests.
7. Run pure Python/static launch tests and the full installer Bash suite as the non-root project
   user; then run focused ROS2 build/tests and formatting checks as that user.
8. Run a no-motion DDS/graph integration test: exact types, QoS, publishers, services, GNSS
   freshness, power mapping fixtures, FCU disconnect/reconnect, and no duplicate backend.
9. Only after software acceptance, execute the hardware gates below in order. Do not enable blade or
   manual control defaults as part of the integration patch.
10. Update this checkpoint from active to historical/retained and update authoritative codemaps/docs
    after implementation actually changes the tree.

## 11. Explicitly forbidden resurrection

Do not restore or copy:

- `sensors/mavros/**`;
- `ros2/src/mowgli_mavros_bridge/**` into MowgliNext;
- the historical local-position-to-wheel-odom relay;
- the separate `mowgli-ntrip` container or a second NTRIP implementation;
- historical u-blox, Unicore, NMEA, or direct receiver launch/config stacks;
- copied `nav2_params*.yaml`, `slam_toolbox.yaml`, localization YAML, dead package-share config seeds,
  robot_localization, slam_toolbox, Kinematic-ICP, or FusionCore;
- the old GUI backend card as a two-key settings toggle;
- old assumptions that `BATT1` is traction and `BATT2` is backup;
- a timeout as a substitute for source/observation identity;
- any code path that lets both backends consume `/cmd_vel` simultaneously;
- any software bypass of physical blade/emergency safety authority.

## 12. Remaining hardware gates

All procedures below are tied to this exact evidence baseline and must be re-stamped at execution:

```text
Pixhawk: Holybro Pixhawk 5X, USB serial 2F002F001050425937353320
Firmware: ArduRover 4.6.3, commit 3fc7011a
Validated passive host: Kilted amd64 workstation
Transport: primary composite USB if00, 921600 baud, MAVLink2, sysid 1 / compid 1
External software: MowgliMAVROS code 6b4f197, audit docs through 4bb0b2c,
                   MAVROS 2.15.1 at 22ae5b7c
Observed mode: MANUAL, disarmed
```

The powered HERE4 on CAN1 was not detected; the operator-identified F9P on GPS2 was detected without
an indoor fix. VESC 6 Pro devices historically shared CAN1. These facts do not validate a final
receiver, CAN, actuator, or mower baseline.

### HW-MAV-001 — ARM64/RPi4 USB deployment (`HARDWARE_PENDING`)

- **Procedure:** with motion and blade disabled, deploy the pinned multi-arch digest on the target
  RPi4; select the exact `if00` by-id path; observe enumeration, MAVROS connection, parameter sync,
  and repeated unplug/replug plus FCU power-cycle recovery.
- **Pass:** no boot loop or interface ambiguity; the same by-id endpoint returns; the bridge creates
  a new source incarnation, invalidates cached observations, and becomes ready only after required
  fresh streams return.
- **Safety:** disarmed, wheels unable to move, blade electrically isolated, no parameter writes or
  firmware flash.

### HW-MAV-002 — Steering/throttle and stop semantics (`HARDWARE_PENDING`)

- **Procedure:** blades removed/isolated, chassis secured with drive wheels safely clear, hard e-stop
  reachable; enable the mapping only for bounded low commands in forward/reverse and left/right,
  then stop command publication, disconnect DDS, disconnect USB, request HOLD, and request disarm.
- **Pass:** signs and magnitudes match commands without saturation/wrap; zero is neutral; every loss
  path reaches zero within the declared bound; HOLD/disarm outcomes are confirmed from FCU and
  physical outputs; reconnect never produces a stale command.
- **Safety:** two-person observation, exclusion zone, current-limited supply where possible, no
  ground motion, no blade circuit.

### HW-MAV-003 — POWER1/POWER2 semantics (`HARDWARE_PENDING`)

- **Procedure:** record exact wiring and ArduPilot battery-monitor parameters; independently connect
  and remove dock POWER1 and traction POWER2 while logging raw `BATTERY_STATUS` IDs and adapter
  outputs; repeat across dock/undock and power cycle.
- **Pass:** configured dock instance alone controls charger/dock fields; configured traction instance
  alone controls `v_battery`, SoC, and battery-failsafe input; dock disappearance while undocked does
  not look like traction failure; stale/missing traction fails closed without inventing a value.
- **Safety:** no actuator enable; use rated measurement equipment and protected power connections;
  do not change ArduPilot failsafe parameters during this audit/acceptance.

### HW-MAV-004 — Blade command, feedback, and emergency authority (`HARDWARE_PENDING`)

- **Procedure:** first validate disconnected output electrically, then a guarded no-blade motor, and
  only under a separate explicitly authorized field protocol consider a real blade. Exercise enable,
  disable, stale command, lift/stop inputs, emergency latch/reset, HOLD, disarm, process crash, and
  transport loss while correlating command acknowledgment with physical feedback.
- **Pass:** enable cannot bypass autopilot/physical safety; disable and every failsafe remove output
  within documented limits; feedback is direct and fresh; reset cannot clear an asserted physical
  trigger; the ROS response never claims physical completion before confirmation.
- **Safety:** this gate requires separate explicit authorization. Blade-on work is not authorized by
  this migration plan.

### HW-MAV-005 — GNSS/RTCM and CAN coexistence (`HARDWARE_PENDING`)

- **Procedure:** outdoors, resolve HERE4 CAN1 wiring/termination/node identity and VESC coexistence;
  establish the selected GPS1/GPS2 receiver; verify fresh per-receiver samples, RTK state, correction
  age, and bounded RTCM injection using known data.
- **Pass:** the configured selected receiver alone drives each observation sequence; no cached
  republication appears new; `<=720`-byte RTCM chunks arrive byte-perfectly; RTK/fix/accuracy mapping
  matches observed receiver/autopilot state.
- **Safety:** disarmed, no motion/blade, passive/read-only receiver audit before any reconfiguration;
  record every receiver firmware, wiring, CAN node, and ArduPilot parameter used.

Operational MAVROS backend acceptance requires all applicable gates, not just a successful container
start or MAVLink heartbeat.
