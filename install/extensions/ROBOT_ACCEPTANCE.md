# Extension Manager: isolated robot-host acceptance

Test date: 2026-11-10. The ARM64 CLI ran on `automower`, over the existing
`mower@automower.local` SSH connection, as normal user `mower` (uid 1000).
During these acceptance runs, no issues, PRs or pushes were created.

## Later GUI work: separate evidence

The GUI/host API and fresh-inventory optimization were added after these exact
Engine baselines. They were validated locally with unit/race tests, a proxy
transport harness and mocked Chromium desktop/mobile flows. They were not
deployed to the robot, and the Engine results below do not validate that later
HTTP/GUI integration. The recorded robot cleanup remains complete.

## Result and scope

**Passed:** actual Docker Engine adoption, upgrade, explicit rollback, recovery
after an interrupted upgrade, automatic rollback on failed application readiness,
disable/remove, consecutive disabled upgrades/rollback and reinstall with retained
bind data and named-volume contents. The lifecycle service ran `/bin/sleep`;
the named-volume fixture additionally ran isolated `/bin/sh` and `/bin/cat` helpers.
Every pre-existing container's ID **and start time**, plus
the modem service's systemd invocation, stayed unchanged across the checks.

The existing LoRa modem was separately registered for observation, queried and
forgotten without restarting it. Its legacy health response has no producer
invocation field, so the CLI correctly reported application/data `unknown` while
reporting the process `running`.

This is host/software acceptance on the robot. It does not validate Connect's
remote-access functionality, GNSS correction delivery, RF performance, navigation,
motion, blades, or field safety. No production integration was taken over.

## Exact baseline

| Item | Observed value |
|---|---|
| Robot checkout | `f1d3f4af9f8db4ba371b9b1f13219e07e724be34` |
| Development checkout base | `b5e46ed4917c8f11cd71a2b6bc4b63cec19ffd98`, with uncommitted extension implementation |
| Initial Engine suites source fingerprint | SHA-256 `3fc075defdcbc4e41050f514a0727aedce2035822c2c06b9a0fc1249789cd27b` |
| Engine-test ARM64 binary | SHA-256 `3edc6435705b761fbe47140feff56517f37292063541f9dffd34f6f64efefacb` |
| Earlier LoRa-observation binary | SHA-256 `45afcca133957945c9758c87ff886d2a718430169a26514a1855c9eed98f24c8` |
| Architecture | `aarch64`; image inspection `linux/arm64` |
| Docker Engine | `29.7.2` |
| Docker Compose | `v5.5.0` |
| GUI readiness | `ready=true`, `maintenance=false`, firmware protocol `7` before/after the lifecycle checks |
| LoRa package | `0.2.0`, existing `/opt/mowgli-lora-link/.venv` installation |
| LoRa unit | `mowgli-lora-robot.service` |
| LoRa invocation | `b4ffc850fbed4704a6f516399a59385a` |

The source fingerprint hashes the sorted paths and contents of
`gui/pkg/extensions/*.go`, `gui/cmd/mowgli-extensions/*.go`, `gui/go.mod` and
`gui/go.sum`, with NUL separators between path/content/entries. Tests are included.
The binary hash is the definitive executable identity for each run. The final
portable run below used the subsequent healthcheck-serialization fix.

The robot is a mixed local deployment, **not** the development checkout's release:

| Running component | Image ID |
|---|---|
| GUI | `sha256:1c20a2862220f7b1490035017280ac0d349b396872331cd5e355426495b17575` |
| ROS2 | `sha256:7ce4c5fbdd2279b79395e624084194c436f46d31c7f243991e7ac81fdf6fdfcb` |
| GNSS | `sha256:ca91aa2047c73a3db175e2518a618f09799ef130f6d53b42f1a8bba9e0cd16dd` |
| RTCM compatibility sidecar | `sha256:122979c78c9e1438c25edb16cab275b635e0ad856ee00608fd85f9baae317bb2` |
| Existing Connect | `sha256:1d4eb6b10bb5dcb7dc9e54881d3c07476195fabc701d38b6e7cf0999d250ea07` |

Firmware protocol 7 is live interface evidence; the mainboard image hash and
GNSS receiver/firmware identities were not acquired. There is no physical
measurement to generalize beyond this exact software/host baseline.

## Private probe

- Project: `mowgli-ext-lab-no5z1f`; extension ID `extension-probe`.
- Image: `ghcr.io/liltaket/mowgli-connect-agent@sha256:1d4eb6b10bb5dcb7dc9e54881d3c07476195fabc701d38b6e7cf0999d250ea07`.
- OCI source revision: `ce52b8cfe36adf1ce079e448c6c91703df2b2162`.
- Only `/bin/sleep` ran: the Connect entrypoint/agent/relay were not executed.
- UID 65532, read-only rootfs, all capabilities dropped, no-new-privileges,
  internal private bridge, no ports/devices/socket, own bind directory at `/data`.
- A separate private temporary state/config root was used. No real Connect
  identity, robot configuration, maps volume, GNSS device or modem config was mounted.
- The GUI's existing readiness API was checked before each requested mutation;
  no new ROS participant/probe or maintenance marker was introduced.

Lifecycle sequence, 19:22:00–19:23:36 UTC:

1. Manually start the private probe, then adopt its exact existing identity.
2. Upgrade fixture version `1.0.0` → `1.0.1`; verify a new process identity.
3. Explicit rollback to `1.0.0`.
4. Run an intentionally invalid target with a short operation deadline. The
   interrupted operation left visible recovery-required state. This proves
   interrupted transaction handling; it does not establish which Docker substep
   reached its deadline or that the invalid command itself executed.
5. A new CLI invocation runs `recover`; the prior running version is restored.
6. Disable/remove; reinstall the retained layout; enable; disable/remove again.
7. Assert the private data marker survives every transition.

Automatic recovery sequence, 19:25:11–19:25:54 UTC:

1. Adopt a new instance of the same private probe.
2. Target fixture `1.0.3` requires application health although the image has no
   configured application probe. Activation is rejected.
3. The manager automatically restores running `1.0.0`, preserves the original
   failure reason and clears recovery-required state.
4. Disable/remove and verify retained data and unrelated process identities again.

Both fixture version numbers describe test bundles, not upstream Connect releases.

The saved [portable test runner](tests/docker_lifecycle.py) then repeated adoption,
upgrade, rollback, automatic recovery, teardown and retained-data reinstall with
a fresh random project. Its final run passed all 14 reported phases:

| Earlier portable run, before the independent Sol review | Value |
|---|---|
| Result timestamp | `2026-11-10T19:47:39.262572Z` |
| Project | `extension-lifecycle-7b554454c908` |
| ARM64 binary SHA-256 | `a8a0edbb6cb576dd4c1561cbf8dfa1d683491bf450bd93303994ff2f05ffb5d5` |
| Go source fingerprint | `c15135a7e9a0199c7ec7c90226bad442f10a82d5a73f82413612cf7754915343` |
| Runner SHA-256 | `84a5e87e984bdd6ead400b02dd1f87d100f3165d613668f34cb99bbc0549a796` |
| Data marker / unrelated identities / cleanup | All verified; temporary files removed |

At the independent host check, `2026-11-10T19:50:32Z`, all seven original
container IDs/start times and the original modem invocation still matched the
first baseline. No resources remained for any test project, and readiness was
still true with maintenance false and protocol 7.

## Findings fixed from the real run

The first attempt stopped during inventory and its private service/network were
cleaned up. Real Engine bind mounts omit `Name`. The limited inspect template
used `$m.Name`, which fails under Docker's `missingkey=error` behavior. It now
uses `index` for that optional key. The regression fake removes bind `Name` and
uses the same strict template option; the focused test failed before the fix
and passed afterward.

The portable runner also exercised an explicitly disabled healthcheck. The old
serializer emitted `test: null`, which the real Compose CLI rejects. Omitting an
unset test field preserves the valid `{disable: true}` form. A new regression
checks the generated artifact and validates it with the installed Compose CLI
without starting containers; it failed before the fix and passed afterward.

Before the probe, cross-project inventory was narrowed to four ownership labels,
container/image identity, mount identities and network names. Full configuration
inspection is only requested for IDs in the exact bound project and is kept in
local memory, not exported or persisted. Docker command errors now identify the
fixed operation without exposing stderr, arguments or configuration values.

The earlier package/CLI race run passed **64 top-level Go tests** including
the new metadata, bind-mount and disabled-healthcheck cases. The broader GUI suite,
Connect and LoRa results remain recorded in [README.md](README.md).

## Independent-review regression acceptance

GPT-6.1 Sol found that disabled upgrades advanced the register while retaining
old stopped containers. Two upgrades could discard authorization for the actual
old release. The manager now verifies stopped state and journals `down` before
committing the new disabled identity; configuration, bind data and external
volumes are retained. The same review found partial external services reported
as stopped, skipping preflight; they now report `partial`, while transitional
systemd states report `unknown`.

Two fresh robot-host Engine runs passed after the manager fix:

| Execution | Bind | External named volume |
|---|---|---|
| Project | `extension-lifecycle-433b7ac7774e` | `extension-lifecycle-864a1aa21a5a` |
| Result timestamp (UTC) | `2026-11-10T20:09:49Z` | `2026-11-10T20:14:34Z` |
| Reported successful phases | 19 | 20 |
| ARM64 binary SHA-256 | `dfc1d1fb026940e11a8fdc723c30968a7de682d9b23dcb88908b8fd1d0c8f2fa` | Same |
| Go source fingerprint | `29f101a4659f3f02189cd16c7c13986da14ad8ea6dc22a8409238ac2b4dbe2c7` | Same |
| Runner SHA-256 | `9b5fdd6430d360663260a4f3c7b57255533e824bb550e5af5548331d70a84cff` | `e74b31a0762d4f8baba3722f6cec403717da47d360dc956656c4d320c31f4314` |
| Data / original IDs and start times / cleanup | Verified | Verified, including external-volume contents |

Each run repeated exact adoption, enabled upgrade/rollback and automatic recovery,
then two consecutive disabled upgrades, disabled rollback, enable, disable/remove,
reinstall, enable and final teardown. The named volume was created with a unique
test ownership label, mounted only in isolated test containers, verified after
each phase and explicitly deleted by the test runner only after successful
checks. The manager never deleted a volume. No real Connect identity was mounted.

The first named-volume attempt failed in fixture setup: its unprivileged root
helper could not write the image's UID-65532-owned directory. The runner now uses
UID 65532. That attempt's exact ownership-labelled volume and temporary files
were removed after verifying the original robot baseline; no service was started
for the failed fixture. Failure reports now distinguish unverified invariants
from actual failed checks.

The subsequent external-state refinement was tested with **69 top-level Go
package/CLI tests under the race detector**. The final binary SHA-256
`3f393358a2e725207fd279838f14dfee480622162290331dfa08b52aeaba073d`, source fingerprint
`241afd93db8480efa9dc4eb87207ac94353e15f8249d9a6b8179be2204b34e51`, then ran
read-only LoRa adopt/status/forget in a fresh temporary register. It reported
process `running`, application/data `unknown` for the legacy endpoint and left
the original modem invocation unchanged. This last executable did not repeat the
Compose suite; its manager/Compose code is identical to the two runs above.

At `2026-11-10T20:16:28Z`, the final baseline check matched all seven original
containers and start times, matched the original modem invocation, found no test
containers/networks/volumes and confirmed ready=true, maintenance=false, protocol 7.
All known test binaries/scripts were hash-checked and removed from the robot's
`/tmp`; temporary registers and directories were removed too. No movement,
blade or robot-mode command was sent.

## Cleanup and retained evidence

Both lifecycle scripts reported PASS. At completion the temporary register had
no entries and no journal; only the deliberately retained test-data ownership
record remained. Directory/file modes were 0700/0600. Docker listed no containers
or networks for the private test project. The robot still reported `ready=true`,
`maintenance=false`, protocol 7.

Logs are temporary evidence outside Git:

- `/tmp/mowgli-extension-robot-lifecycle-result.log`
- `/tmp/mowgli-extension-robot-auto-recovery-result.log`
- `/tmp/mowgli-extension-acceptance-portable-result.json`
- `/tmp/mowgli-extension-sol-fixes-bind-result.log`
- `/tmp/mowgli-extension-sol-fixes-volume-retry-result.json`
- `/tmp/mowgli-extension-final-preservation-result.log`

The temporary binary/test directory on the robot is removed after evidence
collection. No updater, background process, cron entry or persistent service is
installed by this acceptance test.

## Remaining acceptance

Actual Connect adoption/upgrade is still outstanding: the running image's source
revision differs from the analyzed repository head, its update ownership and
identity volume must be reviewed before any takeover. The LoRa source changes
were not deployed; the current unit was observed only. The core updater's
automatic extension compatibility integration also remains a separate change.

The GNSS/RF question remains `HARDWARE_REQUIRED`. Before that separate run, freeze
the MowgliNext/LoRa commits, GNSS and adapter image/platform digests, both modem
firmware hashes, robot/receiver units and receiver firmware. The currently known
software baseline is above; modem firmware and receiver identities were not
acquired, and the corrected ROS adapter was not deployed.

Use a stationary bench with drive/blade power safely isolated, following the
[LoRa integration contract](https://github.com/liltaket/mowgli-lora-link/blob/f1c5fd7ebea9469a218d580a0fdb8dafda672bba/docs/MOWGLI_INTEGRATION.md)
and its RF limits, or a wired/mock transport. Feed known CRC-valid RTCM frames,
observe modem session, exact GNSS ingress and receiver correction progress, then
disconnect/reconnect the source. Pass requires fresh frame progress after reconnect,
visible interruption distinct from process life and no stale/replayed progress;
no motion/STOP/blade function is part of that procedure. Store physical evidence
in LoRa's repository. Engine acceptance does not replace these checks.
