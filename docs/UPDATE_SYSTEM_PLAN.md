# Version monitoring and coordinated updates — implementation plan

Historical design snapshot: 7 September 2026. The implementation now extends
draft PR #540. See [UPDATES.md](UPDATES.md) for the implemented contract, current
limits, platform support and recovery procedures. The proposals below are design
intent, not a statement that every future extension has shipped.

## Current position

- PR #540 is open against `dev`, at `1944f97d`, with an approval of its read-only
  scope. Upstream `dev` is now `4fc91c4f`; integrate the latest upstream before
  implementation and repeat the relevant checks.
- `gui/pkg/api/versions.go` reports running image IDs/digests and selected OCI
  labels. GUI/server build identity, mainboard firmware/protocol freshness and
  desktop/mobile shortcuts already exist.
- `gui/pkg/api/updates.go` and `gui/pkg/updates/` resolve Stable/Dev image tags,
  compare digests and optionally compare source ancestry. Checks are manual;
  their one-minute cache disappears on GUI restart.
- `useUpdateChecks.ts` reads cached state on mount, rather than initiating an
  external check. Keep that separation.
- `useNotificationCenter.tsx` has a bell/drawer, but its notifications are only
  held in React memory. It needs stable server-issued IDs and persisted update
  notice state, not another mount-triggered toast.
- `install/lib/compose.sh` unconditionally includes Watchtower, and the GUI
  fragment enables its Watchtower label. Both need migration.
- `docker/stack.sh update` regenerates Compose, pulls and recreates services. It
  is not a transaction engine: it has no coordinated health verdict or rollback.
- The latest Stable release is still `v1.1.0`. Its `manifest.json` describes
  firmware assets, not a software deployment. Earlier checks found incomplete
  version-tagged container coverage; old releases cannot be assumed installable.

## Product decisions

1. Automatically check and notify; installation remains an explicit user action.
   Default check interval: four hours, with up to 15 minutes of jitter. Offer
   hourly, four-hourly, daily and manual-only settings. Honour persisted due
   times after reboot; run one overdue check, never a catch-up burst.
2. Offer **Production (Stable)**, **Development** and **Custom branch**. Production means
   published stable releases, not a moving `main` image. Development means the
   latest complete published development snapshot, not whichever builds finish
   first or the newest unbuilt source commit. Custom branch means a configured
   repository and branch publishing complete deployments in the same format.
   Support switching in either direction, subject to compatibility checks.
3. Separate the installed deployment, notification track and selected target.
   Browsing another track does not change the installed track or pin. Commit a
   requested track/pin change only after successful installation.
4. Offer **Latest in track** and **Choose version**. Every actual installation
   resolves to immutable digests, including Latest. A user pin holds a particular
   deployment; digest-based deployment alone is not a user pin.
5. Show newer releases while pinned, with a quiet “Pinned; newer version
   available” notice. Do not silently move pins. Retain the current custom/local
   installation on adoption until the operator explicitly chooses a replacement.
6. Container scope: ROS2, GUI, GPS and the installed supported LiDAR variant.
   Optional services participate only when the deployment descriptor supports
   them; list preserved/excluded services in the plan. Do not add unused hardware
   services or claim an unsupported component was coordinated.
7. Mainboard firmware stays visible and participates in compatibility checks,
   but firmware flashing, OS/Docker upgrades and unattended installation are
   separate work. The container updater must preserve LFP/custom firmware.

### Custom branches and switching

- Represent a source by repository owner/name, full branch/ref and publication
  identity. Branch names containing slashes must not be converted into tags by
  guesswork; use the publisher's explicit mapping and exact source metadata.
- The default custom source is a branch in the upstream repository. Permit a
  fork only through an explicit administrator-configured trusted source. A UI
  branch picker does not authorize arbitrary registries or descriptor URLs.
- A branch must have the publication workflow enabled and a complete build for
  the mower's components/platform. The current CI branch/path filters do not
  guarantee this for arbitrary branches. Show “No installable build published”
  when appropriate; do not compile or check out source on the mower.
- Offer Latest published build and a specific retained snapshot for that source.
  Pinning freezes the whole deployment. Preserve a branch's full name and commit
  in inventory/history even if it is later deleted or renamed.
- Preview Production ↔ Development ↔ Custom as a deployment change, including
  downgrades, divergent source and local patches replaced. Changing the selected
  source alone never installs anything. Commit its policy only after success;
  rollback restores the previous deployment and source/pin policy together.

## Complete deployments without a catalogue service

Keep the existing registry tag/digest reader. Add one small immutable
`mowgli-deployment.json` per published deployment, rather than restoring a
catalogue branch, a database of all builds or a separate catalogue service.

The descriptor contains:

- Schema version, deployment ID, track, source commit, publication sequence/time,
  source/release notes and supported platforms.
- Logical components mapped to repository + immutable image digest, with each
  platform's manifest/config digest and source/build metadata.
- Required firmware protocol/capabilities, minimum updater version, deployment
  layout version, data/config schema compatibility and rollback classification.
- Explicit optional-component coverage. Hardware selection comes from the
  installed configuration, not from whichever image tags happen to exist.

Recommended transport: GitHub release assets for Production and namespaced prerelease
snapshots for Development/custom branches, all using the same descriptor format.
Publication metadata must distinguish repository/ref; do not mix all prereleases
into one Dev feed. This gives the
version picker history without a new indexing service. Dev snapshot retention
must be documented; retain at least the last 30 published snapshots initially.
Locally installed and rollback images are retained independently of remote
retention. Keep all supported Production releases available.

Refactor existing image workflows into reusable jobs called by one publication
workflow. Build the deployment's required first-party image/platform matrix from
one exact source revision, using build caches. Publish the descriptor only after
all required jobs and checks pass. Serialize promotion so an older slow build
cannot become Latest after a newer snapshot. A rebuild receives a new immutable
deployment ID even when the source commit is unchanged. Future carry-forward of
unchanged images needs explicit verified build-input rules; do not infer it from
the current moving tags in this first implementation.

Per-image diagnostics remain useful when no descriptor exists. They do not
enable the coordinated Install button. Show missing release assets/platforms and
unsupported legacy versions clearly. Any backfill of v1.1.0 must be an explicit
maintainer publication; do not synthesize a release from unrelated current tags.

## Execution layer decision: Compose or Watchtower

Recommendation: use the Docker Compose installation already present on the host,
controlled by one small Go updater service. This adds our own executable and
systemd unit, but no new interpreter, database server or third-party updater
container. Reuse the existing registry comparison code. A systemd timer with a
one-shot worker is also feasible, but would still require durable job dispatch,
status, locking and reboot recovery; it is not necessary to reduce dependencies.

The repository actually uses `ghcr.io/nicholas-fedor/watchtower`, not the original
containrrr project. Its current documented API includes metadata-only checks,
targeted synchronous/asynchronous updates and SSE events. It is a credible option
for refreshing containers following their current tags. However:

- `/v1/update?image=...` filters containers by their **current** image reference;
  it does not assign a new image/tag/digest. `/v1/images` and `/v1/config` expose
  read-only inventory/configuration. Production/Dev/custom switching and selected
  versions would still require our code to persist a different Compose target
  and recreate containers.
- The documented history is an in-memory scan buffer, not a durable deployment
  journal. The documented API does not provide a whole-deployment rollback
  operation. We would still own maintenance gating, backups, coordinated health
  verification, previous image/data retention and reboot recovery.
- Immutable pins intentionally stop a current-reference updater following a
  moving tag. Notification checks must independently follow the selected source,
  even while pinned; our existing reader already supports the core comparison.
- Current installer configuration enables cleanup, which conflicts with retaining
  previous images for recovery. A controlled Watchtower design would need polling
  disabled, narrow scope, authenticated local-only API access and cleanup off,
  with our coordinator retaining exclusive control over installation.

Consequently Watchtower can perform part of execution, but does not remove the
coordinator required by this feature. Using it alongside Compose would create two
execution paths to maintain. Do not adopt it for the new system; migrate the
Mowgli-managed instance only when the replacement is ready. No live Watchtower
configuration changes are part of this planning decision.

## Updater outside the GUI

Add a small Go executable, initially `gui/cmd/mowgli-updater`, reusing the registry
and comparison packages. Install it as a host systemd service. It owns checking,
policy, release history cache, jobs, notification events and recovery. A host
service avoids stopping its own supervisor when the GUI or ROS2 container is
replaced. Do not run updates as a goroutine in the GUI or as an untracked shell
command launched from an HTTP request.

- Persistent state: `/var/lib/mowgli-updater/`, including policy, cached results,
  notices, exact active/previous deployments and transaction journals.
- A permission-restricted Unix socket serves the GUI backend and local recovery
  CLI. No new public management port. Do not share the GUI's open Bitcask database
  between two processes.
- The GUI exposes narrow check/plan/apply/status/rollback operations. Clients
  submit a known deployment ID and plan ID, never shell commands, arbitrary
  repository URLs, Compose documents or filesystem paths.
- Check jobs are single-flight, bounded, persisted and independent of browser
  connections. Back off on offline/rate-limit errors and honour Retry-After.
  Keep the last good result with its age; failure is never “up to date”.
- GUI reads the cached status on load and receives events or polls this local
  status while open. Neither approach causes registry checks on page load.
- Protect new mutation routes from cross-origin/CSRF requests; do not inherit
  permissive global CORS for writes. Document the existing LAN-trust model and
  require a fresh security review of the new privileged mutation surface.
- Expose the agent version. Use a versioned API compatible with current/previous
  GUI versions. A target requiring a newer agent is blocked with an installer
  upgrade instruction; avoid replacing the agent midway through its own job.

Bootstrap the service through the installer and provide a documented one-time
upgrade path for existing installations. Until installed, existing inventory and
manual comparison still work and Settings explains why Install is unavailable.

## Notifications and Settings

Reuse the bell and drawer on desktop/mobile, with a link to Settings → Updates.
Add a small badge to the existing desktop Versions shortcut and mobile More →
Versions & updates entry; avoid intrusive popups every time the app opens.

Store one current update notice per repository/ref/track/deployment identity on the mower, with
persisted acknowledgement/dismissal shared across browsers. Refresh/reconnect and
multiple tabs must not create duplicates. A different candidate creates a new
notice; successful installation or candidate withdrawal clears the old one.

Notifications distinguish an installable newer deployment, a same-source rebuild,
an incomplete publication, and a custom/diverged installation needing review.
Do not turn every digest difference into an upgrade notification. For custom
images, upstream may be older and choosing it can discard local patches.

Settings layout:

1. Installed deployment/track/pin and existing per-component/firmware inventory.
2. Automatic-check interval, notification preference, last/next check and Check now.
3. Target source (Production/Development/Custom branch), a repository/branch picker
   for configured custom sources, Latest/Choose version, date/source/release notes, compatibility
   and availability. Dates are context; identity and source ancestry remain explicit.
4. Review changes: old → new images, preserved services, custom builds replaced,
   required downloads, expected interruption and rollback availability.
5. One Install action, durable per-phase progress and post-update verification.
6. History with previous successful deployments and Restore previous version.

Mobile uses stacked component cards and a full-width review/progress panel. During
GUI replacement, the page says it is reconnecting and retains the transaction ID;
when the GUI returns it resumes the same job, rather than reporting network loss
as an installation failure or submitting the installation again.

## Coordinated transaction

Compose cannot make several container replacements atomic. Provide staged updates
with a single success decision and compensating rollback, not a claim of atomic
container switching.

`planned → downloading → ready → quiescing → backing_up → applying → verifying
→ succeeded` or `rolling_back → rolled_back / recovery_required`.

1. **Plan:** select the complete descriptor for this host's hardware/platform;
   resolve and validate its immutable images. Validate agent/layout/schema and
   firmware compatibility. Missing metadata blocks coordinated installation.
   Fingerprint the current deployment/config and make the plan expire. Recheck
   on Apply; don't silently install a different Latest or overwrite intervening
   edits. Show track switches, downgrades and custom-patch replacement explicitly.
2. **Download:** pull all required target images before interruption. Verify
   platform, digest and metadata, disk space and rollback-image availability.
   No build on the mower. Failed downloads leave running containers untouched.
   Retain locally built rollback images by image ID/export when no registry
   reference exists; fail planning if recovery cannot be secured.
3. **Quiesce:** acquire an exclusive deployment lock and require fresh idle,
   stationary and blade-off evidence. Require a suitable docked/maintenance state,
   without treating >0.3 A charging current as the contact prerequisite. Do not
   auto-dock or stop an active mowing job as a hidden part of Install. Inhibit
   scheduled/manual autonomous starts across the GUI and ROS2 startup path while
   a transaction or recovery is pending; keep emergency-stop behaviour available.
   This guard must survive GUI replacement and host reboot.
4. **Back up:** after stopping relevant writers, capture consistent config,
   GUI database and supported persistent data, active image references and Compose
   inputs. Preserve project name, volume names, mounts, device selections and
   sparse robot configuration. Image-only rollback is not a data rollback.
5. **Apply:** use an installer-owned image override referencing the resolved
   digests and the existing deployment layout. Integrate it with every installer/
   `mowgli-*`/stack entry point so regeneration cannot undo pins or reintroduce
   Watchtower. Validate the rendered Compose model; block unknown custom layouts
   instead of replacing them. Quiesce GUI writes/scheduler before stack changes,
   stop affected consumers, then start sensor dependencies, ROS2 and GUI in
   dependency order. Keep unaffected optional services intact.
6. **Verify:** require expected image identities, stable container/process health,
   GUI API health and server/browser revision, fresh ROS/firmware communication
   with compatible protocol, and expected configured sensor publishers. No-GPS/
   no-RTK indoors is not itself a software failure. Compare with pre-update
   degradations and use bounded hardware-specific startup grace periods.
   Docker “running” alone is insufficient; the host agent must independently
   verify/recover even if the new GUI cannot start.
7. **Commit:** only after verification publish the new active deployment and
   requested track/pin, persist success and release the maintenance gate. Return
   idle; do not automatically resume mowing. Retain at least the previous two
   successful deployments and their required recovery artifacts, subject to
   explicit storage limits. Never prune an active/pending rollback image.
8. **Recover:** failed verification restores the entire affected set, its image
   override and compatible data/config snapshot, then verifies the old deployment.
   Journaling/fsync and observed Docker state permit recovery after GUI failure,
   daemon restart or power loss at each phase. A failed rollback stays inhibited
   and exposes a local CLI/log path. No endless install/rollback retry loop.

Restrict v1 installations to declared compatible layouts and data schemas, or a
tested reversible migration. Block unsupported downgrades rather than promising
that all old binaries can read newer maps/config/databases. Later layout/OS
migrations need separate support. Manual rollback uses the same planning and
maintenance rules, restores the prior track/pin and suppresses repeated prompts
for the failed target until a new target appears or the operator retries.

## Watchtower migration

- Remove automatic inclusion and GUI update labels from installer sources.
- Adopt the current actual deployment as a baseline; do not retag or upgrade it
  during setup. Register the current Compose project and selected services.
- Retire only the Mowgli-managed Watchtower instance during migration, after
  verifying its scope. If another updater controls these same containers, block
  mutation and report the conflict; do not stop unrelated host services.
- Installer regeneration and legacy CLI update paths must honour the deployment
  lock/override. Reconcile and report out-of-band Docker changes rather than
  silently resetting them.

## Implementation order within PR #540

| Order | Reviewable commit group | Completion evidence |
| --- | --- | --- |
| 1 | Integrate latest dev; define policy, deployment descriptor, agent API and journal contracts | Existing read-only checks remain working; schema/compatibility fixtures |
| 2 | Host agent/bootstrap, persistent scheduled checks and read-only GUI adapter | Checks with browser closed; no extra remote checks on UI load; reboot/offline/rate-limit tests |
| 3 | Persistent UI notices and notification/settings controls | Deduplication, dismissal, pins and desktop/mobile navigation tests |
| 4 | Complete deployment publication, version history, Production/Dev/custom source switching, pins and dry-run plan | Incomplete CI never becomes installable; historical snapshots resolve exact image sets; custom branch/fork mapping explicit |
| 5 | Maintenance gate, staged Compose execution, consistent backup, durable rollback and recovery CLI | Fault-injection tests across every phase; GUI replacement/power-loss recovery; no motion during transaction |
| 6 | Install/progress/history/restore UI, Watchtower migration and end-to-end validation | Disposable-stack tests, controlled Pi validation and refreshed screenshots |

Keep these as separate focused commits in the existing PR, rewrite its title and
description around the expanded scope, then request re-review. The existing
approval explicitly assessed a read-only feature and does not cover container
replacement. Mark the update/maintenance path as safety-critical in the PR.

## Required validation before field activation

- Existing Go, TypeScript, lint, translation and focused browser suites.
- Deterministic scheduler tests: persisted due times, jitter, single-flight,
  restart, offline errors, rate limiting and no UI-triggered external checks.
- Registry/publication tests: incomplete matrices, wrong platform/protocol,
  malformed descriptors, same-source rebuilds and obsolete CI promotion; custom
  branch slash/name collisions, missing publications, deleted branches, trusted
  fork boundaries and Production/Dev/custom switches in both directions.
- Transaction tests with isolated Docker projects and volumes: pull failures,
  stale plan, disk exhaustion, duplicate Apply, unhealthy sensor/ROS2/GUI,
  interrupted GUI connection, agent restart, host-reboot recovery, failed
  rollback, backup consistency and custom local-image restoration.
- Maintenance tests: scheduled starts/teleop cannot race activation, stale
  telemetry blocks activation, safety controls remain intact, LFP low-current
  dock state is not rejected solely for being below 0.3 A.
- Legacy-installer migration tests: preserve hardware/config/project/volumes;
  no Watchtower reintroduction or unrelated orphan removal.
- Hardware validation starts on one explicitly selected docked test mower with
  all local patches recorded, then verifies the other supported profile. Do not
  overwrite either mower's custom ROS2/GUI build merely to demonstrate updates.
- Refresh PR screenshots for notices, target review, active update/reconnect,
  successful installation, pinned version and rollback on desktop/mobile.

References: [PR #540](https://github.com/mowglinext/mowglinext/pull/540),
[Compose digest overrides](https://docs.docker.com/reference/cli/docker/compose/config/),
[Compose up](https://docs.docker.com/reference/cli/docker/compose/up/),
[Compose readiness](https://docs.docker.com/compose/how-tos/startup-order/),
[Watchtower fork update API](https://github.com/nicholas-fedor/watchtower/blob/main/docs/http-api/endpoints/update/index.md),
[Watchtower fork check API](https://github.com/nicholas-fedor/watchtower/blob/main/docs/http-api/endpoints/check/index.md),
[Watchtower fork inventory API](https://github.com/nicholas-fedor/watchtower/blob/main/docs/http-api/endpoints/images/index.md),
[Watchtower fork configuration API](https://github.com/nicholas-fedor/watchtower/blob/main/docs/http-api/endpoints/config/index.md),
[Watchtower fork history](https://github.com/nicholas-fedor/watchtower/blob/main/docs/http-api/endpoints/history/index.md).
