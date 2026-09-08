# Software updates and recovery

Settings → Updates reports installed software, mainboard firmware and web build
identities. On a managed installation it also checks for complete deployments and
installs a reviewed selection. Updates are never installed automatically.

The desktop rail footer shows the verified running version and track; mobile
shows the same summary in More. Clicking it opens Updates. Custom combinations
and unverified installations are labelled explicitly instead of claiming the
base release or available update is running. This reads local cached status only.

## Simple and Advanced views

The screen opens in **Simple**. It shows the installed version and pin, the source
being checked and the available release once. **Check for updates** and
**Review update** sit directly below, before the list of running components.
Simple rows do not repeat a target release number beside each container. Firmware health, errors, active update/recovery
status and recovery actions remain visible. The updater service offers its own
update action only when a different published binary is available.

**Advanced** adds source, repository, branch, check frequency, retained-version
selection, compatible per-service overrides and pin controls. Advanced separates **Published releases** from **Custom images**. Installed container identities, web build details,
manual per-image comparisons and deployment history also live here. The view
switch changes presentation only: it does not check remotely, install anything
or change policy. Unsaved source edits must be saved or reset before review.
Simple always reviews the latest published deployment and preserves an existing
pin; switching back from Advanced cannot install a hidden older selection.

The confirmation names the version, repository/branch and affected components.
It explains downtime, backups and firmware scope. Exact before/after image
identities are under **Container image details**. This keeps the ordinary path
short while preserving the information needed to assess custom builds or
troubleshoot recovery. The same control and order are used on desktop and mobile;
Advanced fields stack into one column on narrow screens.

### Selecting another branch or fork

1. Open **Advanced** and choose **Production**, **Development** or **Custom branch**.
2. Choose an enabled repository. For Custom branch, type the full name, for
   example `feat/settings-updates`; slashes are preserved. This is a branch-name
   field, not a list of every GitHub branch.
3. Click **Save and check**. Choose Latest or a retained deployment, then Review
   installation. Selecting a source alone never replaces containers.

An administrator enables a fork by adding it to the existing host config's list
(preserve the other settings), for example:

```json
"trusted_repositories": [
  "mowglinext/mowglinext",
  "wjcloudy/mowglinext"
]
```

The file is `/etc/mowgli-updater.json`. Restart `mowgli-updater.service` while no
update/recovery is in progress to load the new list. The repository picker is
populated from that list; the browser cannot add arbitrary repositories.

Each selected source must publish the same complete deployment format with
readable release assets and GHCR images for the host architecture. A forked branch
with code but no complete published deployment shows **No installable build**.
Production/dev tracks can also be selected within a trusted fork. The upstream
source remains available for switching back. The current PR branch has not yet
completed an end-to-end deployment publication/field trial; ordinary branch image
builds by themselves do not make it installable through this screen.

## Supported installations

The host updater is a static Go executable for **Linux ARM64 or AMD64**, supervised
by systemd. It uses the installed Docker Engine and Compose v2 CLI, together with
standard host `tar`, `du` and `mv` tools. No Go toolchain, Watchtower, database
service or container orchestrator is needed on the mower.

Raspberry Pi OS 64-bit, Ubuntu and Debian retain their existing architecture
support. Automatic installation requires the standard installer Compose layout,
host networking and the Mowgli mainboard readiness interface. ARM32, Windows,
macOS, rootless Docker, Podman and non-systemd installations are not supported by
this updater. Version reporting and manual image comparison remain available
when the host service is absent. MAVROS/other hardware backends without Mowgli
firmware readiness cannot pass the automatic installation gate.

## First installation or adoption

Run the normal installer from the selected checkout. It downloads the matching
`updater-<full-commit>` release binary and verifies its SHA-256 checksum, installs
`/usr/local/bin/mowgli-updater`, `/etc/mowgli-updater.json` and the systemd unit,
then adds the socket and maintenance mounts to the generated Compose stack.
An unpublished checkout keeps manual operation and prints a warning. Developers
can supply a locally built, trusted binary with `MOWGLI_UPDATER_BINARY`.

The updater only replaces containers after **both installed GUI and ROS2 images
support maintenance API 1**. Older installations need the normal installer image
upgrade first. The initial custom/LFP installation is not silently adopted as an
upstream deployment. Existing image IDs are retained for recovery when the first
reviewed update runs.

The installer stops and removes only the named Watchtower container belonging to
this Compose project. Managed GUI images opt out of Watchtower. The updater
conservatively refuses installation while any Watchtower container is running;
an administrator must stop it before proceeding. Existing trusted repositories
and updater state survive an installer rerun. A different checkout/project/path
requires an explicit configuration migration.

## Tracks, versions and notifications

- **Production** lists stable releases carrying a complete deployment descriptor.
- **Development** lists complete snapshots published from `dev`.
- **Custom branch** uses the full branch name in a trusted repository. Forks must
  first be added by an administrator to `trusted_repositories` in the host config.

Save settings changes the notification source, not the running deployment.
Choose the latest published snapshot or a specific retained version, optionally
pin it, then review and install. The installed source and pin change only after
successful activation. A pin does not suppress notifications or explicit changes.
Production/dev/custom switching uses the same reviewed installation path.

Checks run every 24 hours by default, with up to 15 minutes of persisted jitter.
Hourly, four-hourly and manual-only checks are also available. Existing saved
intervals are preserved on upgrade. A failed check retains the
previous successful results and retries after 15 minutes plus jitter. UI status
polls read local state; opening a page does not start a remote check. Notification
read/dismiss state survives GUI and host restarts. Different source histories are
shown for review; a date alone never proves that source code is newer.

## What an installation does

1. Resolve a complete compatible deployment to immutable platform image digests.
   The review expires after 15 minutes and includes current and target images.
2. Take the deployment lock, verify the installation has not changed, and pull
   all required images before stopping anything. At least 2 GiB must be free in
   the updater state filesystem; Docker can require additional image storage.
3. Require fresh live firmware/status/odometry, idle behavior, stationary wheels
   and a stopped blade. Write the persistent maintenance marker. ROS2 rejects
   starts and inhibits outgoing wheel/blade commands while it exists, including
   after reboot. This does not replace firmware safety or emergency stop.
4. Retain old image IDs, stop all managed writers and archive the GUI database,
   configuration and maps, syncing and checksumming each archive. Require space for backups and failed-deployment data.
5. Activate the reviewed Compose definition, retire owned services without deleting
   volumes, and replace managed containers in their declared dependency order (sensors before
   ROS2, then GUI by default). Verify actual
   image IDs, container state, fresh application readiness, firmware protocol
   and installed sensor publishers before committing. Charging current and RTK
   fix quality are not readiness requirements.
6. On failure, restore previous configuration/data and images. Retain failed data
   for diagnosis. Release maintenance only after verified success or recovery.

ROS2, GUI, GPS and the installed supported LiDAR variant participate by default.
Additional installed first-party services opt in through Compose labels (below).
Unmanaged services, including MQTT by default, remain outside this transaction. Firmware, host OS and Docker
upgrades are excluded; custom/LFP firmware is not flashed. Targets requiring a
different firmware protocol, updater API, layout or data schema are rejected.
Older releases without a deployment descriptor are comparison-only.

## Installed stack, health and component versions

The updater samples **local Docker state every 15 seconds**, independently of
remote update checks. Page loads read that cache. Samples older than one minute,
failed inspections and active transactions report unknown status. Container
health means running and, when Docker defines a healthcheck, healthy; it does
not claim field readiness, positioning accuracy or that the mower is idle.
The stricter application/sensor readiness gates still control installation.

A successful transaction records every managed container's actual image ID.
The current label is a matched release only while those IDs and service membership
still match. A deliberately selected component override shows **Custom combination**;
manually changed images or managed membership show **Installation changed**.
The last recorded base and component versions remain visible as reference. Older
journals lacking recorded IDs show unverified until a coordinated installation.
A custom installation is never inferred to be an upstream release from tags alone.

Simple shows one installed stack and one Check for updates / Review update flow.
Advanced adds a **Release version** selector, pin, and a version selector beside
**every managed service**: ROS2, GUI, GPS, the selected LiDAR driver and future
first-party helpers. Each follows the selected release by default. **Reset all to
release versions** clears the draft exceptions. Settings for source, repository,
branch and frequency are collapsed under **Update settings**; history and diagnostics
are separate. Mobile stacks each service's controls beneath its running version.
Advanced shows **Built** from the installed container image metadata and
**Published** from the deployment descriptor, including date, time and timezone.
Release, component and updater choices include publication dates; the full selected
date remains visible below component selectors on mobile. Missing or invalid image
build timestamps show Unknown, never the release publication date as a substitute.
Unmanaged MQTT has a local badge, and the host updater has a separate reviewed action.

The installed base remains selectable when it has aged out of the release list and
belongs to the selected source. Individual choices come from complete published
releases in that same repository/track/branch. A branch is entered in Advanced →
Update settings → Custom branch using its full upstream name, for example
`feat/gui-dashboard-improvements`. Saving selects the checking source; choose a
published release and review separately to install. For a single-container change,
keep the installed base release selected and choose a compatible build beside
that container. The other image versions remain on the base (the coordinated
backup/restart still covers the stack). Keeping the base on `dev` while selecting
only GUI from `feat/gui-dashboard-improvements` is not currently supported.
Branch names and matching contracts alone do not bypass that source restriction. Both releases must have the same
nonempty `component_compatibility[image-family]`, layout, data schema, updater API,
maintenance API and firmware protocol. The chosen image must support the host's
platform. Missing/incompatible contracts disable mixing, not whole releases.
Legacy GUI assets retain `gui_compatibility` fallback only when neither asset has
a GUI component contract. This published-release selector never accepts raw image references. Cross-source images use the separate Custom images path described below. This is image selection; a GPS/LiDAR driver change still follows the
installer options and selected release bundle.

### Advanced custom images

**Custom images** starts from the installed stack, even when no release descriptor
is available. Each installed managed container has **Keep current image** or
**Choose custom image**. Enter an explicit Docker tag for its current repository,
a complete `registry/repository:tag`, or `registry/repository@sha256:digest`.
`https://` and `docker://` prefixes on registry image paths are accepted;
web pages, embedded credentials, query strings and insecure HTTP URLs are not.
Use a branch's **published image tag**, not a GitHub branch page. Branch images
may come from another repository or registry, but must match an accessible published MowgliNext deployment descriptor. Unpublished branch builds and legacy rolling images are rejected.
Registry authentication uses the updater service account's Docker configuration;
there is no browser credential form and no extra registry client dependency.

The user acknowledges image trust and unverified compatibility, then chooses
**Download and review images**. Docker downloads for the host platform without
starting candidate containers. The returned digest is inspected instead of the
mutable tag. OCI source/revision/build time, component family, image contract 1,
release and deployment identifiers are mandatory. The published, non-draft GitHub
release descriptor must match the image bytes, platform, family, version, source
revision and firmware protocol. Production uses `vMAJOR.MINOR.PATCH`; dev/custom
uses `deployment-<12-char-sha>-<run>-<attempt>`. GUI also requires updater UI 1,
so an old GUI cannot remove the update interface. Missing or forged-looking
metadata, a tag alone, or an unrelated image named like a release is insufficient.
The review pins that immutable reference and shows requested names,
build dates and kept/changed images. A second explicit acknowledgement enables
installation. A tag does not become an automatic tracking rule. Scheduled checks
continue checking published deployments from the configured source.

Custom mode changes images only: installed Compose, hardware selection, services,
mounts and non-selected image IDs are retained. The whole managed stack still
stops for consistent backup and restarts/verifies together. GUI/ROS candidates and
installed rollback images must declare maintenance API 1. Wrong architectures,
untracked image VOLUME storage, pending installer changes, unmanaged/absent services,
and a changed review baseline are rejected. Existing fresh readiness, mainboard
protocol verification, backup, rollback and maintenance recovery remain mandatory.
The complete-deployment publisher supplies the metadata and validates it on both
architectures before writing an `image_contract: 1` descriptor. Older component-only
workflows do not qualify simply because they publish a `dev` or semantic-version
tag. Use a qualifying complete-deployment image/tag or digest; its version label is
the production release number or immutable snapshot number. The first release
containing this publisher and GUI marker establishes the minimum support boundary,
without guessing an unreleased version number.

No cross-release compatibility contract is invented: image authors must be
trusted, and checks cannot guarantee API/data-format compatibility or safe behavior.
Custom images have the container's existing access to hardware, Docker and saved
state; this is stated before download and again before installation.

A verified complete release is **Standard deployment**, including a complete dev
or custom-branch release. Published per-component overrides or explicit custom
image selections are **Custom mix**, with base release (if known) and individual image
provenance. Manual drift and unknown state remain distinct. Even manually choosing
the same digest stays a custom selection until a complete published deployment is
reviewed/installed. Simple cannot inherit custom drafts. Installing a published
release clears custom overrides; successive custom transactions and rollback retain
and restore the exact previous mix, including installations without a base release.
The host updater and unmanaged services such as MQTT use their separate paths.

The publisher derives `service_choices` from that bundle. These provide the UI's
service/image-family/installer-option projection, including newly introduced services.
Planning compares it with the verified bundle and rejects disagreement. Only managed
services in the selected target can receive overrides; removed, disabled and local
services reject them. Older assets without the projection use installed membership
for the UI; the server remains authoritative.

Other containers follow the selected base. Choose the installed base to retain their
versions. All managed writers still stop for a consistent backup, then the stack
restarts and verifies together. The confirmation names every exception and retains
exact image identities. A deployment pin applies to the whole selected combination;
checks continue to notify. `POST /v1/plan` accepts `component_deployments`, a map from
Compose service to published release ID. The legacy `gui_deployment` alias remains
accepted; specifying GUI in both fields is rejected. Capability
`service-version-overrides` advertises general selection to clients.

Simple mode always reviews the latest **matched** deployment and never carries a
hidden Advanced override. **Review update** in Simple (or **Review matched release** in Advanced) clears exceptions after
successful installation. History and rollback track transaction IDs, base release,
overrides, image IDs and policy, so two installations sharing the same base release
can be restored independently. Firmware is not part of a component override.

## Adding a managed or optional container

Each production, dev or custom deployment now carries `mowgli-compose.json`, a
self-contained bundle of the versioned installer Compose fragments. Its SHA-256
is pinned by deployment schema 2 in `mowgli-deployment.json`. The shared
`install/compose/stack.json` map defines required fragments and hardware options:

| Saved installer choice | Selected fragment |
|---|---|
| GNSS disabled | No GPS service |
| Universal GNSS | `docker-compose.gps.yml` |
| LiDAR disabled | No LiDAR service |
| LiDAR enabled | The chosen ldlidar/rplidar/stl27l fragment |

The installer and updater use the same Go selector. New required fragments enter
the reviewed target stack; new optional groups default to an empty `none` choice.
An unsupported previously selected device blocks the plan instead of silently
being disabled. Core GUI/ROS2 remain required. The release's selected fragments,
saved installer options, retained local services and explicit
`docker/stack-overrides.yaml` produce the desired Compose definition.

`stack-selection.json` records desired hardware options and the identity of the
membership-related installer `.env` values. `stack-applied-selection.json` records
what was last applied. Changes to those `.env` choices invalidate an old plan;
rerun the installer to reconcile them. Ordinary environment/config changes do not
select extra containers. `.env` references remain live; existing robot YAML,
calibration, database and maps remain mounted from their current locations.

After a published deployment is installed, installer reruns retain that installed
bundle and definition, even when the checkout is older. They save changed choices
for **Review update**, including when staying on the same release. Startup
continues using the existing definition until the coordinated transaction applies
the new selection. No hidden removal happens during selection. A rollback restores
the prior applied selection; a still-requested hardware change remains pending.

The review lists **Add, Remove, Update, Keep** and **Keep / local**. Only containers
identified by the reviewed project/service/ID may be retired; new containers are
labelled for ownership-checked rollback removal. There is no broad orphan removal
and no volume deletion. Local services such as MQTT are preserved and not restarted
by the update transaction. Existing physical volume/network names are retained for
the same logical resource keys. Driver/resource changes or new writable storage
require an explicit migration. Reusing an occupied container name for another
service is rejected; such renames need separate releases or a migration.

The updater saves the full previous Compose definition, image overrides, environment,
bundle, selections, local overrides and supported data before activation. Membership
and immutable image references switch together in one atomic Compose replacement;
the durable journal can restore them after interruption. Private Compose/recovery
payloads are removed from browser responses, which expose only choices and changes.
The generated-file checksum rejects manual edits; reviewed customizations belong in
`stack-overrides.yaml`. Legacy adoption also refuses unexplained manual differences.

For an additional first-party service, add its image build definition to
`install/deployment.json`, add its installer Compose fragment to the required list
or an optional group in `install/compose/stack.json`, for example:

```yaml
services:
  camera:
    container_name: mowgli-camera
    image: ghcr.io/mowglinext/mowglinext/camera:dev
    labels:
      garden.mowgli.update.image: camera
      garden.mowgli.update.after: mowgli
      garden.mowgli.update.health: container
```

`image` opts in and identifies a first-party image family in the release asset.
`after` is a comma-separated list of installed managed dependencies; missing
references and cycles reject the plan. Core sensor → ROS2 → GUI ordering cannot
be disabled. Shutdown reverses that order. `health` supports `container`, `gps`
and `lidar`; sensor modes additionally require the corresponding fresh application
observation. Add a Docker healthcheck for a service-specific startup check. Existing
unlabelled standard installations retain their original role mappings.

Every installed managed service must have a target image for the host architecture
in the reviewed release. An unknown/missing image fails the plan before pulling or
stopping containers. Images must remain in the trusted source's GHCR namespace.
The workflow uses the single build-definition list for its matrix, image merging
and publication. No updater source edit is needed for a new stateless first-party
container with an existing health contract.

Persistent services need more care: additional managed services may write only existing writable mounts at the
already supported data destinations (`/db`, `/mowgli_config`, `/ros2_ws/maps`,
`/ros2_ws/config`), which are archived and restored. Other writable mounts reject
the plan. Container writable layers are disposable. Unmanaged containers must not
write shared managed data. New persistence or application-health contracts require
an explicit updater implementation and recovery tests; labels are not arbitrary
backup paths or executable hooks. Adding unrelated third-party image namespaces
is intentionally outside this first-party release model.

`install/deployment.json` also declares `component_compatibility` per image family.
These are maintainer-reviewed drop-in compatibility promises covering **all consumed
and provided interfaces**, ROS messages/services/topic semantics, configuration and
persisted formats. A family with incompatible changes needs a new contract before
publication; remove its entry to disable mixing when uncertain. GUI retains the
legacy `gui_compatibility` field for older consumers. Matching dates, tags or firmware
protocols alone never establish compatibility. Maintaining these promises requires
mixed-version integration tests, including changed consumers and producers. A new
service can join whole releases without a contract; it needs one for independent
version selection.

## Recovery and updater self-updates

The durable journal is `/var/lib/mowgli-updater/state.json`. The worker resumes
interrupted recovery after restart. Settings shows the job phase, error and a
Retry recovery action. Restore previous deployment restores the last successful
deployment's saved data too: changes made since that backup are moved aside.

If the GUI cannot start, use SSH:

```sh
sudo mowgli-updater status
sudo mowgli-updater recover
sudo journalctl -u mowgli-updater.service -n 100
```

Do not remove the maintenance marker to work around a failed recovery. Managed
`mowgli-*` helpers and `docker/stack.sh` share the updater lock, preserve the image
override and refuse conflicting lifecycle operations during maintenance. Direct
administrator Docker commands can bypass that coordination.

The UI also reports the running updater version and offers the selected
deployment's updater binary. It validates the checksum and version/API probe,
and journal schema, then stages the replacement. The installer-managed supervisor starts it and
requires three successful API health samples. Startup failure or a 45-second
health timeout restores the previous binary and reports an error. The worker
cannot replace itself during a container transaction. The supervisor itself is
refreshed by the normal installer; incompatible API/schema changes require an
installer migration rather than in-place worker replacement.

The status/history view retains the most recent 20 completed transactions.
Recovery archives and tagged previous images are retained, not automatically
pruned in this first implementation. Monitor storage and keep the backups/tags
referenced by the current journal and rollback history. Capacity failures stop an
update; they never trigger deletion of recovery data.

## Publishing and contributor reference

`.github/workflows/deployment-release.yml` builds the images declared in
`install/deployment.json` (currently six first-party images) for
both architectures from the same commit, waits for GUI/ROS2 quality gates, and
publishes `mowgli-deployment.json`, its checksummed `mowgli-compose.json` bundle
and updater binaries only when complete. All optional image variants are checked. It
runs for main/dev/release tags, or by manual dispatch on a custom branch. A fork
must enable the workflow and publish readable GHCR images and release assets.
The separate `updater.yml` publishes installer bootstrap binaries.

The descriptor is a release asset, not a catalogue service. Stable assets attach
to the firmware workflow's stable release; development/custom snapshots use
prereleases. The picker returns up to 30 deployments, scanning at most 1,000
release headers. Hitting the scan limit reports an error. Publication and storage
retention remain maintainer responsibilities. HTTPS and trusted repository
configuration establish provenance; SHA-256 checks detect corruption, not an
independent signing authority.

Implementation: `gui/pkg/updater/`, `gui/cmd/mowgli-updater/`,
`gui/cmd/publish-deployment/`, `gui/pkg/api/updater.go`,
`gui/web/src/components/settings/HostUpdaterPanel.tsx`, `install/lib/updater.sh`.
The maintenance helper is
`ros2/src/mowgli_interfaces/include/mowgli_interfaces/update_maintenance.hpp`.
HTTP operations are fixed same-origin JSON routes under `/api/system/updater/`,
proxied to a local Unix socket; no arbitrary Docker command endpoint is exposed.

Tests cover journal recovery, stale plans, persisted checks/notices, source
validation, API origin/readiness gates, supervisor success/crash recovery and a
disposable Docker transaction with injected application failure/data rollback.
Desktop/mobile Playwright cases use fixtures. Physical mower validation and a
complete published ARM64 deployment remain required before field rollout.

### Journal compatibility

The HTTP API remains version 1 with explicit feature capabilities (`release-compose`
adds topology planning; `custom-images` adds explicit image selection). Journal schema 4 adds custom-image provenance and rollback state to schema 3 topology payloads;
this worker reads schema 1/2/3 journals and writes schema 4 on mutation, preserving
history. Older workers reject schema 4. Self-update probes require schema 4 and
refuse unsafe worker downgrades. Existing schema-3 workers require an installer/bootstrap upgrade to this worker before using custom images. Deployment schema 2 requires the Compose bundle;
legacy schema 1 remains image-only. Workers predating schema 2 releases need the
installer bootstrap upgrade before discovering those deployments. Keep the current
worker/backup for recovery; incompatible layouts/data formats require migrations.

### Remaining physical acceptance — HARDWARE_REQUIRED

Software tests and screenshots do not establish physical update acceptance. A
complete published ARM64 test deployment remains a prerequisite. Before a trial,
record the exact PR/combined commit, robot unit, firmware binary hash/protocol,
submodule gitlinks, receiver/driver revisions, Compose configuration, image digests
and updater checksum. The existing private hardware baseline is not evidence for
this extension; no robot was changed during this PR extension.

On a parked mower with blade stopped, stationary wheels, no due mission/schedule,
a supervising operator and physical emergency stop available: install a matched
release; select compatible ROS2, GUI and an enabled sensor on the same base; verify every resulting image;
return to matched; test a reviewed cross-branch custom GUI/ROS/sensor mix, repeated custom changes and rollback with/without a published base; return to a standard deployment and verify provenance resets; roll back each transaction; then test an installed optional
service addition/retirement and failure recovery, checking MQTT identity, retained
volumes and desired/applied installer selections. Perform a supervised interruption
test only after
verifying the manual recovery route and safe power conditions. Pass requires no
actuation or firmware change, the reviewed image combination, preserved/restored
data and maintenance retained until application verification. Any unexpected motion,
image, data loss, stale-observation acceptance or premature gate release is a failure.


### Browser build freshness

Every web build emits `web-build.json` with an identity also embedded in its
JavaScript. The page reads that manifest without caching and compares web-build
IDs, independently of the Go backend revision. Web-only development patches can
therefore keep the previous backend without a permanent reload warning. Different
rebuilds of the same commit still differ. Missing or invalid manifests mean unknown;
backend source revisions are not a substitute for served-web identity. Deploy the
manifest and assets together. Diagnostics retain backend, served web and browser
provenance separately.
