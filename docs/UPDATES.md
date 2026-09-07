# Software updates and recovery

Settings → Updates reports installed software, mainboard firmware and web build
identities. On a managed installation it also checks for complete deployments and
installs a reviewed selection. Updates are never installed automatically.

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

Save and check changes the notification source, not the running deployment.
Choose the latest published snapshot or a specific retained version, optionally
pin it, then review and install. The installed source and pin change only after
successful activation. A pin does not suppress notifications or explicit changes.
Production/dev/custom switching uses the same reviewed installation path.

Checks run every four hours by default, with up to 15 minutes of persisted jitter.
Hourly, daily and manual-only checks are available. A failed check retains the
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
4. Retain old image IDs, stop GUI/ROS writers and archive the GUI database,
   configuration and maps, syncing and checksumming each archive. Require space for backups and failed-deployment data.
5. Replace installed GPS/LiDAR dependencies, ROS2 and GUI in order. Verify actual
   image IDs, container state, fresh application readiness, firmware protocol
   and installed sensor publishers before committing. Charging current and RTK
   fix quality are not readiness requirements.
6. On failure, restore previous configuration/data and images. Retain failed data
   for diagnosis. Release maintenance only after verified success or recovery.

Only ROS2, GUI, GPS and the installed supported LiDAR variant participate.
Other services remain outside this transaction. Firmware, host OS and Docker
upgrades are excluded; custom/LFP firmware is not flashed. Targets requiring a
different firmware protocol, updater API, layout or data schema are rejected.
Older releases without a deployment descriptor are comparison-only.

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
then stages the replacement. The installer-managed supervisor starts it and
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

`.github/workflows/deployment-release.yml` builds all six first-party images for
both architectures from the same commit, waits for GUI/ROS2 quality gates, and
publishes `mowgli-deployment.json` plus updater binaries only when complete. It
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
