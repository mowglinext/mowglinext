# Extension Manager v1: local administration and GUI

Implemented in `gui/pkg/extensions`, `gui/cmd/mowgli-extensions` and the lazy-loaded
`/extensions` GUI page. The GUI forwards a closed API through the existing host
updater Unix socket; the manager still runs on the host. There is no additional
daemon, ROS node, automatic update hook or Go dependency.

## What is implemented

- Closed manifest/bundle and host-trust schemas; explicit platform, source,
  capability and external-unit approvals. Duplicate, unknown, null, case-aliased
  and trailing JSON fields are rejected. Source commits are provenance declarations
  approved locally, not cryptographic proof that an image was built from that commit.
- Separate Compose projects and immutable bundle artifacts. Images must use full
  SHA-256 digest references. `pack` resolves an existing local Compose definition
  without starting services, then writes a new private bundle file.
- Install disabled, explicit adoption of an existing project, enable, disable,
  fixed-topology upgrade, rollback and remove. Remove retains configuration,
  data directories and named-volume identity; there is no purge command.
- A persistent JSON register, Linux process lock, atomic writes, file/directory
  fsync and recovery journal. `recover` resumes an interrupted transaction on
  explicit invocation. All changes are serialized per state root.
- Read-only external systemd registration (`adopt`, `status`, `forget`). No
  systemd start/stop, package installation, modem flashing or ROS configuration.
- Fresh runtime observation on every status request. No persisted health cache,
  restart loop or periodic ROS discovery. Container ID plus start time and
  systemd invocation identity distinguish process incarnations.
- Read-only `preflight` against candidate host compatibility declarations.
  The existing robot updater does not invoke it automatically.

## Build and checks

From `gui/`, using the Go version declared in `go.mod`:

```sh
CGO_ENABLED=0 go build -o /tmp/mowgli-extensions ./cmd/mowgli-extensions
go test ./pkg/extensions ./cmd/mowgli-extensions
go test -race ./pkg/extensions
```

The opt-in [real Engine runner](tests/docker_lifecycle.py) uses only Python's
standard library and a preloaded, approved Linux image containing `/bin/sleep`.
See [ROBOT_ACCEPTANCE.md](ROBOT_ACCEPTANCE.md) for recorded baselines and scope.
Run it as the normal Docker user;
always use `--robot-readiness` when testing on a mower:

```sh
python3 install/extensions/tests/docker_lifecycle.py \
  --binary /path/to/mowgli-extensions \
  --image '<approved-preloaded-repository>@sha256:<digest>' \
  --source-repository owner/repository \
  --source-revision '<full-reviewed-image-source-commit>' \
  --robot-readiness
```

Repeat with `--storage volume` to exercise an explicitly named external volume.
The runner creates its own private temporary register, data and random Compose
project; disables inherited image health probes; checks unrelated container IDs
and start times; and removes only its own resources. Failures retain private
artifacts for diagnosis. It does not adopt the real Connect service or modify
robot configuration. The registry may still be contacted by the manager while
preparing the approved pinned image. No Docker daemon is needed for `--help`.
Volume mode additionally uses the approved image's `/bin/sh` and `/bin/cat` in
short-lived containers with no network, no capabilities and only its own volume.
The image must initialize `/data` writable by UID 65532, as the Connect image does.
Only a fresh test-labelled volume is removed at successful test cleanup; the
manager itself never deletes volumes.

The CLI can be installed separately after review. No change to the robot
installer or host services is needed to build or inspect the standalone command. Run it as the local
Docker administrator, preferably the same user who owns the existing Connect
installation/cron, with a private state root owned by that user. Do not share one
Docker project across independent state roots.

V1 targets the local Linux Docker socket `/var/run/docker.sock`. Inherited
remote Docker contexts and Compose overrides are ignored; rootless sockets and
remote engines need a separate future configuration contract. Pulled image
OS/architecture is checked before any existing service is stopped.

Global flags precede the action; action flags precede its id:

```sh
/tmp/mowgli-extensions --config /path/to/host.json \
  --state-dir /path/to/private/state status
```

Start from [host.example.json](host.example.json). Set the actual CPU platform
and unique installation identity; the CLI rejects a configured platform which
differs from the running executable. Trust lists must be explicit arrays.
The `reviewed:*` value is an administrator's baseline assertion, not a contract
automatically exported or refreshed by today's core. Re-review it after a core
change. Do not copy the example as a claim of compatibility with every future image.

The default locations are `/etc/mowgli-extensions/host.json` and
`/var/lib/mowgli-extensions`. They are not created by the repository installer.
The CLI only creates its own private state directories when invoked; it never
creates or rewrites robot runtime configuration.

## GUI and repository ownership

Open **Extensions** from the desktop rail or mobile More menu. The page lists
installed versions, source, compatibility, process/application/data status and
recovery. Import a local JSON bundle, review its source commit, permissions and
per-service image digest, storage mounts, networks and environment key names,
then explicitly apply. Environment values and command arguments are never shown. Installation leaves managed services disabled;
Enable is a separate confirmed action. External registrations offer observation
and Forget only. Updates use the same import/review flow, and rollback confirms
both the active and previous release identities.

Integration code, images and release manifests can live in independent official,
community or private repositories, or in separate directories of a shared repo.
MowgliNext owns the manager and bundle contract, not every integration's code.
`source.repository` identifies the reviewed source; it is not an instruction to
fetch or execute that repository. Publishers can distribute the same local bundle
through their own release pages, a future optional catalog or an administrator.
No official inclusion is required. V1 imports local files only and runs no remote
installer. Existing Compose `pack` supports local preparation of those artifacts.

The host policy decides which repositories, grants, units and contracts are
approved. Custom import can display an unapproved bundle and its blockers, but
cannot silently add trust or weaken isolation. This supports third-party sources
within the closed profiles; arbitrary host devices, privileged services, published
ports and arbitrary health URLs require a separately reviewed adapter contract.
Source commit metadata is a declaration, not verified image-build provenance.

### Host setup and one owner

The host helper must include the optional `/v1/extensions/*` routes. An older
helper returns an unavailable/setup state, while all other GUI features continue
to work. The existing service reads `/etc/mowgli-extensions/host.json` and uses
`/var/lib/mowgli-extensions` by default. Host-admin service environment overrides
are `MOWGLI_EXTENSIONS_CONFIG` and `MOWGLI_EXTENSIONS_STATE_DIR`; the browser cannot
select these paths or edit host trust. The installer does not create this policy.

The standard `mowgli-updater.service` runs as root. Its register therefore belongs
to that account, and any CLI invocation sharing it must use the same operating
account and root. A register created by another user is rejected. Do not create
a second register for the same Docker project or silently change ownership;
review an explicit migration before connecting an existing CLI-managed install.
The optional extension register remains separate from core updater state/jobs.

GUI configuration bindings are restricted to `/etc/mowgli-extensions/<id>` or
its descendants; data bindings must stay within `<state-root>/<id>`. The same
policy is checked under the mutation/recovery lock. Use the local CLI for
administrator-reviewed layouts outside those roots. The host service has
`PrivateTmp=true`, so temporary paths are also excluded from GUI administration.
Existing configuration/data and physical volume names must be provisioned and
reviewed before adoption.
The GUI container never reads host bind paths or owns the extension register.

### Requests, recovery and performance

Mutations require the existing same-origin JSON guard and explicit request header.
Only state, review, apply and closed lifecycle actions are proxied. Original file
text reaches the strict host parser intact, including duplicate-key rejection.
A review digest binds action, bundle, storage binding and current release; rollback
also checks the exact previous digest under the mutation lock. Responses include
sanitized manifests/status and concrete image, physical mount and network
identities, with environment key names only; never environment values, arguments
or resolved Compose.

Confirmed changes survive browser disconnects. Preparation, including image
pull, has a two-minute budget. Activation gets a new two-minute budget after
preparation succeeds, so download time cannot consume the replacement window.
An expired preparation leaves the running version untouched. Explicit recovery
has its own two-minute budget. Host restart recovery uses the existing durable extension
journal, surfaced explicitly through Recover. Only the latest transient GUI job
is kept in memory; there is no second durable queue or growing job history.

Opening the page and Refresh request fresh status. There is no idle background
polling; an active job is polled only while visible. During that job, the response
explicitly shows it running and returns the last status snapshot instead of
contending with the transaction lock. Each fresh Compose observation uses one
Docker inventory and one image verification per service, with no persistent
health cache. There is no background restart loop or unbounded job history.

New extension jobs are refused while a core job is pending, and GUI maintenance
also blocks mutations. This is not a bidirectional core/extension transaction lock;
core compatibility preflight and coordinated update planning remain future work.

## Connect: pack and adopt existing identity

Use the standalone Connect installation. A legacy service embedded in core
`stack-overrides.yaml` needs a separately reviewed migration; this CLI will not
move it or guess its volume identity.

1. Disable the installation's own updater first. Use a Connect installer version
   that supports `--auto-update no` while preserving existing settings and
   per-install cron removal. Its `--uninstall` must retain identity by default. Do not use
   uninstall when transferring a running instance. Inspect the actual owning
   user's crontab too; this CLI does not remove or discover every user's cron.
2. Confirm the existing project, service and physical identity volume using
   Docker read-only inventory. Preserve the volume name exactly. For the current
   standalone defaults they are `mowgli-connect`, `mowgli-connect`, and normally
   `mowgli-connect_data`; the physical volume name must be verified.
3. Prepare a local copy of the standalone Compose definition. Keep its bridge,
   normal entrypoint, read-only rootfs, capability-drop/bootstrap, environment
   and egress policy. Declare the **existing** physical data volume explicitly:

   ```yaml
   volumes:
     data:
       external: true
       name: <verified-physical-volume-name>
   ```

   This avoids making the manager the creator/deleter of an identity volume.
   `pack` deliberately refuses implicit volumes. It does not edit the source
   Compose file. Review the resolved image digest and source revision together;
   adoption must match the image/config actually running, even if the source
   Compose previously used a moving tag.
4. Pack and inspect the private file, then explicitly adopt:

   ```sh
   mowgli-extensions --config /path/host.json --state-dir /path/private/state \
     pack --manifest /path/to/mowgli-connect.json \
     --compose /path/to/reviewed-compose.yaml --env-file /existing/connect/.env \
     --project mowgli-connect --output /path/private/connect.bundle.json mowgli-connect

   mowgli-extensions --config /path/host.json --state-dir /path/private/state \
     adopt --bundle /path/private/connect.bundle.json \
     --project mowgli-connect mowgli-connect
   ```

Use [manifests/mowgli-connect.json](manifests/mowgli-connect.json) as the manifest
starting point. Its version is the local distribution-manifest version, not an
assertion of an upstream release. For an image built from local reference-repo
changes, replace the source revision with the actual reviewed commit when it exists.

The normalized definition is checked against live container image IDs, mounts,
environment, command/entrypoint, isolation, networks and ownership. Explicit
adoption performs no recreation. A subsequent enable/upgrade materializes the
approved definition with manager ownership labels. Another project's containers,
resources, unexpected services or differing identity are rejected.

Resolved bundles can contain environment secrets. The tool writes them 0600,
never prints their contents, and sanitizes Docker/systemd failures. Keep bundles
outside Git. `pack` refuses to overwrite a destination. V1 rejects all dollar
characters in resolved Compose values, including literal/escaped dollars, to
prevent a second Compose interpolation from changing the approved definition.
Configurations needing dollar literals require a future literal-encoding contract.

The generic `isolated` profile requires an explicit private bridge, read-only
rootfs, dropped capabilities, no-new-privileges, declared egress and approved
storage. Connect's specific profile preserves its root nft bootstrap capabilities.
These are trusted local integrations: network access can reach the host GUI;
the manager does not turn arbitrary third-party code into an untrusted plugin sandbox.
Restart policies are limited to `no` and `unless-stopped` so a disabled service
does not silently re-enable after a Docker daemon restart. A partially running
project is reported as `partial`, never as fully stopped.

## LoRa: observe only

The existing modem service and optional ROS adapter remain separate. The
reference bundle includes **only the modem unit** and grants no ROS compatibility:

```sh
mowgli-extensions --config /path/host.json --state-dir /path/private/state \
  adopt --bundle /path/to/mowgli-lora-link.bundle.json \
  --project mowgli-ext-mowgli-lora-link mowgli-lora-link
mowgli-extensions --config /path/host.json --state-dir /path/private/state status
mowgli-extensions --config /path/host.json --state-dir /path/private/state \
  forget mowgli-lora-link
```

`project` reserves the registration's namespace; no Compose project is created
for an external service. [The bundle](manifests/mowgli-lora-link.bundle.json)
references the investigated modem baseline. Declared units must also appear in
the host's external-unit allowlist.

Status distinguishes process state, application health and data freshness.
LoRa modem health is accepted only from its fixed loopback endpoint when the
response reports the same systemd `invocation_id` as the primary modem unit,
unchanged across the request. The LoRa service must support this optional field.
Legacy responses remain usable for legacy consumers but the manager reports
application health `unknown`. RTCM progress and downstream receiver use remain
`unknown`; an HTTP response or callback timestamp never proves new correction data.

## Changes, failures and recovery

```sh
mowgli-extensions --config /path/host.json --state-dir /path/private/state \
  upgrade --bundle /path/private/new.bundle.json mowgli-connect
mowgli-extensions --config /path/host.json --state-dir /path/private/state \
  rollback mowgli-connect
mowgli-extensions --config /path/host.json --state-dir /path/private/state recover
```

Upgrade prepares/pulls before stopping old services, records the durable before
and target state, applies and verifies the target, then atomically commits.
Application probes may be `starting` for a bounded readiness window. A failed
activation restores the previous compatible artifact; failed restoration leaves
the journal and explicit recovery status. Cancellation requires explicit recover.
An interrupted enable cleans up; an interrupted disable/remove completes that
requested operation. Nothing changes the robot's maintenance marker.

V1 rejects service/network topology, data schema, volume layout, profile and
ownership migrations. A same-schema application must have a reviewed backward
compatible data contract; the manager retains data and does not snapshot/restore
application data. Only artifacts and desired state are rolled back. Disabled
upgrades and rollbacks require the actual service to be stopped, then journal
removal of its stopped containers before advancing the release identity. They
never start a service. Automatic
adoption does not stop unrelated updaters, and it is not a substitute for that
data contract. Keep core updates and extension updates as explicit operations.

Disable/remove keep config/data and reserve their ownership after unregistering:

```sh
mowgli-extensions --config /path/host.json --state-dir /path/private/state disable mowgli-connect
mowgli-extensions --config /path/host.json --state-dir /path/private/state remove mowgli-connect
mowgli-extensions --config /path/host.json --state-dir /path/private/state \
  preflight --target-host /path/candidate-host.json
```

`preflight` exits nonzero for conflicts or unresolved recovery. It is advisory
until the existing updater gains a separate, reviewed integration. This change
does not claim that today's core automatically enforces extension compatibility.

## Acceptance boundaries

The isolated Engine tests and exact executable/image baselines are recorded in
[ROBOT_ACCEPTANCE.md](ROBOT_ACCEPTANCE.md). Existing production services were
observed only. Connect adoption, LoRa deployment, GNSS/RF delivery and automatic
core-updater enforcement each require their own acceptance; the CLI does not
claim them. No motion, blade or robot maintenance behavior is changed.
