# Read-only version and update checks

Settings → Updates shows installed software and firmware identities and provides
an explicit **Check now**. **Refresh versions** reads only the installed state.
The comparison selector is remembered in the browser and never changes the
mower's installed channel, image references or Watchtower settings.

## Tag selection and comparison

- **Development** resolves the `dev` tag for each installed first-party image.
- **Stable** reads the latest published non-prerelease GitHub release and resolves
  its version tag, removing the leading `v` (`v1.2.0` → `1.2.0`). Missing release
  images remain unavailable; there is no fallback to `main`, `latest` or `dev`.
- Image tags identify candidates; immutable index, platform and configuration
  digests establish whether images match. Equal source revisions do not hide
  rebuilt images. An older multiarch index can still match when the mower's
  platform image is unchanged.
- For different images, GitHub commit ancestry identifies **newer source**,
  **older source**, **diverged histories**, or the **same source rebuilt**.
  This does not use build dates or claim that every different image is an upgrade.
  Missing revisions, inaccessible fork commits, rate limits and lookup failures
  leave source order unknown without invalidating the digest comparison.

The checker covers installed first-party ROS2, GUI, GPS and the LDLiDAR, RPLiDAR
and STL27L variants. Other installed optional components remain in the inventory
and are marked unsupported for update checks. A custom repository or digest-pinned
installation is identified explicitly and compared with the selected upstream
tag without changing its installed reference.

Each component reports matching, different, unavailable, unknown, missing platform
or unsupported. The overall result says images match only when every installed
component was compared successfully. These are individual image comparisons,
not a declaration of compatibility between releases. Firmware health describes
compatibility with the running robot software using fresh hardware status; it
does not certify an available image or claim the newest firmware is installed.

## Implementation and limits

`GET /api/system/updates?channel=dev` reads the process-local cached result.
`&check=true` explicitly checks remote metadata, at most once per minute per
channel. Last-success time survives subsequent failures until the GUI restarts.
There is no background poll or dependency on a deployment catalogue, additional
Git branch, cross-workflow CI discovery or publication job.

Registry requests read manifests and small configuration documents, never image
layers. Requests have a 45-second overall budget, individual HTTP calls time out
after 15 seconds, and documents are limited to 4 MiB. GHCR and GitHub API hosts are
fixed. `UPDATES_REPOSITORY=owner/repo` lets a host administrator select the source
repository; browser requests cannot supply arbitrary URLs.

Source comparisons run after image checks with at most five seconds of the
remaining overall budget. Full commit SHAs are required; equal revisions need no
GitHub request. Repeated pairs are deduplicated per check, and up to 128 successful
immutable source comparisons are cached across channels until process restart
or cache capacity is reached. No GitHub credentials are required.

GUI and sensor image pipelines publish version tags on release-tag pushes,
alongside existing branch/SHA tags and OCI metadata. This does not retroactively
create missing images for earlier releases such as v1.1.0.

Installation, pin-policy writes, automatic updates, firmware flashing and rollback
are outside these endpoints. A future installer can record local image digests
and use a small release lockfile if coordinated deployments become necessary.

Tests cover registry identity resolution without layer downloads, digest mismatch,
same-revision rebuilds, unchanged platform images, Stable/Dev tag selection,
missing images/platforms, custom and unsupported images, remote failures, cached
read-only routes, and desktop/mobile comparisons.
