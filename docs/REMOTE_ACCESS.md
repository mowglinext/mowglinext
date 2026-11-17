# Remote access (Tailscale sidecar)

Settings → Remote access turns on an **optional** Tailscale node next to the
robot so the web interface is reachable from a phone or laptop anywhere, without
opening a port on the router or running a VPN on the Pi itself. It is off by
default; nothing is installed or exposed until the operator enables it.

## What it does

- Runs one extra container, `mowgli-remote` (`tailscale/tailscale`, pinned to
  a tag published on Docker Hub — the image lags the GitHub release, so check
  hub.docker.com/r/tailscale/tailscale/tags before bumping the pin), on the host network in **userspace networking** mode: no
  `/dev/net/tun`, no `NET_ADMIN`, not privileged, and every Linux capability dropped. Inbound tailnet connections
  are forwarded to the same port on localhost, which on the host network is the
  GUI on `:4006`.
- Optionally publishes the GUI as `https://<name>.<tailnet>.ts.net` through
  **Tailscale Serve** (TLS terminated by tailscaled with the tailnet's automatic
  certificate; Serve preserves the `Host` header, which the GUI's WebSocket
  origin check requires). This needs MagicDNS and HTTPS certificates enabled in
  the Tailscale admin console (DNS page). The plain `http://<tailnet-ip>:4006`
  address always works.
- Keeps the node identity in the Docker volume `mowgli_remote_state`, so
  disabling and re-enabling does not require a new login.

## What it deliberately does not do

- **No public exposure.** Tailscale Funnel is never configured. Only devices
  signed in to the operator's tailnet (subject to its ACLs) can reach the robot.
  The GUI has no login of its own and can drive the mower, flash firmware and
  reboot the host — anyone invited to the tailnet can do all of that.
- **No installer or Compose change.** The container is created and reconciled
  by the GUI backend over the Docker socket from the operator's settings, like
  the GNSS configurator's throwaway containers. It carries no
  `garden.mowgli.update.*` label, so the host updater treats it as unmanaged
  (like MQTT), and it opts out of Watchtower. `mowgli-down` does not stop it;
  turning the setting off removes it.
- **No robot-side coupling.** ROS2 does not know it exists.

## Operator flow

1. Settings → Remote access → switch on. Optionally set a node name (default
   `mowgli`) and paste an **auth key** from the Tailscale admin console
   (Settings → Keys). Save.
2. The status card walks through *Downloading the Tailscale image* → *Starting*
   → either **Connected** (auth key) or **Login required** with an *Open
   Tailscale login* button. Open it, sign in, approve the device; the card
   updates on its own.
3. Once connected the card lists the URLs: the HTTPS name when the tailnet has
   certificates enabled, then `http://<name>.<tailnet>.ts.net:4006` when MagicDNS
   is on, then `http://<ip>:4006` for each tailnet IP.
4. **Log out of tailnet** discards the node key and restarts the sidecar so it
   logs in afresh (with the stored key if one is set).

## Settings (GUI key-value DB, not `mowgli_robot.yaml`)

| Key | Default | Meaning |
|-----|---------|---------|
| `remoteAccess.enabled` | `false` | Create/run the sidecar |
| `remoteAccess.hostname` | `mowgli` | Tailnet node name (DNS label) |
| `remoteAccess.authKey` | unset | Pre-authorised key, write-only; passed to the container as a file (`TS_AUTHKEY=file:…`), never as a plain env var |
| `remoteAccess.serveHttps` | `true` | Apply a Tailscale Serve config (443 → `http://127.0.0.1:<gui-port>`) |
| `remoteAccess.image` | `tailscale/tailscale:v1.102.3` | Sidecar image, restricted to the official `tailscale/tailscale` repository (any tag or sha256 digest) because the API is unauthenticated and the container runs on the host network; blank in the GUI resets to the default |

The GUI port comes from `system.api.addr`, so a non-default listen port is
proxied correctly.

## How the backend reconciles

`gui/pkg/providers/remote_access.go` fingerprints the desired container spec
(image, env, mounts, injected files) into the label `garden.mowgli.remote.spec`.
On every settings change and at GUI start it looks the container up by name:

- disabled → remove the container (volume kept);
- same fingerprint → start it if stopped, else nothing;
- different fingerprint or absent → pull the image, recreate, start.

Status (`GET /api/remote-access/status`) runs `tailscale status --json` inside
the container and projects `BackendState`, `AuthURL`, `Self.DNSName`,
`TailscaleIPs`, `CertDomains` and `Health` into the login link and URL list.

## Health monitoring and bounded recovery

While enabled, the GUI observes the sidecar once a minute, even when no browser
is open. Settings → Remote access shows Docker state separately from daemon
readiness, backend/login state, node online state and Tailscale health warnings.
It also shows the last usable daemon-status observation, cumulative recovery
attempts and the earliest next recovery time. A successful status check is not
proof that a remote phone can reach the GUI.

Tailscale handles ordinary reconnects itself. Offline nodes, WAN/Wi-Fi outages,
health warnings, `Starting`, `Stopped`, `NeedsLogin` (including expired/revoked
keys) and `NeedsMachineAuth` never trigger a restart. Missing/stopped containers,
Docker API errors, malformed status and changed container specs require operator
action rather than speculative repair.

Automatic recovery requires repeated **local** daemon socket failures or a
status command killed by its in-container eight-second timeout, sustained for
at least three minutes on the same running container with the expected spec.
A failed host TCP probe to `controlplane.tailscale.com:443` suppresses recovery
and starts a fresh grace period. This probe is additional reachability evidence,
not a full internet diagnostic, and runs only after a sustained local fault.

Recovery restarts that same container: no image pull, recreation, logout,
credential changes, Serve changes or identity-volume removal. Attempts have a
45-second deadline and a 5/10/20-minute exponential cooldown. At most three
automatic attempts are allowed; the budget is saved **before** each restart and
survives GUI restarts and healthy observations. Storage failures disable further
automatic repair. If a fault persists after the budget is exhausted, the state
is **Degraded** and requires attention. **Retry** or saving settings explicitly
resets the budget. The GUI does not automatically reset authentication.

Docker operations, settings changes, Retry and logout are serialized. An
operator action waits for any already-started, bounded recovery to finish;
subsequent work uses the new settings. Update maintenance pauses observation and
recovery; the updater acknowledges maintenance only after in-flight sidecar work
has drained. If that takes longer than the updater acknowledgement deadline, the
update remains gated and must be retried. Requested reconciliation remains pending
until maintenance clears.
When disabled the health ticks perform no Docker or network operations. The
monitor runs inside the GUI, so it cannot repair a stopped GUI process.

## Routes

`GET/PUT /api/remote-access/settings`, `GET /api/remote-access/status`,
`POST /api/remote-access/apply` (retry), `POST /api/remote-access/logout`.

## Troubleshooting

- **Stuck at "Downloading"**: the Pi is pulling `tailscale/tailscale` from
  Docker Hub; check `docker logs mowgli-remote` and the robot's uplink.
- **"manifest unknown" on pull**: the pinned tag is not on Docker Hub (the
  image publication lags the GitHub release). Set the image field to a tag
  listed at hub.docker.com/r/tailscale/tailscale/tags, e.g.
  `tailscale/tailscale:v1.102.3`, and Save.
- **"Login required" never turns into Connected**: approve the device in the
  admin console; if the tailnet requires device approval, an admin must accept
  it there too.
- **No HTTPS address**: enable MagicDNS + HTTPS certificates in the admin
  console (DNS page). The http:// addresses work regardless.
- **Streams do not load but pages do**: you are going through a proxy that
  rewrites `Host`; use the addresses the status card lists, not a
  reverse proxy of your own.
- **Forget everything**: switch off, then `docker volume rm mowgli_remote_state`
  on the robot.
