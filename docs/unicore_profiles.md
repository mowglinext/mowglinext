# Archived Receiver Profiles

This page is kept only to catch old links.

Receiver-specific runtime profiles are no longer part of the supported MowgliNext install surface. Active GNSS operator configuration now lives in `mowgli_robot.yaml` and the GUI/backend flow; `docker/.env` keeps fallback-only first-boot defaults such as:

```env
GNSS_STACK=universal
GNSS_BACKEND=universal
GNSS_RECEIVER_FAMILY=...
GNSS_TRANSPORT=serial
GNSS_SERIAL_DEVICE=...
GNSS_SERIAL_BAUD=921600
GNSS_NTRIP_ENABLED=true
```

Universal GNSS remains responsible for receiver model/profile/signal validation and apply behavior. Do not add receiver-specific runtime keys or signal-group logic to the installer.
