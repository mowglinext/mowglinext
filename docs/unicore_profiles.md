# Archived Receiver Profiles

This page is kept only to catch old links.

Receiver-specific runtime profiles are no longer part of the supported MowgliNext install surface. The installer now writes the Universal GNSS contract only, centered on:

```env
GNSS_STACK=universal
GNSS_BACKEND=universal
GNSS_RECEIVER_FAMILY=...
GNSS_TRANSPORT=serial
GNSS_SERIAL_DEVICE=...
GNSS_SERIAL_BAUD=921600
GNSS_NTRIP_ENABLED=true
```

Do not add receiver-specific runtime keys to new installs.
