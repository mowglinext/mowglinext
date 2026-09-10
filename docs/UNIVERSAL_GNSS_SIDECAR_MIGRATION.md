# Universal GNSS Sidecar Migration

> **Historical document (2026-06)** — describes the state at that time. Current reference: [`sensors/README.md`](../sensors/README.md) and [`wiki/Sensors.md`](../wiki/Sensors.md).

Archived note.

The detailed migration plan was removed once Universal GNSS became the only supported GNSS runtime in MowgliNext.

Current rule:

```text
The GNSS sidecar is Universal GNSS, configured only through the public GNSS_* contract.
```

Canonical user entrypoint:

```bash
curl -sSL https://mowgli.garden/install.sh | bash
```
