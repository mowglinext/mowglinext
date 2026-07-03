# GNSS Integration Audit

Archived note.

The historical migration details that used to live here were removed during the Universal GNSS cleanup because they described retired install and runtime paths.

Current rule:

```text
Use only the Universal GNSS install entrypoint and the public GNSS_* contract.
```

Canonical user entrypoint:

```bash
curl -sSL https://mowgli.garden/install.sh | bash
```
