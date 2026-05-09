# Runtime Migration / Backup Cleanup

## Current behavior

`migrate_runtime_paths()` currently creates timestamped backups of:
- `docker/.env`
- `docker/docker-compose.yaml`

on every installer run.

This behavior is intentionally kept during the stabilization/hardening phase
to protect user runtime configuration during install refactors.

## Future improvements

- avoid creating backups when files are unchanged
- rotate old backups automatically
- keep only the latest N backups
- optionally move backups into a dedicated backup directory
- add a `--no-backup` or `--safe-backup` policy mode
- detect generated vs user-edited runtime files
- avoid backup spam during repeated reruns/tests

## Important invariant

Never silently destroy:
- `docker/.env`
- `docker/config/*`
- `mower_config.sh`
- `mosquitto.conf`
- user runtime configuration
