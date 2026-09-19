# Runtime Migration / Backup Cleanup

## Current behavior

`migrate_runtime_paths()` currently creates timestamped backups of:
- `docker/.env`
- `docker/docker-compose.yaml`

on every installer run.

This behavior is intentionally kept during the stabilization/hardening phase
to protect user runtime configuration during install refactors.

## Future improvements

- ~~avoid creating backups when files are unchanged~~ — done: `migrate_runtime_paths()`
  still backs the file up unconditionally up front (the regenerated content
  isn't known until the later `setup_env`/compose-write steps run), but
  `prune_backup_if_unchanged()` (`install/lib/deploy.sh`) removes it
  retroactively once the regenerated file turns out byte-identical to it.
  See `install/tests/test_backup_pruning.sh`.
- rotate old backups automatically
- keep only the latest N backups
- optionally move backups into a dedicated backup directory
- add a `--no-backup` or `--safe-backup` policy mode
- detect generated vs user-edited runtime files
- ~~avoid backup spam during repeated reruns/tests~~ — done for the common
  "nothing actually changed" case, same fix as above.

## Important invariant

Never silently destroy:
- `docker/.env`
- `docker/config/*`
- `mower_config.sh`
- `mosquitto.conf`
- user runtime configuration
