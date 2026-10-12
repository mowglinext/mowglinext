# Graph checkpoints

`GraphManager::Save(prefix)` stores optimized pose values and metadata together
in `<prefix>.checkpoint`. The versioned envelope contains metadata/XML byte
lengths and a CRC32 over both payloads. `<prefix>.checkpoint.previous` retains
one complete recovery generation. LiDAR tiles remain an independent store.

The graph snapshot is copied under the existing graph mutex. XML serialization,
validation of the previous checkpoint and all disk operations happen after the
mutex is released. The node's existing save-in-flight guard permits one writer
per prefix; direct callers must provide equivalent serialization.

The save sequence is:

1. Create a unique temporary file in the checkpoint's directory. Check all
   writes, `fflush`, `ferror`, file `fsync`, and `fclose`.
2. If the current generation passes envelope, XML and pose/metadata validation,
   hard-link its inode through a unique temporary name and rename that name to
   `.checkpoint.previous`. Sync the parent directory before replacing current.
   If current is corrupt, keep the existing recovery generation instead.
3. Rename the complete new file to `.checkpoint`, then sync and close the parent
   directory. Return success only if every required operation succeeded.

The publication rename is the single graph/metadata commit point. The recovery
rename moves another complete checkpoint; no pair of file renames is treated as
an atomic graph/metadata transaction. Before publication, an interrupted save
leaves current unchanged. After publication, the new complete generation may be
visible. If the final directory sync/close fails, Save returns false, durability
is uncertain, and the previous generation remains available. A retry may save a
new snapshot; a false result does not prove the publication rename never ran.

Load validates framing/checksum, XML, finite metadata/poses, the latest pose
identity, and the configured datum. It first tries current, then the retained
previous generation if current is unreadable or structurally invalid. It keeps
the existing cross-garden rejection policy. Temporary files are never loaded.
Clear removes and syncs legacy files before removing modern generations, checks
errors, and resets RAM only after disk clearing succeeds. Interrupted saves may
leave ignored temporary files; automatic scanning/deletion of those files is
deliberately avoided.

Existing `.graph` XML and `.meta` text checkpoints remain readable when neither
modern file exists, including metadata without datum fields and obsolete scan
sidecars. The next save migrates to the combined format without truncating the
legacy pair. Once a modern generation exists, Load does not silently resurrect a
stale legacy pair. Legacy sidecars have no generation identity: obvious
pose/index mismatches are rejected, but same-index historical mixing cannot be
authenticated retroactively. Older binaries do not read the new format; preserve
the original legacy files if rollback to an older binary is required (their
contents will remain stale after modern saves).

The manual `~/save_graph` Trigger response is deferred until the worker finishes;
`success` describes the actual graph checkpoint result. Busy or unavailable
workers return failure. Executor callbacks continue while disk I/O runs. Auto
save failures are logged at ERROR level. The response does not acknowledge the
separate asynchronous LiDAR tile store.

Regression coverage is in `test_checkpoint_file`, `test_persistence_faults`,
`test_persistence`, and the DDS-isolated `test_save_service`. It covers checked
I/O failure boundaries, short writes, real kernel file-size failures, process
exits around publication, malformed envelopes, legacy migration, recovery,
save/load/Tick round trips, clearing, and executor progress during held I/O.

These tests establish software and process-interruption behavior, not physical
power-loss behavior or navigation performance on a robot. Durability assumes a
local POSIX filesystem with same-directory atomic rename, hard links, and honest
file/directory `fsync` behavior. CRC32 detects accidental damage, not hostile
tampering. Media/controller failures that ignore sync guarantees remain outside
the software proof. No hardware or physical power-loss validation is claimed.
