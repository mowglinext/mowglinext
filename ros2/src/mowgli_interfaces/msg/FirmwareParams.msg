# STM32 runtime parameters and their flash persistence state (protocol v8).
# Published latched by hardware_bridge_node on ~/firmware_params whenever a
# firmware report changes it.

# BOOT_UNKNOWN: no store status received yet
uint8 BOOT_UNKNOWN=255
# BOOT_DEFAULTS: the board booted on compiled defaults
uint8 BOOT_DEFAULTS=0
# BOOT_FLASH: the board booted on the values in its flash
uint8 BOOT_FLASH=1
# BOOT_FLASH_ERASED: the log was erased after an explicit request or because
#                    no valid record existed at boot
uint8 BOOT_FLASH_ERASED=2

uint8 COMMIT_NONE=0
# COMMIT_WRITTEN: a new record was written
uint8 COMMIT_WRITTEN=1
# COMMIT_UNCHANGED: the set already matched the flash: skipped
uint8 COMMIT_UNCHANGED=2
uint8 COMMIT_PENDING=3
# COMMIT_LOG_FULL: no room: an explicit reset request is required
uint8 COMMIT_LOG_FULL=4
uint8 COMMIT_ERROR=5
# COMMIT_RESET_PENDING: reset is armed or a boot reset remains unresolved
uint8 COMMIT_RESET_PENDING=6

builtin_interfaces/Time stamp
# true once the version handshake completed with a protocol mismatch: the
# board runs none of these values until it is reflashed
bool firmware_incompatible
uint8 boot_source            # BOOT_*
uint8 last_commit            # COMMIT_*
uint16 records_left          # commits that still fit before an explicit reset is needed
uint32 reset_request_id      # latest reset id retained in backup registers; code 6 confirms armed or unresolved reset
uint32 store_status_sequence # per-connection identity of the latest actual PARAM_STORE_STATUS packet; 0 before one is received
FirmwareParam[] params
