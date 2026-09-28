# STM32 runtime parameters and their flash persistence state (protocol v7).
# Published latched by hardware_bridge_node on ~/firmware_params whenever a
# firmware report changes it.

# BOOT_UNKNOWN: no store status received yet
uint8 BOOT_UNKNOWN=255
# BOOT_DEFAULTS: the board booted on compiled defaults
uint8 BOOT_DEFAULTS=0
# BOOT_FLASH: the board booted on the values in its flash
uint8 BOOT_FLASH=1
# BOOT_FLASH_ERASED: the log was full/foreign and was erased at boot
uint8 BOOT_FLASH_ERASED=2

uint8 COMMIT_NONE=0
# COMMIT_WRITTEN: a new record was written
uint8 COMMIT_WRITTEN=1
# COMMIT_UNCHANGED: the set already matched the flash: skipped
uint8 COMMIT_UNCHANGED=2
uint8 COMMIT_PENDING=3
# COMMIT_LOG_FULL: no room: the next boot erases and rewrites
uint8 COMMIT_LOG_FULL=4
uint8 COMMIT_ERROR=5

builtin_interfaces/Time stamp
# true once the version handshake completed with a protocol mismatch: the
# board runs none of these values until it is reflashed
bool firmware_incompatible
uint8 boot_source            # BOOT_*
uint8 last_commit            # COMMIT_*
uint16 records_left          # commits that still fit before a boot-time erase
FirmwareParam[] params
