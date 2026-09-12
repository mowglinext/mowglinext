uint8 MOWER_STATUS_INITIALIZING=0
uint8 MOWER_STATUS_OK=255
uint8 RESET_CAUSE_UNKNOWN=0
uint8 RESET_CAUSE_PIN=1
uint8 RESET_CAUSE_POR_PDR=2
uint8 RESET_CAUSE_BOR=3
uint8 RESET_CAUSE_SFTRST=4
uint8 RESET_CAUSE_IWDG=5
uint8 RESET_CAUSE_WWDG=6
uint8 RESET_CAUSE_LPWR=7

builtin_interfaces/Time stamp
uint8 mower_status
uint8 reset_cause
string reset_cause_name
bool raspberry_pi_power
bool is_charging
bool esc_power  # Legacy field from hardware_bridge; currently blade/ESC-related, not traction PAC5210 power.
bool rain_detected
bool sound_module_available
bool sound_module_busy
bool ui_board_available
bool mow_enabled
bool firmware_debug_enabled

uint8 mower_esc_status
float32 mower_esc_temperature
float32 mower_esc_current
float32 mower_motor_temperature
float32 mower_motor_rpm
# Time of the most recent direct blade-controller report.  This differs from
# `stamp`, which is refreshed for every mainboard status packet even when the
# blade telemetry stream has stopped. A zero stamp means no report received.
builtin_interfaces/Time blade_status_stamp

# Last blade request successfully written to the firmware link:
# unknown (also the empty default), off, forward, or reverse.
# Command intent only: unsigned RPM cannot confirm physical direction, and
# firmware may inhibit the request or wait for a stopped reversal.
string blade_requested_direction

# Firmware version handshake (image <-> firmware compatibility). Populated by
# hardware_bridge from the STM32's CONFIG_RSP reply. firmware_compatible is
# false until a matching reply arrives; if the firmware never answers (too old
# to implement the handshake) it stays false and PreFlightCheck blocks mowing —
# the operator must reflash. firmware_version is "major.minor.patch" ("" or
# "unknown" until resolved); firmware_protocol_version is the wire protocol
# version reported by the firmware (0 if unknown).
string firmware_version
uint8 firmware_protocol_version
bool firmware_compatible
