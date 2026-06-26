uint8 MOWER_STATUS_INITIALIZING=0
uint8 MOWER_STATUS_OK=255

builtin_interfaces/Time stamp
uint8 mower_status
bool raspberry_pi_power
bool is_charging
bool esc_power  # Legacy field from hardware_bridge; currently blade/ESC-related, not traction PAC5210 power.
bool rain_detected
bool sound_module_available
bool sound_module_busy
bool ui_board_available
bool mow_enabled

uint8 mower_esc_status
float32 mower_esc_temperature
float32 mower_esc_current
float32 mower_motor_temperature
float32 mower_motor_rpm

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
