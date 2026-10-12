# One STM32 runtime parameter, as reported by the firmware (protocol v7,
# PARAM_VALUE). Published inside FirmwareParams by hardware_bridge_node.

# STATUS_OK: applied exactly as requested
uint8 STATUS_OK=0
# STATUS_CLAMPED: applied, moved into the firmware envelope
uint8 STATUS_CLAMPED=1
# STATUS_UNKNOWN_ID: the firmware has no such parameter
uint8 STATUS_UNKNOWN_ID=2
# STATUS_REJECTED: non-finite value, nothing changed
uint8 STATUS_REJECTED=3

uint16 id                    # stable wire id (fw_param_catalog.h)
string name                  # hardware_bridge ROS parameter (or measurement) that feeds it
bool requested_valid         # false until the bridge sent a value this session
float32 requested            # last value the bridge sent
bool reported                # false until the firmware answered this session
float32 applied              # value the firmware is running with
float32 default_value        # compiled power-on default
float32 min_value            # absolute envelope, lower bound
float32 max_value            # absolute envelope, upper bound
uint8 status                 # STATUS_* of the last set since the board booted
bool persisted               # applied == the value stored in the board's flash
bool is_volatile             # never persisted (re-measured by the host)
