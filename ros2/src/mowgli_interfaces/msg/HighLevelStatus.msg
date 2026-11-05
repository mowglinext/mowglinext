uint8 HIGH_LEVEL_STATE_NULL=0
uint8 HIGH_LEVEL_STATE_IDLE=1
uint8 HIGH_LEVEL_STATE_AUTONOMOUS=2
uint8 HIGH_LEVEL_STATE_RECORDING=3
uint8 HIGH_LEVEL_STATE_MANUAL_MOWING=4

uint8 state
string state_name
string sub_state_name
int16 current_area
int16 current_path
int16 current_path_index
int16 total_swaths
int16 completed_swaths
int16 skipped_swaths
# Smooth mowing progress for the current area, 0..100. Pose-cursor based
# (primary GUI %); total_swaths/completed_swaths are the coarse sub-path X/Y.
float32 coverage_percent
float32 gps_quality_percent
float32 battery_percent
bool is_charging
bool emergency
# Why the mower is not mowing, for the operator. Set when a session ends or a hold begins without
# finishing (an area given up on, a dock or undock that failed, a charger that never charged);
# cleared by the next command. stop_reason is a stable machine code (AREA_START_BLOCKED,
# AREA_NO_PROGRESS, DOCK_NAV_FAILED, UNDOCK_FAILED, CHARGER_FAILED), stop_detail a short English
# sentence with the numbers. Both are empty when there is nothing to report.
string stop_reason
string stop_detail
