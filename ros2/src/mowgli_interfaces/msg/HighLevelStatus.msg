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
