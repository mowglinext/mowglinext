# Unified typed GNSS runtime status.
# backend / receiver_vendor are optional metadata: leave empty when the
# current layer only sees generic NavSatFix data and cannot identify the
# concrete receiver backend safely.
#
# capability_flags indicate which optional fields this backend/runtime path
# knows how to provide at all.
#
# value_flags use the SAME CAP_* bits and indicate which optional fields have
# a current value in this specific sample. This lets consumers distinguish:
#   capability=0                    -> unavailable / unsupported
#   capability=1, value=0           -> supported but currently unknown/stale
#   capability=1, value=1, bool=0   -> available and false
#   capability=1, value=1, bool=1   -> available and true
#
# quality_percent is a coarse normalized UI score. Backends may provide a
# richer value; otherwise the shared adapter derives it from fix_type.
uint8 FIX_TYPE_NO_FIX=0
uint8 FIX_TYPE_GPS_FIX=1
uint8 FIX_TYPE_RTK_FLOAT=2
uint8 FIX_TYPE_RTK_FIXED=3
uint8 FIX_TYPE_DEAD_RECKONING=4

uint8 RTK_MODE_UNKNOWN=0
uint8 RTK_MODE_NONE=1
uint8 RTK_MODE_FLOAT=2
uint8 RTK_MODE_FIXED=3

uint8 BASELINE_STATUS_UNKNOWN=0
uint8 BASELINE_STATUS_COMPUTED=1
uint8 BASELINE_STATUS_NOT_SOLVED=2
uint8 BASELINE_STATUS_INSUFFICIENT_OBSERVATIONS=3
uint8 BASELINE_STATUS_NO_CONVERGENCE=4
uint8 BASELINE_STATUS_OUT_OF_TOLERANCE=5
uint8 BASELINE_STATUS_COVARIANCE_TRACE_EXCEEDED=6
uint8 BASELINE_STATUS_NOT_CONFIGURED=7

uint8 CORRECTION_STREAM_STATUS_UNKNOWN=0
uint8 CORRECTION_STREAM_STATUS_IDLE=1
uint8 CORRECTION_STREAM_STATUS_WAITING=2
uint8 CORRECTION_STREAM_STATUS_ACTIVE=3
uint8 CORRECTION_STREAM_STATUS_UNAVAILABLE=4
uint8 CORRECTION_STREAM_STATUS_ERROR=5

uint32 CAP_RTK_MODE=1
uint32 CAP_HDOP=2
uint32 CAP_VDOP=4
uint32 CAP_HORIZONTAL_ACCURACY=8
uint32 CAP_VERTICAL_ACCURACY=16
uint32 CAP_HEADING=32
uint32 CAP_HEADING_ACCURACY=64
uint32 CAP_SATELLITES_USED=128
uint32 CAP_SATELLITES_VISIBLE=256
uint32 CAP_SATELLITES_TRACKED=512
uint32 CAP_DIFFERENTIAL_CORRECTIONS=1024
uint32 CAP_CORRECTIONS_ACTIVE=2048
uint32 CAP_CORRECTION_AGE=4096
uint32 CAP_MEAN_CN0=8192
uint32 CAP_MAX_CN0=16384
uint32 CAP_DUAL_ANTENNA_STATUS=32768
uint32 CAP_INTERFERENCE_STATUS=65536
uint32 CAP_JAMMING_STATUS=131072
uint32 CAP_DUAL_ANTENNA_BASELINE=262144
uint32 CAP_BASELINE_AZIMUTH=524288
uint32 CAP_BASELINE_PITCH=1048576
uint32 CAP_BASELINE_LENGTH=2097152
uint32 CAP_BASELINE_SOLUTION_STATUS=4194304
uint32 CAP_CORRECTION_STREAM=8388608
uint32 CAP_MSM_SUMMARY=16777216

std_msgs/Header header
string backend
string receiver_vendor
string receiver_model
string receiver_firmware
uint8 fix_type
bool fix_valid
bool differential_corrections
bool corrections_active
bool dead_reckoning
uint8 rtk_mode
bool dual_antenna_heading
bool interference_detected
bool jamming_detected
float32 quality_percent
uint32 capability_flags
uint32 value_flags
float32 hdop
float32 vdop
float32 horizontal_accuracy_m
float32 vertical_accuracy_m
float32 heading_deg
float32 heading_accuracy_deg
uint16 satellites_used
uint16 satellites_visible
uint16 satellites_tracked
float32 correction_age_s
float32 mean_cn0_db_hz
float32 max_cn0_db_hz

# Canonical dual-antenna baseline projection from Universal GNSS.
# heading_* and dual_antenna_heading remain compatibility fields only.
bool dual_antenna_baseline
float32 baseline_azimuth_deg
float32 baseline_pitch_deg
float32 baseline_length_m
uint8 baseline_solution_status

# Diagnostics-derived correction-stream summary mirrored from Universal GNSS.
uint8 correction_stream_status
bool msm_summary_seen
bool msm_summary_decoded
bool msm_summary_valid
uint16 msm_summary_message_type
uint16 msm_summary_station_id
string msm_summary_constellations_seen
uint16 msm_summary_satellite_count
uint16 msm_summary_signal_count
uint16 msm_summary_cell_count
float32 msm_summary_age_s
