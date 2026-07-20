# Live status of the one-click dock calibration, published at ~1 Hz + on every
# phase change on ~/dock_calibration/status by dock calibration
# (calibrate_imu_yaw_node). This is the foxglove-friendly façade the GUI reads
# (foxglove_bridge has no ROS-action support): the GUI starts a run via the
# ~/dock_calibration/start service (non-blocking) and watches this topic for
# live progress AND the terminal outcome. Phase codes mirror
# CalibrateDock.action; retry_reason mirrors CalibrateDock.action RETRY_*.

uint8 PHASE_WAIT_RTK=0
uint8 PHASE_REVERSING=1
uint8 PHASE_CHECK_COG=2
uint8 PHASE_REDOCKING=3
uint8 PHASE_VERIFY_CHARGE=4
uint8 PHASE_PERSIST=5
uint8 PHASE_IDLE=6      # no run has started this session
uint8 PHASE_DONE=7      # a run finished (see success/retry_reason)

uint8 phase
float32 progress          # 0.0 .. 1.0 over the whole sequence
float32 cog_std_deg       # running circular std of COG samples (deg)
float32 displacement_m    # GPS displacement on the current leg (m)
bool charging             # latest firmware charging state
bool running              # true while a calibration is in progress

# Terminal fields — meaningful once phase == PHASE_DONE (running == false).
bool success
uint8 retry_reason        # mirrors CalibrateDock.action RETRY_* (0 == none)
string message
