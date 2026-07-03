uint8 WHEEL_VALID_FL=1
uint8 WHEEL_VALID_FR=2
uint8 WHEEL_VALID_RL=4
uint8 WHEEL_VALID_RR=8

builtin_interfaces/Time stamp
# Runtime encoder scale [ticks / m]; kept float end-to-end so live tuning
# preserves fractional calibration values such as 319.305.
float32 wheel_tick_factor
uint8 valid_wheels
uint8 wheel_direction_fl
uint32 wheel_ticks_fl
uint8 wheel_direction_fr
uint32 wheel_ticks_fr
uint8 wheel_direction_rl
uint32 wheel_ticks_rl
uint8 wheel_direction_rr
uint32 wheel_ticks_rr
