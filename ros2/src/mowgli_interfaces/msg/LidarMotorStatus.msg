# LidarMotorStatus — the LD19 spin-motor's commanded/observed PWM state.
#
# Published by mowgli_lidar_pwm (sensors/lidar-ldlidar), which owns the
# sysfs PWM device on the sensor's PWM control pin (GPIO12/pin 32 by
# default — the LD19's own TXD5 pin, which UART5 claims but the LD19 never
# actually drives since it is a one-way transmit-only serial device; GPIO13/
# pin 33 stays UART5 RXD for scan data, untouched).
#
# Consumed by:
#  * mowgli_behavior's pre-undock/pre-manual-mow precondition, which blocks
#    (bounded) until state == RUNNING and a fresh /scan has actually arrived
#    before letting motion leave IDLE — closing the gap IsScanStale leaves
#    open (a cold motor with zero scans yet reads the same as "no LiDAR
#    installed", which is deliberately permissive there but must NOT be
#    permissive here).
#  * GUI diagnostics (future).

std_msgs/Header header

# commanded stopped; motor not spinning
uint8 IDLE=0
# commanded running; external-control handshake sent, ramping
uint8 SPINNING_UP=1
# commanded running; motor confirmed healthy
uint8 RUNNING=2
# commanded stopped; ramping down
uint8 SPINNING_DOWN=3
# commanded running but motor did not reach a healthy state in time, or the
# PWM device stopped responding
uint8 FAULT=4

uint8 state

# Duty cycle currently being driven, 0.0-1.0. 0.0 while IDLE/SPINNING_DOWN.
float32 commanded_duty_cycle

# Seconds since the last state transition (IDLE<->SPINNING_UP<->RUNNING etc).
float32 time_in_state_sec
