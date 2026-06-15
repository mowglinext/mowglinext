# Drive friction-feedforward calibration status — STM32 firmware -> host.
#
# Periodic (~1 Hz) report of the firmware's passive friction estimator, plus
# whether host-pushed feedforward parameters are currently overriding it. The
# estimator only ever observes commanded motion; it never initiates any. Watch
# this during a friction-calibration sweep to see the learned plant, and to
# decide whether to promote the estimate into mowgli_robot.yaml. Units are
# PAC5210 effort-byte (PWM_PER_MPS = 300 byte per m/s).
builtin_interfaces/Time stamp
float32 est_coulomb_byte          # learned Coulomb intercept [byte]
float32 est_viscous_byte_per_mps  # learned viscous slope [byte/(m/s)]
float32 est_breakaway_byte        # learned static breakaway level [byte]
uint16 sample_count               # RLS windows accumulated (saturating)
bool est_valid                    # passive estimate has converged
bool host_active                  # host-pushed FF params are currently in effect
