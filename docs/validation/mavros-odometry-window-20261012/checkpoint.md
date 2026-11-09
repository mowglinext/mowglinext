# MAVROS-ODOMETRY-WINDOW-20261012

RETAINED — software PASS; physical acquisition timing/scale HARDWARE_REQUIRED.
Next feat/mavros-refresh HEAD1a2fdae5; MVR811eda2 and GNSS383caba3 unchanged.
Motor/RTK fit is on one acquisition window inside the command plateau, with all
ramp interpolation brackets excluded. Reliable ground motion prevents false
coefficient-based stall; actual stationary ground/missing wheels still rejects.
Missing RTK or counter provenance cannot yield a fit. Diagnostic candidate
43.9678226206 motor rev/m is not applied. Full result/procedure in README.md.
93 pytest cases; colcon96 XML results, zero errors/failures/skips. No robot access,
parameter/firmware/geometry change, commit/push, staging or gitlink modification.
Reports and small fixture remain in Next; original root archive untouched.
