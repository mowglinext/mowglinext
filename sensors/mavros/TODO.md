# MowgliNext work queue

## MowgliMAVROS integration — 2026-09-29

The [integration checkpoint](docs/MAVROS_INTEGRATION_CHECKPOINT_20260929.md) records the earlier verified no-motion Lyrical deployment. The canonical MAVROS GNSS software path is now implemented and validated on Lyrical; ARM64/robot acceptance remains pending. No propulsion or mower acceptance is implied.

1. [ ] Complete target acceptance of the canonical MAVROS GNSS path. The MowgliMAVROS `mowgli_gnss` adapter and RTCM path are implemented and have passed Lyrical software/graph validation; remaining work is ARM64/robot validation of sole `/gps/fix` and `/gps/status` ownership, GPS1/GPS2 selection, ellipsoid altitude pairing, rich RTK/correction provenance, stale/no-fix handling, and FCU reconnect/reboot behavior.
2. [ ] Validate POWER1 current/voltage/MAVLink charge state in four physical cases: off dock, dock attached without charge, active charge, POWER1 disconnected. Preserve POWER1 dock and POWER2 traction mapping. The bridge uses fresh POWER1 voltage >0 V as the operator-defined dock/charging indicator, pending reconciliation with a historical off-dock positive-voltage log. `Power.charge_current = raw POWER1 - raw POWER2` only with two fresh currents; GUI/MQTT display only this result. Battery percent is an approximate estimate from filtered POWER2 voltage; GUI shows unavailable without voltage and preserves a measured 0%. Verify voltage-to-SoC calibration independently.
3. [ ] Calibrate signed wheel RPM → wheel distance (`meters_per_motor_revolution`, effective radius, track width), validate count freshness/wrap and physical direction, then enable `/wheel_odom` only after measured acceptance.
4. [ ] Complete HERE4/CAN2 node124, compass and final RTK/RTCM diagnosis independently of the canonical GPS1/GPS2 transport selection. Do not label a receiver as RTK without validated RTK status.
5. [ ] Install/prove hardware E-stop and guarded drive/steering/mower isolation and feedback. Keep `manual_control_enabled=false` and `blade_control_enabled=false` until separate physical acceptance. The prior one-ARM trial failed `Arm: Battery 1 unhealthy`; resolve its cause without bypass.
6. [ ] Publish and pin the production multiarch Lyrical image, verify rollback, and repeat deployment acceptance against an immutable digest before release. Humble compatibility is deferred until the repository refactor.

## Autopilot firmware lifecycle

- [ ] Add autopilot firmware update/flash support for both official firmware and custom Mowgli firmware.
- [ ] Detect the flight-controller board and installed firmware version reliably before selecting an image.
- [ ] Provide explicit image selection, flash, recovery and rollback workflows with hardware-specific safety gates.
- [ ] Add a future heartbeat/device-info equivalent to the STM32 firmware so the autopilot and native Mowgli backends can eventually share one hardware-inventory model. This is a future protocol change, not part of the current MAVROS GUI exposure.
