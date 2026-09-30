# MowgliNext work queue

## MowgliMAVROS integration — 2026-09-29

The [integration checkpoint](docs/MAVROS_INTEGRATION_CHECKPOINT_20260929.md) records the verified no-motion Lyrical deployment. `GNSS_STACK=universal` remains the target; `disabled` is only the temporary NEO-M9N GPS1 cutover. No propulsion, mower or ARM acceptance is implied.

1. [ ] Add a Universal GNSS `mavros` backend and select it under nominal `GNSS_STACK=universal`. Prove one publisher each for `/gps/fix` and `/gps/status`, matching ROS hashes/QoS, observation freshness and GNSS/RTCM provenance; then remove the temporary `GNSS_STACK=disabled` and `MAVROS_GPS1_CANONICAL=true` cutover.
2. [ ] Validate POWER1 current/voltage/MAVLink charge state in four physical cases: off dock, dock attached without charge, active charge, POWER1 disconnected. Preserve POWER1 dock and POWER2 traction mapping. The bridge uses fresh POWER1 voltage >0 V as the operator-defined dock/charging indicator, pending reconciliation with a historical off-dock positive-voltage log. `Power.charge_current = raw POWER1 - raw POWER2` only with two fresh currents; GUI/MQTT display only this result. Battery percent is an approximate estimate from filtered POWER2 voltage; GUI shows unavailable without voltage and preserves a measured 0%. Verify voltage-to-SoC calibration independently.
3. [ ] Calibrate signed wheel RPM → wheel distance (`meters_per_motor_revolution`, effective radius, track width), validate count freshness/wrap and physical direction, then enable `/wheel_odom` only after measured acceptance.
4. [ ] Complete HERE4/CAN2 node124, compass and final RTK/RTCM diagnosis separately from the temporary NEO-M9N. Do not label GPS1 as RTK.
5. [ ] Install/prove hardware E-stop and guarded drive/steering/mower isolation and feedback. Keep `manual_control_enabled=false` and `blade_control_enabled=false` until separate physical acceptance. The prior one-ARM trial failed `Arm: Battery 1 unhealthy`; resolve its cause without bypass.
6. [ ] Publish and pin production multiarch Kilted/Lyrical images, verify rollback, and repeat deployment acceptance against immutable digests before release.
