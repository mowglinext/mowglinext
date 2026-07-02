# MowgliNext TODO

## Universal GNSS Baseline Integration

- Temporary integration dependency:
  `ros2/src/external/universal-gnss` is pinned in this MowgliNext branch to
  `chore/gnss-geodesy-terminology-audit` at
  `e155211e437dfbf0b32fb7ed230e99341c28c319` until that Universal GNSS work is
  merged upstream.
- After the Universal GNSS branch merges, repoint the submodule from the
  temporary feature-branch commit to the merged upstream `main` commit and
  confirm the public Mowgli `GnssStatus` projection still matches it exactly.
- Keep the public Mowgli GNSS projection canonical:
  - `baseline_azimuth_deg`
  - `baseline_pitch_deg`
  - `baseline_length_m`
  - `baseline_solution_status`
  - `rtk_mode`
  - `correction_stream_status`
  - `msm_summary_*`
- Keep `heading_deg`, `heading_accuracy_deg`, and `dual_antenna_heading`
  compatibility-only. Do not reinterpret `baseline_azimuth_deg` as robot yaw,
  robot heading, or vehicle heading unless a real robot-frame transform is
  applied.
- Field validation checklist on live hardware:
  - verify `/gps/status.baseline_azimuth_deg`, `baseline_pitch_deg`,
    `baseline_length_m`, and `baseline_solution_status` update on solved
    dual-antenna Unicore baseline data
  - verify `/gps/status.correction_stream_status` moves through unknown,
    waiting, active, unavailable, and error states without silently collapsing
    unknown into false
  - verify `/gps/status.msm_summary_*` reflects RTCM semantic diagnostics when
    MSM traffic is present on `/diagnostics`
  - verify generic NMEA + NTRIP still exposes normalized `rtk_mode`
  - verify older Universal GNSS `main` fallback keeps unsupported
    baseline/correction/MSM groups explicitly unavailable instead of inventing
    values
