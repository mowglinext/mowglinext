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

## GNSS Validation Without Hardware

- The existing GUI test/mock seam now carries canonical `GnssStatus` samples
  in `gui/web/src/test/mocks.tsx`. Those samples are intentionally the same
  public Mowgli-facing message shape the real backend exposes.
- Sample scenarios currently covered:
  - `dual_antenna_um982_solved`
  - `baseline_unavailable_unknown`
  - `ntrip_startup_waiting`
  - `correction_stream_active`
  - `correction_stream_unavailable`
  - `correction_stream_error`
  - `msm_summary_present`
  - `msm_malformed_not_decoded`
  - `nmea_gga_fix_quality_float`
- Frontend validation command:
  `cd gui/web && npm test -- --run src/utils/gpsStatus.test.ts src/components/settings/UniversalGnssLiveStatusCard.test.tsx`
- What these samples validate:
  - baseline field labels/formatting stay canonical:
    `baseline_azimuth_deg`, `baseline_pitch_deg`, `baseline_length_m`,
    `baseline_solution_status`
  - correction stream states render as explicit enum-driven states rather than
    ad-hoc booleans
  - malformed MSM samples keep `seen`/`decoded`/`valid` and zero counts
    explicit instead of silently becoming unsupported
  - Generic NMEA samples validate the public `rtk_mode` projection without
    pretending Mowgli has transformed `baseline_azimuth_deg` into robot yaw

## Robot GNSS Validation Checklist

- Open the Settings live GNSS card and the Diagnostics page while tailing:
  - `ros2 topic echo /gps/status`
  - `ros2 topic echo /diagnostics`
- Validate `dual_antenna_um982_solved` behavior on live hardware:
  - use a dual-antenna Unicore UM982 setup with a solved baseline
  - confirm `/gps/status.baseline_azimuth_deg`,
    `baseline_pitch_deg`, `baseline_length_m`, and
    `baseline_solution_status` carry numeric values plus
    `BASELINE_STATUS_COMPUTED`
  - confirm the GUI shows those values under the canonical baseline labels and
    does not rename `baseline_azimuth_deg` to heading or yaw
- Validate `baseline_unavailable_unknown` behavior:
  - power up with the secondary antenna unavailable or before the baseline
    converges
  - confirm baseline capability remains present but unsolved values stay
    unknown/pending instead of silently false
- Validate correction stream transitions:
  - `ntrip_startup_waiting`: start the receiver before the NTRIP session is
    authenticated and confirm `correction_stream_status=WAITING`
  - `correction_stream_active`: confirm live RTCM forwarding drives
    `correction_stream_status=ACTIVE`
  - `correction_stream_unavailable`: remove the caster/network source and
    confirm `correction_stream_status=UNAVAILABLE`
  - `correction_stream_error`: force an auth or transport failure and confirm
    `correction_stream_status=ERROR`
- Validate RTCM semantic MSM projection:
  - `msm_summary_present`: feed valid MSM traffic and confirm
    `msm_summary_seen`, `msm_summary_decoded`, `msm_summary_valid`,
    `msm_summary_satellite_count`, `msm_summary_signal_count`, and
    `msm_summary_cell_count` update together
  - `msm_malformed_not_decoded`: feed malformed or undecodable MSM data and
    confirm `seen=true`, `decoded=false`, `valid=false` remain explicit instead
    of collapsing to unsupported
- Validate Generic NMEA RTK normalization:
  - `nmea_gga_fix_quality_float`: confirm the Generic NMEA path maps GGA
    `fix_quality` into the public `rtk_mode` enum and the GUI reflects the
    normalized RTK state
- Compatibility fallback check:
  - verify older Universal GNSS `main` builds still leave unsupported
    baseline/correction/MSM groups explicitly unavailable instead of inventing
    values
