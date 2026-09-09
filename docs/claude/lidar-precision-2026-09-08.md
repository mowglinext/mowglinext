# LiDAR precision correction and replay record — 2026-09-08/09

Historical validation record for `feat/lidar-map-anchor`, starting at `599c23be`.
Changes affect localization used by physical motion and therefore require safety-critical review before deployment/merge. No robot deployment is part of this validation.

## Implementation

- Shared acquisition timing: each scan is consumed once; IMU-stamped continuous odometry brackets its pose. ICP factors connect the historical acquisition nodes with node-to-scan offsets. Loop-closure clouds are expressed in their associated node frames.
- Scan-to-map: XY measurements target the historical node with a body-frame odometry offset. No absolute LiDAR yaw measurement is added. Pending observations expire and yield immediately to fresh Fixed or dock state.
- Covariance: floor every principal variance; reject nonfinite or indefinite estimates instead of replacing them with a tight isotropic covariance.
- Map import: incompatible resolution, rotated origins and wrong frames are rejected. No centre-only resampling that silently removes occupied area.
- Shadow calibration: evaluate before inserting the current scan; clear calibration on map replacement/reset. Residuals against fused RTK remain correlated and are not independent ground truth.
- Beluga odometry memory is synchronized with acquisition time before particle seeding. ROS time drives anchor gates, including replay; explicit pose resets invalidate LiDAR history.

## Parameters

Package configuration and declarations are in `ros2/src/fusion_graph/config/fusion_graph.yaml` and `src/fusion_graph_node_setup_params.cpp`.

| Parameter | Default | Purpose |
|---|---|---|
| `lidar_scan_max_age_s` | 0.5 s | Maximum scan age and queued observation lifetime. IMU interpolation gaps above 0.25 s are refused. |
| `lidar_anchor_apply_age_s` | 20 s | Minimum Fixed age before applying a validated map observation. |
| `lidar_anchor_engage_age_s` | 1 s | Existing freshness limit for map learning and filter tracking; independent of the application delay. |

The 20 s delay protects short Float episodes observed in the available bags; it does not establish accuracy during longer outages. Candidate covariance slot 14 indicates acceptance into the queue; expiration/cancellation can still prevent insertion. The graph factor counter records actual insertion.

## Validation protocol

Pinned ROS Kilted/GTSAM/Beluga Docker image: `sha256:870c244d326b3aa6962bcba2e1acfdd59b9a913c5fad5be6d241f40e9bc82544`. Build and tests run as UID 501, with current `mowgli_interfaces` headers explicitly selected in CMake. The image's older helper differs only in repeated nonzero GNSS sequence handling; fusion_graph's receipt-only path uses sequence zero.

Replays use production scan deskew, hardware charging status and COG. GPS/COG are masked by message timestamps from 200 s through 360 s. Original Fixed observations remain evaluator-only during the mask. Predicted antenna position is compared at GNSS header stamps, using the physical lever arm and the configured datum, without fitted trajectory alignment. Initial maps are empty when the bag does not provide a map; no later-session map is imported.

## Automated checks

Release build succeeds. Package validation reports 324 checks, zero errors and zero failures; 72 cppcheck checks are skipped by the image's ament integration because of cppcheck 2.13 performance issues. Nine regression cases were added for historical factor targeting, motion offsets, queue expiration/cancellation, timestamp interpolation and bounds, missing IMU coverage, angle wrap, principal covariance flooring and incompatible map resolution. The exact final log is `fix-final-checks.log` in the validation bundle.

clang-format 18.1.8 passes both full touched-file verification and the changed-lines check against the merge-base with `origin/main`. No deployment or repository commit was performed.

## Remaining limitations

- At rest, the existing graph throttle creates a node every 5 seconds (`GraphParams::stationary_node_period_s`). Returning GNSS factors wait for that tick. In the mowing replay, the corrected run waited roughly four seconds longer than the baseline due to timer phase: return-window P90 was 6.51 cm versus 2.35 cm, with unchanged wrong-fix rejection counters. This is a separate GNSS application-latency issue.
- Shadow calibration still shares odometry and earlier map scans with the fused reference. Evaluating before insertion removes direct reuse of the current scan but does not make calibration independent.
- The fixed application delay and covariance changes do not guarantee accurate localization outside mapped support. The validator still rejects unsupported estimates.
- The native-outage bag from September 6 lacks hardware charging status. The fail-closed charger gate correctly produced zero map candidates and zero map factors; no claim about scan-to-map accuracy during its outage can be made.
- Particle-filter and callback scheduling variability require reporting repeated runs, not selecting a best trajectory. Two repetitions are observed ranges, not statistical confidence intervals.

## Local reproduction bundle

Artifacts and scripts live at `/Users/cedric/Dev/git/mowglinext-lidar-review-20260908` (outside the repository, alongside the existing review bundle). `fix-source-hashes.json`, `fix-binary-hashes.json` and `fix.patch` identify the implementation; every run stores the exact invocation, parameters, node logs, trajectories, candidates, diagnostics and relay counters.

Use `python3 fix_batch.py paired`, `python3 fix_batch.py repeat`, `python3 fix_batch.py baseline_repeat` and `python3 fix_batch.py mow` from that directory. Runs use ROS time at playback rate 1. `fix_launch.py` also runs the native-outage bag and the odometry-only control. Evaluate them with `evaluate.py`, consolidate timestamp-matched errors with `fix_compare.py`, and render with `.venv/bin/python fix_plot.py`. Tags ending in `_invalid_integer_arg` are failed harness attempts and are excluded.

## Measured results

Twelve completed replays across three bags. On `bag_full_20260908_1515`, eight paired/repeated LiDAR runs and one odometry-only control share 790 unique held-out GNSS timestamps in [201, 359.5] s. All nine mask exactly 800 GPS publications and 99 COG publications. No trajectory alignment is fitted.

| Mode | Before P90 (m), two runs | Corrected P90 (m), two runs |
|---|---:|---:|
| Scan-to-map only | 0.308–0.513 | 0.224–0.314 |
| ICP + scan-to-map (LC enabled) | 0.962–1.662 | 0.169–0.381 |

The odometry-only control has P90 **0.646 m** on the same 790 timestamps. Thus the corrected combined path improves substantially over both the previous implementation and the control in these trials. The anchor-only ranges partly overlap; this evidence does not establish a universal ranking between anchor-only and combined modes.

On `bag_mow_lidar_20260907_0644`, the low-motion masked interval has 792 shared reference timestamps: P90 **0.0647 m before** and **0.0654 m corrected**, essentially unchanged. The GNSS-return latency difference is described above. The September 6 native-outage run verifies the missing-charger-status gate, not scan-to-map precision.

Corrected candidate timestamps are unique and their observed publication ages remain below the 0.5 s freshness bound. The full-bag corrected runs queue no map observation outside the artificial outage; the short natural Float episode does not trigger map application. Each corrected combined full-bag run accepts three loop closures over the full replay.

### Per-run held-out errors

| Run | N | P50 (m) | P90 (m) | Max (m) |
|---|---:|---:|---:|---:|
| `before_anchor20` | 790 | 0.161 | 0.308 | 0.431 |
| `before_both20` | 790 | 0.750 | 1.662 | 1.767 |
| `before_anchor20_r2` | 790 | 0.192 | 0.513 | 0.612 |
| `before_both20_r2` | 790 | 0.258 | 0.962 | 1.068 |
| `after_anchor20` | 790 | 0.130 | 0.224 | 0.345 |
| `after_both20` | 790 | 0.253 | 0.381 | 0.494 |
| `after_anchor20_r2` | 790 | 0.144 | 0.314 | 0.385 |
| `after_both20_r2` | 790 | 0.088 | 0.169 | 0.382 |
| `mow_before_both20` | 792 | 0.054 | 0.065 | 0.078 |
| `mow_after_both20` | 792 | 0.054 | 0.065 | 0.078 |
| `after_odom` | 790 | 0.432 | 0.646 | 0.752 |

Machine-readable comparison: `fix-comparison.json`. Plots: `fix_precision.png` and `fix_precision.pdf`, in the local validation bundle.


