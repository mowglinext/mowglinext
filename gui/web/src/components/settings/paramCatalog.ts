// Curated metadata for ROS2 parameters surfaced by the live parameter editor.
//
// The foxglove bridge exposes EVERY declared parameter (300+). Most concern only
// a handful of developers, so each known parameter is tagged with a tier:
//   - basic:  things a normal operator may reasonably want to change
//   - middle: tuning that affects behaviour but needs some understanding
//   - expert: deep internals (estimator gains, scan-match thresholds, PID, ...)
// Parameters not listed here default to the "expert" tier and the "Other" group,
// so nothing is ever hidden from the expert view — the catalog only curates the
// label, description and grouping for the ones we understand.

export type ParamTier = "basic" | "middle" | "expert";

export interface ParamMeta {
  label: string;
  description: string;
  tier: ParamTier;
  group: string;
  unit?: string;
}

export const TIER_ORDER: ParamTier[] = ["basic", "middle", "expert"];

export const TIER_LABEL: Record<ParamTier, string> = {
  basic: "paramCatalog.tier.basic",
  middle: "paramCatalog.tier.middle",
  expert: "paramCatalog.tier.expert",
};

// rank lets us show a profile cumulatively: the "middle" profile shows basic +
// middle, "expert" shows everything.
export const TIER_RANK: Record<ParamTier, number> = {basic: 0, middle: 1, expert: 2};

// Keyed by the parameter's short name (the segment after the last '.' or '/').
// This keeps the catalog independent of how the bridge fully-qualifies names.
const CATALOG: Record<string, ParamMeta> = {
  // ── Coverage / mowing ────────────────────────────────────────────────────
  tool_width: {label: "paramCatalog.tool_width.label", description: "paramCatalog.tool_width.description", tier: "basic", group: "Coverage", unit: "m"},
  transit_speed: {label: "paramCatalog.transit_speed.label", description: "paramCatalog.transit_speed.description", tier: "basic", group: "Coverage", unit: "m/s"},
  mowing_speed: {label: "paramCatalog.mowing_speed.label", description: "paramCatalog.mowing_speed.description", tier: "basic", group: "Coverage", unit: "m/s"},
  num_headland_passes: {label: "paramCatalog.num_headland_passes.label", description: "paramCatalog.num_headland_passes.description", tier: "basic", group: "Coverage"},
  headland_width: {label: "paramCatalog.headland_width.label", description: "paramCatalog.headland_width.description", tier: "middle", group: "Coverage", unit: "m"},
  swath_overlap: {label: "paramCatalog.swath_overlap.label", description: "paramCatalog.swath_overlap.description", tier: "middle", group: "Coverage", unit: "m"},
  swath_angle: {label: "paramCatalog.swath_angle.label", description: "paramCatalog.swath_angle.description", tier: "middle", group: "Coverage", unit: "rad"},
  chassis_safety_inset: {label: "paramCatalog.chassis_safety_inset.label", description: "paramCatalog.chassis_safety_inset.description", tier: "middle", group: "Coverage", unit: "m"},
  progress_timeout_sec: {label: "paramCatalog.progress_timeout_sec.label", description: "paramCatalog.progress_timeout_sec.description", tier: "middle", group: "Coverage", unit: "s"},

  // ── Docking ──────────────────────────────────────────────────────────────
  dock_approach_overshoot: {label: "paramCatalog.dock_approach_overshoot.label", description: "paramCatalog.dock_approach_overshoot.description", tier: "middle", group: "Docking", unit: "m"},
  use_gps_dock_detection: {label: "paramCatalog.use_gps_dock_detection.label", description: "paramCatalog.use_gps_dock_detection.description", tier: "middle", group: "Docking"},
  dock_pose_yaw_sigma_rad: {label: "paramCatalog.dock_pose_yaw_sigma_rad.label", description: "paramCatalog.dock_pose_yaw_sigma_rad.description", tier: "expert", group: "Docking", unit: "rad"},

  // ── GNSS / datum ─────────────────────────────────────────────────────────
  declination_deg: {label: "paramCatalog.declination_deg.label", description: "paramCatalog.declination_deg.description", tier: "middle", group: "GNSS", unit: "°"},
  mag_yaw_variance: {label: "paramCatalog.mag_yaw_variance.label", description: "paramCatalog.mag_yaw_variance.description", tier: "expert", group: "GNSS"},
  enable_mag_cal: {label: "paramCatalog.enable_mag_cal.label", description: "paramCatalog.enable_mag_cal.description", tier: "expert", group: "GNSS"},
  min_horizontal_uT: {label: "paramCatalog.min_horizontal_uT.label", description: "paramCatalog.min_horizontal_uT.description", tier: "expert", group: "GNSS", unit: "µT"},
  datum_lat: {label: "paramCatalog.datum_lat.label", description: "paramCatalog.datum_lat.description", tier: "expert", group: "GNSS", unit: "°"},
  datum_lon: {label: "paramCatalog.datum_lon.label", description: "paramCatalog.datum_lon.description", tier: "expert", group: "GNSS", unit: "°"},

  // ── Fusion graph (localizer) ─────────────────────────────────────────────
  node_period_s: {label: "paramCatalog.node_period_s.label", description: "paramCatalog.node_period_s.description", tier: "expert", group: "Localization", unit: "s"},
  use_scan_matching: {label: "paramCatalog.use_scan_matching.label", description: "paramCatalog.use_scan_matching.description", tier: "middle", group: "Localization"},
  use_loop_closure: {label: "paramCatalog.use_loop_closure.label", description: "paramCatalog.use_loop_closure.description", tier: "middle", group: "Localization"},
  icp_max_iter: {label: "paramCatalog.icp_max_iter.label", description: "paramCatalog.icp_max_iter.description", tier: "expert", group: "Localization"},
  icp_max_corresp_dist: {label: "paramCatalog.icp_max_corresp_dist.label", description: "paramCatalog.icp_max_corresp_dist.description", tier: "expert", group: "Localization", unit: "m"},
  icp_max_rmse_m: {label: "paramCatalog.icp_max_rmse_m.label", description: "paramCatalog.icp_max_rmse_m.description", tier: "expert", group: "Localization", unit: "m"},

  // ── LiDAR filtering ──────────────────────────────────────────────────────
  dock_blank_range: {label: "paramCatalog.dock_blank_range.label", description: "paramCatalog.dock_blank_range.description", tier: "expert", group: "LiDAR", unit: "m"},
  post_undock_blank_sec: {label: "paramCatalog.post_undock_blank_sec.label", description: "paramCatalog.post_undock_blank_sec.description", tier: "expert", group: "LiDAR", unit: "s"},
  min_obstacle_z_m: {label: "paramCatalog.min_obstacle_z_m.label", description: "paramCatalog.min_obstacle_z_m.description", tier: "expert", group: "LiDAR", unit: "m"},
  max_obstacle_z_m: {label: "paramCatalog.max_obstacle_z_m.label", description: "paramCatalog.max_obstacle_z_m.description", tier: "expert", group: "LiDAR", unit: "m"},
  lidar_height_m: {label: "paramCatalog.lidar_height_m.label", description: "paramCatalog.lidar_height_m.description", tier: "expert", group: "LiDAR", unit: "m"},

  // ── Obstacles (local_costmap avoidance tuning) ────────────────────────────
  // task #35, 2026-07-17 field analysis: obstacles only pushed the path from
  // 0.4-0.6 m out at ~0.17 m/s — too late for a smooth deviation.
  // task #49/#51, 2026-07-17: this only affects Nav2 TRANSIT (MPPI/RPP read
  // the inflation gradient) — FTC's coverage/mowing deviation checks raw
  // lethal cells only and never reads this radius. See
  // obstacle_detection_range_m below for the mowing-time equivalent.
  obstacle_inflation_radius: {label: "paramCatalog.obstacle_inflation_radius.label", description: "paramCatalog.obstacle_inflation_radius.description", tier: "middle", group: "Obstacles", unit: "m"},
  // task #51: the real "avoid from further out" knob for MOWING/coverage —
  // how far ahead along the path FTC scans for a lethal cell before it
  // starts skirting. Injected into FollowCoveragePath.obstacle_lookahead as
  // a pose count (navigation.launch.py, F2C 0.05 m sampling).
  obstacle_detection_range_m: {label: "paramCatalog.obstacle_detection_range_m.label", description: "paramCatalog.obstacle_detection_range_m.description", tier: "middle", group: "Obstacles", unit: "m"},
  // task #36, 2026-07-17 field analysis: a wait-clock reset bug let the
  // coverage controller hold zero velocity ~40s (vs. the intended
  // obstacle_wait_timeout_s cap) when skirting a marginal obstacle. Fixed
  // in ftc_controller.cpp; surfacing the timeout itself as GUI-tunable.
  obstacle_wait_timeout_s: {label: "paramCatalog.obstacle_wait_timeout_s.label", description: "paramCatalog.obstacle_wait_timeout_s.description", tier: "middle", group: "Obstacles", unit: "s"},
  // Sideways room left beyond the chassis when skirting. Separate from
  // obstacle_body_half_width on purpose: that one also sets DETECTION reach,
  // and widening it re-opens the over-reach stalls. Injected into
  // FollowCoveragePath.obstacle_clearance_margin (navigation.launch.py).
  // Note obstacle_inflation_radius cannot substitute — the deviation checks
  // threshold at cost 253, a band sized by the footprint inscribed radius.
  obstacle_clearance_margin: {label: "paramCatalog.obstacle_clearance_margin.label", description: "paramCatalog.obstacle_clearance_margin.description", tier: "middle", group: "Obstacles", unit: "m"},

  // ── Motor control (firmware-adjacent PID) ────────────────────────────────
  // 2026-07-17 Option C (task #34): angular_rate_kp/ki/kff (the host-side
  // yaw-rate PI, Option B task #24) are removed — the loop now runs in
  // firmware (task #33), tuned via these params (PACKET_ID_LL_SET_YAW_PID).
  yaw_kp: {label: "paramCatalog.yaw_kp.label", description: "paramCatalog.yaw_kp.description", tier: "expert", group: "Motor control"},
  yaw_ki: {label: "paramCatalog.yaw_ki.label", description: "paramCatalog.yaw_ki.description", tier: "expert", group: "Motor control"},
  yaw_trim_limit_mps: {label: "paramCatalog.yaw_trim_limit_mps.label", description: "paramCatalog.yaw_trim_limit_mps.description", tier: "expert", group: "Motor control", unit: "m/s"},
  yaw_loop_enabled: {label: "paramCatalog.yaw_loop_enabled.label", description: "paramCatalog.yaw_loop_enabled.description", tier: "expert", group: "Motor control"},
  yaw_gyro_sign: {label: "paramCatalog.yaw_gyro_sign.label", description: "paramCatalog.yaw_gyro_sign.description", tier: "expert", group: "Motor control"},

  // ── IMU calibration ──────────────────────────────────────────────────────
  imu_cal_samples: {label: "paramCatalog.imu_cal_samples.label", description: "paramCatalog.imu_cal_samples.description", tier: "expert", group: "IMU"},
  imu_cal_auto_rest_sec: {label: "paramCatalog.imu_cal_auto_rest_sec.label", description: "paramCatalog.imu_cal_auto_rest_sec.description", tier: "expert", group: "IMU", unit: "s"},
  imu_cal_periodic_recal_sec: {label: "paramCatalog.imu_cal_periodic_recal_sec.label", description: "paramCatalog.imu_cal_periodic_recal_sec.description", tier: "expert", group: "IMU", unit: "s"},

  // ── Diagnostics thresholds ───────────────────────────────────────────────
  battery_warn_pct: {label: "paramCatalog.battery_warn_pct.label", description: "paramCatalog.battery_warn_pct.description", tier: "basic", group: "Diagnostics", unit: "%"},
  battery_error_pct: {label: "paramCatalog.battery_error_pct.label", description: "paramCatalog.battery_error_pct.description", tier: "basic", group: "Diagnostics", unit: "%"},
  motor_temp_warn_c: {label: "paramCatalog.motor_temp_warn_c.label", description: "paramCatalog.motor_temp_warn_c.description", tier: "middle", group: "Diagnostics", unit: "°C"},
  motor_temp_error_c: {label: "paramCatalog.motor_temp_error_c.label", description: "paramCatalog.motor_temp_error_c.description", tier: "middle", group: "Diagnostics", unit: "°C"},
};

/** shortName returns the trailing segment of a fully-qualified parameter name. */
export function paramShortName(name: string): string {
  const parts = name.split(/[./]/).filter(Boolean);
  return parts.length ? parts[parts.length - 1] : name;
}

/** paramMeta returns curated metadata, defaulting unknown params to expert/Other. */
export function paramMeta(name: string): ParamMeta {
  const short = paramShortName(name);
  return (
    CATALOG[short] ?? {
      label: short,
      description: "paramCatalog.uncurated.description",
      tier: "expert",
      group: "Other",
    }
  );
}
