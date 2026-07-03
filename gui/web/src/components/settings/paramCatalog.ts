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

  // ── Motor control (firmware-adjacent PID) ────────────────────────────────
  angular_rate_kp: {label: "paramCatalog.angular_rate_kp.label", description: "paramCatalog.angular_rate_kp.description", tier: "expert", group: "Motor control"},
  angular_rate_ki: {label: "paramCatalog.angular_rate_ki.label", description: "paramCatalog.angular_rate_ki.description", tier: "expert", group: "Motor control"},
  angular_rate_kff: {label: "paramCatalog.angular_rate_kff.label", description: "paramCatalog.angular_rate_kff.description", tier: "expert", group: "Motor control"},

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
