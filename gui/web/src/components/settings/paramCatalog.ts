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
  basic: "Basic",
  middle: "Middle",
  expert: "Expert",
};

// rank lets us show a profile cumulatively: the "middle" profile shows basic +
// middle, "expert" shows everything.
export const TIER_RANK: Record<ParamTier, number> = {basic: 0, middle: 1, expert: 2};

// Keyed by the parameter's short name (the segment after the last '.' or '/').
// This keeps the catalog independent of how the bridge fully-qualifies names.
const CATALOG: Record<string, ParamMeta> = {
  // ── Coverage / mowing ────────────────────────────────────────────────────
  tool_width: {label: "Tool width", description: "Effective blade cut width. Drives both the mowed-area stamp radius and the F2C swath spacing.", tier: "basic", group: "Coverage", unit: "m"},
  transit_speed: {label: "Transit speed", description: "Travel speed when driving between areas or to the dock.", tier: "basic", group: "Coverage", unit: "m/s"},
  mowing_speed: {label: "Mowing speed", description: "Forward speed while mowing a swath.", tier: "basic", group: "Coverage", unit: "m/s"},
  num_headland_passes: {label: "Headland passes", description: "Number of concentric border rings mowed before the back-and-forth swaths.", tier: "basic", group: "Coverage"},
  headland_width: {label: "Headland width", description: "Total border band width; rings are spaced by the tool width.", tier: "middle", group: "Coverage", unit: "m"},
  swath_overlap: {label: "Swath overlap", description: "Extra overlap between adjacent swaths to avoid thin un-mowed strips.", tier: "middle", group: "Coverage", unit: "m"},
  swath_angle: {label: "Swath angle", description: "Fixed mowing-line angle. Leave at auto unless you need a specific stripe direction.", tier: "middle", group: "Coverage", unit: "rad"},
  chassis_safety_inset: {label: "Chassis safety inset", description: "Pre-inset of the field boundary to keep the chassis inside the polygon.", tier: "middle", group: "Coverage", unit: "m"},
  progress_timeout_sec: {label: "Progress timeout", description: "How long the controller may make no progress before recovery kicks in.", tier: "middle", group: "Coverage", unit: "s"},

  // ── Docking ──────────────────────────────────────────────────────────────
  dock_approach_overshoot: {label: "Dock approach overshoot", description: "Extra distance driven past the dock pose to guarantee charge-contact.", tier: "middle", group: "Docking", unit: "m"},
  use_gps_dock_detection: {label: "GPS dock detection", description: "Detect the dock from the GPS dock pose instead of relying on isDocked().", tier: "middle", group: "Docking"},
  dock_pose_yaw_sigma_rad: {label: "Dock yaw uncertainty", description: "Assumed heading uncertainty of the configured dock pose.", tier: "expert", group: "Docking", unit: "rad"},

  // ── GNSS / datum ─────────────────────────────────────────────────────────
  declination_deg: {label: "Magnetic declination", description: "Local magnetic declination applied to the magnetometer yaw.", tier: "middle", group: "GNSS", unit: "°"},
  mag_yaw_variance: {label: "Mag yaw variance", description: "Sensor-noise variance for the magnetometer yaw factor.", tier: "expert", group: "GNSS"},
  datum_lat: {label: "Datum latitude", description: "Map origin latitude. Set during onboarding from an RTK-Fixed position.", tier: "expert", group: "GNSS", unit: "°"},
  datum_lon: {label: "Datum longitude", description: "Map origin longitude. Set during onboarding from an RTK-Fixed position.", tier: "expert", group: "GNSS", unit: "°"},

  // ── Fusion graph (localizer) ─────────────────────────────────────────────
  node_period_s: {label: "Graph node period", description: "Cadence at which the factor graph adds a new Pose2 node.", tier: "expert", group: "Localization", unit: "s"},
  use_scan_matching: {label: "LiDAR scan matching", description: "Add LiDAR scan-match between-factors to ride through RTK-Float windows.", tier: "middle", group: "Localization"},
  use_loop_closure: {label: "LiDAR loop closure", description: "Add loop-closure factors from revisited LiDAR scans.", tier: "middle", group: "Localization"},
  icp_max_iter: {label: "ICP max iterations", description: "Maximum iterations for the scan-match ICP solve.", tier: "expert", group: "Localization"},
  icp_max_corresp_dist: {label: "ICP correspondence dist", description: "Maximum point-pair distance considered a correspondence.", tier: "expert", group: "Localization", unit: "m"},
  icp_max_rmse_m: {label: "ICP max RMSE", description: "Reject a scan match whose RMSE exceeds this.", tier: "expert", group: "Localization", unit: "m"},

  // ── LiDAR filtering ──────────────────────────────────────────────────────
  dock_blank_range: {label: "Dock blank range", description: "Blank LiDAR returns within this range of the dock to avoid self-detection.", tier: "expert", group: "LiDAR", unit: "m"},
  post_undock_blank_sec: {label: "Post-undock blank", description: "Suppress obstacle detection for this long after undocking.", tier: "expert", group: "LiDAR", unit: "s"},
  min_obstacle_z_m: {label: "Min obstacle height", description: "Ignore LiDAR points below this height (ground filter).", tier: "expert", group: "LiDAR", unit: "m"},
  max_obstacle_z_m: {label: "Max obstacle height", description: "Ignore LiDAR points above this height.", tier: "expert", group: "LiDAR", unit: "m"},
  lidar_height_m: {label: "LiDAR mount height", description: "Height of the LiDAR above ground, used by the ground filter.", tier: "expert", group: "LiDAR", unit: "m"},

  // ── Motor control (firmware-adjacent PID) ────────────────────────────────
  angular_rate_kp: {label: "Angular rate Kp", description: "Proportional gain of the ROS-side angular-rate assist loop.", tier: "expert", group: "Motor control"},
  angular_rate_ki: {label: "Angular rate Ki", description: "Integral gain of the angular-rate assist loop.", tier: "expert", group: "Motor control"},
  angular_rate_kff: {label: "Angular rate Kff", description: "Feed-forward gain of the angular-rate assist loop.", tier: "expert", group: "Motor control"},

  // ── IMU calibration ──────────────────────────────────────────────────────
  imu_cal_samples: {label: "IMU cal samples", description: "Number of samples averaged during the on-dock IMU bias calibration.", tier: "expert", group: "IMU"},
  imu_cal_periodic_recal_sec: {label: "IMU recal interval", description: "How often the IMU bias is re-estimated while parked.", tier: "expert", group: "IMU", unit: "s"},

  // ── Diagnostics thresholds ───────────────────────────────────────────────
  battery_warn_pct: {label: "Battery warn %", description: "Battery level below which a warning diagnostic is raised.", tier: "basic", group: "Diagnostics", unit: "%"},
  battery_error_pct: {label: "Battery error %", description: "Battery level below which an error diagnostic is raised.", tier: "basic", group: "Diagnostics", unit: "%"},
  motor_temp_warn_c: {label: "Motor temp warn", description: "Motor temperature above which a warning is raised.", tier: "middle", group: "Diagnostics", unit: "°C"},
  motor_temp_error_c: {label: "Motor temp error", description: "Motor temperature above which an error is raised.", tier: "middle", group: "Diagnostics", unit: "°C"},
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
      description: "Uncurated parameter — exposed in the Expert profile only.",
      tier: "expert",
      group: "Other",
    }
  );
}
