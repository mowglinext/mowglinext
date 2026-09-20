// Declarative field groups rendered by SettingsFieldCard.
//
// Labels and tooltips come from i18n (`settingsFields.<key>.label|tooltip`),
// group headings from `settingsFieldGroups.<id>.title|description`. Defaults are
// NOT declared here: they live in asserts/mower_config.schema.json (pinned to
// the ROS2 template by schema_template_parity_test.go). The min/max below are
// input guards only and mirror the clamps the consumers apply themselves.

export type SettingsFieldSpec =
    | { key: string; kind: "switch" }
    | { key: string; kind: "select"; options: number[] }
    | {
          key: string;
          kind: "number";
          min: number;
          max: number;
          step: number;
          precision: number;
          unit?: string;
      };

export interface SettingsFieldGroup {
    id: string;
    /** Physical-motion / safety-relevant group: renders a warning banner. */
    isSafetyRelevant?: boolean;
    fields: SettingsFieldSpec[];
}

const num = (
    key: string,
    min: number,
    max: number,
    step: number,
    precision: number,
    unit?: string,
): SettingsFieldSpec => ({ key, kind: "number", min, max, step, precision, unit });

export const YAW_LOOP_GROUP: SettingsFieldGroup = {
    id: "yawLoop",
    isSafetyRelevant: true,
    fields: [
        { key: "yaw_loop_enabled", kind: "switch" },
        num("yaw_kp", 0, 5, 0.01, 2),
        num("yaw_ki", 0, 20, 0.05, 2),
        num("yaw_trim_limit_mps", 0, 0.5, 0.01, 2, "m/s"),
        { key: "yaw_gyro_sign", kind: "select", options: [1, -1] },
    ],
};

export const CHARGE_LIMITS_GROUP: SettingsFieldGroup = {
    id: "chargeLimits",
    fields: [
        num("max_charge_voltage", 0, 30, 0.1, 1, "V"),
        num("max_charge_current", 0, 5, 0.1, 1, "A"),
    ],
};

export const LOCALIZATION_GUARD_GROUP: SettingsFieldGroup = {
    id: "localizationGuard",
    fields: [
        num("loc_gnss_acc_pause_m", 0.02, 5, 0.05, 2, "m"),
        num("loc_gnss_acc_resume_m", 0.01, 5, 0.05, 2, "m"),
        num("loc_gnss_stale_s", 1, 60, 1, 1, "s"),
        num("loc_sigma_pause_persist_s", 0, 60, 0.5, 1, "s"),
        num("loc_sigma_resume_persist_s", 0, 60, 0.5, 1, "s"),
        num("loc_sigma_pause_m", 0, 20, 0.5, 2, "m"),
        num("loc_sigma_resume_m", 0, 20, 0.5, 2, "m"),
        num("loc_sigma_backstop_persist_s", 0, 120, 1, 1, "s"),
    ],
};

export const DOCK_DETECTION_GROUP: SettingsFieldGroup = {
    id: "dockDetection",
    fields: [{ key: "use_gps_dock_detection", kind: "switch" }],
};

export const DOCK_CALIBRATION_GROUP: SettingsFieldGroup = {
    id: "dockCalibration",
    fields: [
        num("dock_calib_reverse_distance_m", 0.5, 5, 0.1, 2, "m"),
        num("dock_calib_reverse_speed_ms", 0.05, 0.3, 0.01, 2, "m/s"),
        num("dock_calib_redock_overshoot_m", 0, 1, 0.05, 2, "m"),
        num("dock_calib_rtk_wait_timeout_s", 1, 300, 1, 1, "s"),
        num("dock_calib_redock_charge_timeout_s", 5, 300, 1, 1, "s"),
        num("dock_calib_cog_min_samples", 1, 200, 1, 0),
        num("dock_calib_cog_std_max_rad", 0.05, 3.14, 0.05, 2, "rad"),
        num("dock_calib_cog_bearing_match_max_rad", 0.05, 3.14, 0.05, 2, "rad"),
        num("dock_calib_min_baseline_displacement_m", 0.1, 5, 0.1, 2, "m"),
    ],
};

export const TURN_SPEED_GROUP: SettingsFieldGroup = {
    id: "turnSpeed",
    fields: [num("turn_speed_ratio", 0.1, 1, 0.05, 2)],
};

export const REVERSE_ESCAPE_GROUP: SettingsFieldGroup = {
    id: "reverseEscape",
    isSafetyRelevant: true,
    fields: [
        { key: "obstacle_reverse_enabled", kind: "switch" },
        num("obstacle_reverse_max_dist_m", 0, 1, 0.05, 2, "m"),
        num("obstacle_reverse_speed_mps", 0, 0.3, 0.01, 2, "m/s"),
    ],
};

export const AREA_RECORDING_GROUP: SettingsFieldGroup = {
    id: "areaRecording",
    fields: [
        num("area_simplification_tolerance", 0.01, 0.5, 0.01, 2, "m"),
        num("area_record_rate_hz", 1, 50, 1, 1, "Hz"),
    ],
};

export const BEHAVIOR_TREE_GROUP: SettingsFieldGroup = {
    id: "behaviorTree",
    fields: [
        num("tick_rate", 1, 50, 1, 1, "Hz"),
        { key: "bt_debug_logging", kind: "switch" },
        { key: "idle_nav2_suspend", kind: "switch" },
    ],
};

// Upper bounds equal the compiled ceilings in
// mowgli_behavior/start_blocked_escape.hpp (kEscapeMax*).
export const START_ESCAPE_GROUP: SettingsFieldGroup = {
    id: "startEscape",
    isSafetyRelevant: true,
    fields: [
        { key: "start_blocked_escape_enabled", kind: "switch" },
        num("start_blocked_escape_speed", 0, 0.15, 0.01, 2, "m/s"),
        num("start_blocked_escape_distance", 0, 0.6, 0.05, 2, "m"),
        num("start_blocked_escape_timeout_s", 0, 15, 0.5, 1, "s"),
        num("start_blocked_escape_min_signal_speed", 0, 0.2, 0.01, 2, "m/s"),
        num("start_blocked_escape_signal_max_age_s", 0, 300, 5, 0, "s"),
    ],
};

export const ALL_FIELD_GROUPS: SettingsFieldGroup[] = [
    YAW_LOOP_GROUP,
    CHARGE_LIMITS_GROUP,
    LOCALIZATION_GUARD_GROUP,
    DOCK_DETECTION_GROUP,
    DOCK_CALIBRATION_GROUP,
    TURN_SPEED_GROUP,
    REVERSE_ESCAPE_GROUP,
    AREA_RECORDING_GROUP,
    BEHAVIOR_TREE_GROUP,
    START_ESCAPE_GROUP,
];

export const groupKeys = (group: SettingsFieldGroup): string[] =>
    group.fields.map((field) => field.key);
