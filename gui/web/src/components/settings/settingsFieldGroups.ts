// Declarative field groups rendered by SettingsFieldCard.
//
// Labels and tooltips come from i18n (`settingsFields.<key>.label|tooltip`),
// group headings from `settingsFieldGroups.<id>.title|description`. Defaults are
// NOT declared here: they live in asserts/mower_config.schema.json (pinned to
// the ROS2 template by schema_template_parity_test.go). The min/max below are
// input guards only and mirror the clamps the consumers apply themselves.

import { appliesToBackend, HardwareBackend } from "../../constants/hardwareBackends.ts";

type FieldBase = {
    key: string;
    /** Hardware backends this field means something on (absent = all). */
    backends?: readonly HardwareBackend[];
};

export type SettingsFieldSpec = FieldBase &
    (
        | { kind: "switch" }
        | { kind: "select"; options: (number | string)[] }
        | { kind: "text"; placeholder?: string }
        | {
              kind: "number";
              min: number;
              max: number;
              step: number;
              precision: number;
              unit?: string;
          }
    );

export interface SettingsFieldGroup {
    id: string;
    /** Physical-motion / safety-relevant group: renders a warning banner. */
    isSafetyRelevant?: boolean;
    /** Hardware backends this group means something on (absent = all). */
    backends?: readonly HardwareBackend[];
    fields: SettingsFieldSpec[];
}

/**
 * The group as shown on `backend`: null when it does not apply there, else
 * only its fields that do (a STM32-only trip delay is meaningless behind the
 * OpenMower LowLevel board, which has no such setting).
 */
export const groupForBackend = (
    group: SettingsFieldGroup,
    backend: HardwareBackend,
): SettingsFieldGroup | null => {
    if (!appliesToBackend(group.backends, backend)) return null;
    const fields = group.fields.filter((f) => appliesToBackend(f.backends, backend));
    return fields.length > 0 ? { ...group, fields } : null;
};

const num = (
    key: string,
    min: number,
    max: number,
    step: number,
    precision: number,
    unit?: string,
    backends?: readonly HardwareBackend[],
): SettingsFieldSpec => ({ key, kind: "number", min, max, step, precision, unit, backends });

/** Settings only the Mowgli STM32 firmware implements. */
const STM32_ONLY: readonly HardwareBackend[] = ["mowgli"];

export const YAW_LOOP_GROUP: SettingsFieldGroup = {
    id: "yawLoop",
    isSafetyRelevant: true,
    backends: STM32_ONLY,
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
    isSafetyRelevant: true,
    fields: [
        num("max_charge_voltage", 25.2, 29.4, 0.1, 1, "V"),
        num("max_charge_current", 0.1, 1.2, 0.1, 1, "A"),
    ],
};

// Bounds = the firmware's absolute envelope (fw_param_catalog.h). The
// firmware coerces anything outside it and reports what it applied
// (FirmwareParamsCard). On OpenMower the bridge feeds the lift delays to the
// LowLevel board (lift / tilt period) under the SAME envelope
// (lowlevel_config.hpp); the board has no counterpart for the others (its
// stop button is a fixed 20 ms, and v1 has no IMU tilt e-stop).
export const FIRMWARE_SAFETY_GROUP: SettingsFieldGroup = {
    id: "firmwareSafety",
    isSafetyRelevant: true,
    fields: [
        num("max_mps", 0.1, 0.6, 0.01, 2, "m/s"),
        num("one_wheel_lift_emergency_ms", 10, 5000, 10, 0, "ms", ["mowgli", "openmower"]),
        num("both_wheels_lift_emergency_ms", 10, 3000, 10, 0, "ms", ["mowgli", "openmower"]),
        num("tilt_emergency_ms", 10, 1000, 10, 0, "ms", STM32_ONLY),
        num("stop_button_emergency_ms", 10, 250, 10, 0, "ms", STM32_ONLY),
        num("play_button_clear_emergency_ms", 500, 10000, 100, 0, "ms", STM32_ONLY),
        num("imu_inclination_threshold", 44, 64, 1, 0, undefined, STM32_ONLY),
    ],
};

// OpenMower v1 electronics: how the LowLevel board and the three xESC are
// wired. Defaults come from the installer (docker/.env) until set here.
export const OPENMOWER_WIRING_GROUP: SettingsFieldGroup = {
    id: "openmowerWiring",
    isSafetyRelevant: true,
    backends: ["openmower"],
    fields: [
        { key: "openmower_xesc_type", kind: "select", options: ["xesc_mini", "xesc_2040"] },
        { key: "openmower_ll_port", kind: "text", placeholder: "/dev/ttyAMA0" },
        { key: "openmower_xesc_left_port", kind: "text", placeholder: "/dev/ttyAMA5" },
        { key: "openmower_xesc_right_port", kind: "text", placeholder: "/dev/ttyAMA3" },
        { key: "openmower_xesc_mow_port", kind: "text", placeholder: "/dev/ttyAMA4" },
        { key: "openmower_emergency_input_config", kind: "text", placeholder: "!L,!L,S,S" },
    ],
};

// The bridge's host-side wheel loop (the xESC only takes a duty cycle). The
// STM32's wheel_pid_* gains are PWM counts and do not transfer.
export const OPENMOWER_WHEEL_LOOP_GROUP: SettingsFieldGroup = {
    id: "openmowerWheelLoop",
    isSafetyRelevant: true,
    backends: ["openmower"],
    fields: [
        { key: "openmower_wheel_loop_enabled", kind: "switch" },
        num("openmower_wheel_duty_per_mps", 0.1, 5, 0.05, 2),
        num("openmower_wheel_kp", 0, 10, 0.05, 2),
        num("openmower_wheel_ki", 0, 50, 0.1, 2),
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
    FIRMWARE_SAFETY_GROUP,
    LOCALIZATION_GUARD_GROUP,
    DOCK_DETECTION_GROUP,
    DOCK_CALIBRATION_GROUP,
    TURN_SPEED_GROUP,
    REVERSE_ESCAPE_GROUP,
    AREA_RECORDING_GROUP,
    BEHAVIOR_TREE_GROUP,
    START_ESCAPE_GROUP,
    OPENMOWER_WIRING_GROUP,
    OPENMOWER_WHEEL_LOOP_GROUP,
];

export const groupKeys = (group: SettingsFieldGroup): string[] =>
    group.fields.map((field) => field.key);
