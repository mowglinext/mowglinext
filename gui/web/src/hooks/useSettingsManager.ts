import {validAdvancedKeyName, type AdvancedKeyError} from '../components/settings/advancedSettings';
import type {PersistedDockPose} from '../utils/robotEditorContract';
import {DOCK_VISUAL_KEYS} from "../components/robot/dockGeometry";
import {materializeLegacyCasterPosition} from "../utils/casterPosition";
import { useCallback, useEffect, useEffectEvent, useMemo, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { useApi } from "./useApi.ts";
import { App } from "antd";
import { dirtyKeysRequireGpsRestart, restartGps } from "../utils/containers.ts";
import { useContainerRestart } from "./useContainerRestart.ts";
import { getQuaternionFromHeading } from "../utils/map.tsx";
import { ContentType } from "../api/Api.ts";
import { matchesSettingSearch, settingSearchText } from "../utils/settingsSearch.ts";
import { valuesMatch } from "../utils/settingsValues.ts";
import {
    AREA_RECORDING_GROUP,
    BEHAVIOR_TREE_GROUP,
    CHARGE_LIMITS_GROUP,
    DOCK_CALIBRATION_GROUP,
    DOCK_DETECTION_GROUP,
    FIRMWARE_SAFETY_GROUP,
    LOCALIZATION_GUARD_GROUP,
    OPENMOWER_WHEEL_LOOP_GROUP,
    OPENMOWER_WIRING_GROUP,
    REVERSE_ESCAPE_GROUP,
    START_ESCAPE_GROUP,
    TURN_SPEED_GROUP,
    YAW_LOOP_GROUP,
    groupKeys,
} from "../components/settings/settingsFieldGroups.ts";
import { planHardwareParameters, settingsSectionsForBackend } from "../constants/hardwareBackends.ts";
import { useHardwareBackend } from "./useHardwareBackend.ts";
import {isJsonObject} from '../utils/json';
import {apiErrorMessage, errorMessage} from "../utils/errors";
import type {SettingsValues} from "../types/settings.generated";


// The canonical YAML write precedes sidecar/env synchronization. A failed
// response may explicitly confirm persistence; never infer it from error text.
const persistedFailure = (response: unknown): string | undefined => {
    if (typeof response !== 'object' || response === null || !('error' in response)) return undefined;
    const body: unknown = response.error;
    return isJsonObject(body) && body.persisted === true && typeof body.error === 'string' ? body.error : undefined;
};

// A hardware node may accept a live write at a different value than requested
// (e.g. clamped). POST /params then lists it under `adjustments` and echoes the
// applied value; a refused write is an error response instead.
const adjustedLiveValue = (response: Record<string, unknown>, name: string): {applied: unknown} | undefined => {
    if (!Array.isArray(response.adjustments)) return undefined;
    const match: unknown = response.adjustments.find(item => isJsonObject(item) && item.name === name);
    return isJsonObject(match) && 'applied_value' in match ? {applied: match.applied_value} : undefined;
};

/** A section that saves outside mowgli_robot.yaml but wants the page's Save button. */
export interface ExternalSaver {
    /** Number of pending edits (0 = clean). Feeds the Save button count. */
    dirtyCount: number;
    /** Keep the section's reconciliation owner alive until all writes settle. */
    pending?: boolean;
    /** Persist; resolve true on success, false after showing its own error. */
    save: () => Promise<boolean>;
    /** Drop pending edits (page-level Revert). */
    revert?: () => void;
}


export type SettingsSection =
    | "updates"
    | "appearance"
    | "hardware"
    | "drive_motor"
    | "ntrip"
    | "positioning"
    | "sensors"
    | "localization"
    | "mowing"
    | "docking"
    | "battery"
    | "safety"
    | "obstacles"
    | "navigation"
    | "weather"
    | "leds"
    | "mqtt"
    | "remote_access"
    | "notifications"
    | "advanced";


export type SectionMeta = {
    id: SettingsSection;
    label: string;
    icon: string;
    description: string;
    keys: string[];
};


const SECTION_DEFINITIONS: SectionMeta[] = [
    {
        id: "updates",
        label: "settingsSections.updates.label",
        icon: "cloud-sync",
        description: "settingsSections.updates.description",
        keys: [],
    },
    {
        id: "appearance",
        label: "settingsSections.appearance.label",
        icon: "bg-colors",
        description: "settingsSections.appearance.description",
        keys: [],
    },
    {
        id: "hardware",
        label: "settingsSections.hardware.label",
        icon: "tool",
        description: "settingsSections.hardware.description",
        keys: [
            "robot_name", "mower_model", "wheel_radius", "wheel_track", "wheel_width",
            "wheel_x_offset", "chassis_center_x", "chassis_length", "chassis_width",
            "mower_visual_style", "mower_visual_colour", ...DOCK_VISUAL_KEYS, "chassis_height", "chassis_z_offset", "chassis_mass_kg", "caster_radius", "caster_track", "caster_x_offset",
            "ticks_per_meter", "tool_width", "blade_radius", "blade_x_offset", "blade_y_offset",
            // The hardware backend's own wiring (ports, controller type), in
            // the same section: one place for "what this robot is made of".
            ...groupKeys(OPENMOWER_WIRING_GROUP),
        ],
    },
    {
        id: "drive_motor",
        label: "settingsSections.drive_motor.label",
        icon: "dashboard",
        description: "settingsSections.drive_motor.description",
        keys: [
            "wheel_pid_kp", "wheel_pid_ki", "wheel_pid_kd",
            "wheel_pid_integral_limit", "wheel_pid_pwm_per_mps",
            ...groupKeys(YAW_LOOP_GROUP),
            ...groupKeys(OPENMOWER_WHEEL_LOOP_GROUP),
        ],
    },
    {
        id: "ntrip",
        label: "settingsSections.ntrip.label",
        icon: "wifi",
        description: "settingsSections.ntrip.description",
        keys: [
            "ntrip_enabled", "ntrip_host", "ntrip_port",
            "ntrip_user", "ntrip_password", "ntrip_mountpoint",
        ],
    },
    {
        id: "positioning",
        label: "settingsSections.positioning.label",
        icon: "global",
        description: "settingsSections.positioning.description",
        keys: [
            "datum_lat", "datum_lon", "datum_alt",
            "gnss_receiver_family", "gnss_serial_device", "gnss_serial_baud",
            "gnss_config_baud", "gnss_execution_baud", "gnss_profile", "gnss_signal_profile",
            "gnss_profile_rate_hz", "gnss_signal_group",
            "gnss_unicore_pvt_algorithm", "gnss_unicore_rtk_reliability",
            "gnss_unicore_rtk_timeout_s", "gnss_unicore_dgps_timeout_s",
            "gps_wait_after_undock_sec", "gps_timeout_sec",
            "gnss_receiver_model",
        ],
    },
    {
        id: "sensors",
        label: "settingsSections.sensors.label",
        icon: "aim",
        description: "settingsSections.sensors.description",
        keys: [
            "lidar_enabled", "lidar_x", "lidar_y", "lidar_z", "lidar_yaw",
            "imu_x", "imu_y", "imu_z", "imu_yaw", "imu_pitch", "imu_roll",
            "gps_x", "gps_y", "gps_z",
            "dock_pose_yaw",
            "imu_cal_samples", "imu_cal_auto_rest_sec", "imu_cal_periodic_recal_sec",
        ],
    },
    {
        id: "localization",
        label: "settingsSections.localization.label",
        icon: "node-index",
        description: "settingsSections.localization.description",
        keys: [
            "use_lidar_map_anchor", "lidar_anchor_shadow_mode",
            "use_magnetometer",
            "enable_mag_cal", "declination_deg", "min_horizontal_uT", "mag_yaw_variance",
            ...groupKeys(LOCALIZATION_GUARD_GROUP),
        ],
    },
    {
        id: "mowing",
        label: "settingsSections.mowing.label",
        icon: "scissor",
        description: "settingsSections.mowing.description",
        keys: [
            // NOTE: path_spacing intentionally omitted — it is a dead knob. F2C
            // swath spacing = tool_width (coverage_server.operation_width); a
            // separate spacing value re-opens the swath-gap bug. The preview and
            // tool_width (Geometry section) are the real controls.
            // The outline_*/mow_angle_* knobs were removed: NO ros2 node reads
            // them (navigation.launch.py forwards only the keys below to
            // coverage_server, and the BT hardcodes mow_angle_deg=-1.0 "auto"),
            // so they were dead controls. swath_overlap (a real coverage_server
            // param) is surfaced here instead.
            "mowing_enabled", "blade_auto_reverse", "mowing_speed", "transit_speed",
            // Blade-load slowdown (FollowCoveragePath.blade_load_*, injected by
            // navigation.launch.py): slow the feed when the blade RPM sags.
            "blade_load_slowdown_enabled", "blade_load_rpm_full",
            "blade_load_rpm_min", "blade_load_min_speed_ratio",
            "headland_width", "num_headland_passes", "swath_overlap",
            "chassis_safety_inset", "min_turning_radius", "mow_direction", "mow_cross_hatch",
            "connector_max_headland_passes",
            ...groupKeys(TURN_SPEED_GROUP),
        ],
    },
    {
        id: "docking",
        label: "settingsSections.docking.label",
        icon: "home",
        description: "settingsSections.docking.description",
        keys: [
            "undock_distance", "undock_speed", "dock_approach_distance",
            "dock_max_retries", "dock_use_charger_detection",
            "dock_charging_threshold",
            "dock_approach_overshoot", "dock_pose_yaw_sigma_rad",
            ...groupKeys(DOCK_DETECTION_GROUP),
            ...groupKeys(DOCK_CALIBRATION_GROUP),
        ],
    },
    {
        id: "battery",
        label: "settingsSections.battery.label",
        icon: "thunderbolt",
        description: "settingsSections.battery.description",
        keys: [
            "battery_full_voltage", "battery_empty_voltage", "battery_critical_voltage",
            "battery_full_percent", "battery_low_percent", "battery_critical_percent",
            "battery_critical_recovery_percent", "battery_manual_resume_percent",
            "battery_charge_tail_current_a",
            ...groupKeys(CHARGE_LIMITS_GROUP),
        ],
    },
    {
        id: "safety",
        label: "settingsSections.safety.label",
        icon: "safety",
        description: "settingsSections.safety.description",
        keys: [
            // motor_temp_high_c / motor_temp_low_c REMOVED (issue #195): no
            // layer of the stack implements a thermal blade cutoff — the
            // firmware only measures and reports blade temperature. The real
            // surface is mowgli_monitoring's motor_temp_warn_c /
            // motor_temp_error_c diagnostics thresholds.
            // The two lift keys below stay CLAIMED by this section but are
            // deliberately NOT rendered (see SafetySection.tsx):
            // lift_recovery_mode suppresses the ROS2 lift emergency and
            // auto-releases the firmware latch, and the delay is inert unless
            // that mode is on. Listing them here is what keeps them out of
            // AdvancedSection's free-form editor, exactly as before.
            "lift_blade_resume_delay_sec", "lift_recovery_mode",
            ...groupKeys(FIRMWARE_SAFETY_GROUP),
        ],
    },
    {
        id: "obstacles",
        label: "settingsSections.obstacles.label",
        icon: "warning",
        description: "settingsSections.obstacles.description",
        keys: [
            "obstacle_inflation_radius", "max_obstacle_avoidance_distance",
            "obstacle_clearance_margin", "obstacle_detection_range_m",
            "obstacle_wait_timeout_s",
            "obstacle_margin", "obstacle_slowdown_ratio", "dig_obstacle_enabled",
            "dig_sensitivity",
            ...groupKeys(REVERSE_ESCAPE_GROUP),
        ],
    },
    {
        id: "navigation",
        label: "settingsSections.navigation.label",
        icon: "compass",
        description: "settingsSections.navigation.description",
        keys: [
            "xy_goal_tolerance", "yaw_goal_tolerance", "coverage_xy_tolerance",
            "progress_timeout_sec",
            "boundary_inner_margin_m", "dock_inner_margin_exempt_radius_m", "obstacle_transit_margin_m",
            ...groupKeys(AREA_RECORDING_GROUP),
            ...groupKeys(BEHAVIOR_TREE_GROUP),
            ...groupKeys(START_ESCAPE_GROUP),
        ],
    },
    {
        // Rain sensor behaviour + the IrriSense soil-moisture skip: both
        // decide whether the weather lets the robot mow.
        id: "weather",
        label: "settingsSections.weather.label",
        icon: "cloud",
        description: "settingsSections.weather.description",
        keys: ["rain_mode", "rain_delay_minutes", "rain_debounce_sec"],
    },
    {
        id: "leds",
        label: "settingsSections.leds.label",
        icon: "bulb",
        description: "settingsSections.leds.description",
        keys: [
            // Every led_* key is claimed here so none of them leaks into
            // AdvancedSection's free-form editor, where a raw SPI device path
            // or clock would be edited with no context.
            "led_enabled", "led_count", "led_spi_device", "led_spi_speed_hz",
            "led_brightness", "led_idle_scale", "led_refresh_hz",
            "led_low_battery_percent", "led_charge_full_percent",
            "led_charge_complete_timeout_s", "led_charge_complete_dim_scale",
            "led_charge_complete_indicator_count", "led_charge_complete_indicator_scale",
            "led_charge_complete_indicator_ids",
            "led_status_timeout_s", "led_keepalive_s", "led_device_retry_s",
        ],
    },
    {
        id: "mqtt",
        label: "settingsSections.mqtt.label",
        icon: "wifi",
        description: "settingsSections.mqtt.description",
        keys: [
            // Every mqtt_* key is claimed here so none of them leaks into
            // AdvancedSection's free-form editor — same rationale as "leds"
            // above (a raw broker password with no context).
            "mqtt_enabled", "mqtt_host", "mqtt_port", "mqtt_username",
            "mqtt_password", "mqtt_topic_prefix", "mqtt_use_ssl",
            "mqtt_home_assistant_discovery_enabled",
        ],
    },
    {
        id: "remote_access",
        label: "settingsSections.remote_access.label",
        icon: "global",
        description: "settingsSections.remote_access.description",
        // No yaml keys: the remote-access settings (auth key included) live
        // in the GUI's key-value DB and the section loads/saves them itself.
        keys: [],
    },
    {
        id: "notifications",
        label: "settingsSections.notifications.label",
        icon: "bell",
        description: "settingsSections.notifications.description",
        // Same as IrriSense: DB-backed, the section owns load/save.
        keys: [],
    },
    {
        id: "advanced",
        label: "settingsSections.advanced.label",
        icon: "code",
        description: "settingsSections.advanced.description",
        keys: [],
    },
];


const HIDDEN_FROM_ADVANCED = new Set([
    "dock_pose_x",
    "dock_pose_y",
    "slam_mode",
    "map_save_on_dock",
    "map_save_path",
    "gps_antenna_x",
    "gps_antenna_y",
    "gps_antenna_z",
    "automatic_mode",
    // Retired in issue #195: removed from the ROS2 template AND the GUI
    // schema because no node ever read them. Belt-and-braces for a robot
    // whose installed yaml still carries them between the schema removal
    // and the retired-key scrub that runs on the next Settings save
    // (gui/pkg/api/settings.go retiredParamKeys) — same rationale as
    // slam_mode / map_save_on_dock above: dead config that would silently
    // mislead anyone who edits it.
    "outline_passes",
    "outline_offset",
    "outline_overlap",
    "mow_angle_offset_deg",
    "mow_angle_increment_deg",
    "motor_temp_high_c",
    "motor_temp_low_c",
    // Also block retired localizer keys scrubbed by the YAML producer.
    "use_scan_matching",
    "use_loop_closure",
    "icp_max_iter",
    "icp_max_corresp_dist",
    "icp_source_subsample",
    "scan_min_inliers",
    "icp_sigma_xy_base",
    "icp_sigma_theta_base",
    "icp_max_rmse_m",
    "icp_max_delta_xy_m",
    "icp_max_delta_theta_rad",
    "icp_max_divergence_xy_m",
    "icp_max_divergence_theta_rad",
    "scan_yield_to_rtk",
    "scan_yield_timeout_s",
    "scan_yield_sigma_xy",
    "scan_yield_sigma_theta",
    "scan_yaw_sigma_floor_rad",
    "lc_max_dist_m",
    "lc_min_age_s",
    "lc_max_candidates",
    "lc_min_delta_m",
    "lc_min_delta_theta",
    "lc_max_rmse",
    "lc_sigma_xy",
    "lc_sigma_theta",
    "lc_skip_when_rtk_fixed",
    "lc_min_travel_m",
    "lc_min_interval_s",
    "lc_gps_sigma_ratio",
    "scan_retention_nodes",
    "lidar_map_half_extent_m",
    "ticks_per_revolution",
]);

export const useSettingsManager = () => {
    const { t } = useTranslation();
    const guiApi = useApi();
    const hardware = useHardwareBackend();
    const { notification } = App.useApp();
    const [savedValues, setSavedValues] = useState<SettingsValues>({});
    // A GPS write can have persisted even when its acknowledgement/readback fails.
    const dockCaptureUnconfirmed = useRef(false);
    const dockCapturePending = useRef(false);
    const settingsWritesInFlight = useRef(0);
    const [localValues, setLocalValues] = useState<SettingsValues>({});
    // Schema defaults = the GUI's source of "default value" for each key.
    // (The backend derives these from the JSON schema, which stands in for the
    // ROS2 package template it cannot read at runtime.) Used for the
    // per-field "reset to default" affordance and the overridden indicator.
    const [defaults, setDefaults] = useState<SettingsValues>({});
    const [loading, setLoading] = useState(true);
    const [saving, setSaving] = useState(false);
    const [restartRequired, setRestartRequired] = useState(() => {
        try { return sessionStorage.getItem('mowgli.pendingRosRestart') === 'true'; }
        catch { return false; }
    });
    useEffect(() => {
        try { sessionStorage.setItem('mowgli.pendingRosRestart', String(restartRequired)); }
        catch { /* Storage can be disabled in embedded browsers. */ }
    }, [restartRequired]);
    const acknowledgeRestart = useCallback(() => {
        try { sessionStorage.removeItem('mowgli.pendingRosRestart'); } catch { /* optional storage */ }
        setRestartRequired(false);
    }, []);
    // GPS restart skips the rosbridge readiness probe (ROS2 is unaffected).
    const gpsRestart = useContainerRestart({
        pendingLabel: t("settingsManager.gpsRestartPending"),
        successMessage: t("settingsManager.gpsRestartSuccess"),
        errorMessage: t("settingsManager.gpsRestartError"),
        skipReadinessProbe: true,
    });
    const [searchQuery, setSearchQuery] = useState("");
    const initialLoadDone = useRef(false);



    // Load values on mount
    const reportLoadError = useEffectEvent((error: unknown) => {
        notification.error({message: t('settingsSections.toasts.loadFailed'), description: errorMessage(error)});
    });

    const adoptInitialValues = useEffectEvent((data: SettingsValues) => {
        const baseline = savedValues;
        setSavedValues(data);
        setLocalValues(current => {
            const merged = {...data};
            for (const [key, value] of Object.entries(current)) {
                if (!valuesMatch(value, baseline[key])) merged[key] = value;
            }
            return merged;
        });
        initialLoadDone.current = true;
    });

    useEffect(() => {
        let active = true;
        void (async () => {
            try {
                setLoading(true);
                const res = await guiApi.settings.yamlList();
                if (!active) return;
                if (res.error) throw new Error(apiErrorMessage(res.error));
                const data = (res.data) || {};
                adoptInitialValues(data);
            } catch (e) {
                if (active) reportLoadError(e);
            } finally {
                if (active) setLoading(false);
            }
        })();
        return () => { active = false; };
    }, [guiApi.settings]);

    // The reset value follows the draft preset, including before Save.
    // Discard responses from an earlier selection to avoid stale defaults.
    const draftModel: unknown = localValues.mower_model;
    const draftMowerModel = typeof draftModel === "string" ? draftModel : undefined;

    useEffect(() => {
        if (loading) return;

        let active = true;
        setDefaults({});

        guiApi.request({
            path: "/settings/yaml/defaults",
            method: "GET",
            format: "json",
            query: draftMowerModel === undefined
                ? {}
                : {mower_model: draftMowerModel},
        }).then(res => {
            if (active && !res.error) {
                setDefaults((res.data as SettingsValues) || {});
            }
        }).catch(() => {
            /* defaults unavailable: reset affordance stays hidden */
        });

        return () => {
            active = false;
        };
    }, [loading, draftMowerModel, guiApi]);

    const handleChange = useCallback((key: string, value: unknown) => {
        setLocalValues((prev) => ({
            ...materializeLegacyCasterPosition(prev),
            [key]: value,
        }));
    }, []);

    const handleBulkChange = useCallback((changes: SettingsValues) => {
        setLocalValues((prev) => ({
            ...materializeLegacyCasterPosition(prev),
            ...changes,
        }));
    }, []);

    // hasDefault: the schema knows a default for this key (so a reset is
    // meaningful). isDefault: the current local value already equals that
    // default (tolerating int/float JSON churn, so 5 == 5.0). isOverridden:
    // has a default AND the local value differs from it — the operator has
    // pinned a non-default value that we should visually flag.
    // valuesMatch is imported from ../utils/settingsValues.ts (null-safe;
    // the old inline copy treated valuesMatch(null, 0) as true via Number(null)).
    const hasDefault = useCallback(
        (key: string): boolean => key in defaults,
        [defaults]
    );


    const isDefault = useCallback(
        (key: string): boolean =>
            key in defaults && valuesMatch(localValues[key], defaults[key]),
        [defaults, localValues]
    );


    const isOverridden = useCallback(
        (key: string): boolean =>
            key in defaults && !valuesMatch(localValues[key], defaults[key]),
        [defaults, localValues]
    );


    // resetToDefault reverts a field to its schema default in the local (unsaved)
    // state; the operator still presses Save to persist. On save the backend
    // prunes default-valued keys, so the installed config stays sparse.
    const resetToDefault = useCallback(
        (key: string) => {
            if (!(key in defaults)) return;
            setLocalValues((prev) => ({ ...materializeLegacyCasterPosition(prev), [key]: defaults[key] }));
        },
        [defaults]
    );


    // Dirty detection
    const dirtyKeys = useMemo(() => {
        const dirty = new Set<string>();
        for (const key of Object.keys(localValues)) {
            if (JSON.stringify(localValues[key]) !== JSON.stringify(savedValues[key])) {
                dirty.add(key);
            }
        }
        for (const key of Object.keys(savedValues)) {
            if (!(key in localValues)) {
                dirty.add(key);
            }
        }
        return dirty;
    }, [localValues, savedValues]);


    // External savers: sections whose settings do not live in mowgli_robot.yaml
    // (IrriSense keeps its token in the GUI DB) register here so the page's
    // ONE Save button covers them too. Refs hold the callbacks (no stale
    // closure in persistSettings); the state mirror only drives rendering.
    const externalSaversRef = useRef<Record<string, ExternalSaver>>({});
    const [externalDirtyCounts, setExternalDirtyCounts] = useState<Record<string, number>>({});
    const [externalPending, setExternalPending] = useState<Record<string, boolean>>({});
    const registerExternalSaver = useCallback((id: string, saver: ExternalSaver) => {
        externalSaversRef.current[id] = saver;
        setExternalDirtyCounts((prev) =>
            prev[id] === saver.dirtyCount ? prev : { ...prev, [id]: saver.dirtyCount }
        );
        setExternalPending(prev => prev[id] === !!saver.pending ? prev : {...prev, [id]: !!saver.pending});
    }, []);
    const unregisterExternalSaver = useCallback((id: string) => {
        delete externalSaversRef.current[id];
        setExternalDirtyCounts((prev) => {
            if (!(id in prev)) return prev;
            const next = { ...prev };
            delete next[id];
            return next;
        });
    }, []);
    const externalDirtyCount = useMemo(
        () => Object.values(externalDirtyCounts).reduce((a, b) => a + b, 0),
        [externalDirtyCounts]
    );
    const dirtyCount = dirtyKeys.size + externalDirtyCount;
    const isDirty = dirtyCount > 0;
    const isSectionPending = (sectionId: SettingsSection) =>
        externalPending[sectionId === "weather" ? "irrisense" : sectionId] ?? false;


    const isSectionDirty = useCallback(
        (sectionId: SettingsSection): boolean => {
            const section = SECTION_DEFINITIONS.find((s) => s.id === sectionId);
            if (!section) return false;
            // IrriSense is rendered within Weather but retains its saver ID.
            const saverId = section.id === "weather" ? "irrisense" : section.id;
            if ((externalDirtyCounts[saverId] ?? 0) > 0) return true;
            if (section.id === "advanced") {
                // Advanced section: any key not in other sections
                const knownKeys = new Set(
                    SECTION_DEFINITIONS.filter((s) => s.id !== "advanced").flatMap((s) => s.keys)
                );
                for (const key of dirtyKeys) {
                    if (!knownKeys.has(key)) return true;
                }
                return false;
            }
            return section.keys.some((k) => dirtyKeys.has(k));
        },
        [dirtyKeys, externalDirtyCounts]
    );


    const persistSettings = useCallback(async (options?: { forceGpsRestart?: boolean }) => {
        let yamlPersisted = false;
        settingsWritesInFlight.current++;
        try {
            if (dockCapturePending.current) {
                notification.warning({message: t('settingsSections.toasts.dockCapturePending')});
                return;
            }
            setSaving(true);
            const forceGpsRestart = options?.forceGpsRestart ?? false;
            // Capture which keys were dirty BEFORE we mark them saved, so we
            // can decide whether GPS needs an auto-restart and whether we
            // need to refresh map_server's dock pose at runtime.
            const gpsDirty = dirtyKeysRequireGpsRestart(dirtyKeys);
            const shouldRestartGps = forceGpsRestart || gpsDirty;
            const dockDirty =
                dirtyKeys.has("dock_pose_x") ||
                dirtyKeys.has("dock_pose_y") ||
                dirtyKeys.has("dock_pose_yaw");
            if (dockDirty && dockCaptureUnconfirmed.current) {
                notification.warning({message: t('settingsSections.toasts.dockCaptureUnconfirmed')});
                return;
            }
            // Only runtime-confirmed routes may be pushed live. OpenMower and
            // pending MAVROS routes must be persisted and applied on restart.
            const plan = planHardwareParameters(dirtyKeys, localValues, hardware);
            const liveParameters = plan.parameters;
            const liveHardwareKeys = plan.liveKeys;
            const liveHardwareDirty = liveHardwareKeys.size > 0;
            // Appearance-only changes are persisted without restarting ROS;
            // geometry and unavailable runtime routes still require a restart.
            const requiresRosRestart = [...dirtyKeys].some((key) =>
                !liveHardwareKeys.has(key) && !DOCK_VISUAL_KEYS.includes(key) &&
                key !== "mower_visual_style" && key !== "mower_visual_colour",
            );
            const hasDirtyChanges = dirtyKeys.size > 0;
            const externalSavers = Object.values(externalSaversRef.current).filter((x) => x.dirtyCount > 0);
            if (!hasDirtyChanges && !shouldRestartGps && externalSavers.length === 0) {
                notification.info({
                    message: t("settingsSections.toasts.noChanges"),
                });
                return;
            }
            // Only send keys the user actually changed. The backend merges
            // payload over the on-disk YAML, so omitting unchanged keys
            // preserves anything other processes may have written between
            // load and save (e.g. dock_pose_x/y from the calibration
            // service or the map "set dock pose" action).
            const dirtyPayload: SettingsValues = {};
            for (const key of dirtyKeys) {
                if (key in localValues) {
                    const value = localValues[key];
                    if (typeof value === 'number' && !Number.isFinite(value)) throw new Error(t('settingsSections.toasts.invalidNumericValue'));
                    dirtyPayload[key] = value;
                }
            }
            if (hasDirtyChanges) {
                let synchronizationFailure: string | undefined;
                try {
                    const res = await guiApi.settings.yamlCreate(dirtyPayload);
                    synchronizationFailure = persistedFailure(res);
                    if (res.error && synchronizationFailure === undefined) throw new Error(apiErrorMessage(res.error));
                } catch (e) {
                    synchronizationFailure = persistedFailure(e);
                    if (synchronizationFailure === undefined) throw e;
                }
                yamlPersisted = true;
                // AdvancedSection deletes surface as null-valued keys. The
                // backend drops null keys from the YAML on save, so prune them
                // from local state too — otherwise the deleted rows reappear.
                const pruned: SettingsValues = {};
                for (const [key, value] of Object.entries(localValues)) {
                    if (value !== null) {
                        pruned[key] = value;
                    }
                }
                setSavedValues(pruned);
                setLocalValues(previous => {
                    const next = {...previous};
                    for (const [key, value] of Object.entries(localValues)) {
                        if (value === null && previous[key] === null) delete next[key];
                    }
                    return next;
                });
                if (requiresRosRestart || synchronizationFailure !== undefined) setRestartRequired(true);
                if (synchronizationFailure !== undefined) {
                    notification.warning({message: t('settingsSections.toasts.savedRuntimeFailed'), description: synchronizationFailure});
                    return;
                }
                if (plan.deferredState !== undefined) {
                    notification.warning({message: t('settingsSections.toasts.saved'),
                        description: t(`settingsSections.toasts.routing.${plan.deferredState}`)});
                } else {
                    notification.success({
                        message: t("settingsSections.toasts.saved"),
                        description: shouldRestartGps
                            ? t("settingsSections.toasts.savedGpsRestartDescription")
                            : t(requiresRosRestart ? "settingsSections.toasts.savedDescription" : liveHardwareDirty ? "settingsSections.toasts.savedApplyingLive" : "settingsSections.toasts.savedDisplaySettings"),
                    });
                }
            } else if (shouldRestartGps) {
                notification.info({
                    message: t("settingsSections.toasts.restartingGps"),
                });
            }
            // Persisting YAML does not guarantee the runtime accepted a live
            // parameter update. Only send routes the active backend reports as available.
            if (hasDirtyChanges && liveParameters.length > 0) {
                try {
                    const liveResponse = await guiApi.request({
                        path: "/params",
                        method: "POST",
                        type: ContentType.Json,
                        format: "json",
                        body: { parameters: liveParameters },
                    });

                    if (liveResponse.error) throw new Error(apiErrorMessage(liveResponse.error));
                    const raw: unknown = liveResponse.data;
                    if (!isJsonObject(raw) || !Array.isArray(raw.parameters)) throw new Error(t('settingsSections.toasts.runtimeUnconfirmed'));
                    const echoed: unknown[] = raw.parameters;
                    const expectedValue = (want: {name: string; value: unknown}) =>
                        adjustedLiveValue(raw, want.name)?.applied ?? want.value;
                    if (!liveParameters.every(want => echoed.some(item => isJsonObject(item)
                        && item.name === want.name && item.value === expectedValue(want)))) throw new Error(t('settingsSections.toasts.runtimeUnconfirmed'));
                    const adjusted = liveParameters.flatMap(want => {
                        const adjustment = adjustedLiveValue(raw, want.name);
                        return adjustment === undefined ? [] : [t('settingsSections.toasts.liveAdjustedItem',
                            {name: want.name, applied: String(adjustment.applied), requested: String(want.value)})];
                    });
                    if (adjusted.length > 0) {
                        notification.warning({message: t('settingsSections.toasts.liveAdjusted'), description: adjusted.join('; ')});
                    } else {
                        notification.success({message: t('settingsSections.toasts.liveApplied')});
                    }
                } catch (e) {
                    setRestartRequired(true);
                    notification.warning({
                        message: t("settingsSections.toasts.drivePidUpdateFailed"),
                        description: errorMessage(e) ??
                            t("settingsSections.toasts.drivePidUpdateFailedDescription"),
                    });
                }
            }
            // Sections that persist outside the yaml (IrriSense) save through
            // their own endpoint; each reports its own toast on failure.
            for (const saver of externalSavers) {
                if (!(await saver.save())) return;
            }
            // Auto-restart the GPS container when GPS/NTRIP fields changed —
            // ROS2 keeps running, the user just sees RTCM stop briefly. This
            // unblocks the "Set Datum from current GPS" path in onboarding,
            // which silently fails when the old (un-credentialled) GPS
            // container is still running.
            if (shouldRestartGps) {
                await gpsRestart.run(() => restartGps(guiApi));
            }
            // Push the new dock pose into map_server at runtime so the
            // dock body / corridor / exclusion polygons + keepout mask
            // get rebuilt without a restart. yamlCreate above persisted
            // dock_pose_* to mowgli_robot.yaml; this call is the
            // counterpart to the old in-process auto-persist behavior
            // that lived in RobotComponentEditor's "Set dock pose"
            // button (was triggering before the user clicked save, so
            // the save button never glowed).
            if (hasDirtyChanges && dockDirty) {
                const px = Number(localValues["dock_pose_x"] ?? 0);
                const py = Number(localValues["dock_pose_y"] ?? 0);
                const yawRad = Number(localValues["dock_pose_yaw"] ?? 0);
                const q = getQuaternionFromHeading(yawRad);
                try {
                    await guiApi.mowglinext.mapDockingCreate({
                        docking_pose: {
                            orientation: {x: q.x!, y: q.y!, z: q.z!, w: q.w!},
                            position: {x: px, y: py, z: 0},
                        },
                        // Manual settings edit: use the typed dock_pose_x/y as-is
                        // (operator entered the value explicitly — no GPS override).
                        use_gps_position: false,
                        // Honour the typed heading (SetDockingPoint yaw_source REQUEST=1);
                        // an operator-entered yaw is explicit, never circular.
                        yaw_source: 1,
                    });
                } catch (e) {
                    notification.warning({
                        message: t("settingsSections.toasts.dockRefreshFailed"),
                        description: errorMessage(e) ??
                            t("settingsSections.toasts.dockRefreshFailedDescription"),
                    });
                }
            }

        } catch (e) {
            if (yamlPersisted) {
                setRestartRequired(true);
                notification.warning({message: t('settingsSections.toasts.savedRuntimeFailed'), description: errorMessage(e)});
            } else {
                notification.error({message: t("settingsSections.toasts.saveFailed"), description: errorMessage(e)});
            }
        } finally {
            settingsWritesInFlight.current--;
            setSaving(false);
        }
    }, [
        localValues,
        dirtyKeys,
        guiApi,
        notification,
        gpsRestart,
        t,
        hardware,
    ]);


    const savePartialValues = useCallback(async (
        partialValues: SettingsValues,
        options?: {
            successMessage?: string;
            successDescription?: string;
            errorMessage?: string;
            silentSuccess?: boolean;
            markRestartRequired?: boolean;
        },
    ) => {
        settingsWritesInFlight.current++;
        try {
            if (dockCapturePending.current) {
                notification.warning({message: t('settingsSections.toasts.dockCapturePending')});
                return false;
            }
            if (dockCaptureUnconfirmed.current && ['dock_pose_x', 'dock_pose_y', 'dock_pose_yaw'].some(key => key in partialValues)) {
                notification.warning({message: t('settingsSections.toasts.dockCaptureUnconfirmed')});
                return false;
            }
            const changedPayload: SettingsValues = {};
            for (const [key, value] of Object.entries(partialValues)) {
                if (JSON.stringify(value) !== JSON.stringify(savedValues[key])) {
                    changedPayload[key] = value;
                }
            }


            if (Object.keys(changedPayload).length === 0) {
                return true;
            }


            setSaving(true);
            let synchronizationFailure: string | undefined;
            try {
                const res = await guiApi.settings.yamlCreate(changedPayload);
                synchronizationFailure = persistedFailure(res);
                if (res.error && synchronizationFailure === undefined) throw new Error(apiErrorMessage(res.error));
            } catch (e) {
                synchronizationFailure = persistedFailure(e);
                if (synchronizationFailure === undefined) throw e;
            }


            setSavedValues((prev) => ({ ...prev, ...changedPayload }));
            setLocalValues((prev) => ({ ...prev, ...changedPayload }));


            if (synchronizationFailure !== undefined || (options?.markRestartRequired ?? true)) {
                setRestartRequired(true);
            }


            if (synchronizationFailure !== undefined) {
                notification.warning({message: t('settingsSections.toasts.savedRuntimeFailed'), description: synchronizationFailure});
                return false;
            }
            if (!options?.silentSuccess) {
                notification.success({
                    message: options?.successMessage ?? t("settingsSections.toasts.saved"),
                    description: options?.successDescription,
                });
            }


            return true;
        } catch (e) {
            notification.error({
                message: options?.errorMessage ?? t("settingsSections.toasts.saveFailed"),
                description: errorMessage(e),
            });
            return false;
        } finally {
            settingsWritesInFlight.current--;
            setSaving(false);
        }
    }, [guiApi, notification, savedValues, t]);
    const acceptPersistedValues = useCallback((persistedValues: SettingsValues) => {
        setSavedValues((prev) => ({ ...prev, ...persistedValues }));
        setLocalValues((prev) => ({ ...prev, ...persistedValues }));
    }, []);


    const beginDockCapture = useCallback(() => {
        if (settingsWritesInFlight.current > 0 || dockCapturePending.current) {
            notification.warning({message: t('settingsSections.toasts.dockCaptureWhileSaving')});
            return false;
        }
        dockCaptureUnconfirmed.current = true;
        dockCapturePending.current = true;
        return true;
    }, [notification, t]);
    const finishDockCapture = useCallback(() => {dockCapturePending.current = false;}, []);
    const acceptPersistedDockPose = useCallback((pose: PersistedDockPose) => {
        acceptPersistedValues(pose);
        dockCaptureUnconfirmed.current = false;
    }, [acceptPersistedValues]);

    const save = useCallback(async () => {
        await persistSettings();
    }, [persistSettings]);


    const saveAndRestartGps = useCallback(async () => {
        await persistSettings({ forceGpsRestart: true });
    }, [persistSettings]);


    const revert = useCallback(() => {
        setLocalValues({ ...savedValues });
        for (const saver of Object.values(externalSaversRef.current)) {
            saver.revert?.();
        }
    }, [savedValues]);


    // Get keys that don't belong to any defined section.
    // dock_pose_x/y/yaw are excluded because they are written by the
    // calibration service and the "set dock pose" GUI action, not by
    // free-form numeric input — even though they live in mowgli_robot.yaml.
    // slam_mode / map_save_* are leftovers from the slam_toolbox era that
    // ended with the FusionCore → iSAM2 migration; they survive in old
    // YAMLs as dead config that would silently mislead anyone who edits
    // them. gps_antenna_* is the legacy name for what is now gps_x/y/z —
    // we read/write the new names and hide the old ones so operators don't
    // edit a key that has no consumer. automatic_mode is OpenMower legacy
    // only consumed by the migration script, not by ROS2 itself.
    const protectedAdvancedKeys = useMemo(() => new Set([
        ...SECTION_DEFINITIONS.filter(section => section.id !== 'advanced').flatMap(section => section.keys),
        ...HIDDEN_FROM_ADVANCED,
    ]), []);
    const validateAdvancedKey = useCallback((key: string): AdvancedKeyError | undefined => {
        if (!validAdvancedKeyName(key)) return 'invalidName';
        if (protectedAdvancedKeys.has(key)) return 'protectedKey';
        if (Object.prototype.hasOwnProperty.call(localValues, key)) return 'duplicateKey';
        return undefined;
    }, [localValues, protectedAdvancedKeys]);
    const advancedKeys = useMemo(() => Object.keys(localValues).filter(key => !protectedAdvancedKeys.has(key)),
        [localValues, protectedAdvancedKeys]);


    // Search filtering
    const matchesSearch = useCallback(
        (key: string, label?: string): boolean => {
            if (!searchQuery) return true;
            return matchesSettingSearch(searchQuery, key, label ?? '', ...settingSearchText(key, t));
        },
        [searchQuery, t]
    );


    return {
        sections: settingsSectionsForBackend(SECTION_DEFINITIONS, hardware.backend).map(section =>
            hardware.backend === "mavros" && section.id === "drive_motor"
                ? { ...section, description: "settingsDriveMotor.odometry.sectionDescription" }
                : section
        ),
        hardwareBackend: hardware,
        backendDefaultOverrides: hardware.defaultOverrides,
        values: localValues,
        savedValues,
        defaults,
        loading: loading || hardware.loading,
        saving,
        gpsRestarting: gpsRestart.pending,
        isDirty,
        dirtyKeys,
        dirtyCount,
        registerExternalSaver,
        unregisterExternalSaver,
        restartRequired,
        acknowledgeRestart,
        searchQuery,
        advancedKeys,
        validateAdvancedKey,
        setSearchQuery,
        handleChange,
        handleBulkChange,
        hasDefault,
        isDefault,
        isOverridden,
        resetToDefault,
        isSectionDirty,
        isSectionPending,
        matchesSearch,
        save,
        savePartialValues,
        saveAndRestartGps,
        acceptPersistedValues,
        acceptPersistedDockPose,
        beginDockCapture,
        finishDockCapture,
        revert,
    };
};
