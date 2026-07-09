import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { useApi } from "./useApi.ts";
import { App } from "antd";
import { dirtyKeysRequireGpsRestart, restartGps } from "../utils/containers.ts";
import { useContainerRestart } from "./useContainerRestart.ts";
import { getQuaternionFromHeading } from "../utils/map.tsx";
import { ContentType } from "../api/Api.ts";

export type SettingsSection =
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
    | "navigation"
    | "rain"
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
        id: "hardware",
        label: "settingsSections.hardware.label",
        icon: "tool",
        description: "settingsSections.hardware.description",
        keys: [
            "mower_model", "wheel_radius", "wheel_track", "wheel_width",
            "wheel_x_offset", "chassis_center_x", "chassis_length", "chassis_width",
            "chassis_height", "chassis_mass_kg", "caster_radius", "caster_track",
            "ticks_per_meter", "tool_width", "blade_radius",
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
        ],
    },
    {
        id: "localization",
        label: "settingsSections.localization.label",
        icon: "node-index",
        description: "settingsSections.localization.description",
        keys: [
            "use_scan_matching", "use_loop_closure",
            "use_magnetometer",
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
            "mowing_enabled", "mowing_speed", "transit_speed",
            "headland_width", "num_headland_passes", "swath_overlap",
            "chassis_safety_inset", "min_turning_radius", "mow_direction",
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
            "battery_critical_recovery_percent",
        ],
    },
    {
        id: "safety",
        label: "settingsSections.safety.label",
        icon: "safety",
        description: "settingsSections.safety.description",
        keys: [
            "motor_temp_high_c", "motor_temp_low_c",
            "max_obstacle_avoidance_distance",
            "lift_blade_resume_delay_sec", "lift_recovery_mode",
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
        ],
    },
    {
        id: "rain",
        label: "settingsSections.rain.label",
        icon: "cloud",
        description: "settingsSections.rain.description",
        keys: ["rain_mode", "rain_delay_minutes", "rain_debounce_sec"],
    },
    {
        id: "advanced",
        label: "settingsSections.advanced.label",
        icon: "code",
        description: "settingsSections.advanced.description",
        keys: [],
    },
];

export const useSettingsManager = () => {
    const { t } = useTranslation();
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [savedValues, setSavedValues] = useState<Record<string, any>>({});
    const [localValues, setLocalValues] = useState<Record<string, any>>({});
    // Schema defaults = the GUI's source of "default value" for each key.
    // (The backend derives these from the JSON schema, which stands in for the
    // ROS2 package template it cannot read at runtime.) Used for the
    // per-field "reset to default" affordance and the overridden indicator.
    const [defaults, setDefaults] = useState<Record<string, any>>({});
    const [loading, setLoading] = useState(true);
    const [saving, setSaving] = useState(false);
    const [restartRequired, setRestartRequired] = useState(false);
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
    useEffect(() => {
        (async () => {
            try {
                setLoading(true);
                const res = await guiApi.settings.yamlList();
                if (res.error) throw new Error((res.error as any).error);
                const data = (res.data as Record<string, any>) || {};
                setSavedValues(data);
                setLocalValues(data);
                // Best-effort: the reset-to-default UI degrades gracefully
                // (no reset icons) if this fails, so it must not block load.
                try {
                    const defRes = await guiApi.request({
                        path: "/settings/yaml/defaults",
                        method: "GET",
                        format: "json",
                    });
                    if (!defRes.error) {
                        setDefaults((defRes.data as Record<string, any>) || {});
                    }
                } catch {
                    /* defaults unavailable — reset affordance hidden */
                }
                initialLoadDone.current = true;
            } catch (e: any) {
                notification.error({
                    message: t("settingsSections.toasts.loadFailed"),
                    description: e.message,
                });
            } finally {
                setLoading(false);
            }
        })();
    }, []);

    const handleChange = useCallback((key: string, value: any) => {
        setLocalValues((prev) => ({ ...prev, [key]: value }));
    }, []);

    const handleBulkChange = useCallback((changes: Record<string, any>) => {
        setLocalValues((prev) => ({ ...prev, ...changes }));
    }, []);

    // hasDefault: the schema knows a default for this key (so a reset is
    // meaningful). isDefault: the current local value already equals that
    // default (tolerating int/float JSON churn, so 5 == 5.0). isOverridden:
    // has a default AND the local value differs from it — the operator has
    // pinned a non-default value that we should visually flag.
    const valuesMatch = (a: any, b: any): boolean => {
        if (typeof a === "number" && typeof b === "number") return a === b;
        // Number vs numeric-string / int-vs-float from JSON round-trips.
        const an = Number(a);
        const bn = Number(b);
        if (!Number.isNaN(an) && !Number.isNaN(bn) &&
            (typeof a !== "string" || a.trim() !== "") &&
            (typeof b !== "string" || b.trim() !== "")) {
            return an === bn;
        }
        return JSON.stringify(a) === JSON.stringify(b);
    };

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
            setLocalValues((prev) => ({ ...prev, [key]: defaults[key] }));
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

    const isDirty = dirtyKeys.size > 0;

    const isSectionDirty = useCallback(
        (sectionId: SettingsSection): boolean => {
            const section = SECTION_DEFINITIONS.find((s) => s.id === sectionId);
            if (!section) return false;
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
        [dirtyKeys]
    );

    const persistSettings = useCallback(async (options?: { forceGpsRestart?: boolean }) => {
        try {
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
            const driveKeys = [
                "wheel_pid_kp", "wheel_pid_ki", "wheel_pid_kd",
                "wheel_pid_integral_limit", "wheel_pid_pwm_per_mps",
            ];
            const liveHardwareKeys = ["ticks_per_meter", ...driveKeys];
            const liveHardwareDirty = liveHardwareKeys.some((k) => dirtyKeys.has(k));
            const hasDirtyChanges = dirtyKeys.size > 0;
            if (!hasDirtyChanges && !shouldRestartGps) {
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
            const dirtyPayload: Record<string, any> = {};
            for (const key of dirtyKeys) {
                if (key in localValues) {
                    dirtyPayload[key] = localValues[key];
                }
            }
            if (hasDirtyChanges) {
                const res = await guiApi.settings.yamlCreate(dirtyPayload);
                if (res.error) throw new Error((res.error as any).error);
                setSavedValues({ ...localValues });
                setRestartRequired(true);
                notification.success({
                    message: t("settingsSections.toasts.saved"),
                    description: shouldRestartGps
                        ? t("settingsSections.toasts.savedGpsRestartDescription")
                        : t("settingsSections.toasts.savedDescription"),
                });
            } else if (shouldRestartGps) {
                notification.info({
                    message: t("settingsSections.toasts.restartingGps"),
                });
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
                    });
                } catch (e: any) {
                    notification.warning({
                        message: t("settingsSections.toasts.dockRefreshFailed"),
                        description: e?.message ??
                            t("settingsSections.toasts.dockRefreshFailedDescription"),
                    });
                }
            }
            // Push live-tunable wheel/drive parameters to the running
            // hardware_bridge node so they take effect immediately (no
            // restart). yamlCreate above persisted them to mowgli_robot.yaml
            // for the next boot; this sets the live ROS params too. The
            // hardware_bridge callback applies ticks_per_meter in-process and
            // re-sends the full drive runtime tuning packet to the STM32 firmware.
            if (hasDirtyChanges && liveHardwareDirty) {
                const parameters = liveHardwareKeys
                    .filter((k) => dirtyKeys.has(k) && k in localValues)
                    .map((k) => ({ name: `hardware_bridge.${k}`, value: Number(localValues[k]) }));
                if (parameters.length > 0) {
                    try {
                        await guiApi.request({
                            path: "/params",
                            method: "POST",
                            type: ContentType.Json,
                            format: "json",
                            body: { parameters },
                        });
                    } catch (e: any) {
                        notification.warning({
                            message: "Settings saved, but live wheel/drive update failed",
                            description: e?.message ??
                                "Restart ROS2 to apply the new wheel and drive-motor settings.",
                        });
                    }
                }
            }
        } catch (e: any) {
            notification.error({
                message: t("settingsSections.toasts.saveFailed"),
                description: e.message,
            });
        } finally {
            setSaving(false);
        }
    }, [localValues, dirtyKeys, guiApi, notification, gpsRestart, t]);

    const savePartialValues = useCallback(async (
        partialValues: Record<string, any>,
        options?: {
            successMessage?: string;
            successDescription?: string;
            errorMessage?: string;
            silentSuccess?: boolean;
            markRestartRequired?: boolean;
        },
    ) => {
        try {
            const changedPayload: Record<string, any> = {};
            for (const [key, value] of Object.entries(partialValues)) {
                if (JSON.stringify(value) !== JSON.stringify(savedValues[key])) {
                    changedPayload[key] = value;
                }
            }

            if (Object.keys(changedPayload).length === 0) {
                return true;
            }

            setSaving(true);
            const res = await guiApi.settings.yamlCreate(changedPayload);
            if (res.error) {
                throw new Error((res.error as any).error);
            }

            setSavedValues((prev) => ({ ...prev, ...changedPayload }));
            setLocalValues((prev) => ({ ...prev, ...changedPayload }));

            if (options?.markRestartRequired ?? true) {
                setRestartRequired(true);
            }

            if (!options?.silentSuccess) {
                notification.success({
                    message: options?.successMessage ?? t("settingsSections.toasts.saved"),
                    description: options?.successDescription,
                });
            }

            return true;
        } catch (e: any) {
            notification.error({
                message: options?.errorMessage ?? t("settingsSections.toasts.saveFailed"),
                description: e.message,
            });
            return false;
        } finally {
            setSaving(false);
        }
    }, [guiApi, notification, savedValues, t]);

    const acceptPersistedValues = useCallback((persistedValues: Record<string, any>) => {
        setSavedValues((prev) => ({ ...prev, ...persistedValues }));
        setLocalValues((prev) => ({ ...prev, ...persistedValues }));
    }, []);

    const save = useCallback(async () => {
        await persistSettings();
    }, [persistSettings]);

    const saveAndRestartGps = useCallback(async () => {
        await persistSettings({ forceGpsRestart: true });
    }, [persistSettings]);

    const revert = useCallback(() => {
        setLocalValues({ ...savedValues });
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
    ]);
    const advancedKeys = useMemo(() => {
        const knownKeys = new Set(
            SECTION_DEFINITIONS.filter((s) => s.id !== "advanced").flatMap((s) => s.keys)
        );
        return Object.keys(localValues).filter(
            (k) => !knownKeys.has(k) && !HIDDEN_FROM_ADVANCED.has(k),
        );
    }, [localValues]);

    // Search filtering
    const matchesSearch = useCallback(
        (key: string, label?: string): boolean => {
            if (!searchQuery) return true;
            const q = searchQuery.toLowerCase();
            return (
                key.toLowerCase().includes(q) ||
                (label?.toLowerCase().includes(q) ?? false)
            );
        },
        [searchQuery]
    );

    return {
        sections: SECTION_DEFINITIONS,
        values: localValues,
        savedValues,
        defaults,
        loading,
        saving,
        gpsRestarting: gpsRestart.pending,
        isDirty,
        dirtyKeys,
        restartRequired,
        searchQuery,
        advancedKeys,
        setSearchQuery,
        handleChange,
        handleBulkChange,
        hasDefault,
        isDefault,
        isOverridden,
        resetToDefault,
        isSectionDirty,
        matchesSearch,
        save,
        savePartialValues,
        saveAndRestartGps,
        acceptPersistedValues,
        revert,
    };
};
