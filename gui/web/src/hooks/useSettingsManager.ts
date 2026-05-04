import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useApi } from "./useApi.ts";
import { App } from "antd";
import { dirtyKeysRequireGpsRestart, restartGps } from "../utils/containers.ts";
import { useContainerRestart } from "./useContainerRestart.ts";

export type SettingsSection =
    | "hardware"
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
        label: "Hardware",
        icon: "tool",
        description: "Robot model, wheels, chassis, and blade dimensions",
        keys: [
            "mower_model", "wheel_radius", "wheel_track", "wheel_width",
            "wheel_x_offset", "chassis_center_x", "chassis_length", "chassis_width",
            "chassis_height", "chassis_mass_kg", "caster_radius", "caster_track",
            "ticks_per_meter", "tool_width", "blade_radius",
        ],
    },
    {
        id: "positioning",
        label: "GPS & Positioning",
        icon: "global",
        description: "GPS datum, NTRIP corrections, protocol",
        keys: [
            "datum_lat", "datum_lon", "datum_alt",
            "gps_port", "gps_baudrate",
            "gps_protocol", "gps_wait_after_undock_sec",
            "gps_timeout_sec", "ntrip_enabled", "ntrip_host", "ntrip_port",
            "ntrip_user", "ntrip_password", "ntrip_mountpoint",
        ],
    },
    {
        id: "sensors",
        label: "Sensors",
        icon: "aim",
        description: "LiDAR, IMU, and GPS antenna placement on the robot",
        keys: [
            "lidar_enabled", "lidar_x", "lidar_y", "lidar_z", "lidar_yaw",
            "imu_x", "imu_y", "imu_z", "imu_yaw", "imu_pitch", "imu_roll",
            "gps_x", "gps_y", "gps_z",
            "use_lidar",
            "dock_pose_yaw",
        ],
    },
    {
        id: "localization",
        label: "Localization",
        icon: "node-index",
        description: "Map-frame fusion strategy and optional LiDAR factors",
        keys: [
            "use_fusion_graph", "use_scan_matching", "use_loop_closure",
            "use_magnetometer",
        ],
    },
    {
        id: "mowing",
        label: "Mowing",
        icon: "scissor",
        description: "Speed, path spacing, angle, and outline settings",
        keys: [
            "mowing_enabled", "mowing_speed", "transit_speed", "outline_passes",
            "outline_offset", "outline_overlap", "path_spacing", "headland_width",
            "min_turning_radius", "mow_angle_offset_deg", "mow_angle_increment_deg",
        ],
    },
    {
        id: "docking",
        label: "Docking",
        icon: "home",
        description: "Undock distance, approach, retry settings",
        keys: [
            "undock_distance", "undock_speed", "dock_approach_distance",
            "dock_max_retries", "dock_use_charger_detection",
        ],
    },
    {
        id: "battery",
        label: "Battery",
        icon: "thunderbolt",
        description: "Voltage and percentage thresholds for charging",
        keys: [
            "battery_full_voltage", "battery_empty_voltage", "battery_critical_voltage",
            "battery_full_percent", "battery_low_percent", "battery_critical_percent",
        ],
    },
    {
        id: "safety",
        label: "Safety",
        icon: "safety",
        description: "Emergency stops, temperature limits, obstacle avoidance",
        keys: [
            "motor_temp_high_c", "motor_temp_low_c", "emergency_stop_on_tilt",
            "emergency_stop_on_lift", "max_obstacle_avoidance_distance",
            "lift_blade_resume_delay_sec", "lift_recovery_mode",
        ],
    },
    {
        id: "navigation",
        label: "Navigation",
        icon: "compass",
        description: "Goal tolerances and progress timeout",
        keys: [
            "xy_goal_tolerance", "yaw_goal_tolerance", "coverage_xy_tolerance",
            "progress_timeout_sec",
        ],
    },
    {
        id: "rain",
        label: "Rain",
        icon: "cloud",
        description: "Rain detection behavior and delay",
        keys: ["rain_mode", "rain_delay_minutes", "rain_debounce_sec"],
    },
    {
        id: "advanced",
        label: "Advanced",
        icon: "code",
        description: "Raw parameters and custom key-value pairs",
        keys: [],
    },
];

export const useSettingsManager = () => {
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [savedValues, setSavedValues] = useState<Record<string, any>>({});
    const [localValues, setLocalValues] = useState<Record<string, any>>({});
    const [loading, setLoading] = useState(true);
    const [saving, setSaving] = useState(false);
    const [restartRequired, setRestartRequired] = useState(false);
    // GPS restart skips the rosbridge readiness probe (ROS2 is unaffected).
    const gpsRestart = useContainerRestart({
        pendingLabel: "Redémarrage GPS…",
        successMessage: "GPS redémarré — patientez ~10–30 s pour le RTK Fix",
        errorMessage: "Échec du redémarrage GPS",
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
                initialLoadDone.current = true;
            } catch (e: any) {
                notification.error({
                    message: "Failed to load settings",
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

    const save = useCallback(async () => {
        try {
            setSaving(true);
            // Capture which keys were dirty BEFORE we mark them saved, so we
            // can decide whether GPS needs an auto-restart.
            const gpsDirty = dirtyKeysRequireGpsRestart(dirtyKeys);
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
            const res = await guiApi.settings.yamlCreate(dirtyPayload);
            if (res.error) throw new Error((res.error as any).error);
            setSavedValues({ ...localValues });
            setRestartRequired(true);
            notification.success({
                message: "Settings saved",
                description: gpsDirty
                    ? "Restarting GPS to apply NTRIP/serial changes. Restart ROS2 to apply other changes."
                    : "Restart ROS2 to apply changes.",
            });
            // Auto-restart the GPS container when GPS/NTRIP fields changed —
            // ROS2 keeps running, the user just sees RTCM stop briefly. This
            // unblocks the "Set Datum from current GPS" path in onboarding,
            // which silently fails when the old (un-credentialled) GPS
            // container is still running.
            if (gpsDirty) {
                await gpsRestart.run(() => restartGps(guiApi));
            }
        } catch (e: any) {
            notification.error({
                message: "Failed to save settings",
                description: e.message,
            });
        } finally {
            setSaving(false);
        }
    }, [localValues, dirtyKeys, guiApi, notification, gpsRestart]);

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
        isSectionDirty,
        matchesSearch,
        save,
        revert,
    };
};
