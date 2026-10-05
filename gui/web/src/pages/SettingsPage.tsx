// Hardware backends (HARDWARE_BACKEND: which bridge drives the robot) as the
// Settings page sees them. The active one comes from GET
// /settings/hardware-backend (gui/pkg/api/settings_backend.go).

export type HardwareBackend = "mowgli" | "mavros" | "openmower";

export type HardwareParameterRoute = {
    parameter: string;
    runtime: "available" | "pending_image";
};

export interface HardwareBackendInfo {
    backend: HardwareBackend;
    /**
     * Settings whose DEFAULT this backend replaces
     * (ros2/src/mowgli_bringup/config/backends/<backend>.yaml), e.g. the
     * OpenMower xESC's ticks_per_meter.
     */
    defaultOverrides: Record<string, unknown>;
    /** Routes for writing live ROS parameters on the selected backend. */
    parameterRoutes: Record<string, HardwareParameterRoute>;
    /** Whether the backend's live parameter routing is deployed. */
    runtimeRouting: "available" | "pending_image";
}

export const DEFAULT_HARDWARE_BACKEND: HardwareBackend = "mowgli";

export const normalizeHardwareBackend = (value: unknown): HardwareBackend =>
    value === "openmower" || value === "mavros" ? value : DEFAULT_HARDWARE_BACKEND;

/** True when an item limited to `backends` applies to `active` (no limit = all). */
export const appliesToBackend = (
    backends: readonly HardwareBackend[] | undefined,
    active: HardwareBackend,
): boolean => !backends || backends.includes(active);

/**
 * The values a mower-model preset should write on this backend. A preset
 * describes the MACHINE; a key the backend overrides describes its
 * ELECTRONICS (an OpenMower xESC counts 1600 hall ticks/m whatever the
 * chassis), so it takes the backend's default instead — which the settings
 * backend then prunes, leaving the robot on that same default.
 */
export const presetValuesForBackend = (
    preset: Record<string, number>,
    defaultOverrides: Record<string, unknown>,
): Record<string, unknown> => {
    const out: Record<string, unknown> = { ...preset };
    for (const key of Object.keys(preset)) {
        if (key in defaultOverrides) out[key] = defaultOverrides[key];
    }
    return out;
};

/**
 * MAVROS currently has no drive tuning section. Mowgli shows STM32 PID
 * controls; OpenMower shows its own xESC wheel-loop controls.
 */
export const settingsSectionsForBackend = <T extends { id: string }>(
    sections: readonly T[],
    backend: HardwareBackend,
): T[] => sections.filter((section) => backend !== "mavros" || section.id !== "drive_motor");

/**
 * Build ROS parameter updates only for routes confirmed as live. A pending
 * MAVROS image must not receive parameter updates that it cannot handle.
 */
export const liveHardwareParameters = (
    dirtyKeys: ReadonlySet<string>,
    values: Record<string, unknown>,
    routes: Record<string, HardwareParameterRoute>,
) => Object.entries(routes)
    .filter(([key, route]) => route.runtime === "available" && dirtyKeys.has(key) && key in values)
    .map(([key, route]) => ({ name: route.parameter, value: Number(values[key]) }));