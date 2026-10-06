export type HardwareBackend = "mowgli" | "mavros";

export type HardwareParameterRoute = {
    parameter: string;
    runtime: "available" | "pending_image";
};

export type HardwareBackendInfo = {
    backend: HardwareBackend;
    parameterRoutes: Record<string, HardwareParameterRoute>;
    runtimeRouting: "available" | "pending_image";
};

export const DEFAULT_HARDWARE_BACKEND: HardwareBackend = "mowgli";

export const normalizeHardwareBackend = (value: unknown): HardwareBackend =>
    value === "mavros" ? "mavros" : DEFAULT_HARDWARE_BACKEND;

export const settingsSectionsForBackend = <T extends { id: string }>(
    sections: readonly T[],
    backend: HardwareBackend,
): T[] => sections.filter((section) => backend === "mowgli" || section.id !== "drive_motor");

export const liveHardwareParameters = (
    dirtyKeys: ReadonlySet<string>,
    values: Record<string, unknown>,
    routes: Record<string, HardwareParameterRoute>,
) => Object.entries(routes)
    .filter(([key, route]) => route.runtime === "available" && dirtyKeys.has(key) && key in values)
    .map(([key, route]) => ({ name: route.parameter, value: Number(values[key]) }));
