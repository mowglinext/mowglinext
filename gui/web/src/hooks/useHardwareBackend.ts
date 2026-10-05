import { useEffect, useState } from "react";
import { useApi } from "./useApi.ts";
import {
    DEFAULT_HARDWARE_BACKEND,
    normalizeHardwareBackend,
} from "../constants/hardwareBackends.ts";
import type {
    HardwareBackendInfo,
    HardwareParameterRoute,
} from "../constants/hardwareBackends.ts";

// An older GUI backend might not expose the metadata endpoint yet. Keep the
// established Mowgli live tuning behavior in that case, but never assume that
// the same ROS parameter routes exist on MAVROS or OpenMower.
const MOWGLI_FALLBACK_ROUTES: Record<string, HardwareParameterRoute> = {
    ticks_per_meter: { parameter: "hardware_bridge.ticks_per_meter", runtime: "available" },
    wheel_pid_kp: { parameter: "hardware_bridge.wheel_pid_kp", runtime: "available" },
    wheel_pid_ki: { parameter: "hardware_bridge.wheel_pid_ki", runtime: "available" },
    wheel_pid_kd: { parameter: "hardware_bridge.wheel_pid_kd", runtime: "available" },
    wheel_pid_integral_limit: { parameter: "hardware_bridge.wheel_pid_integral_limit", runtime: "available" },
    wheel_pid_pwm_per_mps: { parameter: "hardware_bridge.wheel_pid_pwm_per_mps", runtime: "available" },
};

const FALLBACK: HardwareBackendInfo = {
    backend: DEFAULT_HARDWARE_BACKEND,
    defaultOverrides: {},
    parameterRoutes: MOWGLI_FALLBACK_ROUTES,
    runtimeRouting: "available",
};

const isRecord = (value: unknown): value is Record<string, unknown> =>
    value !== null && typeof value === "object" && !Array.isArray(value);

const parseParameterRoutes = (value: unknown): Record<string, HardwareParameterRoute> => {
    if (!isRecord(value)) return {};

    const routes: Record<string, HardwareParameterRoute> = {};
    for (const [key, candidate] of Object.entries(value)) {
        if (
            isRecord(candidate) &&
            typeof candidate.parameter === "string" &&
            (candidate.runtime === "available" || candidate.runtime === "pending_image")
        ) {
            routes[key] = {
                parameter: candidate.parameter,
                runtime: candidate.runtime,
            };
        }
    }
    return routes;
};

/**
 * The active hardware backend comes from the installer via the GUI API.
 * While loading, or if the endpoint is unavailable, retain the stock Mowgli
 * behavior. On a successful response, use only the selected backend's routes.
 */
export const useHardwareBackend = (): HardwareBackendInfo & { loading: boolean } => {
    const guiApi = useApi();
    const [info, setInfo] = useState<HardwareBackendInfo>(FALLBACK);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        let cancelled = false;

        void (async () => {
            try {
                const response = await guiApi.request({
                    path: "/settings/hardware-backend",
                    method: "GET",
                    format: "json",
                });
                if (cancelled || response.error) return;

                const data = isRecord(response.data) ? response.data : {};
                const backend = normalizeHardwareBackend(data.backend);
                const defaultOverrides = isRecord(data.default_overrides)
                    ? data.default_overrides
                    : {};
                const parameterRoutes = data.parameter_routes === undefined && backend === "mowgli"
                    ? MOWGLI_FALLBACK_ROUTES
                    : parseParameterRoutes(data.parameter_routes);
                const runtimeRouting = data.runtime_routing === "pending_image" ||
                    (data.runtime_routing !== "available" && backend === "mavros")
                    ? "pending_image"
                    : "available";

                setInfo({
                    backend,
                    defaultOverrides,
                    parameterRoutes,
                    runtimeRouting,
                });
            } catch {
                // Keep the safe Mowgli fallback if the best-effort request fails.
            } finally {
                if (!cancelled) setLoading(false);
            }
        })();

        return () => {
            cancelled = true;
        };
        // Hardware backend selection changes at installation/restart, not during
        // a settings session; fetch once to avoid loops if useApi() is unstable.
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    return { ...info, loading };
};