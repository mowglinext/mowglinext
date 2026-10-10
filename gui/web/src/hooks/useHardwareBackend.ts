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

// An older GUI backend might not expose the metadata endpoint yet.
// Preserve Mowgli's existing live tuning fallback, but never assume
// the same ROS parameter routes exist on MAVROS or OpenMower.
const MOWGLI_FALLBACK_ROUTES: Record<string, HardwareParameterRoute> = {
    ticks_per_meter: {
        parameter: "hardware_bridge.ticks_per_meter",
        runtime: "available",
    },
    wheel_pid_kp: {
        parameter: "hardware_bridge.wheel_pid_kp",
        runtime: "available",
    },
    wheel_pid_ki: {
        parameter: "hardware_bridge.wheel_pid_ki",
        runtime: "available",
    },
    wheel_pid_kd: {
        parameter: "hardware_bridge.wheel_pid_kd",
        runtime: "available",
    },
    wheel_pid_integral_limit: {
        parameter: "hardware_bridge.wheel_pid_integral_limit",
        runtime: "available",
    },
    wheel_pid_pwm_per_mps: {
        parameter: "hardware_bridge.wheel_pid_pwm_per_mps",
        runtime: "available",
    },
};

const FALLBACK: HardwareBackendInfo = {
    backend: DEFAULT_HARDWARE_BACKEND,
    defaultOverrides: {},
    parameterRoutes: MOWGLI_FALLBACK_ROUTES,
    runtimeRouting: "available",
    robotName: "",
};

const isRecord = (
    value: unknown,
): value is Record<string, unknown> =>
    value !== null &&
    typeof value === "object" &&
    !Array.isArray(value);

const parseParameterRoutes = (
    value: unknown,
): Record<string, HardwareParameterRoute> => {
    if (!isRecord(value)) {
        return {};
    }

    const routes: Record<string, HardwareParameterRoute> = {};

    for (const [key, candidate] of Object.entries(value)) {
        if (
            isRecord(candidate) &&
            typeof candidate.parameter === "string" &&
            (
                candidate.runtime === "available" ||
                candidate.runtime === "pending_image"
            )
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
 *
 * While loading, or if the endpoint is unavailable, retain the stock
 * Mowgli behavior.
 *
 * On a successful response, use only the selected backend's routes.
 *
 * robotName is provided by the installed robot configuration and is
 * used by the GUI header badge.
 */
export const useHardwareBackend = ():
    HardwareBackendInfo & { loading: boolean } => {

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

                if (cancelled || response.error) {
                    return;
                }

                const data = isRecord(response.data)
                    ? response.data
                    : {};

                const backend = normalizeHardwareBackend(data.backend);

                // Preserve backend-specific defaults, particularly for
                // OpenMower xESC hardware configuration.
                const defaultOverrides = isRecord(data.default_overrides)
                    ? data.default_overrides
                    : {};

                // Mowgli keeps its legacy fallback if the endpoint is
                // older and does not yet expose parameter_routes.
                //
                // MAVROS and OpenMower must never inherit these routes.
                const parameterRoutes =
                    data.parameter_routes === undefined &&
                    backend === "mowgli"
                        ? MOWGLI_FALLBACK_ROUTES
                        : parseParameterRoutes(data.parameter_routes);

                // An older MAVROS endpoint without runtime routing
                // confirmation must not be assumed to support live updates.
                const runtimeRouting =
                    data.runtime_routing === "pending_image" ||
                    (
                        data.runtime_routing !== "available" &&
                        backend === "mavros"
                    )
                        ? "pending_image"
                        : "available";

                // Robot name is supplied by the new dev backend.
                // Keep an empty string until a valid name is available.
                const robotName =
                    typeof data.robot_name === "string"
                        ? data.robot_name.trim()
                        : "";

                if (cancelled) {
                    return;
                }

                setInfo({
                    backend,
                    defaultOverrides,
                    parameterRoutes,
                    runtimeRouting,
                    robotName,
                });
            } catch {
                // Keep the Mowgli fallback if the best-effort request fails.
            } finally {
                if (!cancelled) {
                    setLoading(false);
                }
            }
        })();

        return () => {
            cancelled = true;
        };

        // Hardware backend selection changes at installation/restart,
        // not during a settings session. Fetch once to avoid loops
        // if useApi() is unstable.
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    return { ...info, loading };
};