import { useEffect, useState } from "react";
import { useApi } from "./useApi.ts";
import {
    DEFAULT_HARDWARE_BACKEND,
    HardwareBackendInfo,
    HardwareParameterRoute,
    normalizeHardwareBackend,
} from "../constants/hardwareBackends.ts";

const FALLBACK: HardwareBackendInfo = {
    backend: DEFAULT_HARDWARE_BACKEND,
    // Preserve the pre-backend-aware Mowgli live-update behavior if this
    // best-effort metadata request fails. A successful MAVROS response always
    // replaces these routes with backend-specific live destinations.
    parameterRoutes: {
        ticks_per_meter: { parameter: "hardware_bridge.ticks_per_meter", runtime: "available" },
        wheel_pid_kp: { parameter: "hardware_bridge.wheel_pid_kp", runtime: "available" },
        wheel_pid_ki: { parameter: "hardware_bridge.wheel_pid_ki", runtime: "available" },
        wheel_pid_kd: { parameter: "hardware_bridge.wheel_pid_kd", runtime: "available" },
        wheel_pid_integral_limit: { parameter: "hardware_bridge.wheel_pid_integral_limit", runtime: "available" },
        wheel_pid_pwm_per_mps: { parameter: "hardware_bridge.wheel_pid_pwm_per_mps", runtime: "available" },
    },
    runtimeRouting: "available",
};

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
                const data = (response.data ?? {}) as {
                    backend?: unknown;
                    parameter_routes?: Record<string, HardwareParameterRoute>;
                    runtime_routing?: "available" | "pending_image";
                };
                setInfo({
                    backend: normalizeHardwareBackend(data.backend),
                    parameterRoutes: data.parameter_routes ?? {},
                    runtimeRouting: data.runtime_routing === "pending_image" ? "pending_image" : "available",
                });
            } catch {
                // Older backends retain the established Mowgli behavior.
            } finally {
                if (!cancelled) setLoading(false);
            }
        })();
        return () => { cancelled = true; };
    }, [guiApi]);

    return { ...info, loading };
};
