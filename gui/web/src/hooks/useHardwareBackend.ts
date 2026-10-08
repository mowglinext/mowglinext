import { useEffect, useState } from "react";
import { useApi } from "./useApi.ts";
import {
    DEFAULT_HARDWARE_BACKEND,
    HardwareBackendInfo,
    normalizeHardwareBackend,
} from "../constants/hardwareBackends.ts";

const FALLBACK: HardwareBackendInfo = { backend: DEFAULT_HARDWARE_BACKEND, defaultOverrides: {} };

/**
 * The robot's hardware backend. Falls back to "mowgli" (the stock path) while
 * loading or when the backend predates the route, so the page never hides the
 * STM32 settings of a robot that has them.
 */
export const useHardwareBackend = (): HardwareBackendInfo & { loading: boolean } => {
    const guiApi = useApi();
    const [info, setInfo] = useState<HardwareBackendInfo>(FALLBACK);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        let cancelled = false;
        void (async () => {
            try {
                const res = await guiApi.request({
                    path: "/settings/hardware-backend",
                    method: "GET",
                    format: "json",
                });
                if (cancelled || res.error) return;
                const data = (res.data ?? {}) as { backend?: unknown; default_overrides?: unknown };
                const overrides =
                    data.default_overrides && typeof data.default_overrides === "object"
                        ? (data.default_overrides as Record<string, unknown>)
                        : {};
                setInfo({ backend: normalizeHardwareBackend(data.backend), defaultOverrides: overrides });
            } catch {
                /* keep the mowgli fallback */
            } finally {
                if (!cancelled) setLoading(false);
            }
        })();
        return () => {
            cancelled = true;
        };
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    return { ...info, loading };
};
