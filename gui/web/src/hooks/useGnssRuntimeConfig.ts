import { useEffect, useState } from "react";
import { useApi } from "./useApi.ts";

export type GnssRuntimeValue = {
    configured_value?: string;
    fallback_env_value?: string;
    active_value?: string;
    source?: "config" | "env" | "default" | string;
};

export type GnssSerialDeviceOption = {
    path: string;
    resolved_path?: string;
    display_label: string;
    exists: boolean;
};

export type GnssRuntimeConfig = {
    receiver_family?: GnssRuntimeValue;
    serial_device?: GnssRuntimeValue;
    serial_baud?: GnssRuntimeValue;
    serial_devices?: GnssSerialDeviceOption[];
    configured_serial_device_exists?: boolean;
    active_serial_device_exists?: boolean;
    configured_serial_resolved_path?: string;
    active_serial_resolved_path?: string;
};

export const useGnssRuntimeConfig = () => {
    const guiApi = useApi();
    const [runtimeConfig, setRuntimeConfig] = useState<GnssRuntimeConfig | null>(null);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        let cancelled = false;

        const load = async () => {
            try {
                const response = await guiApi.request({
                    path: "/settings/gnss/runtime-config",
                    method: "GET",
                    format: "json",
                });
                if (cancelled) {
                    return;
                }

                const payload = ((response as any)?.data ?? response) as GnssRuntimeConfig;
                setRuntimeConfig(payload);
                setError(null);
            } catch (nextError: any) {
                if (cancelled) {
                    return;
                }
                setError(nextError?.error?.error ?? nextError?.message ?? "Failed to load GNSS runtime config");
            }
        };

        void load();
        const intervalId = window.setInterval(() => {
            void load();
        }, 5000);

        return () => {
            cancelled = true;
            window.clearInterval(intervalId);
        };
    }, [guiApi]);

    return {
        runtimeConfig,
        error,
    };
};
