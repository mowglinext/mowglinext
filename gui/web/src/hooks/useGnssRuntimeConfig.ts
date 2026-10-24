import { useEffect, useState } from "react";
import { useApi } from "./useApi.ts";

export type {GnssRuntimeValue, GnssSerialDeviceOption, GnssRuntimeConfig} from '../utils/gnssConfigContract';
import {decodeGnssRuntimeConfig, type GnssRuntimeConfig} from '../utils/gnssConfigContract';
import {requestErrorMessage} from '../utils/errors';

export const useGnssRuntimeConfig = () => {
    const guiApi = useApi();
    const [runtimeConfig, setRuntimeConfig] = useState<GnssRuntimeConfig | null>(null);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        let cancelled = false;
        let latestCompleted = 0;
        let nextRequest = 0;

        const load = async () => {
            const request = ++nextRequest;
            try {
                const response = await guiApi.request<unknown>({
                    path: "/settings/gnss/runtime-config",
                    method: "GET",
                    format: "json",
                });
                if (cancelled || request < latestCompleted) {
                    return;
                }

                if (response.error || response.ok === false || response.status >= 400) throw new Error(requestErrorMessage(response) ?? 'Failed to load GNSS runtime config');
                const payload = decodeGnssRuntimeConfig(response.data);
                latestCompleted = request;
                setRuntimeConfig(payload);
                setError(null);
            } catch (nextError) {
                if (cancelled || request < latestCompleted) {
                    return;
                }
                latestCompleted = request;
                setRuntimeConfig(null);
                setError(requestErrorMessage(nextError) ?? 'Failed to load GNSS runtime config');
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
