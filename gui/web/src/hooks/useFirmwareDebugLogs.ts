import {useEffect, useRef, useState} from "react";
import {useApi} from "./useApi.ts";
import {useWS} from "./useWS.ts";

const MAX_LINES = 400;
const ANSI_REGEX = /\x1b\[[0-9;]*m/g;

const isFirmwareDiagnosticLine = (line: string): boolean => line.includes("[FW_DIAG]");

type FirmwareDebugLine = {
    id: number;
    plain: string;
};

const findRos2ContainerId = (containers: Array<{
    id?: string;
    names?: string[];
    labels?: Record<string, string>;
}>): string | null => {
    const ros2 = containers.find((container) => {
        const names = container.names ?? [];
        if (names.some((name) => name.includes("mowgli-ros2") || name.includes("ros2"))) {
            return true;
        }
        return container.labels?.app === "ros2";
    });
    return ros2?.id ?? null;
};

export const useFirmwareDebugLogs = (enabled: boolean) => {
    const guiApi = useApi();
    const [lines, setLines] = useState<FirmwareDebugLine[]>([]);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const nextIdRef = useRef(0);

    const stream = useWS<string>(
        () => {
            if (enabled) {
                setError("stream_closed");
            }
        },
        () => {
            setError(null);
        },
        (line, first) => {
            const plain = line.replace(ANSI_REGEX, "");
            if (!isFirmwareDiagnosticLine(plain)) {
                return;
            }
            setLines((prev) => {
                const entry: FirmwareDebugLine = {id: nextIdRef.current++, plain};
                const base = first ? [] : prev;
                const next = [...base, entry];
                return next.length > MAX_LINES ? next.slice(next.length - MAX_LINES) : next;
            });
        },
    );

    useEffect(() => {
        let cancelled = false;

        const stop = () => {
            stream.stop();
            setLoading(false);
        };

        if (!enabled) {
            stop();
            setError(null);
            setLines([]);
            nextIdRef.current = 0;
            return;
        }

        async function connect() {
            setLoading(true);
            setError(null);
            try {
                const res = await guiApi.containers.containersList();
                if (cancelled) {
                    return;
                }
                if (res.error) {
                    throw new Error(res.error.error);
                }
                const containerId = findRos2ContainerId(res.data.containers ?? []);
                if (!containerId) {
                    throw new Error("ros2_container_not_found");
                }
                nextIdRef.current = 0;
                setLines([]);
                stream.start(`/api/containers/${containerId}/logs`);
            } catch (err) {
                if (!cancelled) {
                    setError(err instanceof Error ? err.message : String(err));
                }
            } finally {
                if (!cancelled) {
                    setLoading(false);
                }
            }
        }

        connect();
        return () => {
            cancelled = true;
            stop();
        };
    }, [enabled, guiApi]);

    return {lines, loading, error};
};
