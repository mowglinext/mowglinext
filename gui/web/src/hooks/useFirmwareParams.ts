import { useEffect, useState } from "react";
import { useWS } from "./useWS.ts";
import type { FirmwareParams } from "../types/ros.generated.ts";

/**
 * useFirmwareParams — latest /hardware_bridge/firmware_params report: what the
 * STM32 actually runs for every runtime parameter, its envelope, and the state
 * of the board's parameter flash (protocol v7). null until the first message.
 */
export const useFirmwareParams = (): FirmwareParams | null => {
    const [params, setParams] = useState<FirmwareParams | null>(null);
    const stream = useWS<string>(
        () => {},
        () => {},
        (e) => {
            setParams(e as unknown as FirmwareParams);
        },
    );

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/firmwareParams");
        return () => stream.stop();
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    return params;
};
