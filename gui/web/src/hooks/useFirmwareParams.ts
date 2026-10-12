import { useEffect, useRef, useState } from "react";
import { useWS } from "./useWS.ts";
import type { FirmwareParams } from "../types/ros.generated.ts";

/**
 * useFirmwareParams — latest /hardware_bridge/firmware_params report: what the
 * STM32 actually runs for every runtime parameter, its envelope, and the state
 * of the board's parameter flash (protocol v8), plus local receipt time. Null
 * until the first message.
 */
export const useFirmwareParams = (): {
    report: FirmwareParams | null;
    lastMessageAt: number | null;
    storeStatusSequence: number;
    lastStoreStatusAt: number | null;
} => {
    const [params, setParams] = useState<FirmwareParams | null>(null);
    const [lastMessageAt, setLastMessageAt] = useState<number | null>(null);
    const [storeStatus, setStoreStatus] = useState<{ sequence: number; receivedAt: number | null }>({
        sequence: 0,
        receivedAt: null,
    });
    const lastSequenceRef = useRef(0);
    const hasSequenceBaselineRef = useRef(false);
    const stream = useWS<string>(
        () => {
            // A reconnect can replay the retained ROS message. Require a new
            // store-status identity after every disconnect before treating it
            // as fresh evidence again.
            lastSequenceRef.current = 0;
            hasSequenceBaselineRef.current = false;
            setStoreStatus({ sequence: 0, receivedAt: null });
        },
        () => {},
        (e) => {
            const report = e as unknown as FirmwareParams;
            const receivedAt = Date.now();
            setParams(report);
            setLastMessageAt(receivedAt);
            const sequence = report.store_status_sequence ?? 0;
            if (sequence === 0) {
                lastSequenceRef.current = 0;
                hasSequenceBaselineRef.current = true;
                setStoreStatus({ sequence: 0, receivedAt: null });
            } else if (!hasSequenceBaselineRef.current) {
                // The first nonzero value may be the retained ROS latch, not a
                // packet received during this browser session. Treat it only
                // as a baseline; a changed identity proves a newer frame.
                lastSequenceRef.current = sequence;
                hasSequenceBaselineRef.current = true;
                setStoreStatus({ sequence, receivedAt: null });
            } else if (sequence !== lastSequenceRef.current) {
                lastSequenceRef.current = sequence;
                setStoreStatus({ sequence, receivedAt });
            }
        },
    );

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/firmwareParams");
        return () => stream.stop();
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    return {
        report: params,
        lastMessageAt,
        storeStatusSequence: storeStatus.sequence,
        lastStoreStatusAt: storeStatus.receivedAt,
    };
};
