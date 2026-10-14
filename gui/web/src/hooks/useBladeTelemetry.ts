import {useEffect, useRef, useState} from 'react';
import type {Status} from '../types/ros';
import {useTopic} from './useTopic';
import {bladeTelemetry} from '../utils/motorTelemetry';

export function useBladeTelemetry(backend?: string) {
    const acquisition = useRef<{identity: string | null; at: number | null}>({identity: null, at: null});
    const {data, lastMessageAt} = useTopic<{status: Status; observedAt: number | null}>(
        'status', {status: {}, observedAt: null}, {withTimestamp: true, select: raw => {
            const status = (raw !== null && typeof raw === "object" ? raw : {}) as Status;
            const stamp = status.blade_status_stamp;
            const identity = stamp ? `${stamp.sec}:${stamp.nanosec}` : null;
            if (identity !== acquisition.current.identity) acquisition.current = {identity, at: Date.now()};
            return {status, observedAt: acquisition.current.at};
        }});
    const [now, setNow] = useState(Date.now);
    useEffect(() => {
        const timer = window.setInterval(() => setNow(Date.now()), 1000);
        return () => window.clearInterval(timer);
    }, []);
    return bladeTelemetry(data.status,
        data.observedAt === null || lastMessageAt === null ? null : Math.min(data.observedAt, lastMessageAt),
        Math.max(now, lastMessageAt ?? now), backend);
}
