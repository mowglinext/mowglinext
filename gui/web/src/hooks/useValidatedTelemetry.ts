import {useEffect, useState} from 'react';
import {isJsonObject} from '../utils/json';
import {isRosStamp} from '../utils/physicalTelemetry';
import {useWS} from './useWS';
import {telemetryStampIdentity, type TelemetryDecode} from '../utils/statusTelemetry';

export interface ValidatedTelemetry<T> {
    data: T;
    /** Delivery history only: even a rejected frame establishes that this stream sent data. */
    hasReceivedFrame: boolean;
    /** Last reported assertions; not acknowledged cleared without producer ordering. */
    assertions: T;
    /** Display consumers may require progression after a known disconnect. */
    lastDisconnectAt?: number;
    /** Optional ordered publication metadata for display-only health checks. */
    /** Producer contract: receipt liveness is not ordered publication/acquisition evidence. */
    publicationPolicy?: 'ordered' | 'receipt';
    publicationStamp?: {sec: number; nanosec: number};
    publicationProgressAt?: number;
    publicationRegressed?: boolean;
    publicationBlocked?: boolean;
    condition: 'missing' | 'valid' | 'invalid' | 'unsupported' | 'disconnected';
    /** Browser monotonic receipt time of the last structurally valid frame. */
    lastValidReceiptAt: number | null;
    /** Opaque producer-clock identity, when supplied. Not proof of acquisition. */
    stampIdentity: string | null;
    /** Browser monotonic time when that identity changed, not ROS age. */
    stampChangedAt: number | null;
}
/** Limited status boundary; retains the unthrottled useWS start/stop contract. */
export function useValidatedTelemetry<T>(
    topic: string,
    initial: T,
    decode: (raw: unknown) => TelemetryDecode<T>,
    retainWarnings: (previous: T, next: T) => T,
    accumulateAssertions?: (previous: T, next: T) => T,
    readIdentity: (raw: unknown) => string | null = telemetryStampIdentity,
    publicationPolicy: 'ordered' | 'receipt' = 'ordered',
) {
    const [telemetry, setTelemetry] = useState<ValidatedTelemetry<T>>({
        data: initial, hasReceivedFrame: false, assertions: initial, condition: 'missing', lastValidReceiptAt: null,
        stampIdentity: null, stampChangedAt: null,
    });
    const stream = useWS<unknown>(
        () => setTelemetry(previous => previous.condition === 'disconnected' ? previous : {
            ...previous, publicationBlocked: true, lastDisconnectAt: performance.now(), data: retainWarnings(previous.data, initial), condition: 'disconnected',
        }),
        () => { /* Open is transport liveness, not a validated sample. */ },
        raw => {
            const decoded = decode(raw);
            if (decoded.kind !== 'valid') {
                setTelemetry(previous => previous.condition === decoded.kind ? previous : {
                    ...previous, publicationBlocked: true, hasReceivedFrame: true, data: retainWarnings(previous.data, initial), condition: decoded.kind,
                });
                return;
            }
            const at = performance.now();
            const identity = publicationPolicy === 'receipt' ? null : readIdentity(raw);
            const stamp = publicationPolicy === 'ordered' && isJsonObject(raw) && isRosStamp(raw.stamp)
                && (raw.stamp.sec !== 0 || raw.stamp.nanosec !== 0) ? {sec: raw.stamp.sec, nanosec: raw.stamp.nanosec} : undefined;
            setTelemetry(previous => {
                const high = previous.publicationStamp;
                const advances = stamp && (!high || stamp.sec > high.sec || stamp.sec === high.sec && stamp.nanosec > high.nanosec);
                const regressed = high !== undefined && (!stamp || stamp.sec < high.sec || stamp.sec === high.sec && stamp.nanosec < high.nanosec);
                return {
                    publicationPolicy,
                    // Unstamped HighLevelStatus can recover delivery liveness, never
                    // observation identity. Stamped contracts keep their high-water barrier.
                    publicationBlocked: publicationPolicy === 'ordered' && (regressed || previous.publicationBlocked === true && !advances),
                    ...(high || advances ? {publicationStamp: advances ? stamp : high,
                        publicationProgressAt: advances ? at : previous.publicationProgressAt, publicationRegressed: regressed} : {}),
                    ...(previous.lastDisconnectAt !== undefined ? {lastDisconnectAt: previous.lastDisconnectAt} : {}),
                    data: retainWarnings(previous.data, decoded.data), hasReceivedFrame: true, condition: 'valid',
                    assertions: accumulateAssertions ? accumulateAssertions(previous.assertions, decoded.data) : initial,
                    lastValidReceiptAt: at, stampIdentity: identity,
                    stampChangedAt: identity === null ? null
                        : identity === previous.stampIdentity ? previous.stampChangedAt : at,
                };
            });
        },
    );
    const {start, stop} = stream;
    useEffect(() => {
        start(`/api/mowglinext/subscribe/${topic}`);
        return stop;
    }, [topic, start, stop]);
    return {...stream, telemetry};
}
