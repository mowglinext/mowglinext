import {useEffect, useState} from 'react';
import {useWS} from './useWS';
import {telemetryStampIdentity, type TelemetryDecode} from '../utils/statusTelemetry';

export interface ValidatedTelemetry<T> {
    data: T;
    /** Last reported assertions; not acknowledged cleared without producer ordering. */
    assertions: T;
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
) {
    const [telemetry, setTelemetry] = useState<ValidatedTelemetry<T>>({
        data: initial, assertions: initial, condition: 'missing', lastValidReceiptAt: null,
        stampIdentity: null, stampChangedAt: null,
    });
    const stream = useWS<unknown>(
        () => setTelemetry(previous => previous.condition === 'disconnected' ? previous : {
            ...previous, data: retainWarnings(previous.data, initial), condition: 'disconnected',
        }),
        () => { /* Open is transport liveness, not a validated sample. */ },
        raw => {
            const decoded = decode(raw);
            if (decoded.kind !== 'valid') {
                setTelemetry(previous => previous.condition === decoded.kind ? previous : {
                    ...previous, data: retainWarnings(previous.data, initial), condition: decoded.kind,
                });
                return;
            }
            const at = performance.now();
            const identity = telemetryStampIdentity(raw);
            setTelemetry(previous => ({
                data: retainWarnings(previous.data, decoded.data), condition: 'valid',
                assertions: accumulateAssertions ? accumulateAssertions(previous.assertions, decoded.data) : initial,
                lastValidReceiptAt: at, stampIdentity: identity,
                stampChangedAt: identity === null ? null
                    : identity === previous.stampIdentity ? previous.stampChangedAt : at,
            }));
        },
    );
    const {start, stop} = stream;
    useEffect(() => {
        start(`/api/mowglinext/subscribe/${topic}`);
        return stop;
    }, [topic, start, stop]);
    return {...stream, telemetry};
}
