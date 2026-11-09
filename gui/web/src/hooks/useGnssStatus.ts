import {useEffect, useMemo, useState} from 'react';
import {GnssStatusConstants as C, type GnssStatus} from '../types/ros';
import {DIAGNOSTIC_STALE_MS, useDiagnostics} from './useDiagnostics';
import type {ValidatedTelemetry} from './useValidatedTelemetry';
import {useWS} from './useWS';
import {decodeGnssStatus, gnssObservationIdentity} from '../utils/gnssTelemetry';
import {deriveGnssStatusFromDiagnostics} from '../utils/gpsStatus';

// Reuse the existing GUI diagnostic delivery window, not a ROS acquisition age.
export const GNSS_DISPLAY_STALE_MS = DIAGNOSTIC_STALE_MS;
const initialTelemetry = (): ValidatedTelemetry<GnssStatus> => ({data: {}, assertions: {}, hasReceivedFrame: false,
    condition: 'missing', lastValidReceiptAt: null, stampIdentity: null, stampChangedAt: null});

const laterHeader = (a: GnssStatus['header'], b: GnssStatus['header']): boolean => !!a && (!b
    || a.stamp.sec > b.stamp.sec || a.stamp.sec === b.stamp.sec && a.stamp.nanosec > b.stamp.nanosec);

/** GNSS-only lifecycle: transport delivery and producer observation identity stay separate. */
export const useGnssStatusState = () => {
    const {diagnostics} = useDiagnostics();
    // This disconnect boundary is used only for local diagnostic delivery ages.
    const [state, setState] = useState<{telemetry: ValidatedTelemetry<GnssStatus>; disconnectedAt: number;
        blockedIdentity: string | null; blockedDiagnosticStamps: (string | undefined)[];
        sequence: number; blockedSequence: number; source: string | undefined; requiresIdentity: boolean;
        header: GnssStatus['header']; blockedHeader: GnssStatus['header'];
        sources: ReadonlyMap<string | undefined, {sequence: number; header: GnssStatus['header']}>}>(() => ({telemetry: initialTelemetry(), disconnectedAt: -Infinity,
        blockedIdentity: null, blockedDiagnosticStamps: [], sequence: 0, blockedSequence: 0, source: undefined, requiresIdentity: false, header: undefined, blockedHeader: undefined, sources: new Map()}));
    const [clock, setClock] = useState(() => ({mono: performance.now()}));
    const stream = useWS<unknown>(
        () => {
            const at = performance.now();
            setState(previous => ({...previous, disconnectedAt: at,
                blockedIdentity: previous.telemetry.stampIdentity, blockedSequence: previous.sequence, blockedHeader: previous.header,
                requiresIdentity: previous.requiresIdentity || previous.telemetry.lastValidReceiptAt !== null,
                blockedDiagnosticStamps: (diagnostics.status ?? []).map(entry => entry.publicationStamp),
                telemetry: {...previous.telemetry, data: {}, condition: 'disconnected'}}));
        },
        () => { /* Reconnection is not a new publication. */ },
        raw => {
            const decoded = decodeGnssStatus(raw);
            const at = performance.now();
            setState(current => {
                const old = current.telemetry;
                let previous = current;
                if (decoded.kind !== 'valid') return {...previous, blockedIdentity: old.stampIdentity, blockedSequence: previous.sequence, blockedHeader: previous.header,
                    requiresIdentity: previous.requiresIdentity || old.lastValidReceiptAt !== null,
                    telemetry: {...old, data: {}, hasReceivedFrame: true, condition: decoded.kind}};
                const identity = gnssObservationIdentity(raw);
                const sequence = decoded.data.position_observation_sequence ?? 0;
                const source = decoded.data.backend;
                // An absent label cannot establish an anonymous producer after a
                // named source. Preserve its barriers until attributable progress.
                if (source === undefined && [...current.sources.keys()].some(key => key !== undefined)) {
                    return {...current, blockedSequence: current.sequence, blockedHeader: current.header,
                        requiresIdentity: true, telemetry: {...old, data: {}, hasReceivedFrame: true, condition: 'unsupported'}};
                }
                const switched = source !== current.source;
                const known = current.sources.get(source);
                const firstAfterSwitch = switched && !known && old.hasReceivedFrame;
                if (switched) previous = {...current, sequence: known?.sequence ?? 0,
                    header: known?.header, blockedSequence: known?.sequence ?? 0,
                    blockedHeader: known?.header, blockedIdentity: null};
                const regressed = sequence < previous.sequence
                    || sequence === 0 && laterHeader(previous.header, decoded.data.header);
                const header = laterHeader(decoded.data.header, previous.header) ? decoded.data.header : previous.header;
                const expired = old.lastValidReceiptAt !== null
                    && at - (old.stampChangedAt ?? old.lastValidReceiptAt) >= GNSS_DISPLAY_STALE_MS;
                const requiresIdentity = previous.requiresIdentity || expired;
                const blockedHeader = expired ? previous.header : previous.blockedHeader;
                const blocked = firstAfterSwitch || switched && old.hasReceivedFrame && identity === null || regressed || sequence === 0 && blockedHeader !== undefined && !laterHeader(decoded.data.header, blockedHeader) || sequence > 0 && sequence <= previous.blockedSequence
                    || identity !== null && identity === previous.blockedIdentity || requiresIdentity && identity === null;
                // A replay cannot reinstate a positive state after loss/invalidity/no-fix.
                // Explicit negative positioning is always allowed to clear the display.
                const negative = decoded.data.fix_valid === false || decoded.data.fix_type === C.FIX_TYPE_NO_FIX
                    || decoded.data.fix_type === C.FIX_TYPE_DEAD_RECKONING || decoded.data.dead_reckoning === true;
                const sources = new Map(current.sources);
                sources.set(source, {sequence: Math.max(sequence, previous.sequence), header});
                return {...previous, sources, source, header, blockedHeader: negative || blocked ? header : blockedHeader,
                    requiresIdentity: requiresIdentity || negative || blocked,
                    sequence: Math.max(sequence, previous.sequence),
                    blockedIdentity: negative ? identity : previous.blockedIdentity,
                    blockedSequence: negative || blocked ? Math.max(sequence, previous.sequence) : previous.blockedSequence,
                    telemetry: {...old, hasReceivedFrame: true, data: blocked && !negative ? {} : decoded.data,
                        condition: blocked && !negative ? 'unsupported' : 'valid',
                        lastValidReceiptAt: blocked && !negative ? old.lastValidReceiptAt : at,
                        stampIdentity: blocked && !negative ? old.stampIdentity : identity,
                        stampChangedAt: blocked && !negative ? old.stampChangedAt : identity === null ? null
                            : identity === old.stampIdentity ? old.stampChangedAt : at}};
            });
        },
    );
    const {start, stop} = stream;
    useEffect(() => {
        start('/api/mowglinext/subscribe/gnssStatus');
        const timer = setInterval(() => setClock({mono: performance.now()}), 1000);
        return () => {clearInterval(timer); stop();};
    }, [start, stop]);
    const {telemetry} = state;
    const projection = useMemo(() => deriveGnssStatusFromDiagnostics({status: diagnostics.status?.filter(entry =>
        (entry.receivedMonotonic > state.disconnectedAt || entry.receivedMonotonic === state.disconnectedAt
            && entry.publicationStamp !== undefined && !state.blockedDiagnosticStamps.includes(entry.publicationStamp))
        && Math.max(clock.mono, entry.receivedMonotonic) - entry.receivedMonotonic < DIAGNOSTIC_STALE_MS)}),
    [diagnostics, state.disconnectedAt, state.blockedDiagnosticStamps, clock.mono]);
    const status = useMemo(() => {
        if (!telemetry.hasReceivedFrame) return projection ?? {};
        if (telemetry.condition !== 'valid' || telemetry.lastValidReceiptAt === null) return {};
        const now = Math.max(clock.mono, telemetry.lastValidReceiptAt);
        const identityAt = telemetry.stampChangedAt ?? telemetry.lastValidReceiptAt;
        if (now - telemetry.lastValidReceiptAt >= GNSS_DISPLAY_STALE_MS || now - identityAt >= GNSS_DISPLAY_STALE_MS) return {};
        // Do not assemble typed values and independently timed receiver diagnostics.
        return telemetry.data;
    }, [telemetry, projection, clock.mono]);
    return {status, telemetry};
};

export const useGnssStatus = (): GnssStatus => useGnssStatusState().status;
