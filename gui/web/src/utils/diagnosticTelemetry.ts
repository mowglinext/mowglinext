import type {DiagnosticStatus, TimestampedDiagnosticStatus} from '../hooks/useDiagnostics';
import {isJsonObject} from './json';
import {finiteMeasurement, isRosStamp} from './physicalTelemetry';

export function isDiagnosticStatus(value: unknown): value is DiagnosticStatus {
    if (!isJsonObject(value) || !finiteMeasurement(value.level) || !Number.isInteger(value.level)
        || value.level < 0 || value.level > 3 || typeof value.name !== 'string'
        || typeof value.message !== 'string' || typeof value.hardware_id !== 'string'
        || !Array.isArray(value.values)) return false;
    const pairs: unknown[] = value.values;
    const keys = new Set<string>();
    return pairs.every(pair => {
        if (!isJsonObject(pair) || typeof pair.key !== 'string' || typeof pair.value !== 'string' || keys.has(pair.key)) return false;
        keys.add(pair.key);
        return true;
    });
}
export function isTimestampedDiagnosticStatus(value: unknown): value is TimestampedDiagnosticStatus {
    return isDiagnosticStatus(value) && 'receivedAt' in value && finiteMeasurement(value.receivedAt)
        && 'receivedMonotonic' in value && finiteMeasurement(value.receivedMonotonic);
}

/** Publication identity only; not a new physical acquisition. */
export function diagnosticPublicationIdentity(value: unknown): string | null {
    if (!isJsonObject(value) || !isJsonObject(value.header) || !isRosStamp(value.header.stamp)) return null;
    const {sec, nanosec} = value.header.stamp;
    return sec === 0 && nanosec === 0 ? null : `${sec}:${nanosec}`;
}

export interface DiagnosticProgress {
    stamp?: {sec: number; nanosec: number};
    at: number;
    blocked: boolean;
}

/** Ordered diagnostic publication evidence, never a physical acquisition time. */
export function advanceDiagnosticProgress(previous: DiagnosticProgress | undefined, raw: unknown, at: number): DiagnosticProgress {
    const stamp = isJsonObject(raw) && isJsonObject(raw.header) && isRosStamp(raw.header.stamp)
        && (raw.header.stamp.sec !== 0 || raw.header.stamp.nanosec !== 0)
        ? {sec: raw.header.stamp.sec, nanosec: raw.header.stamp.nanosec} : undefined;
    const high = previous?.stamp;
    if (previous && high && (!stamp || stamp.sec < high.sec || stamp.sec === high.sec && stamp.nanosec < high.nanosec)) {
        return {...previous, at: previous.at, blocked: true};
    }
    if (stamp && (!high || stamp.sec > high.sec || stamp.sec === high.sec && stamp.nanosec > high.nanosec)) {
        return {stamp, at, blocked: false};
    }
    if (previous?.blocked || stamp && previous) return previous;
    return {at, blocked: false}; // Initial unstamped legacy delivery remains explicitly unproven.
}
