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
    return isDiagnosticStatus(value) && 'receivedAt' in value && finiteMeasurement(value.receivedAt);
}

/** Publication identity only; not a new physical acquisition. */
export function diagnosticPublicationIdentity(value: unknown): string | null {
    if (!isJsonObject(value) || !isJsonObject(value.header) || !isRosStamp(value.header.stamp)) return null;
    const {sec, nanosec} = value.header.stamp;
    return sec === 0 && nanosec === 0 ? null : `${sec}:${nanosec}`;
}
