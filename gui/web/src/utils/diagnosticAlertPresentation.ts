import type {HardwareBackend} from '../constants/hardwareBackends';
import {HighLevelStatusConstants as H} from '../types/ros';
import type {deriveDiagnosticHealth} from './diagnosticHealth';

export type CurrentDiagnosticAlert = ReturnType<typeof deriveDiagnosticHealth>['alerts'][number];
export const RECEPTION_ALERT_DELAY_MS = 60_000;
/** Publication tokens come from the existing admission boundaries, never render time.
 * ageMs is delivery/progression age, not physical acquisition age. */
export interface RequiredDiagnosticSource {
    key: string;
    label: string;
    context?: string;
    publication: number | null;
    ageMs: number;
}
export interface ReceptionWatch {
    key: string;
    label: string;
    context?: string;
    publication: number | null;
    lastAdmissibleAt: number;
}
/** Presentation only. A replay/invalid frame/reconnect supplies no new token.
 * Sources no longer required are removed, so storage is bounded by configuration. */
export function advanceReceptionWatches(previous: readonly ReceptionWatch[], sources: readonly RequiredDiagnosticSource[], now: number): ReceptionWatch[] {
    const known = new Map(previous.map(source => [source.key, source]));
    return sources.map(source => {
        const candidate = known.get(source.key);
        const old = candidate?.context === source.context ? candidate : undefined;
        const progresses = source.publication !== null && Number.isFinite(source.publication)
            && (!old || old.publication === null || source.publication > old.publication)
            && Number.isFinite(source.ageMs) && source.ageMs >= 0;
        return {key: source.key, label: source.label, context: source.context,
            publication: progresses ? source.publication : old?.publication ?? null,
            lastAdmissibleAt: progresses ? Math.max(old?.lastAdmissibleAt ?? -Infinity, now - source.ageMs)
                : old?.lastAdmissibleAt ?? now};
    });
}
export const missingReceptionSources = (sources: readonly ReceptionWatch[], now: number) =>
    sources.filter(source => now - source.lastAdmissibleAt >= RECEPTION_ALERT_DELAY_MS);

/** Unknown operational context cannot cancel a previously established requirement. */
export function autonomousReceptionRequired(previous: boolean, state: number | undefined): boolean {
    return state === H.HIGH_LEVEL_STATE_AUTONOMOUS ? true
        : state !== undefined && [H.HIGH_LEVEL_STATE_IDLE, H.HIGH_LEVEL_STATE_RECORDING, H.HIGH_LEVEL_STATE_MANUAL_MOWING].some(value => state === Number(value)) ? false : previous;
}

/** diagnostics_node monitors the shared hardware contract, not a native node.
 * External MAVROS odometry is required only when its producer says so. */
export function backendReceptionReports(backend: HardwareBackend, mavrosWheelRequired: boolean) {
    if (backend === 'unknown') return [];
    return [['hardware', 'Hardware Bridge', 'mowgli/hardware_bridge'],
        ...(backend !== 'mavros' || mavrosWheelRequired ? [['odometry', 'Odometry', 'mowgli/odometry']] : [])];
}
