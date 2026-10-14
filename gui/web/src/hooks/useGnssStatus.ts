import {useMemo} from 'react';
import type {GnssStatus} from '../types/ros';
import {useDiagnostics} from './useDiagnostics';
import {useValidatedTelemetry} from './useValidatedTelemetry';
import {diagnosticPublicationIdentity} from '../utils/diagnosticTelemetry';
import {decodeGnssStatus} from '../utils/gnssTelemetry';
import {deriveGnssStatusFromDiagnostics, mergeGnssStatusDiagnosticProjection} from '../utils/gpsStatus';

const replaceStatus = (_previous: GnssStatus, next: GnssStatus): GnssStatus => next;

/** Receipt/identity metadata describes delivery, not physical GNSS acquisition. */
export const useGnssStatusState = () => {
    const {telemetry} = useValidatedTelemetry<GnssStatus>('gnssStatus', {}, decodeGnssStatus, replaceStatus, undefined, diagnosticPublicationIdentity);
    const {diagnostics} = useDiagnostics();
    const projection = useMemo(() => deriveGnssStatusFromDiagnostics(diagnostics), [diagnostics]);
    const status = useMemo(() => {
        if (!telemetry.hasReceivedFrame) return projection ?? {};
        // Invalid frames or transport loss must not resurrect a diagnostic fix.
        if (telemetry.condition !== 'valid') return {};
        return mergeGnssStatusDiagnosticProjection(telemetry.data, projection);
    }, [telemetry.condition, telemetry.hasReceivedFrame, telemetry.data, projection]);
    return {status, telemetry};
};

export const useGnssStatus = (): GnssStatus => useGnssStatusState().status;
