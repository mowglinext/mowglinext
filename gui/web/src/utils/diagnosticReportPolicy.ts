import type {TimestampedDiagnosticStatus} from '../hooks/useDiagnostics';

/** Local evidence window, provisionally unchanged; never a physical acquisition age. */
export const DIAGNOSTIC_STALE_MS = 30_000;
export interface DiagnosticReportContext {
    connected: boolean;
    nowMonotonic: number;
    diagnosticsDisconnectedAt?: number | null;
    lidarEnabled?: unknown;
    ready?: boolean;
}
type ReportEvidence = Pick<TimestampedDiagnosticStatus, 'healthProgressAt' | 'receivedMonotonic' | 'healthBlocked' | 'publicationStamp'>;

/** Exact shared health predicate, including legacy and post-disconnect barriers. */
export function diagnosticReportFresh(entry: ReportEvidence, input: DiagnosticReportContext): boolean {
    const at = entry.healthProgressAt ?? entry.receivedMonotonic;
    return input.connected && entry.healthBlocked !== true && Number.isFinite(at)
        && input.nowMonotonic >= at && input.nowMonotonic - at < DIAGNOSTIC_STALE_MS
        && (entry.healthBlocked === false || input.diagnosticsDisconnectedAt == null
            || entry.publicationStamp !== undefined && entry.receivedMonotonic > input.diagnosticsDisconnectedAt);
}

/** Optional producer declaration, never inferred from a running container. */
export function mavrosWheelRequirement(entry: TimestampedDiagnosticStatus): boolean | undefined {
    if (entry.name !== 'mowgli_mavros_bridge/wheel_odometry_source' || entry.hardware_id !== '') return undefined;
    const value = entry.values.find(pair => pair.key === 'required')?.value;
    return value === 'true' ? true : value === 'false' ? false : undefined;
}

/** These monitoring checks report only absence/age, not a measured hardware fault. */
export function unavailableReport(entry: TimestampedDiagnosticStatus): boolean {
    if (entry.level === 0) return false;
    // This exact readiness WARN reports missing input, not a measured fault.
    // An ERROR or a different message retains its normal immediate severity.
    if (entry.level === 1 && entry.message === 'missing_or_stale' && mavrosWheelRequirement(entry) !== undefined) return true;
    const ageOnly = new Map([['Hardware Bridge', 'mowgli/hardware_bridge'], ['IMU', 'mowgli/imu'],
        ['LiDAR', 'mowgli/lidar'], ['Odometry', 'mowgli/odometry']]);
    if (ageOnly.get(entry.name) === entry.hardware_id) return true;
    if (entry.name === 'GPS' && entry.hardware_id === 'mowgli/gps') {
        const status = entry.values.find(pair => pair.key === 'fix_status')?.value;
        // A negative NavSatStatus confirms No Fix; otherwise this check reports missing/old input.
        return status === undefined || status.trim() === '' || !Number.isFinite(Number(status)) || Number(status) >= 0;
    }
    if (entry.name === 'EKF Map' && entry.hardware_id === 'robot_localization/filtered_map') {
        return !entry.values.some(pair => ['flat_ok', 'z_drift_ok'].includes(pair.key) && pair.value === 'false');
    }
    if (entry.name === 'Emergency System' && entry.hardware_id === 'mowgli/emergency') return entry.values.length === 0;
    return false;
}

/** Keep the health exclusion exact: a disabled sensor cannot erase an unrelated driver fault. */
export function diagnosticReportNotApplicable(entry: TimestampedDiagnosticStatus, lidarEnabled: unknown): boolean {
    return lidarEnabled === false && entry.name === 'LiDAR' && entry.hardware_id === 'mowgli/lidar' && unavailableReport(entry);
}

type ReportQualification =
    | {admission: 'accepted'; classification: 'fault'; severity: 1 | 2}
    | {admission: 'accepted'; classification: 'explicit_ok'}
    | {admission: 'control'; classification: 'expired' | 'unverified' | 'unavailable' | 'not_applicable'};

/** Qualifies a logical reported condition, never a measured hardware fault by ROS level alone. */
export function qualifyDiagnosticReport(entry: TimestampedDiagnosticStatus, input: DiagnosticReportContext): ReportQualification {
    if (input.ready === false) return {admission: 'control', classification: 'unverified'};
    if (diagnosticReportNotApplicable(entry, input.lidarEnabled)) return {admission: 'control', classification: 'not_applicable'};
    if (!diagnosticReportFresh(entry, input)) {
        const at = entry.healthProgressAt ?? entry.receivedMonotonic;
        return {admission: 'control', classification: input.connected && entry.healthBlocked !== true
            && Number.isFinite(at) && input.nowMonotonic >= at && input.nowMonotonic - at >= DIAGNOSTIC_STALE_MS ? 'expired' : 'unverified'};
    }
    if (entry.level === 3 || unavailableReport(entry)) return {admission: 'control', classification: 'unavailable'};
    if (entry.level === 0) return {admission: 'accepted', classification: 'explicit_ok'};
    if (entry.level === 1 || entry.level === 2) return {admission: 'accepted', classification: 'fault', severity: entry.level};
    return {admission: 'control', classification: 'unverified'};
}
