import type {GnssStatus} from '../types/ros';
import {HighLevelStatusConstants as H, GnssStatusConstants as G} from '../types/ros';
import type {EmergencyDisplay, HighLevelDisplay} from './statusTelemetry';
import {hasEmergency, emergencyClearanceUnverified, emergencyIsKnown} from './statusTelemetry';
import {deriveGpsStatus, displayHorizontalAccuracyM} from './gpsStatus';
import {getBatteryLevel} from './battery';
import type {DiagnosticsSnapshot} from '../hooks/useDiagnosticsSnapshot';
import type {TimestampedDiagnosticStatus} from '../hooks/useDiagnostics';
import {DIAGNOSTIC_STALE_MS} from '../hooks/useDiagnostics';
import type {ValidatedTelemetry} from '../hooks/useValidatedTelemetry';
import type {FusionGraphStats} from '../hooks/useFusionGraphDiagnostics';

export type DiagnosticHealthLevel = 'critical' | 'warning' | 'unknown' | 'ok' | 'not_applicable';
export interface HealthCheck {id: string; level: DiagnosticHealthLevel; reason: string; evidence?: string[]}
export interface DiagnosticHealthInput {
    emergency: EmergencyDisplay; highLevel: HighLevelDisplay;
    emergencyFresh: boolean; operationalFresh: boolean; batteryFresh: boolean;
    connected: boolean; gnss: GnssStatus; batteryPercent: number;
    snapshot: DiagnosticsSnapshot | null; snapshotFresh: boolean;
    diagnostics: TimestampedDiagnosticStatus[]; nowMs: number; nowMonotonic: number;
    /** Browser monotonic disconnect boundary, never a civil date or ROS epoch. */
    diagnosticsDisconnectedAt?: number | null;
    settings: Record<string, unknown>; recoveryStuck?: boolean; fusion?: FusionGraphStats | null;
}

/** Delivery/progression evidence only. This is never a motion authorization. */
export function diagnosticTelemetryFresh<T>(telemetry: ValidatedTelemetry<T>, now: number): boolean {
    if (telemetry.publicationPolicy === 'receipt') {
        const at = telemetry.lastValidReceiptAt;
        // Only the validity and liveness of the reported context are known.
        // This does not establish a new observation, physical state or replay freedom.
        return telemetry.condition === 'valid' && at !== null && now >= at && now - at < DIAGNOSTIC_STALE_MS;
    }
    const identityAt = telemetry.publicationProgressAt ?? telemetry.stampChangedAt ?? telemetry.lastValidReceiptAt;
    return telemetry.condition === 'valid' && telemetry.publicationRegressed !== true && telemetry.publicationBlocked !== true && identityAt !== null && now >= identityAt
        && now - identityAt < DIAGNOSTIC_STALE_MS
        && (telemetry.publicationBlocked === false || telemetry.lastDisconnectAt === undefined || (telemetry.publicationProgressAt ?? telemetry.stampChangedAt) !== null
            && (telemetry.publicationProgressAt ?? telemetry.stampChangedAt ?? -Infinity) > telemetry.lastDisconnectAt);
}

const stoppedContainerStates = new Set(['created', 'restarting', 'removing', 'paused', 'exited', 'dead']);
/** Unknown Docker strings are not evidence that a service stopped. */
export function containerObservationLevel(state: string): 'ok' | 'warning' | 'unknown' {
    return state === 'running' ? 'ok' : stoppedContainerStates.has(state) ? 'warning' : 'unknown';
}

const operationalStates = new Set<number>([H.HIGH_LEVEL_STATE_IDLE, H.HIGH_LEVEL_STATE_AUTONOMOUS, H.HIGH_LEVEL_STATE_RECORDING, H.HIGH_LEVEL_STATE_MANUAL_MOWING]);
const rank: Record<DiagnosticHealthLevel, number> = {critical: 4, warning: 3, unknown: 2, ok: 1, not_applicable: 0};
const check = (id: string, level: DiagnosticHealthLevel, reason: string, evidence?: string[]): HealthCheck => ({id, level, reason, evidence});

function diagnosticNumber(entry: TimestampedDiagnosticStatus, key: string): number | undefined {
    const text = entry.values.find(pair => pair.key === key)?.value.trim();
    if (!text || !/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?$/i.test(text)) return undefined;
    const value = Number(text);
    return Number.isFinite(value) ? value : undefined;
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

/** One display verdict and per-check conclusions; all inputs are already decoded. */
export function deriveDiagnosticHealth(input: DiagnosticHealthInput) {
    const {emergency, highLevel, connected, nowMonotonic, settings} = input;
    const checks: HealthCheck[] = [];
    const clearance = emergencyClearanceUnverified(emergency, highLevel);
    checks.push(check('emergency', hasEmergency(emergency, highLevel) ? 'critical'
        : connected && input.emergencyFresh && emergencyIsKnown(emergency, highLevel) ? 'ok' : 'unknown',
    clearance ? 'clearance' : hasEmergency(emergency, highLevel) ? 'emergency' : 'emergency'));
    const operational = connected && input.operationalFresh && highLevel.state !== undefined && operationalStates.has(highLevel.state);
    checks.push(check('operation', operational ? 'ok' : 'unknown', 'operation'));
    const autonomous = operational && highLevel.state === H.HIGH_LEVEL_STATE_AUTONOMOUS;
    const snapshot = input.snapshotFresh ? input.snapshot : null;
    const containers = snapshot?.containers;
    // Installed standard Compose names (versions.go), corroborated by the
    // updater image role. An old renamed backup is not a required running service.
    const core = [{name: 'mowgli-ros2', component: 'mowgli-ros2'}, {name: 'mowgli-gui', component: 'mowglinext-gui'}];
    const required = (c: {name: string; component?: string}) => core.some(role => role.name === c.name && role.component === c.component);
    const stoppedCore = containers?.filter(c => required(c) && containerObservationLevel(c.state) === 'warning');
    const otherApplicable = containers?.filter(c => c.component && !core.some(role => role.component === c.component)
        && !(c.component.startsWith('lidar-') && settings.lidar_enabled === false));
    const stoppedOther = otherApplicable?.filter(c => containerObservationLevel(c.state) === 'warning');
    const unknownOther = otherApplicable?.some(c => containerObservationLevel(c.state) === 'unknown');
    const coreConfirmed = core.every(role => containers?.some(c => c.name === role.name && c.component === role.component && c.state === 'running'));
    checks.push(check('containers', !snapshot || snapshot.containers_available === false ? 'unknown'
        : stoppedCore?.length ? 'critical' : stoppedOther?.length ? 'warning'
            : snapshot.containers_available === true && coreConfirmed && !unknownOther ? 'ok' : 'unknown', 'containers',
    containers?.filter(c => c.state !== 'running').map(c => c.name)));
    const cross = snapshot?.cross_checks;
    checks.push(check('configuration', !cross || !['ok', 'warn', 'error'].includes(cross.overall_status) ? 'unknown'
        : cross.overall_status === 'error' ? 'warning' : cross.overall_status !== 'ok' || cross.warnings.length ? 'unknown' : 'ok', 'configuration', cross?.warnings));
    const temperature = snapshot?.system.cpu_temperature;
    // Legacy producers used zero for read failure; explicit modern availability disambiguates zero.
    const cpuKnown = typeof temperature === 'number' && Number.isFinite(temperature)
        && (snapshot?.system.cpu_temperature_available === true
            || snapshot?.system.cpu_temperature_available === undefined && temperature !== 0);
    checks.push(check('cpu', !cpuKnown ? 'unknown' : temperature > 70 ? 'critical' : temperature > 55 ? 'warning' : 'ok', 'cpu', cpuKnown ? [`${temperature} °C`] : undefined));
    const battery = input.batteryFresh && connected ? getBatteryLevel(input.batteryPercent) : 'unknown';
    checks.push(check('battery', battery === 'danger' ? 'critical' : battery === 'warn' ? 'warning' : battery, 'battery', battery !== 'unknown' ? [`${input.batteryPercent}%`] : undefined));
    const gps = deriveGpsStatus(input.gnss);
    const accuracy = displayHorizontalAccuracyM(input.gnss);
    // gnss_stack=disabled only disables the direct bridge; MAVROS can still own GNSS.
    const negative = input.gnss.fix_valid === false || gps.fixType === 'NO_FIX' || input.gnss.fix_type === G.FIX_TYPE_DEAD_RECKONING || input.gnss.dead_reckoning === true;
    checks.push(check('gnss', !connected ? 'unknown' : negative ? 'warning'
        : gps.fixType === 'UNKNOWN' ? 'unknown' : gps.fixType !== 'RTK_FIX' ? 'warning'
            : accuracy === undefined ? 'unknown' : accuracy > 0.1 ? 'warning' : 'ok', 'gnss', gps.fixType !== 'UNKNOWN' ? [gps.label, ...(accuracy !== undefined ? [`${accuracy} m`] : [])] : undefined));
    const entries = input.diagnostics;
    const fresh = (entry: TimestampedDiagnosticStatus) => {
        const at = entry.healthProgressAt ?? entry.receivedMonotonic;
        return connected && entry.healthBlocked !== true && Number.isFinite(at)
            && nowMonotonic >= at && nowMonotonic - at < DIAGNOSTIC_STALE_MS
            && (entry.healthBlocked === false || input.diagnosticsDisconnectedAt == null
                || entry.publicationStamp !== undefined && entry.receivedMonotonic > input.diagnosticsDisconnectedAt);
    };
    // No text-based exclusions: a driver phrase cannot erase an ERROR.
    const notApplicableReport = (entry: TimestampedDiagnosticStatus) => settings.lidar_enabled === false
        && entry.name === 'LiDAR' && entry.hardware_id === 'mowgli/lidar' && unavailableReport(entry);
    const current = entries.filter(entry => fresh(entry) && !notApplicableReport(entry));
    const failures = current.filter(entry => (entry.level === 1 || entry.level === 2) && !unavailableReport(entry));
    const stale = entries.filter(entry => !notApplicableReport(entry) && (!fresh(entry) || entry.level === 3 || unavailableReport(entry)));
    const criticalReports = failures.filter(entry => entry.level === 2 && (
        entry.name === 'Motors' && entry.hardware_id === 'mowgli/motors'
            && ['esc_temperature_c', 'motor_temperature_c'].some(key => diagnosticNumber(entry, key) !== undefined)
        || entry.name === 'Emergency System' && entry.hardware_id === 'mowgli/emergency'
            && entry.values.some(pair => ['active', 'latched'].includes(pair.key) && pair.value === 'true')
        || autonomous && entry.name === 'Path Tracking' && entry.hardware_id === 'mowgli/controller_server'
            && diagnosticNumber(entry, 'recent_p95_lateral_error_m') !== undefined));
    const batteryReport = failures.find(entry => entry.level === 2 && entry.name === 'Battery' && entry.hardware_id === 'mowgli/battery'
        && (diagnosticNumber(entry, 'voltage_v') ?? 0) > 0 && diagnosticNumber(entry, 'percentage') !== undefined
        && (diagnosticNumber(entry, 'percentage') ?? -1) >= 0 && (diagnosticNumber(entry, 'percentage') ?? Infinity) <= 100);
    if (batteryReport) {
        criticalReports.push(batteryReport);
        const index = checks.findIndex(item => item.id === 'battery');
        checks[index] = {...checks[index], level: 'critical', evidence: [...(checks[index].evidence ?? []),
            `ROS: ${diagnosticNumber(batteryReport, 'percentage')}% · ${diagnosticNumber(batteryReport, 'voltage_v')} V`]};
    }
    checks.push(check('ros', criticalReports.length ? 'critical' : failures.length ? 'warning' : stale.length || !current.length ? 'unknown' : 'ok', 'ros',
        (criticalReports.length ? criticalReports : failures).map(entry => `${entry.name}: ${entry.message}`)));
    checks.push(check('lidar', settings.lidar_enabled === false ? 'not_applicable' : 'unknown', 'lidar'));
    // An absent optional configuration is not proof of a missing required sensor.
    if (settings.lidar_enabled === true) {
        const lidar = current.find(entry => entry.name === 'LiDAR' && entry.hardware_id === 'mowgli/lidar');
        checks[checks.length - 1] = check('lidar', !lidar || lidar.level === 3 || unavailableReport(lidar) ? 'unknown' : lidar.level === 0 ? 'ok' : 'warning', 'lidar');
    } else if (settings.lidar_enabled !== false) checks.pop();
    if (autonomous) {
        const fusion = input.fusion;
        const at = fusion?.healthProgressAt ?? fusion?.distinctMonotonic;
        const recent = fusion && connected && fusion.healthBlocked !== true && at !== undefined
            && input.nowMonotonic >= at && input.nowMonotonic - at < DIAGNOSTIC_STALE_MS
            && (fusion.healthBlocked === false || fusion.lastDisconnectAt === undefined || at > fusion.lastDisconnectAt);
        checks.push(check('localizer', !recent || fusion.level === 3 ? 'unknown' : fusion.level === 2 ? 'critical'
            : fusion.level === 1 ? 'warning' : !/^[1-9]\d*$/.test(fusion.values.total_nodes ?? '') ? 'unknown' : 'ok', 'localizer', recent ? [fusion.message] : undefined));
    }
    if (input.recoveryStuck) checks.push(check('recovery', 'warning', 'recovery'));
    const primary = checks.reduce((a, b) => rank[b.level] > rank[a.level] ? b : a);
    return {level: primary.level, primary, checks, available: checks.filter(c => c.level !== 'unknown' && c.level !== 'not_applicable'), unknown: checks.filter(c => c.level === 'unknown'),
        notApplicable: checks.filter(c => c.level === 'not_applicable'), critical: checks.filter(c => c.level === 'critical'),
        alerts: entries.filter(entry => entry.level > 0 || !fresh(entry)).map(entry => ({...entry,
            reportedLevel: entry.level, notApplicable: notApplicableReport(entry), level: fresh(entry) && !unavailableReport(entry) ? entry.level : 3}))};
}
