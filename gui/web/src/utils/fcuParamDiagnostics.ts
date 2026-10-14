import type {ParamDocument} from './fcuParamDocument';
import type {ParamIdentity} from './fcuParamIdentity';
import {profileForIdentity, type ParamHardware, type ParamReview} from './fcuParamProfile';

export const diagnosticGroups = ['communication', 'actuators', 'can', 'safety', 'navigation', 'telemetry'] as const;
export type DiagnosticGroup = typeof diagnosticGroups[number];
export type GnssDiagnosticBackend = 'unknown' | 'mavros' | 'universal';
export type DiagnosticState = 'compatible' | 'review' | 'missing' | 'incompatible';
export function diagnosticState(row: ParamReview): DiagnosticState {
    return row.state === 'compliant' ? 'compatible' : row.state === 'missing' ? 'missing'
        : row.state === 'mismatch' && !row.provisional && row.proposed !== undefined ? 'incompatible' : 'review';
}
/** Presentation projection only: original document and comparison rules remain intact. */
export function groupDiagnosticRows(rows: readonly ParamReview[], hardware: ParamHardware, gnss: GnssDiagnosticBackend) {
    const grouped: Record<DiagnosticGroup, ParamReview[]> = {communication: [], actuators: [], can: [], safety: [], navigation: [], telemetry: []};
    for (const original of rows) {
        // Acceptance/stream-override options are critical context even though the generic profile classifies them optional.
        const row: ParamReview = original.parameter === 'MAV_OPTIONS' || original.parameter === `MAV${hardware.mavlinkChannel}_OPTIONS`
            ? {...original, category: 'conditional'} : original;
        if (row.category !== 'required' && row.category !== 'conditional') continue;
        if (row.id.startsWith('personal.')) continue;
        if (row.id.startsWith('hardware.') && row.parameter !== `CAN_P${hardware.canPort}_BITRATE` && !/^BATT\d*_MONITOR$/.test(row.parameter)) continue;
        if (row.id.startsWith('stream.')) continue; // Dedicated flow diagnostics replace duplicate stream rows.
        if (row.id.startsWith('navigation.GPS') && gnss === 'universal') continue;
        const group = row.id.startsWith('link.') || row.parameter === 'MAV_OPTIONS' ? 'communication'
            : row.parameter === `MAV${hardware.mavlinkChannel}_OPTIONS` ? 'telemetry'
            : row.id.startsWith('drive.') || row.id.startsWith('blade.') ? 'actuators'
            : row.id.startsWith('can.') || row.id.startsWith('rpm.') || row.parameter.startsWith('CAN_P') ? 'can'
            : /^BATT\d*_MONITOR$/.test(row.parameter) ? 'telemetry'
            : row.id.startsWith('safety.') || row.id.startsWith('arming.') ? 'safety' : 'navigation';
        if (group === 'can' && hardware.drive !== 'vesc-dronecan') continue;
        grouped[group].push(row);
    }
    return grouped;
}
export function summarizeDiagnostics(states: readonly DiagnosticState[]) {
    return {compatible: states.filter(s => s === 'compatible').length, review: states.filter(s => s === 'review').length,
        missing: states.filter(s => s === 'missing').length, incompatible: states.filter(s => s === 'incompatible').length};
}
export interface TelemetryFlow {
    id: string; messages: string; consumer: string; parameters: {name: string; configuredHz?: number; ambiguous: boolean}[];
    state: 'review'; freshnessReferenceMs?: number; freshnessSource: string;
}
const owner = 'https://github.com/Pepeuch/MowgliMAVROS/blob/bf5bb0db3e4e3a0784ecd486e4c4c7909c0063e7/';
/** Offline inventory, never acquisition-rate evidence. No reliable runtime frequency measurement is exposed to this tool. */
export function telemetryDiagnostics(document: ParamDocument | null, identity: ParamIdentity, h: ParamHardware, gnss: GnssDiagnosticBackend): TelemetryFlow[] {
    if (!profileForIdentity(identity)) return [];
    const flows: TelemetryFlow[] = [];
    const add = (id: string, messages: string, consumer: string, suffixes: string[], freshnessReferenceMs?: number,
        source = 'ros2/src/mowgli_mavros_bridge/config/hardware_bridge_mavros.yaml') => {
        flows.push({id, messages, consumer, state: 'review', freshnessReferenceMs, freshnessSource: owner + source,
            parameters: suffixes.map(suffix => {
                const name = `MAV${h.mavlinkChannel}_${suffix}`;
                const ambiguous = document?.duplicates.has(name) ?? false;
                const value = document?.entries.get(name)?.value;
                return {name, ambiguous, configuredHz: !ambiguous && value !== undefined && value >= 0 ? value : undefined};
            })});
    };
    add('state', 'HEARTBEAT (#0), SYS_STATUS (#1)', 'sys_status → /mavros/state, /mavros/sys_status → hardware_bridge', ['EXT_STAT']);
    add('imu', 'ATTITUDE (#30), RAW_IMU (#27) / SCALED_IMU (#26)', 'imu → /mavros/imu/data → hardware_bridge', ['EXTRA1', 'RAW_SENS'], 5000);
    if (gnss !== 'universal') add('gnss', 'GPS_RAW_INT (#24) / GPS2_RAW (#124), GPS_RTK (#127) / GPS2_RTK (#128)', 'universal_gnss → mowgli_gnss (mavros_gnss_adapter) → /gps/status', ['EXT_STAT'], 3000, 'ros2/src/mavros_gnss_adapter/include/mavros_gnss_adapter/adapter_state.hpp');
    if (gnss === 'universal') add('gnssExternal', '—', 'Universal GNSS → /gps/status → hardware_bridge (outside MAVROS)', [], 5000);
    if (h.drive === 'vesc-dronecan') {
        add('esc', h.legacyRpm ? 'ESC_TELEMETRY_1_TO_4 (#11030), RPM (#226)' : 'ESC_STATUS (#291), ESC_INFO (#290) / legacy ESC_TELEMETRY (#11030)',
            'esc_wheel_odometry → EscObservation / wheel_ticks / wheel_odom', h.legacyRpm ? ['EXTRA1'] : [], 3000);
    } else add('wheel', '—', 'Configured encoder / wheel odometry source → /wheel_odom (if required)', [], 5000);
    if (h.blade) add('bladeRpm', 'ESC_STATUS (#291) / ESC_TELEMETRY_1_TO_4 (#11030)', 'esc_wheel_odometry → hardware_bridge → BladeControl', h.legacyRpm ? ['EXTRA1'] : [], 1000,
            'ros2/src/mowgli_mavros_bridge/include/mowgli_mavros_bridge/blade_control.hpp');
    add('battery', 'BATTERY_STATUS (#147)', 'battery_observer → /hardware_bridge/power → hardware_bridge', ['EXTRA3'], 5000,
        'ros2/src/mavros_battery_observer/src/battery_observer_plugin.cpp');
    add('rc', 'RC_CHANNELS (#65), RC_CHANNELS_RAW (#35)', 'rc_io → /mavros/rc/in (if RC is used)', ['RC_CHAN']);
    if (h.blade) add('bladeOutput', 'SERVO_OUTPUT_RAW (#36)', 'rc_io → /mavros/rc/out → hardware_bridge / BladeControl', ['RC_CHAN']);
    return flows;
}
