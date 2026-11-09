import type {ParamDocument} from './fcuParamDocument';
import type {ParamIdentity, FirmwareFamily} from './fcuParamIdentity';

export type Requirement = 'required' | 'conditional' | 'recommended' | 'optional' | 'calibration';
export interface ParamHardware {
    drive: 'unknown' | 'vesc-dronecan' | 'pwm-skid'; blade: boolean; legacyRpm: boolean;
    serialPort: number | null; mavlinkChannel: number; gcsSysId: number; targetSysId: number;
    rightOutput: number; leftOutput: number; bladeOutput: number;
    rightEscIndex: number; leftEscIndex: number; canPort: number; canDriver: number;
}
/** Editable reference bindings, not universally validated device settings. */
export const referenceParamHardware = (): ParamHardware => ({drive: 'unknown', blade: false, legacyRpm: false,
    serialPort: null, mavlinkChannel: 1, gcsSysId: 255, targetSysId: 1,
    rightOutput: 1, leftOutput: 2, bladeOutput: 3, rightEscIndex: 0, leftEscIndex: 1, canPort: 1, canDriver: 1});
export interface ParamRule {id: string; parameter: string; category: Requirement; reason: string;
    proposed?: number; accepted?: readonly number[]; sensitive: boolean; provisional: boolean; source: string; versions: readonly string[]; applicability: string; contract: string}
export interface ParamReview extends ParamRule {current?: number; state: 'compliant' | 'mismatch' | 'missing' | 'ambiguous' | 'review' | 'preserved'; editable: boolean}
const MAVLINK_DOC = 'https://ardupilot.org/rover/docs/common-mavlink-configuration.html';
const SERVO_DOC = 'https://ardupilot.org/dev/docs/mavlink-move-servo.html';
const RENAME_DOC = 'https://ardupilot.org/rover/docs/common-param-name-changes.html';
export interface RuntimeParamProfile {id: string; family: FirmwareFamily; vehicle: string; versions: readonly string[]; buildRules: (document: ParamDocument, hardware: ParamHardware) => ParamRule[]}
export const runtimeParamProfiles: readonly RuntimeParamProfile[] = [{id: 'ardurover-4.7.1-minimal-v1', family: 'ardupilot', vehicle: 'rover', versions: ['4.7.1'], buildRules: arduRover471Rules}];
export function profileForIdentity(identity: ParamIdentity) {
    return runtimeParamProfiles.find(p => p.family === identity.family && p.vehicle === identity.vehicle && p.versions.includes(identity.version));
}
export function validParamHardware(h: ParamHardware): boolean {
    const integer = (v: number, lo: number, hi: number) => Number.isInteger(v) && v >= lo && v <= hi;
    const link = integer(h.gcsSysId, 1, 255) && integer(h.targetSysId, 1, 255) && integer(h.mavlinkChannel, 1, 10)
        && (h.serialPort === null || integer(h.serialPort, 0, 9));
    if (!link || (h.blade && !integer(h.bladeOutput, 1, 16))) return false;
    if (h.drive === 'unknown') return true;
    const drive = integer(h.rightOutput, 1, 16) && integer(h.leftOutput, 1, 16) && h.rightOutput !== h.leftOutput
        && (!h.blade || (h.bladeOutput !== h.rightOutput && h.bladeOutput !== h.leftOutput));
    if (!drive || h.drive === 'pwm-skid') return drive;
    return integer(h.canPort, 1, 2) && integer(h.canDriver, 1, 2)
        && (!h.legacyRpm || (integer(h.rightEscIndex, 0, 15) && integer(h.leftEscIndex, 0, 15) && h.rightEscIndex !== h.leftEscIndex));
}
function arduRover471Rules(document: ParamDocument, h: ParamHardware): ParamRule[] {
    const rules: ParamRule[] = [];
    const add = (id: string, parameter: string, category: Requirement, reason: string, proposed?: number,
        sensitive = false, provisional = false, source = '', accepted?: readonly number[]) => {
        // A provisional hypothesis is a review item, never a guessed correction.
        const contractPath = id.startsWith('rpm.') ? 'ros2/src/mavros_esc_wheel_odometry/include/mavros_esc_wheel_odometry/wheel_tick_projection.hpp'
            : id.startsWith('arming.') ? 'ros2/src/mowgli_mavros_bridge/include/mowgli_mavros_bridge/fcu_session_controller.hpp'
            : id.startsWith('link.') ? 'ros2/src/mowgli_mavros_bridge/launch/mavros_backend.launch.py'
            : id.startsWith('blade.') || id.startsWith('safety.') || id.startsWith('navigation.') ? 'ros2/src/mowgli_mavros_bridge/src/mavros_hardware_bridge_node.cpp'
            : 'ros2/src/mowgli_mavros_bridge/src/firmware_provider.cpp';
        rules.push({id, parameter, category, reason, proposed: provisional ? undefined : proposed,
            accepted: provisional ? undefined : accepted, sensitive, provisional, source: source || `https://github.com/Pepeuch/MowgliMAVROS/blob/bf5bb0db3e4e3a0784ecd486e4c4c7909c0063e7/${contractPath}`,
            versions: ['4.7.1'], applicability: id.startsWith('can.') || id.startsWith('rpm.') ? 'vesc-dronecan'
                : id.startsWith('drive.') ? h.drive : id.startsWith('blade.') ? 'blade' : 'mavros',
            contract: `https://github.com/Pepeuch/MowgliMAVROS/blob/bf5bb0db3e4e3a0784ecd486e4c4c7909c0063e7/${contractPath}`});
    };
    // Launch defaults and editable input fields do not prove effective installed IDs.
    // The resolution may belong in MAVROS, especially in a multi-robot installation.
    add('link.target', 'MAV_SYSID', 'required', 'target', undefined, false, false, MAVLINK_DOC);
    const gcsRange = document.entries.get('MAV_GCS_SYSID_HI')?.value;
    add('link.gcs', 'MAV_GCS_SYSID', 'required', 'gcs', undefined, false, false, MAVLINK_DOC);
    if (gcsRange !== undefined && gcsRange !== 0) add('link.gcsRange', 'MAV_GCS_SYSID_HI', 'conditional', 'gcs', undefined, true, true, MAVLINK_DOC);
    if (h.serialPort !== null) add('link.protocol', `SERIAL${h.serialPort}_PROTOCOL`, 'conditional', 'protocol', 2, false, false, MAVLINK_DOC, [1, 2]);
    for (const stream of ['EXT_STAT', 'EXTRA1', 'EXTRA3']) {
        add(`stream.${stream}`, `MAV${h.mavlinkChannel}_${stream}`, 'conditional', 'stream', undefined, false, true, MAVLINK_DOC);
    }
    if (h.drive !== 'unknown') {
        add('drive.right', `SERVO${h.rightOutput}_FUNCTION`, 'conditional', 'driveRight', undefined, true, true);
        add('drive.left', `SERVO${h.leftOutput}_FUNCTION`, 'conditional', 'driveLeft', undefined, true, true);

        if (h.drive === 'vesc-dronecan') {
            add('can.driver', `CAN_P${h.canPort}_DRIVER`, 'conditional', 'canDriver', undefined, true, true);
            add('can.protocol', `CAN_D${h.canDriver}_PROTOCOL`, 'conditional', 'canProtocol', undefined, true, true);
            add('can.outputs', `CAN_D${h.canDriver}_UC_ESC_BM`, 'conditional', 'canOutputs', undefined, true, true);
            if (h.legacyRpm) for (const instance of [1, 2]) {
                add(`rpm.${instance}.type`, `RPM${instance}_TYPE`, 'conditional', 'rpmType', undefined, false, true);
                add(`rpm.${instance}.mask`, `RPM${instance}_ESC_MASK`, 'conditional', 'rpmMask', undefined, false, true);
                add(`rpm.${instance}.scale`, `RPM${instance}_SCALING`, 'conditional', 'rpmScale', undefined, false, true);
            }
        }
    }
    if (h.blade) add('blade.output', `SERVO${h.bladeOutput}_FUNCTION`, 'conditional', 'blade', undefined, true, true, SERVO_DOC);
    add('arming.require', 'ARMING_REQUIRE', 'conditional', 'arming', undefined, true, true);
    // Exclusion bitmasks, electrical safety and failsafe actions are never
    // copied from the reference robot or disabled to pass this comparison.
    for (const parameter of ['ARMING_SKIPCHK', 'BRD_SAFETY_DEFLT', 'BRD_SAFETY_MASK', 'BRD_SAFETYOPTION',
        'FS_GCS_ENABLE', 'FS_GCS_TIMEOUT', 'FS_ACTION', 'FS_EKF_ACTION', 'FS_THR_ENABLE']) {
        add(`safety.${parameter}`, parameter, 'conditional', parameter === 'ARMING_SKIPCHK' ? 'skipChecks' : 'safety', undefined, true, true, parameter === 'ARMING_SKIPCHK' ? RENAME_DOC : '');
    }
    for (const parameter of ['AHRS_EKF_TYPE', 'GPS1_TYPE', 'GPS2_TYPE', 'EK3_SRC1_POSXY', 'COMPASS_USE', 'INS_USE']) {
        add(`navigation.${parameter}`, parameter, 'conditional', 'navigation');
    }
    for (const parameter of ['MAV_OPTIONS', `MAV${h.mavlinkChannel}_OPTIONS`]) add(`optional.${parameter}`, parameter, 'optional', 'optional');
    const covered = new Set(rules.map(rule => rule.parameter));
    for (const parameter of document.entries.keys()) {
        if (!covered.has(parameter) && /^BATT\d*_(FS_|ARM_|LOW_|CRT_)/.test(parameter)) {
            add(`safety.${parameter}`, parameter, 'conditional', 'safety', undefined, true, true);
        } else if (!covered.has(parameter) && /^(BATT\d*_(MONITOR|I2C_ADDR|I2C_BUS)$|CAN_P\d+_BITRATE$)/.test(parameter)) {
            add(`hardware.${parameter}`, parameter, 'conditional', 'powerCan');
        } else if (!covered.has(parameter) && /^(ATC_|CRUISE_|WP_|INS_|COMPASS_|BATT|SERVO\d+_(MIN|MAX|TRIM|REVERSED)$|CAN_D\d+_UC_ESC_(OF|RV)$|CAN_P\d+_BITRATE$)/.test(parameter)) {
            add(`personal.${parameter}`, parameter, 'calibration', 'calibration');
        }
    }
    return rules;
}
export function reviewParamDocument(document: ParamDocument, identity: ParamIdentity, h: ParamHardware): ParamReview[] {
    const profile = profileForIdentity(identity);
    if (!profile || !validParamHardware(h)) return [];
    return profile.buildRules(document, h).map(rule => {
        const current = document.entries.get(rule.parameter)?.value;
        const state = document.duplicates.has(rule.parameter) ? 'ambiguous' : current === undefined ? 'missing'
            : rule.proposed === undefined ? rule.category === 'calibration' ? 'preserved' : 'review'
            : (rule.accepted ?? [rule.proposed]).includes(current) ? 'compliant' : 'mismatch';
        return {...rule, current, state, editable: state === 'mismatch' && rule.proposed !== undefined};
    });
}
export function admissibleParamCorrections(rows: readonly ParamReview[]) {
    return rows.filter(row => row.editable && !row.sensitive && !row.provisional);
}
export function selectedParamCorrections(rows: readonly ParamReview[], selected: readonly string[], acknowledgement?: {confirmed: boolean; reason: string}): Map<string, number> {
    const corrections = new Map<string, number>();
    for (const id of selected) {
        const row = rows.find(candidate => candidate.id === id);
        if (!row?.editable || row.proposed === undefined) throw new Error('ambiguousParameter');
        if ((row.sensitive || row.provisional) && (!acknowledgement?.confirmed || !acknowledgement.reason.trim())) throw new Error('confirmationRequired');
        corrections.set(row.parameter, row.proposed);
    }
    return corrections;
}
