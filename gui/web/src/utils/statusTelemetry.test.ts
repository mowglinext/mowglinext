import {describe, expect, it} from 'vitest';
import type {Emergency, HighLevelStatus, Status} from '../types/ros';
import {
    decodeEmergency, decodeHighLevelStatus, decodeStatus, emergencyIsKnown,
    hasEmergency, retainEmergency, retainHighLevelEmergency, retainStatusWarning,
    telemetryStampIdentity,
} from './statusTelemetry';

const emergency: Required<Emergency> = {
    stamp: {sec: 12, nanosec: 5}, active_emergency: false, latched_emergency: true,
    lift_warning: false, lift_duration_sec: 0, reason: 'STOP',
};
const highLevel: Required<HighLevelStatus> = {
    state: 2, state_name: 'PLANNING', sub_state_name: '', current_area: -1,
    current_path: -1, current_path_index: -1, total_swaths: 0, completed_swaths: 0,
    skipped_swaths: 0, coverage_percent: 0, gps_quality_percent: 100,
    battery_percent: 42, is_charging: false, emergency: false,
};
const status: Required<Status> = {
    stamp: {sec: 12, nanosec: 5}, mower_status: 255, reset_cause: 0,
    reset_cause_name: 'unknown', raspberry_pi_power: true, is_charging: false,
    esc_power: true, rain_detected: false, sound_module_available: false,
    sound_module_busy: false, ui_board_available: true, mow_enabled: false,
    firmware_debug_enabled: false, dig_escalated: false, dig_escalated_distance_m: 0,
    dig_escalated_required_distance_m: 0.3, mower_esc_status: 0, mower_esc_temperature: -2,
    mower_esc_current: -1, mower_motor_temperature: 20, mower_motor_rpm: 0,
    blade_status_stamp: {sec: 0, nanosec: 0}, blade_requested_direction: 'unknown',
    firmware_version: '1.2.3', firmware_protocol_version: 8, firmware_compatible: true,
};
const decoders = [
    {name: 'emergency', decode: decodeEmergency, complete: emergency},
    {name: 'high level', decode: decodeHighLevelStatus, complete: highLevel},
    {name: 'status', decode: decodeStatus, complete: status},
];
describe.each(decoders)('$name boundary', ({decode, complete}) => {
    it('accepts the complete contract without coercion or mutation', () => {
        const payload = Object.freeze({...complete});
        expect(decode(payload)).toEqual({kind: 'valid', data: payload});
    });
    it('accepts sparse and empty records without adding observed defaults', () => {
        expect(decode({})).toEqual({kind: 'valid', data: {}});
        expect(decode({is_charging: false})).toEqual({kind: 'valid', data: {is_charging: false}});
    });
    it('retains backend-specific future extensions', () => {
        const payload = {...complete, backend_extension: {capability: 'unsupported'}};
        expect(decode(payload)).toEqual({kind: 'valid', data: payload});
    });
    it.each([null, undefined, [], [complete], 'JSON text', 1, false])('rejects a non-record %j', payload => {
        expect(decode(payload).kind).toBe('invalid');
    });
    it('rejects a wrong type for every known field', () => {
        for (const key of Object.keys(complete)) {
            expect(decode({...complete, [key]: []}).kind, key).toBe('invalid');
            expect(decode({...complete, [key]: null}).kind, key).toBe('invalid');
        }
    });
});
describe('status field domains', () => {
    it.each([NaN, Infinity, -Infinity, '0', null, undefined])('rejects invalid numeric measurements %j', value => {
        expect(decodeStatus({mower_motor_rpm: value}).kind).toBe('invalid');
        expect(decodeEmergency({lift_duration_sec: value}).kind).toBe('invalid');
        expect(decodeHighLevelStatus({battery_percent: value}).kind).toBe('invalid');
    });
    it.each([-1, 101, NaN, Infinity])('rejects invalid percentages %j', value => {
        for (const key of ['coverage_percent', 'gps_quality_percent', 'battery_percent']) {
            expect(decodeHighLevelStatus({[key]: value}).kind).toBe('invalid');
        }
    });
    it.each([{sec: 1}, {sec: '1', nanosec: 0}, {sec: 1.5, nanosec: 0},
        {sec: 1, nanosec: -1}, {sec: 1, nanosec: 1e9}, {sec: NaN, nanosec: 0},
        {sec: Infinity, nanosec: 0}, null, []])('rejects a malformed stamp %j', stamp => {
        expect(decodeEmergency({stamp}).kind).toBe('invalid');
        expect(decodeStatus({stamp}).kind).toBe('invalid');
        expect(decodeStatus({blade_status_stamp: stamp}).kind).toBe('invalid');
    });
    it.each([0, 1, 2, 3, 4])('accepts high-level enum %i', state => {
        expect(decodeHighLevelStatus({state}).kind).toBe('valid');
    });
    it.each([5, 255])('distinguishes unsupported high-level enum %i', state => {
        expect(decodeHighLevelStatus({state}).kind).toBe('unsupported');
    });
    it.each([-1, 256, 0.5, '2'])('rejects an invalid enum representation %j', state => {
        expect(decodeHighLevelStatus({state}).kind).toBe('invalid');
    });
    it('distinguishes unsupported operational codes from known initializing/OK', () => {
        expect(decodeStatus({mower_status: 0}).kind).toBe('valid');
        expect(decodeStatus({mower_status: 255}).kind).toBe('valid');
        expect(decodeStatus({mower_status: 1}).kind).toBe('unsupported');
        expect(decodeStatus({reset_cause: 8}).kind).toBe('unsupported');
        expect(decodeStatus({blade_requested_direction: 'sideways'}).kind).toBe('unsupported');
    });
    it('preserves MAVROS unavailable and OpenMower optional shapes without handshake defaults', () => {
        expect(decodeStatus({mower_status: 0, stamp: {sec: 0, nanosec: 0}})).toEqual({kind: 'valid', data: {mower_status: 0, stamp: {sec: 0, nanosec: 0}}});
        const openmower = {mower_status: 255, firmware_version: 'LowLevel', mower_esc_current: -0.1};
        expect(decodeStatus(openmower)).toEqual({kind: 'valid', data: openmower});
    });
    it('accepts extensible high-level names and signed sentinel indices', () => {
        expect(decodeHighLevelStatus({state: 2, state_name: 'FUTURE_AUTONOMOUS_SUBSTATE', current_area: -1}).kind).toBe('valid');
        expect(decodeHighLevelStatus({current_path: 32768}).kind).toBe('invalid');
    });
});
describe('assertion retention and identity', () => {
    it('does not clear emergency or lift assertions from a missing bit', () => {
        expect(retainEmergency(emergency, {})).toEqual({latched_emergency: true});
        expect(retainEmergency({active_emergency: true, lift_warning: true}, {reason: 'partial'}))
            .toEqual({active_emergency: true, lift_warning: true, reason: 'partial'});
        expect(retainHighLevelEmergency({emergency: true, state: 2}, {})).toEqual({emergency: true});
        expect(retainStatusWarning({dig_escalated: true, mower_status: 255}, {})).toEqual({dig_escalated: true});
    });
    it('clears only explicitly reported false bits and never retains old healthy bits', () => {
        expect(retainEmergency(emergency, {latched_emergency: false})).toEqual({latched_emergency: false});
        expect(retainEmergency({active_emergency: false}, {})).toEqual({});
        expect(retainHighLevelEmergency({emergency: true}, {emergency: false})).toEqual({emergency: false});
    });
    it('does not let a high-level false bit mask a hardware assertion', () => {
        expect(hasEmergency({active_emergency: true}, {emergency: false})).toBe(true);
        expect(hasEmergency({latched_emergency: true}, {emergency: false})).toBe(true);
        expect(hasEmergency({active_emergency: false}, {emergency: true})).toBe(true);
    });
    it('requires hardware active and latch information before declaring emergency known clear', () => {
        expect(emergencyIsKnown({}, {emergency: false})).toBe(false);
        expect(emergencyIsKnown({active_emergency: false}, {})).toBe(false);
        expect(emergencyIsKnown({active_emergency: false, latched_emergency: false}, {})).toBe(true);
    });
    it('treats stamps as opaque identities, including negative ROS simulation time', () => {
        expect(telemetryStampIdentity({stamp: {sec: -1, nanosec: 2}})).toBe('-1:2');
        expect(telemetryStampIdentity({stamp: {sec: 0, nanosec: 0}})).toBeNull();
        expect(telemetryStampIdentity(highLevel)).toBeNull();
    });
});
