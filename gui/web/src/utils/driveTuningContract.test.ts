import {describe, expect, it} from 'vitest';
import {decodeDriveTuningReport, decodeDriveTuningRollback, decodeDriveTuningStart, decodeDriveTuningStatus, validateDriveTuningRequest} from './driveTuningContract';

import {validStatus, validReport, validStart} from '../test/driveTuningFixtures';

describe('drive tuning response contracts', () => {
    it('retains zero, false and sparse optional fields', () => {
        const report = decodeDriveTuningReport(validReport());
        expect(report.parsed?.proposed_params).toEqual({ticks_per_meter: 343.596123});
        expect(report.parsed?.distance_m).toBe(0); expect(report.parsed?.applied_live).toBe(false);
        expect(report.parsed?.test_speed_mps).toBeUndefined(); expect(report.parsed?.status_snapshot).toBeUndefined();
        expect(report.parsed?.trials).toEqual([]); expect(report.parsed?.reasons).toEqual([]);
    });
    it('accepts an empty latest-report response with no fabricated report', () => {
        expect(decodeDriveTuningReport(validStatus()).parsed).toBeUndefined();
    });
    it('preserves unknown snapshot availability and explicit negatives', () => {
        const input = validReport();
        const parsed = {...input.parsed, status_snapshot: {active_emergency: false, esc_power: null}};
        expect(decodeDriveTuningReport({...input, parsed}).parsed?.status_snapshot).toEqual({active_emergency: false, esc_power: null});
    });
    it.each([null, [], {}, {...validStatus(), feed_forward: {status: 'future'}}, {...validStatus(), job: {state: 'succeeded'}},
        {...validStatus(), pid: {status: 'validated', message: false}}])('rejects malformed status %j', input => {
        expect(() => decodeDriveTuningStatus(input)).toThrow();
    });
    it.each([NaN, Infinity, -Infinity, '10', false, null])('rejects invalid proposed values %s', value => {
        const input = validReport();
        expect(() => decodeDriveTuningReport({...input, parsed: {...input.parsed, proposed_params: {ticks_per_meter: value}}})).toThrow();
    });
    it.each([{trials: [{}]}, {reasons: [false]}, {raw_yaml: false}, {status_snapshot: {active_emergency: 'false'}}, {drivetrain_diagnostics: {wheel_radius_m: '0.1'}}])('rejects malformed report field %j', patch => {
        const input = validReport();
        const response = 'raw_yaml' in patch ? {...input, ...patch} : {...input, parsed: {...input.parsed, ...patch}};
        expect(() => decodeDriveTuningReport(response)).toThrow();
    });
    it('does not hide missing mandatory collections as empty', () => {
        const input = validReport();
        expect(() => decodeDriveTuningReport({...input, parsed: {...input.parsed, trials: undefined}})).toThrow();
    });
    it('keeps rollback rejection separate from successful restored values', () => {
        expect(decodeDriveTuningRollback({success: true, restored: {ticks_per_meter: 0}}).restored).toEqual({ticks_per_meter: 0});
        expect(() => decodeDriveTuningRollback({success: false, message: 'Rejected'})).toThrow('Rejected');
        expect(() => decodeDriveTuningRollback({success: true, restored: {ticks_per_meter: '0'}})).toThrow();
    });
    it('requires a running start acknowledgement', () => {
        expect(decodeDriveTuningStart(validStart()).apply).toBe(false);
        expect(() => decodeDriveTuningStart({...validStart(), state: 'failed'})).toThrow();
        expect(() => decodeDriveTuningStart({})).toThrow();
    });
    it('validates outgoing numeric values before they can stringify to null', () => {
        const request = {apply: false, allow_undock: false, undock_distance_m: 2, passes: 1, max_speed_mps: .3, segment_duration_s: 5};
        expect(() => validateDriveTuningRequest(request)).not.toThrow();
        expect(() => validateDriveTuningRequest({...request, max_speed_mps: NaN})).toThrow();
        expect(() => validateDriveTuningRequest({...request, passes: 1.5})).toThrow();
    });
});

it.each(['ff', 'pid'])('accepts a populated %s report with optional diagnostic fields', mode => {
    const input = validReport();
    const trial = {name: 'pass-1', phase: 'forward', target_speed: .2, measured_speed_mean: .19, overshoot: 0,
        stall_detected: false, oscillation_detected: false, integral_saturation_suspected: false, rtk_accepted: true,
        ticks_per_meter: 343.596123, left_ticks_seen: 0, right_ticks_seen: 12, settling_time: null};
    const parsed = {...input.parsed, mode, trials: [trial], reasons: ['Measured pass'], robot_mass_kg: null,
        drivetrain_diagnostics: {wheel_radius_m: .1, notes: ['Derived']}, status_snapshot: {active_emergency: false}};
    const report = decodeDriveTuningReport({...input, parsed});
    expect(report.parsed?.trials[0]).toEqual(trial);
    expect(report.parsed?.trials[0].live_oscillation_detected).toBeUndefined();
    expect(report.parsed?.robot_mass_kg).toBeNull();
});
