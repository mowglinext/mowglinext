import {expect, it} from 'vitest';
import {decodeGnssActionResponse, decodeGnssRuntimeConfig, gnssActionOutcome} from './gnssConfigContract';

it.each([['plan', 'plan'], ['apply', 'apply'], ['factory-reset-apply', 'factory_reset_apply'], ['restart', 'restart']] as const)('accepts current %s and legacy sparse acknowledgements', (action, producerAction) => {
    expect(gnssActionOutcome(decodeGnssActionResponse({success: true, action: producerAction}, action))).toBe('success');
    expect(decodeGnssActionResponse({success: true}, action)).toEqual({success: true});
});
it.each([null, [], {}, {success: 'true'}, {success: 0}, {success: true, action: 'restart'},
    {success: true, restart_attempted: 'false'}, {success: true, warnings: [0]}, {success: true, executions: {}},
    {success: true, executions: [{}]}, {success: true, detected_baud: 0}, {success: true, detected_baud: '0'},
    {success: true, restart_succeeded: true, restart_error: 'failed'}, {success: true, restart_succeeded: true, restart_attempted: false}, {success: true, active_verified_baud: 'fast'}, {success: true, target_baud: Infinity}])('rejects malformed action %j', raw => {
    expect(() => decodeGnssActionResponse(raw, 'apply')).toThrow();
});
it('preserves failed detection followed by successful manual recovery', () => {
    const raw = {success: true, execution_baud: 'auto', detected_baud: '115200', target_baud: '921600', active_verified_baud: '921600',
        executions: [{tool: 'detect', command: ['detect'], exit_code: 2, success: false},
            {tool: 'apply', command: ['apply', '--baud', '115200'], exit_code: 0, success: true, stdout: '', stderr: ''}]};
    expect(decodeGnssActionResponse(raw, 'apply')).toEqual(raw);
});
it.each([{exit_code: .5, success: false}, {exit_code: 0, success: false}, {exit_code: 2, success: true}, {exit_code: 0, success: true, command: [0]}])('rejects inconsistent/malformed execution %j', patch => {
    expect(() => decodeGnssActionResponse({success: true, executions: [{tool: 'apply', command: ['apply'], ...patch}]}, 'apply')).toThrow();
});
it.each([
    [{success: true, manual_baud_required: true}, 'failed'], [{success: true, restart_error: 'failed'}, 'failed'], [{success: false}, 'failed'], [{success: false, partial_failure: true, restart_attempted: true, restart_error: 'failed'}, 'partial'],
    [{success: true, partial_failure: true}, 'partial'], [{success: true, restart_attempted: true}, 'unconfirmed'],
    [{success: true, restart_attempted: true, restart_succeeded: false}, 'failed'],
    [{success: true, restart_attempted: true, restart_succeeded: true}, 'success'],
] as const)('classifies operation and restart independently: %j', (raw, outcome) => {
    expect(gnssActionOutcome(decodeGnssActionResponse(raw, 'apply'))).toBe(outcome);
});
it('retains zero exit status, explicit negative flags and absent optional baud fields', () => {
    const response = decodeGnssActionResponse({success: false, gps_container_was_running: false, manual_baud_required: true,
        executions: [{tool: 'detect', command: [], exit_code: 0, success: true}]}, 'apply');
    expect(response.detected_baud).toBeUndefined(); expect(response.gps_container_was_running).toBe(false);
    expect(response.executions?.[0].exit_code).toBe(0);
});
it('preserves configuration provenance, legitimate empty strings and unknown sources', () => {
    const raw = {receiver_family: {active_value: 'auto', source: 'default'}, serial_device: {configured_value: '', fallback_env_value: '/dev/example', active_value: '/dev/example', source: 'env'},
        serial_baud: {active_value: '921600', source: 'future'}, active_serial_device_exists: false, serial_devices: []};
    expect(decodeGnssRuntimeConfig(raw)).toEqual(raw);
    expect(decodeGnssRuntimeConfig({serial_device: {active_value: ''}}).active_serial_device_exists).toBeUndefined();
});
it.each([null, {}, [], {serial_device: '/dev/ttyUSB0'}, {serial_device: {active_value: 0}},
    {serial_device: {}, active_serial_device_exists: 'true'}, {serial_device: {}, serial_devices: null},
    {serial_device: {}, serial_devices: [{path: '/dev/example', display_label: 'Example', exists: 1}]}])('rejects malformed runtime config %j', raw => {
    expect(() => decodeGnssRuntimeConfig(raw)).toThrow();
});

it.each([true, false])('accepts the exact Go factory reset identifier for success=%s', success => {
    const raw = {action: 'factory_reset_apply', success, partial_failure: !success,
        gps_container_was_running: true, runtime_baud_differs_from_config: false,
        restart_attempted: true, ...(success ? {restart_succeeded: true} : {restart_error: 'restore failed'})};
    expect(decodeGnssActionResponse(raw, 'factory-reset-apply')).toEqual(raw);
    expect(gnssActionOutcome(decodeGnssActionResponse(raw, 'factory-reset-apply'))).toBe(success ? 'success' : 'partial');
});
it.each(['plan', 'apply', 'restart', 'factory-reset-apply', 'unrelated'])('rejects unrelated/wrong factory reset identifier %s', action => {
    expect(() => decodeGnssActionResponse({success: true, action}, 'factory-reset-apply')).toThrow();
});
it.each(['plan', 'apply', 'restart'] as const)('does not accept the reset producer identifier for %s', route => {
    expect(() => decodeGnssActionResponse({success: true, action: 'factory_reset_apply'}, route)).toThrow();
});
