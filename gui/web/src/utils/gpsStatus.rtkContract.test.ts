import observed from '../test/fixtures/gnss-hotfix-passive-wire.json';
import {describe, expect, it} from 'vitest';
import {decodeGnssStatus} from './gnssTelemetry';
import {currentRtkMode, deriveGpsStatus} from './gpsStatus';
import {bridgeFixedWireFixture, capturedBridgeFixed} from '../test/gnssBridgeFixedFixture';

describe('Mowgli Universal bridge positioning dimension and RTK carrier mode', () => {
    it('accepts the captured fields/masks/sequence at the decoder boundary without fabricating an identity', () => {
        const decoded = decodeGnssStatus(bridgeFixedWireFixture);
        expect(decoded.kind).toBe('valid');
        if (decoded.kind !== 'valid') throw new Error('Bridge fixture rejected');
        expect(decoded.data).toMatchObject(capturedBridgeFixed);
        expect(decoded.data.header).toBeUndefined(); expect(decoded.data.receiver_model).toBeUndefined();
    });
    it('classifies the operator-observed 3D + Fixed bridge frame as FIXED with 100% display quality', () => {
        const decoded = decodeGnssStatus(bridgeFixedWireFixture);
        if (decoded.kind !== 'valid') throw new Error('Bridge fixture rejected');
        expect(deriveGpsStatus(decoded.data)).toMatchObject({fixType: 'RTK_FIX', percent: 100});
    });
    it('retains the available Fixed carrier mode of that 3D frame', () => {
        expect(currentRtkMode(capturedBridgeFixed)).toBe(3);
    });
    it('classifies available 3D + Float without needing RTCM activity or centimetric accuracy', () => {
        const decoded = decodeGnssStatus({...bridgeFixedWireFixture, rtk_mode: 2, horizontal_accuracy_m: 1});
        if (decoded.kind !== 'valid') throw new Error('Float fixture rejected');
        expect(deriveGpsStatus(decoded.data)).toMatchObject({fixType: 'RTK_FLOAT', percent: 80});
        expect(currentRtkMode(decoded.data)).toBe(2);
    });
});

const combinations = [
    {fix: 0, types: ['NO_FIX', 'NO_FIX', 'NO_FIX', 'NO_FIX'], modes: [undefined, undefined, undefined, undefined]},
    {fix: 1, types: ['UNKNOWN', 'UNKNOWN', 'RTK_FLOAT', 'RTK_FIX'], modes: [0, 1, 2, 3]},
    {fix: 2, types: ['RTK_FLOAT', 'UNKNOWN', 'RTK_FLOAT', 'UNKNOWN'], modes: [0, undefined, 2, undefined]},
    {fix: 3, types: ['RTK_FIX', 'UNKNOWN', 'UNKNOWN', 'RTK_FIX'], modes: [0, undefined, undefined, 3]},
    {fix: 4, types: ['UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN'], modes: [undefined, undefined, undefined, undefined]},
    {fix: 5, types: ['2D_FIX', '2D_FIX', '2D_FIX', '2D_FIX'], modes: [0, 1, undefined, undefined]},
    {fix: 6, types: ['3D_FIX', '3D_FIX', 'RTK_FLOAT', 'RTK_FIX'], modes: [0, 1, 2, 3]},
    {fix: 7, types: ['DGPS', 'DGPS', 'DGPS', 'DGPS'], modes: [0, 1, undefined, undefined]},
];
it.each(combinations.flatMap(row => row.types.map((type, mode) => ({fix: row.fix, mode, type, acceptedMode: row.modes[mode]}))))(
    'classifies available fix=$fix mode=$mode as $type without hiding incompatible RTK claims', ({fix, mode, type, acceptedMode}) => {
        const decoded = decodeGnssStatus({fix_valid: true, fix_type: fix, rtk_mode: mode, capability_flags: 1, value_flags: 1});
        if (decoded.kind !== 'valid') throw new Error('Valid enum combination rejected');
        expect(deriveGpsStatus(decoded.data).fixType).toBe(type);
        expect(currentRtkMode(decoded.data)).toBe(acceptedMode);
    });
it.each([5, 6, 7])('retains base category %s with absent/unavailable RTK, regardless of correction activity or small accuracy', fix_type => {
    const expected = fix_type === 5 ? '2D_FIX' : fix_type === 6 ? '3D_FIX' : 'DGPS';
    for (const masks of [{capability_flags: 0, value_flags: 0}, {capability_flags: 1, value_flags: 0}]) {
        const status = {fix_type, fix_valid: true, rtk_mode: 3, corrections_active: true, horizontal_accuracy_m: 0.001, ...masks};
        expect(deriveGpsStatus(status).fixType).toBe(expected);
        expect(currentRtkMode(status)).toBeUndefined();
    }
    expect(deriveGpsStatus({fix_type, fix_valid: true}).fixType).toBe(expected);
    expect(deriveGpsStatus({fix_type, fix_valid: true, corrections_active: true, horizontal_accuracy_m: 0.001}).fixType).toBe(expected);
});
it.each([{fix_valid: false}, {fix_type: 0}, {fix_type: 4}, {dead_reckoning: true}])('never promotes negative positioning to RTK: %j', negative => {
    const status = {...bridgeFixedWireFixture, ...negative};
    expect(['RTK_FIX', 'RTK_FLOAT']).not.toContain(deriveGpsStatus(status).fixType);
    expect(currentRtkMode(status)).toBeUndefined();
});
it.each([{rtk_mode: 4}, {rtk_mode: '3'}, {value_flags: 0xffffffff}, {horizontal_accuracy_m: NaN}])('preserves strict decoder rejection of malformed external data %j', invalid => {
    expect(decodeGnssStatus({...bridgeFixedWireFixture, ...invalid}).kind).toBe('invalid');
});
it('keeps the Rock 5B MAVROS Float witness unchanged', () => {
    const status = {fix_valid: true, fix_type: 2, rtk_mode: 2};
    expect(deriveGpsStatus(status)).toMatchObject({fixType: 'RTK_FLOAT', percent: 80});
    expect(currentRtkMode(status)).toBe(2);
});

it('also restores the initial operator observation (sequence 306451, accuracy approximately 0.014 m)', () => {
    const decoded = decodeGnssStatus({...bridgeFixedWireFixture, position_observation_sequence: 306451, horizontal_accuracy_m: 0.014});
    if (decoded.kind !== 'valid') throw new Error('Initial operator fixture rejected');
    expect(deriveGpsStatus(decoded.data)).toMatchObject({fixType: 'RTK_FIX', percent: 100});
    expect(currentRtkMode(decoded.data)).toBe(3);
});

it.each([
    {name: 'Mowgli', wire: observed.mowgli, type: 'RTK_FIX', mode: 3, percent: 100},
    {name: 'Rock 5B', wire: observed.rock5b, type: 'RTK_FLOAT', mode: 2, percent: 80},
])('decodes passively captured $name wire fields without synthetic scaffolding', ({wire, type, mode, percent}) => {
    const decoded = decodeGnssStatus(wire);
    if (decoded.kind !== 'valid') throw new Error('Observed GUI transport frame rejected');
    expect(deriveGpsStatus(decoded.data)).toMatchObject({fixType: type, percent});
    expect(currentRtkMode(decoded.data)).toBe(mode);
    expect(decoded.data.position_observation_sequence).toBe(wire.position_observation_sequence);
    expect(decoded.data.horizontal_accuracy_m).toBe(wire.horizontal_accuracy_m);
});
