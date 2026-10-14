import {describe, expect, it} from 'vitest';
import {decodeSettings, flattenConfig, settingsCoordinate, isIntegerSetting} from './settingsContract';

describe('settings JSON and flattening contract', () => {
    it('preserves null, arrays, empty containers, empty strings, zero and false as leaves', () => {
        expect(flattenConfig({nested: {nil: null, list: [0, {x: false}], empty: {}, text: '', zero: 0, disabled: false}, 'already.dotted': 12}))
            .toEqual({'nested.nil': null, 'nested.list': [0, {x: false}], 'nested.empty': {}, 'nested.text': '', 'nested.zero': 0, 'nested.disabled': false, 'already.dotted': 12});
    });
    it('preserves own prototype-like JSON keys without prototype mutation', () => {
        const result = flattenConfig(JSON.parse('{"__proto__":{"value":1}}'));
        expect(Object.entries(result)).toEqual([['__proto__.value', 1]]);
        expect(Object.getPrototypeOf(result)).toBe(Object.prototype);
    });
    it('rejects collisions and empty keys rather than changing settings', () => {
        expect(() => flattenConfig({'a.b': 1, a: {b: 2}})).toThrow('Conflicting');
        expect(() => flattenConfig({'': 1})).toThrow('Empty');
    });
    it.each([null, [], 1, {x: undefined}, {x: NaN}, {x: Infinity}, {x: new Date()}, {x: () => 1}, {x: Array(2)}])('rejects unsupported settings %j', raw => {
        expect(() => decodeSettings(raw)).toThrow('Invalid settings payload');
    });
    it('rejects cycles, accessors and invisible values', () => {
        const cyclic: Record<string, unknown> = {}; cyclic.self = cyclic;
        expect(() => decodeSettings(cyclic)).toThrow();
        expect(() => decodeSettings(Object.defineProperty({}, 'x', {value: 1}))).toThrow();
        expect(() => decodeSettings({get x() {throw new Error('Getter must not execute');}})).toThrow('Invalid');
        expect(() => decodeSettings({[Symbol('x')]: 1})).toThrow();
    });
    it('keeps legacy coordinate strings and integer guards without implicit object coercion', () => {
        expect(settingsCoordinate('0')).toBe(0);
        expect(settingsCoordinate('-1.25')).toBe(-1.25);
        expect(settingsCoordinate(undefined)).toBe(0);
        for (const raw of [false, [], {}, '', Infinity]) expect(settingsCoordinate(raw)).toBeNaN();
        expect(isIntegerSetting(0)).toBe(true);
        for (const raw of ['0', null, 1.5, Infinity]) expect(isIntegerSetting(raw)).toBe(false);
    });
});
