import {describe, expect, it} from 'vitest';
import {decodeStringRecord, isJsonObject} from './json';
import {decodeConfigEnvironment} from './configEnvironment';

describe('configuration JSON boundaries', () => {
    it.each([undefined, null, false, 0, 'text', [], new Date(0)])('rejects a non-object configuration document: %j', payload => {
        expect(isJsonObject(payload)).toBe(false);
        expect(() => decodeStringRecord(payload)).toThrow('Expected a configuration object');
        expect(() => decodeConfigEnvironment(payload)).toThrow('Expected a configuration environment object');
    });

    it.each([undefined, null, false, 12, [], {nested: 'value'}])('rejects invalid configuration scalars without coercing them: %j', value => {
        expect(() => decodeStringRecord({valid: 'keep', invalid: value})).toThrow('must be a string');
    });

    it('accepts empty documents and exact string values without interpreting them as measurements', () => {
        expect(decodeStringRecord({})).toEqual({});
        const source = {empty: '', zero: '0', enabled: 'false', whitespace: ' '};
        const decoded = decodeStringRecord(source);
        expect(decoded).toEqual(source);
        source.zero = '12';
        expect(decoded.zero).toBe('0');
    });

    it('preserves own special JSON keys without polluting object prototypes', () => {
        const payload: unknown = JSON.parse('{"__proto__":"text","constructor":"stored"}');
        const decoded = decodeStringRecord(payload);
        expect(Object.getPrototypeOf(decoded)).toBe(Object.prototype);
        expect(Object.prototype.hasOwnProperty.call(decoded, '__proto__')).toBe(true);
        expect(decoded['__proto__']).toBe('text');
        expect(decoded.constructor).toBe('stored');
    });

    it('accepts absent or empty optional tileUri and ignores unrelated future environment fields', () => {
        expect(decodeConfigEnvironment({})).toEqual({});
        expect(decodeConfigEnvironment({tileUri: ''})).toEqual({tileUri: ''});
        expect(decodeConfigEnvironment({tileUri: '/tiles/{z}/{x}/{y}.png', future: {enabled: true}}))
            .toEqual({tileUri: '/tiles/{z}/{x}/{y}.png'});
    });

    it.each([null, false, 0, [], {url: '/tiles'}])('does not accept an invalid tileUri as an overlay: %j', tileUri => {
        expect(() => decodeConfigEnvironment({tileUri})).toThrow('tileUri must be a string');
    });
});
