import {describe, expect, it} from 'vitest';
import {parseAdvancedNumber, validAdvancedKeyName, structuredAdvancedPreview} from './advancedSettings';

describe('advanced decimal scalar contract', () => {
    it.each(['0', '-0', '-2', '.25', '-0.75', '1e-2', ' 42 '])('accepts finite decimal %s', text => {
        expect(Object.is(parseAdvancedNumber(text), Number(text.trim()))).toBe(true);
    });
    it.each(['', ' ', '-', '.', '1e', '1e+', '1.', 'NaN', 'Infinity', '-Infinity', '1e999', '0x10', '0b10', '0o10', '2m', '1,2'])('rejects incomplete/nondecimal/nonfinite %s', text => {
        expect(parseAdvancedNumber(text)).toBeUndefined();
    });
    it.each(['unknown_param', 'UPPER', '_private', 'parameter2'])('preserves valid exact key %s', name => expect(validAdvancedKeyName(name)).toBe(true));
    it.each(['', ' ', ' spaced ', 'my-key', 'a.b', '/a', '2param', '__proto__', 'constructor', 'prototype'])('rejects ambiguous or reserved key %s without normalization', name => expect(validAdvancedKeyName(name)).toBe(false));
    it('presents structured JSON without coercion or modification', () => {
        const value = {list: [0, false, '', null], nested: {x: 2}};
        const before = JSON.stringify(value);
        expect(structuredAdvancedPreview(value)).toBe(JSON.stringify(value, null, 2));
        expect(JSON.stringify(value)).toBe(before);
        expect(structuredAdvancedPreview([0, 1])).toBe('[\n  0,\n  1\n]');
        expect(structuredAdvancedPreview({long: 'x'.repeat(2000)})?.length).toBe(1001);
        expect(structuredAdvancedPreview({x: Infinity})).toBeUndefined();
        const cycle: Record<string, unknown> = {}; cycle.self = cycle;
        expect(structuredAdvancedPreview(cycle)).toBeUndefined();
    });
});


describe('advanced numeric conversion fidelity', () => {
    it.each(['1e-400', '-1e-400', '0.00001e-400', '1e-999999'])('rejects nonzero underflow %s', text => {
        expect(parseAdvancedNumber(text)).toBeUndefined();
    });
    it.each(['9007199254740993', '-9007199254740993', '9007199254740993.0', '900719925474099300e-2', '9.007199254740993e15', '1000000000000000100'])('rejects inaccurately represented decimal integer %s', text => {
        expect(parseAdvancedNumber(text)).toBeUndefined();
    });
    it.each(['0', '-0', '0e-400', '-0e999999', '5e-324', '-5e-324', '1e-300', '-.125', '9007199254740991', '9007199254740992', '-9007199254740992', '1e20', '1000000000000000128'])('preserves representable finite decimal %s', text => {
        expect(Object.is(parseAdvancedNumber(text), Number(text))).toBe(true);
    });
});
