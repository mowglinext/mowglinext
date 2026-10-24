import {decodeSettings} from '../../utils/settingsContract';
export type AdvancedScalar = string | number | boolean;
export type AdvancedKeyError = 'invalidName' | 'duplicateKey' | 'protectedKey';
export function validAdvancedKeyName(key: string): boolean {
    return /^[A-Za-z_][A-Za-z0-9_]*$/.test(key) && !['__proto__', 'constructor', 'prototype'].includes(key);
}
export function parseAdvancedNumber(text: string): number | undefined {
    const decimal = /^([+-]?)(\d+(?:\.\d+)?|\.\d+)(?:[eE]([+-]?\d+))?$/.exec(text.trim());
    if (!decimal) return undefined;
    const value = Number(text.trim());
    if (!Number.isFinite(value)) return undefined;
    const mantissa = decimal[2];
    const coefficient = mantissa.replace('.', '').replace(/^0+/, '');
    // A zero mantissa is a real zero regardless of exponent; a nonzero one
    // must not disappear through IEEE-754 underflow (including negative zero).
    if (value === 0) return coefficient === '' ? value : undefined;
    if (!Number.isSafeInteger(value) && Number.isInteger(value)) {
        const significant = coefficient.replace(/0+$/, '');
        const fractionLength = mantissa.includes('.') ? mantissa.length - mantissa.indexOf('.') - 1 : 0;
        const scale = Number(decimal[3] ?? '0') - fractionLength + coefficient.length - significant.length;
        // Only mathematically integral decimals need an exact integer check.
        // Ordinary fractional decimals retain their existing binary rounding.
        if (scale >= 0) {
            const exact = BigInt(significant) * 10n ** BigInt(scale) * (decimal[1] === '-' ? -1n : 1n);
            if (BigInt(value) !== exact) return undefined;
        }
    }
    return value;
}
export function structuredAdvancedPreview(value: unknown): string | undefined {
    if (typeof value !== 'object' || value === null) return undefined;
    try {
        const json = JSON.stringify(decodeSettings({value}).value, null, 2);
        return json.length > 1000 ? `${json.slice(0, 1000)}…` : json;
    } catch {return undefined;}
}
