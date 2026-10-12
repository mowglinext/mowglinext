import {describe, expect, it} from 'vitest';
import {createUtf8StreamDecoder} from './utf8Stream.ts';

const base64 = (bytes: Uint8Array) => btoa(String.fromCharCode(...bytes));

describe('UTF-8 log transport', () => {
    it('preserves microseconds, accents and ASCII across every byte boundary', () => {
        const text = '179.373µs — météo 🌿\nINFO ready';
        const bytes = new TextEncoder().encode(text);
        for (let split = 0; split <= bytes.length; split++) {
            const decode = createUtf8StreamDecoder();
            expect(decode(base64(bytes.slice(0, split))) + decode(base64(bytes.slice(split)))).toBe(text);
        }
    });
    it('does not carry partial bytes into a new connection', () => {
        const old = createUtf8StreamDecoder();
        expect(old(base64(new Uint8Array([0xc2])))).toBe('');
        expect(createUtf8StreamDecoder()(btoa('new stream'))).toBe('new stream');
    });
});
