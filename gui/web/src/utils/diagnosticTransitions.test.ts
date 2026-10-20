import {describe, expect, it} from 'vitest';
import type {TimestampedDiagnosticStatus} from '../hooks/useDiagnostics';
import {DIAGNOSTIC_TRANSITION_CAPACITY, DIAGNOSTIC_TRANSITION_IDENTITY_LIMIT, DiagnosticTransitionMemory} from './diagnosticTransitions';

const entry = (level: number, hardware_id = 'receiver'): TimestampedDiagnosticStatus => ({
    name: 'status', hardware_id, level, message: 'native message', values: [{key: 'measurement', value: '123.4567'}],
    receivedAt: 1000, publicationStamp: '1:0', healthBlocked: false,
});
const memory = () => {
    const result = new DiagnosticTransitionMemory();
    result.begin('robot/direct', []);
    return result;
};

describe('compact observed diagnostic transitions, separate from physical validity', () => {
    it('records level changes and first non-OK observations, without retaining the full status', () => {
        const result = memory();
        result.observe(undefined, entry(0));
        expect(result.readAfter().transitions).toEqual([]);
        result.observe(entry(0), entry(1));
        result.observe(entry(1), {...entry(0), healthBlocked: true});
        result.observe(undefined, entry(3, 'other'));
        expect(result.readAfter().transitions).toEqual([
            {order: 1, name: 'status', hardware_id: 'receiver', previousLevel: 0, level: 1, publicationStamp: '1:0', healthBlocked: false},
            {order: 2, name: 'status', hardware_id: 'receiver', previousLevel: 1, level: 0, publicationStamp: '1:0', healthBlocked: true},
            {order: 3, name: 'status', hardware_id: 'other', previousLevel: null, level: 3, publicationStamp: '1:0', healthBlocked: false},
        ]);
        expect(Object.isFrozen(result.readAfter().transitions[0])).toBe(true);
    });

    it('does not allocate another retained record for republications or measurements at an unchanged level', () => {
        const result = memory();
        const warning = entry(1);
        result.observe(undefined, warning);
        const first = result.readAfter();
        for (let i = 0; i < 1000; i++) result.observe(warning, {...warning, message: `sample ${i}`, publicationStamp: `${i}:0`});
        const final = result.readAfter();
        expect(final.cursor).toEqual(first.cursor);
        expect(final.transitions).toHaveLength(1);
        expect(final.transitions[0]).toBe(first.transitions[0]);
        expect(final.evicted).toBe(0);
        expect(result.readAfter(first.cursor).transitions).toEqual([]);
    });

    it('does not confuse homonymous hardware IDs or duplicate their retained level observations', () => {
        const result = memory();
        result.observe(undefined, entry(1, 'A'));
        result.observe(entry(1, 'A'), entry(2, 'B'));
        result.observe(entry(2, 'B'), entry(1, 'A'));
        result.observe(entry(1, 'A'), entry(2, 'B'));
        result.observe(entry(2, 'B'), entry(0, 'A'));
        expect(result.readAfter().transitions.map(e => [e.hardware_id, e.previousLevel, e.level]))
            .toEqual([['A', null, 1], ['B', null, 2], ['A', 1, 0]]);
    });

    it('keeps exactly 64 records in order and reports gaps to independent incremental readers', () => {
        const result = memory();
        for (let i = 1; i <= 5; i++) result.observe(entry((i - 1) % 2), entry(i % 2));
        const firstReader = result.readAfter();
        const secondReader = result.readAfter();
        for (let i = 6; i <= 75; i++) result.observe(entry((i - 1) % 2), entry(i % 2));
        const next = result.readAfter(firstReader.cursor);
        expect(next.transitions).toHaveLength(DIAGNOSTIC_TRANSITION_CAPACITY);
        expect(next.transitions.map(e => e.order)).toEqual(Array.from({length: 64}, (_, i) => i + 12));
        expect(next.evicted).toBe(11);
        expect(next.missed).toBe(6);
        expect(result.readAfter(secondReader.cursor)).toEqual(next);
        expect(result.readAfter().missed).toBe(11);
        expect(result.readAfter(next.cursor).transitions).toEqual([]);
        expect(result.readAfter(next.cursor).missed).toBe(0);
    });

    it('omits oversized identities explicitly instead of truncating or retaining payloads', () => {
        const result = memory();
        const tooLarge = {...entry(1), name: 'x'.repeat(DIAGNOSTIC_TRANSITION_IDENTITY_LIMIT + 1)};
        result.observe(undefined, tooLarge);
        result.observe(undefined, entry(2));
        const read = result.readAfter();
        expect(read.omittedOversizedIdentity).toBe(1);
        expect(read.evicted).toBe(0);
        expect(read.missed).toBe(1);
        expect(read.transitions.map(e => e.order)).toEqual([2]);
        expect(tooLarge.name).toHaveLength(DIAGNOSTIC_TRANSITION_IDENTITY_LIMIT + 1);
    });

    it('clears context, counters and predecessors without carrying a previous robot into the next', () => {
        const result = memory();
        const old = entry(2);
        result.observe(undefined, old);
        const first = result.readAfter();
        result.begin('other/mavros', [old]);
        result.observe(old, entry(1));
        const changed = result.readAfter(first.cursor);
        expect(changed.contextChanged).toBe(true);
        expect(changed.context).toBe('other/mavros');
        expect(changed.transitions[0].previousLevel).toBeNull();
        expect(changed.transitions[0].order).toBe(1);
        expect(changed.evicted).toBe(0);
        result.clear();
        result.observe(undefined, entry(2));
        expect(result.readAfter(changed.cursor)).toMatchObject({active: false, context: null, contextChanged: true,
            transitions: [], missed: 0, evicted: 0, omittedOversizedIdentity: 0});
    });

    it('rejects a cursor from another scope and handles invalid positions conservatively', () => {
        const a = memory(), b = memory();
        a.observe(undefined, entry(1));
        b.observe(undefined, entry(2));
        expect(a.readAfter(b.readAfter().cursor).contextChanged).toBe(true);
        for (const order of [-1, 2, 0.5, Infinity]) {
            expect(a.readAfter({...a.readAfter().cursor, order}).contextChanged).toBe(true);
        }
    });
});
