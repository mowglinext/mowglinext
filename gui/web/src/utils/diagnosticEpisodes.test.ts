import {describe, expect, it} from 'vitest';
import {acknowledgeDiagnosticEpisodes as ack, createDiagnosticEpisodeState as create, readDiagnosticEpisodes as read,
    reduceDiagnosticEpisodes as reduce, type DiagnosticEpisodeDecision, type DiagnosticEpisodeState} from './diagnosticEpisodes';

const scope = Symbol('A');
const context = 'endpoint-A/robot-A/mowgli';
const identity = {name: 'report', hardware_id: 'source'};
const base = (order: number) => ({scope, context, identity, order, continuity: 'continuous' as const});
const fault = (order: number, severity: 1 | 2 = 1): DiagnosticEpisodeDecision => ({...base(order), admission: 'accepted', classification: 'fault', severity});
const ok = (order: number): DiagnosticEpisodeDecision => ({...base(order), admission: 'accepted', classification: 'explicit_ok'});
const rejected = (order: number, classification: 'fault' | 'explicit_ok'): DiagnosticEpisodeDecision => ({...base(order), admission: 'rejected', classification});
const control = (order: number, classification: 'expired' | 'unverified' | 'unavailable' | 'not_applicable'): DiagnosticEpisodeDecision =>
    ({...base(order), admission: 'control', classification, continuity: 'unknown'});
const apply = (...inputs: DiagnosticEpisodeDecision[]) => inputs.reduce(reduce, create(scope, context));
const first = (state: DiagnosticEpisodeState) => state.episodes[0];

describe('qualified diagnostic report episodes, no clock or ROS admission inside', () => {
    it('opens then explicitly resolves even in the same synchronous batch', () => {
        const s = apply(fault(1), ok(2));
        expect(s.episodes).toHaveLength(1);
        expect(first(s)).toMatchObject({id: 1, state: 'resolved', resolvedOrder: 2, unconsulted: true});
    });
    it('keeps WARN -> ERROR -> OK in one episode with its peak and no active WARN after OK', () => {
        const s = apply(fault(1), fault(2, 2), ok(3));
        expect(s.episodes).toHaveLength(1); expect(first(s)).toMatchObject({state: 'resolved', peakSeverity: 2});
    });
    it('reuses episode objects and arrays for identical repeated qualified decisions', () => {
        const a = apply(fault(1)), b = reduce(a, fault(2));
        expect(b.lastOrder).toBe(2); expect(b.episodes).toBe(a.episodes); expect(first(b)).toBe(first(a));
    });
    it('keeps repeated clear as a no-op and creates no initial OK history', () => {
        expect(apply(ok(1)).episodes).toEqual([]);
        const a = apply(fault(1), ok(2)); expect(reduce(a, ok(3)).episodes).toBe(a.episodes);
    });
    it('creates separate episodes after explicit OK without merging their identities', () => {
        const s = apply(fault(1), ok(2), fault(3));
        expect(s.episodes.map(e => [e.id, e.state])).toEqual([[1, 'resolved'], [3, 'active']]);
        expect(s.episodes[1].opening).toBe('after_explicit_ok');
    });
    it('keeps admissible equal stamps and excludes a regressive stamp rejected by the caller', () => {
        const s = apply({...fault(1), publicationStamp: '10:0'}, {...ok(2), publicationStamp: '10:0'},
            {...rejected(3, 'fault'), publicationStamp: '9:0'}, {...fault(4), publicationStamp: '11:0'});
        expect(s.episodes.map(e => e.state)).toEqual(['resolved', 'active']);
        expect(s.episodes[1].id).toBe(4);
        expect(first(s)).not.toHaveProperty('publicationStamp');
    });
    it('does not open on rejected WARN or resolve on rejected OK', () => {
        expect(apply(rejected(1, 'fault')).episodes).toEqual([]);
        const s = apply(fault(1), rejected(2, 'explicit_ok')); expect(first(s).state).toBe('active');
    });
    it.each(['fault', 'explicit_ok'] as const)('does not count an oversized rejected %s as lost admissible evidence', classification => {
        const s = create(scope, context);
        const next = reduce(s, {...rejected(1, classification), identity: {name: 'x'.repeat(4097), hardware_id: ''}});
        expect(next.lastOrder).toBe(1); expect(next.episodes).toBe(s.episodes); expect(next.losses).toBe(s.losses);
        expect(next.historyIncomplete).toBe(false); expect(read(next).hasUnconsulted).toBe(false);
        expect(reduce(next, fault(1))).toBe(next);
        expect(first(reduce(next, fault(2))).opening).toBe('first_observed');
    });
    it.each(['fault', 'explicit_ok'] as const)('keeps existing consultation and losses unchanged for an oversized rejected %s', classification => {
        const s = apply(fault(1)), consulted = ack(s, read(s).receipt);
        const next = reduce(consulted, {...rejected(2, classification), identity: {name: 'x'.repeat(4097), hardware_id: ''}});
        expect(next.lastOrder).toBe(2); expect(next.episodes).toBe(consulted.episodes); expect(next.losses).toBe(consulted.losses);
        expect(next.historyIncomplete).toBe(false); expect(read(next).hasUnconsulted).toBe(false);
    });
    it('preserves a genuine pending loss and its receipt when an oversized rejected observation follows', () => {
        const large = {name: 'x'.repeat(4097), hardware_id: ''};
        const s = apply({...fault(1), identity: large}), receipt = read(s).receipt;
        const next = reduce(s, {...rejected(2, 'fault'), identity: large});
        expect(next.lastOrder).toBe(2); expect(next.losses).toBe(s.losses);
        expect(next.losses).toMatchObject({pending: true, omitted: 1, version: 1});
        expect(ack(next, receipt).losses.pending).toBe(false);
    });
    it.each(['expired', 'unverified', 'unavailable', 'not_applicable'] as const)('does not invent history or attention from oversized control %s', classification => {
        const s = create(scope, context);
        const next = reduce(s, {...control(1, classification), identity: {name: 'x'.repeat(4097), hardware_id: ''}});
        expect(next.lastOrder).toBe(1); expect(next.episodes).toBe(s.episodes); expect(next.episodes).toEqual([]);
        expect(next.losses).toBe(s.losses); expect(next.losses).toMatchObject({omitted: 0, pending: false});
        expect(next.historyIncomplete).toBe(false); expect(read(next).hasUnconsulted).toBe(false);
        expect(reduce(next, fault(1))).toBe(next);
        expect(first(reduce(next, fault(2))).opening).toBe('first_observed');
    });
    it.each(['expired', 'unverified', 'unavailable', 'not_applicable'] as const)('preserves a consulted episode on oversized control %s', classification => {
        const s = apply(fault(1)), consulted = ack(s, read(s).receipt);
        const next = reduce(consulted, {...control(2, classification), identity: {name: 'x'.repeat(4096), hardware_id: 'y'}});
        expect(next.lastOrder).toBe(2); expect(next.episodes).toBe(consulted.episodes); expect(first(next)).toBe(first(consulted));
        expect(next.losses).toBe(consulted.losses); expect(next.historyIncomplete).toBe(false);
        expect(first(next)).toMatchObject({state: 'active', continuity: 'continuous', unconsulted: false});
        expect(read(next).hasUnconsulted).toBe(false);
    });
    it.each(['expired', 'unverified', 'unavailable', 'not_applicable'] as const)('preserves a prior accepted omission and its acknowledgement on oversized control %s', classification => {
        const large = {name: 'x'.repeat(4097), hardware_id: ''};
        const s = apply({...fault(1), identity: large}), receipt = read(s).receipt;
        const next = reduce(s, {...control(2, classification), identity: large});
        expect(next.lastOrder).toBe(2); expect(next.episodes).toBe(s.episodes); expect(next.losses).toBe(s.losses);
        expect(next.historyIncomplete).toBe(true); expect(next.losses).toMatchObject({omitted: 1, pending: true, version: 1});
        const acknowledged = ack(next, receipt);
        expect(acknowledged.losses).toMatchObject({omitted: 1, pending: false, version: 1});
        expect(acknowledged.historyIncomplete).toBe(true); expect(read(acknowledged).hasUnconsulted).toBe(false);
    });
    it('can resolve with an admissible clear after an earlier rejected clear', () => {
        expect(first(apply(fault(1), rejected(2, 'explicit_ok'), ok(3))).state).toBe('resolved');
    });
    it.each(['expired', 'unverified', 'unavailable', 'not_applicable'] as const)('does not fabricate an opening or a resolution from %s', classification => {
        expect(apply(control(1, classification)).episodes).toEqual([]);
        const s = apply(fault(1), control(2, classification));
        expect(first(s)).toMatchObject({state: classification === 'expired' ? 'expired' : 'unknown', unconsulted: true, continuity: 'unknown'});
        expect(first(s)).not.toHaveProperty('resolvedOrder');
    });
    it('resumes an expired proof without inventing another episode or erasing its continuity gap', () => {
        const s = apply(fault(1), control(2, 'expired'), fault(3));
        expect(s.episodes).toHaveLength(1); expect(first(s)).toMatchObject({id: 1, state: 'active', continuity: 'unknown'});
    });
    it('can explicitly resolve after a gap, while retaining that gap', () => {
        const s = apply(fault(1), control(2, 'unverified'), ok(3));
        expect(first(s)).toMatchObject({state: 'resolved', continuity: 'unknown', resolvedOrder: 3});
    });
    it('does not invalidate an already resolved episode on later expiry', () => {
        const a = apply(fault(1), ok(2)); expect(reduce(a, control(3, 'expired')).episodes).toBe(a.episodes);
    });
    it('does not rearm a consulted episode on same-level recovery after a continuity gap', () => {
        let s = apply(fault(1)); s = ack(s, read(s).receipt);
        s = reduce(reduce(s, control(2, 'expired')), fault(3));
        expect(first(s)).toMatchObject({state: 'active', unconsulted: false, continuity: 'unknown'});
    });
    it('rearms only above the consulted peak, not on repeated severity oscillations', () => {
        let s = apply(fault(1)); s = ack(s, read(s).receipt); s = reduce(s, fault(2, 2));
        expect(first(s)).toMatchObject({id: 1, peakSeverity: 2, unconsulted: true});
        s = ack(s, read(s).receipt); s = reduce(s, fault(3));
        expect(first(s)).toMatchObject({peakSeverity: 2, lastFaultSeverity: 1, unconsulted: false});
        s = reduce(s, fault(4, 2)); expect(first(s).unconsulted).toBe(false);
    });
    it('does not rearm consulted ERROR -> WARN -> ERROR without explicit OK', () => {
        let s = apply(fault(1, 2)); s = ack(s, read(s).receipt);
        s = reduce(s, fault(2)); expect(first(s)).toMatchObject({lastFaultSeverity: 1, peakSeverity: 2, unconsulted: false});
        s = reduce(s, fault(3, 2));
        expect(first(s)).toMatchObject({id: 1, state: 'active', peakSeverity: 2, unconsulted: false});
    });
    it('keeps an unread aggravation unread after downgrade', () => {
        let s = apply(fault(1)); s = ack(s, read(s).receipt);
        s = reduce(s, fault(2, 2)); s = reduce(s, fault(3));
        expect(first(s)).toMatchObject({id: 1, peakSeverity: 2, lastFaultSeverity: 1, unconsulted: true});
    });
    it('opens a new unread WARN after consulted ERROR and explicit OK', () => {
        let s = apply(fault(1, 2)); s = ack(s, read(s).receipt);
        s = reduce(reduce(s, ok(2)), fault(3));
        expect(s.episodes.map(e => [e.id, e.state, e.unconsulted])).toEqual([[1, 'resolved', false], [3, 'active', true]]);
        expect(s.episodes[1]).toMatchObject({peakSeverity: 1, opening: 'after_explicit_ok'});
    });
    it('requires a current receipt covering the peak even when severity has fallen again', () => {
        let s = apply(fault(1)); const oldReceipt = read(s).receipt;
        s = reduce(reduce(s, fault(2, 2)), fault(3));
        expect(first(ack(s, oldReceipt)).unconsulted).toBe(true);
        const presented = read(s);
        expect(presented.episodes[0]).toMatchObject({peakSeverity: 2, lastFaultSeverity: 1, version: 3});
        s = ack(s, presented.receipt); s = reduce(s, fault(4, 2));
        expect(first(s).unconsulted).toBe(false);
    });
    it('does not rearm a consulted episode merely because it is resolved', () => {
        let s = apply(fault(1)); s = ack(s, read(s).receipt); s = reduce(s, ok(2));
        expect(first(s)).toMatchObject({state: 'resolved', unconsulted: false});
    });
    it('does not merge logical sources or let B clear A', () => {
        const b = {name: 'report', hardware_id: 'other'};
        const s = apply(fault(1), {...fault(2, 2), identity: b}, {...ok(3), identity: b});
        expect(s.episodes.map(e => [e.identity.hardware_id, e.state])).toEqual([['source', 'active'], ['other', 'resolved']]);
    });
    it('ignores duplicate, regressive and unsafe local orders without inspecting ROS stamps', () => {
        const s = apply(fault(10));
        for (const order of [10, 9, -1, 0, 1.5, Infinity, Number.MAX_SAFE_INTEGER + 1]) expect(reduce(s, ok(order))).toBe(s);
        expect(reduce(s, ok(20)).episodes[0].state).toBe('resolved');
    });
    it('refuses old or mismatching scopes and starts a new context empty', () => {
        const s = apply(fault(1)); expect(reduce(s, {...ok(2), scope: Symbol()})).toBe(s);
        expect(reduce(s, {...ok(2), context: 'robot-B'})).toBe(s);
        const next = create(Symbol(), 'robot-B'); expect(next.episodes).toEqual([]);
        expect(ack(next, read(s).receipt)).toBe(next);
    });
    it('keeps input state/identity immutable and does not retain caller payload extensions', () => {
        const source = {...identity}; const input = {...fault(1), identity: source, payload: {status: ['not retained']}};
        const empty = create(scope, context); const s = reduce(empty, input); source.name = 'changed later';
        expect(empty.episodes).toEqual([]); expect(first(s).identity.name).toBe('report');
        expect(first(s)).not.toHaveProperty('payload'); expect(Object.isFrozen(first(s))).toBe(true);
        expect(Object.isFrozen(s.episodes)).toBe(true);
    });
});

describe('consultation applies only to the presented scope and content versions', () => {
    it('reading is non destructive and does not acknowledge anything', () => {
        const s = apply(fault(1)); expect(read(s).hasUnconsulted).toBe(true); expect(read(s).episodes).toEqual(read(s).episodes);
        expect(first(s).unconsulted).toBe(true);
    });
    it('does not acknowledge an event arriving between read and acknowledgement', () => {
        let s = apply(fault(1)); const receipt = read(s).receipt;
        s = reduce(s, {...fault(2), identity: {name: 'B', hardware_id: ''}}); s = ack(s, receipt);
        expect(s.episodes.map(e => e.unconsulted)).toEqual([false, true]);
    });
    it('does not acknowledge an aggravation or resolution that changed a presented episode', () => {
        const s = apply(fault(1)), receipt = read(s).receipt;
        expect(first(ack(reduce(s, fault(2, 2)), receipt)).unconsulted).toBe(true);
        expect(first(ack(reduce(s, ok(2)), receipt)).unconsulted).toBe(true);
    });
    it('acknowledges only selected presented episodes and not an omitted loss indicator', () => {
        let s = apply(fault(1), {...fault(2), identity: {name: 'B', hardware_id: ''}});
        s = ack(s, read(s, {ids: [1], includeLosses: false}).receipt);
        expect(s.episodes.map(e => e.unconsulted)).toEqual([false, true]);
    });
    it('identical republication does not invalidate an otherwise current receipt', () => {
        let s = apply(fault(1)); const receipt = read(s).receipt; s = ack(reduce(s, fault(2)), receipt);
        expect(read(s).hasUnconsulted).toBe(false); expect(ack(s, receipt)).toBe(s);
    });
});

describe('bounded capacity and explicit neutral losses', () => {
    const otherFault = (order: number, name: string): DiagnosticEpisodeDecision => ({...fault(order), identity: {name, hardware_id: ''}});
    const otherOk = (order: number, name: string): DiagnosticEpisodeDecision => ({...ok(order), identity: {name, hardware_id: ''}});

    it('keeps active sources at capacity without fabricating OK or an episode for an omitted source', () => {
        let s = create(scope, context, 2);
        for (let i = 1; i <= 3; i++) s = reduce(s, otherFault(i, `${i}`));
        expect(s.episodes.map(e => e.id)).toEqual([1, 2]);
        expect(s.losses).toMatchObject({evicted: 0, omitted: 1, unconsultedEvicted: 0, pending: true});
        expect(s.historyIncomplete).toBe(true);
        s = reduce(s, otherOk(4, '3')); expect(s.episodes.map(e => e.id)).toEqual([1, 2]);
        s = reduce(s, otherFault(5, '3'));
        expect(s.episodes.map(e => [e.id, e.state])).toEqual([[1, 'active'], [2, 'active']]);
        expect(s.losses.omitted).toBe(2);
    });
    it('counts eviction only of resolved consulted records without manufacturing an unread material fault', () => {
        let s = reduce(create(scope, context, 1), fault(1)); s = reduce(s, ok(2)); s = ack(s, read(s).receipt);
        s = reduce(s, otherFault(3, 'B'));
        expect(s.episodes.map(e => e.id)).toEqual([3]);
        expect(s.losses).toMatchObject({evicted: 1, omitted: 0, unconsultedEvicted: 0, pending: false});
        expect(first(s).opening).toBe('first_observed');
        expect(s.historyIncomplete).toBe(true);
    });
    it.each([false, true])('retains consulted active A in A/B/A without rearming A; omission consulted=%s', consultLoss => {
        let s = reduce(create(scope, context, 1), fault(1)); s = ack(s, read(s).receipt);
        const a = first(s);
        s = reduce(s, otherFault(2, 'B'));
        if (consultLoss) s = ack(s, read(s, {ids: []}).receipt);
        expect(read(s).hasUnconsulted).toBe(!consultLoss);
        s = reduce(s, fault(3));
        expect(first(s)).toBe(a);
        expect(first(s)).toMatchObject({id: 1, state: 'active', unconsulted: false, opening: 'first_observed'});
        expect(s.losses).toMatchObject({evicted: 0, omitted: 1, unconsultedEvicted: 0, pending: !consultLoss});
        expect(first(s)).not.toHaveProperty('resolvedOrder');
    });
    it('avoids the former indistinguishable A/C eviction histories by retaining the unresolved identity', () => {
        const prepare = (name: string) => {
            let s = reduce(create(scope, context, 1), otherFault(1, name));
            s = ack(s, read(s).receipt); s = reduce(s, otherFault(2, 'B'));
            return ack(s, read(s, {ids: []}).receipt);
        };
        const returning = prepare('A'), neverObserved = prepare('C');
        expect(returning).not.toEqual(neverObserved);
        const returned = reduce(returning, otherFault(3, 'A')), omitted = reduce(neverObserved, otherFault(3, 'A'));
        expect(first(returned)).toMatchObject({id: 1, identity: {name: 'A'}, unconsulted: false});
        expect(first(omitted)).toMatchObject({id: 1, identity: {name: 'C'}, unconsulted: false});
        expect(returned.losses.omitted).toBe(1); expect(omitted.losses.omitted).toBe(2);
    });
    it('protects resolved unread episodes until their actual content is consulted', () => {
        let s = reduce(create(scope, context, 1), fault(1)); s = reduce(s, ok(2));
        const resolved = first(s);
        s = reduce(s, otherFault(3, 'B'));
        expect(first(s)).toBe(resolved); expect(s.losses.omitted).toBe(1);
        s = ack(s, read(s, {ids: []}).receipt);
        expect(first(s).unconsulted).toBe(true);
        s = reduce(s, otherFault(4, 'B')); expect(s.losses.omitted).toBe(2);
        s = ack(s, read(s).receipt); s = reduce(s, otherFault(5, 'B'));
        expect(first(s)).toMatchObject({id: 5, identity: {name: 'B'}, opening: 'unknown_history', unconsulted: true});
        expect(s.losses).toMatchObject({evicted: 1, omitted: 2, unconsultedEvicted: 0});
    });
    it('does not free a slot merely through consultation, then frees it after explicit resolution', () => {
        let s = reduce(create(scope, context, 1), fault(1)); s = ack(s, read(s).receipt);
        s = reduce(s, otherFault(2, 'B')); expect(first(s).id).toBe(1);
        s = reduce(s, ok(3)); s = reduce(s, otherFault(4, 'B'));
        expect(first(s)).toMatchObject({id: 4, state: 'active', unconsulted: true});
        expect(s.losses).toMatchObject({evicted: 1, omitted: 1, pending: true});
    });
    it.each(['expired', 'unverified', 'unavailable', 'not_applicable'] as const)('never evicts consulted but unresolved %s evidence', classification => {
        let s = reduce(create(scope, context, 1), fault(1)); s = reduce(s, control(2, classification));
        s = ack(s, read(s).receipt); const previous = first(s);
        s = reduce(s, otherFault(3, 'B'));
        expect(first(s)).toBe(previous); expect(s.losses).toMatchObject({evicted: 0, omitted: 1});
        expect(first(s)).not.toHaveProperty('resolvedOrder');
    });
    it('evicts the oldest eligible episode even when the oldest overall episode is active', () => {
        let s = reduce(create(scope, context, 3), fault(1));
        s = reduce(reduce(s, otherFault(2, 'B')), otherOk(3, 'B'));
        s = reduce(reduce(s, otherFault(4, 'C')), otherOk(5, 'C'));
        s = ack(s, read(s).receipt);
        const a = s.episodes[0], c = s.episodes[2];
        s = reduce(s, otherFault(6, 'D'));
        expect(s.episodes.map(e => e.id)).toEqual([1, 4, 6]);
        expect(s.episodes[0]).toBe(a); expect(s.episodes[1]).toBe(c);
        expect(s.losses).toMatchObject({evicted: 1, omitted: 0, pending: false});
    });
    it('counts consecutive omitted observations, not a certain number of distinct incidents', () => {
        let s = reduce(create(scope, context, 1), fault(1));
        for (let i = 2; i <= 10; i++) s = reduce(s, otherFault(i, 'same B'));
        expect(s.lastOrder).toBe(10); expect(s.episodes.map(e => e.id)).toEqual([1]);
        expect(s.losses).toMatchObject({omitted: 9, evicted: 0, version: 10, pending: true});
        expect(reduce(s, otherFault(10, 'same B'))).toBe(s);
        const rejectedB = {...rejected(11, 'fault'), identity: {name: 'same B', hardware_id: ''}};
        const next = reduce(s, rejectedB);
        expect(next.lastOrder).toBe(11); expect(next.losses).toBe(s.losses);
    });
    it('does not acknowledge a newer omission with a receipt for an older loss version', () => {
        let s = reduce(create(scope, context, 1), fault(1)); s = reduce(s, otherFault(2, 'B'));
        const receipt = read(s, {ids: []}).receipt;
        s = reduce(s, otherFault(3, 'B')); s = ack(s, receipt);
        expect(s.losses.pending).toBe(true); expect(s.losses.omitted).toBe(2);
        s = ack(s, read(s, {ids: []}).receipt); expect(s.losses.pending).toBe(false);
        expect(first(s).unconsulted).toBe(true); expect(s.historyIncomplete).toBe(true);
    });
    it('retains loss information until explicitly presented and acknowledged', () => {
        let s = reduce(create(scope, context, 1), fault(1)); s = reduce(s, otherFault(2, 'B'));
        s = ack(s, read(s, {includeLosses: false}).receipt); expect(s.losses.pending).toBe(true);
        expect(read(s).hasUnconsulted).toBe(true); s = ack(s, read(s).receipt); expect(read(s).hasUnconsulted).toBe(false);
    });
    it('bounds identities without truncation or retaining arbitrary strings', () => {
        const s = apply({...fault(1), identity: {name: 'x'.repeat(4097), hardware_id: ''}});
        expect(s.episodes).toEqual([]); expect(s.losses).toMatchObject({omitted: 1, pending: true});
        expect(s.historyIncomplete).toBe(true);
    });
    it('does not silently accept unbounded capacity or context', () => {
        for (const capacity of [0, -1, 65, 1.5, Infinity]) expect(() => create(scope, context, capacity)).toThrow(RangeError);
        expect(() => create(scope, 'x'.repeat(4097))).toThrow(RangeError);
    });
    it('keeps the first 64 active episodes under many sources, with exact observation omission counts', () => {
        let s = create(scope, context);
        for (let i = 1; i <= 1000; i++) s = reduce(s, otherFault(i, `${i}`));
        expect(s.episodes.map(e => e.id)).toEqual(Array.from({length: 64}, (_, i) => i + 1));
        expect(s.losses).toMatchObject({evicted: 0, omitted: 936, unconsultedEvicted: 0, pending: true});
    });
    it('keeps omission counters finite at saturation with newer versions for concurrent acknowledgement', () => {
        let s = reduce(create(scope, context, 1), fault(1));
        s = {...s, losses: {...s.losses, omitted: Number.MAX_SAFE_INTEGER}};
        const receipt = read(s).receipt;
        s = reduce(s, otherFault(2, 'B'));
        expect(s.losses).toMatchObject({omitted: Number.MAX_SAFE_INTEGER, evicted: 0, unconsultedEvicted: 0, version: 2, pending: true});
        expect(ack(s, receipt).losses.pending).toBe(true);
    });
    it('also saturates safe eviction counts without turning consulted history removal into an unread loss', () => {
        let s = reduce(create(scope, context, 1), fault(1)); s = reduce(s, ok(2)); s = ack(s, read(s).receipt);
        s = {...s, losses: {...s.losses, evicted: Number.MAX_SAFE_INTEGER}};
        s = reduce(s, otherFault(3, 'B'));
        expect(s.losses).toMatchObject({evicted: Number.MAX_SAFE_INTEGER, omitted: 0, unconsultedEvicted: 0, pending: false});
    });
    it('accepts an identity exactly at the bound and counts the hardware ID in that bound', () => {
        const s = apply({...fault(1), identity: {name: 'x'.repeat(4095), hardware_id: 'y'}});
        expect(s.episodes).toHaveLength(1);
        expect(reduce(s, {...fault(2), identity: {name: 'x'.repeat(4095), hardware_id: 'yy'}}).losses.omitted).toBe(1);
    });
    it('never lets a receipt for an eligible evicted opening acknowledge a later opening of the same identity', () => {
        let s = reduce(create(scope, context, 1), fault(1)); const receipt = read(s).receipt;
        s = reduce(s, ok(2)); s = ack(s, read(s).receipt); s = reduce(s, otherFault(3, 'B'));
        s = reduce(s, otherOk(4, 'B')); s = ack(s, read(s).receipt); s = reduce(s, fault(5));
        expect(first(ack(s, receipt))).toMatchObject({id: 5, unconsulted: true, opening: 'unknown_history'});
    });
    it('does not admit or acknowledge omissions from another scope or context', () => {
        let s = reduce(create(scope, context, 1), fault(1)); s = reduce(s, otherFault(2, 'B'));
        expect(reduce(s, {...otherFault(3, 'C'), scope: Symbol('other')})).toBe(s);
        expect(reduce(s, {...otherFault(3, 'C'), context: 'other'})).toBe(s);
        expect(ack(s, {...read(s).receipt, scope: Symbol('other')})).toBe(s);
        const next = create(Symbol('new'), context, 1);
        expect(ack(next, read(s).receipt)).toBe(next);
        expect(next.losses).toMatchObject({omitted: 0, pending: false});
    });
    it('bounds presentation and acknowledgement work as well as stored episodes', () => {
        const s = reduce(create(scope, context, 1), fault(1));
        expect(() => read(s, {ids: [1, 2]})).toThrow(RangeError);
        expect(ack(s, {...read(s).receipt, context: 'other-context'})).toBe(s);
        expect(ack(s, {...read(s).receipt, episodes: [{id: 1, version: 1}, {id: 2, version: 2}]})).toBe(s);
    });
});
