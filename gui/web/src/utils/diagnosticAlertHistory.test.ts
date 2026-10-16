import {describe, expect, it} from 'vitest';
import {advanceDiagnosticAlertHistory as advance, alertEpisodeVisible, diagnosticAlertGroupKey, ALERT_HISTORY_LIMIT,
    type CurrentDiagnosticAlert, type DiagnosticAlertEpisode} from './diagnosticAlertHistory';
import {deriveDiagnosticHealth} from './diagnosticHealth';
import {healthy} from '../test/diagnosticHealthFixture';

const report = (level = 1, name = 'component/detail', hardware_id = 'source-A'): CurrentDiagnosticAlert => ({
    name, hardware_id, level, reportedLevel: level, message: `Reported ${level}`, values: [{key: 'value', value: '0'}], receivedAt: 0, notApplicable: false,
});
const step = (previous: readonly DiagnosticAlertEpisode[], now: number, alerts: CurrentDiagnosticAlert[], options: {
    connected?: boolean; observations?: CurrentDiagnosticAlert[]; openGroups?: string[];
} = {}) => advance(previous, {now, alerts, observations: options.observations ?? alerts,
    connected: options.connected ?? true, openGroups: options.openGroups ?? []});
const resolve = (previous: readonly DiagnosticAlertEpisode[], now: number, entry = report()) => step(previous, now, [], {observations: [{...entry, level: 0, message: 'OK'}]});

describe('visual diagnostic episodes independent of current health', () => {
    it('retains WARN → OK → WARN as one source row with an identifiable new episode', () => {
        const first = step([], 0, [report()]);
        const resolved = resolve(first, 1000);
        expect(resolved[0]).toMatchObject({state: 'resolved', firstSeen: 0, report: {level: 1, message: 'Reported 1'}});
        expect(alertEpisodeVisible(resolved[0], 29999, [], false)).toBe(true);
        expect(alertEpisodeVisible(resolved[0], 30000, [], false)).toBe(false);
        const again = step(resolved, 2000, [{...report(), message: 'New warning'}]);
        expect(again).toHaveLength(1);
        expect(again[0]).toMatchObject({episode: 2, firstSeen: 2000, state: 'active', report: {message: 'New warning'}});
        expect(again[0].key).toBe(first[0].key);
    });
    it('does not reset the minimum lifetime on repeated 1 Hz publications or repeated OK', () => {
        let entries = step([], 0, [report()]);
        for (let i = 1; i < 20; i++) entries = step(entries, i * 1000, [{...report(), receivedAt: i * 1000}]);
        expect(entries).toHaveLength(1); expect(entries[0].firstSeen).toBe(0);
        entries = resolve(entries, 20000);
        entries = resolve(entries, 29000);
        expect(entries[0].endedAt).toBe(20000);
        expect(alertEpisodeVisible(entries[0], 30000, [], false)).toBe(false);
    });
    it('updates ERROR immediately and retains its last message, severity and values on OK', () => {
        const warning = step([], 0, [report()]);
        const error = step(warning, 10, [{...report(2), message: 'Confirmed error'}]);
        expect(error[0].report.level).toBe(2); expect(error[0].firstSeen).toBe(0);
        const resolved = resolve(error, 20);
        expect(resolved[0]).toMatchObject({state: 'resolved', report: {reportedLevel: 2, message: 'Confirmed error', values: [{key: 'value', value: '0'}]}});
    });
    it.each(['disconnect', 'missing', 'stale', 'malformed'])('marks %s unverified, never resolved without matching admissible OK', reason => {
        const first = step([], 0, [report(2)]);
        const stale = {...report(2), level: 3};
        const next = step(first, 1000, reason === 'stale' ? [stale] : [], {connected: reason !== 'disconnect'});
        expect(next[0]).toMatchObject({state: 'unverified', report: {reportedLevel: 2, message: 'Reported 2'}});
        const cachedOk = step(next, 2000, [{...report(0), level: 3}], {observations: [report(0)]});
        expect(cachedOk[0].state).toBe('unverified'); expect(cachedOk[0].report.reportedLevel).toBe(2);
        expect(resolve(cachedOk, 3000)[0].state).toBe('resolved');
    });
    it('distinguishes actual ROS STALE from confirmed ERROR while preserving reported provenance', () => {
        const entries = step([], 0, [report(3), report(2, 'other/detail')]);
        expect(entries.map(entry => [entry.state, entry.report.reportedLevel])).toEqual([['unverified', 3], ['active', 2]]);
    });
    it('never accepts OK from a different source and retains individual keys within a component', () => {
        const first = step([], 0, [report(), report(1, 'component/second'), report(2, 'component/detail', 'source-B')]);
        expect(new Set(first.map(entry => entry.key)).size).toBe(3);
        expect(new Set(first.map(entry => entry.groupKey)).size).toBe(2);
        const next = step(first, 1000, [], {observations: [report(0, 'component/detail', 'source-C')]});
        expect(next.every(entry => entry.state === 'unverified')).toBe(true);
    });
    it('protects every entry of an open group beyond expiry and allows purge only after explicit collapse', () => {
        const first = step([], 0, [report()]);
        const openGroups = [first[0].groupKey];
        const resolved = resolve(first, 1000);
        const pinned = step(resolved, 900000, [], {observations: [report(0)], openGroups});
        expect(pinned).toHaveLength(1);
        expect(alertEpisodeVisible(pinned[0], 900000, openGroups, false)).toBe(true);
        expect(step(pinned, 900001, [], {observations: [report(0)]})).toEqual([]);
    });
    it('exposes recent history explicitly then expires it after five minutes', () => {
        const entries = resolve(step([], 0, [report()]), 1000);
        expect(alertEpisodeVisible(entries[0], 31000, [], false)).toBe(false);
        expect(alertEpisodeVisible(entries[0], 31000, [], true)).toBe(true);
        expect(step(entries, 300999, [], {observations: [report(0)]})).toHaveLength(1);
        expect(step(entries, 301000, [], {observations: [report(0)]})).toEqual([]);
    });
    it('bounds eligible history without discarding young, current or open alerts', () => {
        const alerts = Array.from({length: ALERT_HISTORY_LIMIT + 10}, (_, index) => report(1, `component-${index}/detail`));
        const first = step([], 0, alerts);
        const ok = alerts.map(entry => ({...entry, level: 0}));
        const resolved = step(first, 1000, [], {observations: ok});
        expect(resolved).toHaveLength(110);
        const bounded = step(resolved, 31000, [], {observations: ok, openGroups: [first[0].groupKey]});
        expect(bounded).toHaveLength(101); expect(bounded[0].key).toBe(first[0].key);
        expect(step(first, 31000, alerts)).toHaveLength(110);
    });
    it('keeps insertion order as severity changes, and never mutates input', () => {
        const first = step([], 0, [report(1, 'first/detail'), report(1, 'second/detail')]);
        const before = structuredClone(first);
        const next = step(first, 100, [report(1, 'first/detail'), report(2, 'second/detail')]);
        expect(next.map(entry => entry.key)).toEqual(first.map(entry => entry.key)); expect(first).toEqual(before);
        expect(diagnosticAlertGroupKey(report(1, 'first/detail'))).toBe(first[0].groupKey);
    });
    it('does not create a fictional alert from healthy observations', () => {
        expect(step([], 0, [], {observations: [report(0)]})).toEqual([]);
    });
    it('never feeds retained severity into the current verdict; emergency remains immediately critical', () => {
        const input = healthy();
        const fault = {...report(2, 'Motors', 'mowgli/motors'), values: [{key: 'esc_temperature_c', value: '100'}], receivedAt: input.nowMs};
        input.diagnostics = [fault];
        const failed = deriveDiagnosticHealth(input);
        const retained = step([], 0, failed.alerts);
        expect(failed.level).toBe('critical');
        input.diagnostics = [{...fault, level: 0, message: 'OK'}];
        const current = deriveDiagnosticHealth(input);
        const history = advance(retained, {now: 1000, alerts: current.alerts, observations: input.diagnostics, connected: true, openGroups: []});
        expect(history[0].report.reportedLevel).toBe(2); expect(history[0].state).toBe('resolved');
        expect(current.level).toBe('ok');
        input.emergency.active_emergency = true;
        expect(deriveDiagnosticHealth(input).level).toBe('critical');
    });
});
