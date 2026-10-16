import type {TimestampedDiagnosticStatus} from '../hooks/useDiagnostics';
import type {deriveDiagnosticHealth} from './diagnosticHealth';

export type CurrentDiagnosticAlert = ReturnType<typeof deriveDiagnosticHealth>['alerts'][number];
export type AlertEpisodeState = 'active' | 'resolved' | 'unverified';
export interface DiagnosticAlertEpisode {
    key: string;
    groupKey: string;
    episode: number;
    firstSeen: number;
    endedAt?: number;
    state: AlertEpisodeState;
    /** Current health projection still contains this source/name. */
    present: boolean;
    /** Last non-OK report, retained independently of any later OK report. */
    report: CurrentDiagnosticAlert;
}
export const ALERT_MIN_VISIBLE_MS = 30_000;
export const ALERT_HISTORY_MS = 5 * 60_000;
export const ALERT_HISTORY_LIMIT = 100;
export const diagnosticAlertKey = (entry: Pick<TimestampedDiagnosticStatus, 'name' | 'hardware_id'>) => JSON.stringify([entry.hardware_id, entry.name]);
export const diagnosticAlertGroupKey = (entry: Pick<TimestampedDiagnosticStatus, 'name' | 'hardware_id'>) => JSON.stringify([entry.hardware_id, entry.name.split('/')[0]]);

/** Visual history only. Never feeds health, freshness, commands or permissions.
 * now is a browser monotonic time supplied by the caller, not a producer clock. */
export function advanceDiagnosticAlertHistory(previous: readonly DiagnosticAlertEpisode[], input: {
    alerts: readonly CurrentDiagnosticAlert[];
    observations: readonly TimestampedDiagnosticStatus[];
    connected: boolean;
    now: number;
    openGroups: readonly string[];
}): DiagnosticAlertEpisode[] {
    const {alerts, observations, connected, now, openGroups} = input;
    const current = new Map(alerts.map(entry => [diagnosticAlertKey(entry), entry]));
    const observationsByKey = new Map(observations.map(entry => [diagnosticAlertKey(entry), entry]));
    const known = new Map(previous.map(entry => [entry.key, entry]));
    const next = previous.map(entry => {
        const alert = current.get(entry.key);
        if (alert) return updateEpisode(entry, alert, connected, now);
        // Absence from health.alerts proves OK only when the same source/name
        // has an explicit OK observation admitted by the current health model.
        const observed = observationsByKey.get(entry.key);
        const state = connected && observed?.level === 0 ? 'resolved' : 'unverified';
        return {...entry, state, present: false,
            endedAt: state === entry.state ? entry.endedAt : now} satisfies DiagnosticAlertEpisode;
    });
    for (const alert of alerts) {
        const key = diagnosticAlertKey(alert);
        if (known.has(key)) continue;
        next.push({key, groupKey: diagnosticAlertGroupKey(alert), episode: 1, firstSeen: now,
            state: connected && alert.level !== 3 ? 'active' : 'unverified', present: true,
            ...(connected && alert.level !== 3 ? {} : {endedAt: now}), report: alert});
    }
    const protectedEntry = (entry: DiagnosticAlertEpisode) => entry.present || openGroups.includes(entry.groupKey)
        || now - entry.firstSeen < ALERT_MIN_VISIBLE_MS;
    const recent = next.filter(entry => protectedEntry(entry) || now - (entry.endedAt ?? entry.firstSeen) < ALERT_HISTORY_MS);
    // Bound eligible history, never evict a currently reported/open episode or
    // violate its minimum visibility. Bursts can temporarily exceed the cap.
    const eligible = recent.filter(entry => !protectedEntry(entry))
        .sort((a, b) => (a.endedAt ?? a.firstSeen) - (b.endedAt ?? b.firstSeen));
    const keep = new Set(eligible.slice(-ALERT_HISTORY_LIMIT).map(entry => entry.key));
    return recent.filter(entry => protectedEntry(entry) || keep.has(entry.key));
}
function updateEpisode(previous: DiagnosticAlertEpisode, alert: CurrentDiagnosticAlert, connected: boolean, now: number): DiagnosticAlertEpisode {
    const state = connected && alert.level !== 3 ? 'active' : 'unverified';
    const reappeared = previous.state === 'resolved' && state === 'active';
    return {...previous, state, present: true,
        episode: previous.episode + (reappeared ? 1 : 0), firstSeen: reappeared ? now : previous.firstSeen,
        endedAt: state === 'active' ? undefined : previous.state === state ? previous.endedAt : now,
        // A stale OK must not erase the last warning/error or imply recovery.
        report: alert.reportedLevel === 0 && previous.report.reportedLevel !== 0 ? previous.report : alert};
}
export function alertEpisodeVisible(entry: DiagnosticAlertEpisode, now: number, openGroups: readonly string[], showRecent: boolean): boolean {
    return entry.present || now - entry.firstSeen < ALERT_MIN_VISIBLE_MS || openGroups.includes(entry.groupKey) || showRecent;
}
